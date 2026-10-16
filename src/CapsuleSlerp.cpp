// R59_LOG_POLICY: per-frame R35 telemetry moved to debug; physical violations warning-throttled.
// BUILD_ID: INCESSUS-C1-V13-FIREWALL-PMD-R1R2-2026-10-03
//
// R74 intentionally starts from the exact R72/R35 CapsuleSlerp baseline. The experimental
// R73 locomotion overshoot envelope is NOT carried forward because runtime observation did
// not eliminate the visible wolf tremor. R35 locomotion remains PASS NATIVE at V13 and
// SteeringIntentController -> ProcessMotionData R1/R2 remains the only procedural yaw source.
//
// New R74 policy is deliberately orthogonal: when the profile's own graph witness says the
// actor is STANDING, V13 may not rotate the physical capsule unless an authored RotationClip
// or an explicit action bypass owns the frame. This prevents invisible capsule turns while
// idle/standing without reintroducing the broad R64 no-carrier freeze. Unknown graph witness
// fails open to native behavior. Moving locomotion, authored turns and actions are unchanged.
//
#include "PCH.h"
#include "CapsuleSlerp.h"
#include "MotionProfileDatabase.h"
#include "SyntheticMotionInjector.h"
#include "SteeringIntentController.h"
#include "SpeedIntentTranslator.h"

#include "RE/A/AIProcess.h"
#include "RE/B/bhkCharacterController.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BShkbAnimationGraph.h"
#include "RE/C/Character.h"
#include "RE/H/hkTransform.h"
#include "RE/M/MiddleHighProcessData.h"

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstring>
#include <cmath>
#include <cstdint>
#include <immintrin.h>
#include <memory>
#include <limits>
#include <mutex>
#include <unordered_map>

#include <Windows.h>

namespace CapsuleSlerp
{
    namespace
    {
        constexpr std::size_t kAngularTransformVFunc = 0x13;
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 6.28318530717958647692f;
        constexpr float kRadToDeg = 57.29577951308232f;
        constexpr float kDegToRad = 0.01745329251994329577f;
        constexpr float kMinDeltaTime = 1.0e-5f;
        constexpr float kMaxDeltaTime = 0.100f;
        constexpr float kFallbackDeltaTime = 1.0f / 60.0f;

        using AngularTransformFn = void (*)(RE::bhkCharacterController*, RE::hkTransform&);

        struct ControllerHook
        {
            std::uintptr_t* vtable{ nullptr };
            AngularTransformFn original{ nullptr };
        };

        enum class LocomotionWitnessSource : std::uint8_t
        {
            kNone = 0,
            kAnimationPriority,
            kSpeedIntent,
            kUniversalMovementDemand
        };

        struct LocomotionGraphWitness
        {
            bool readable{ false };
            bool active{ false };
            LocomotionWitnessSource source{ LocomotionWitnessSource::kNone };
        };


        // R109: final behavior-output decoder. This deliberately reads the BShkbAnimationGraph
        // generator output AFTER Havok has composed state machines/blenders. It does not inspect
        // MTTurn child generators, animationControl weights, localTime, TurnDelta, or playback rate.
        enum class ExactMemoryAccess : std::uint8_t { kRead, kWrite };

        [[nodiscard]] bool IsExactRangeAccessible(
            const void* pointer,
            std::size_t bytes,
            ExactMemoryAccess access) noexcept
        {
            if (!pointer || bytes == 0) return false;
            const auto begin = reinterpret_cast<std::uintptr_t>(pointer);
            constexpr std::uintptr_t kMinUser = 0x0000000000010000ULL;
            constexpr std::uintptr_t kMaxUser = 0x00007FFFFFFFFFFFULL;
            if (begin < kMinUser || begin > kMaxUser || bytes - 1 > kMaxUser - begin) return false;
            const auto end = begin + bytes;
            auto cursor = begin;
            while (cursor < end) {
                MEMORY_BASIC_INFORMATION mbi{};
                if (::VirtualQuery(reinterpret_cast<LPCVOID>(cursor), &mbi, sizeof(mbi)) != sizeof(mbi) ||
                    mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) != 0) {
                    return false;
                }
                const DWORD prot = mbi.Protect & 0xFFu;
                const bool readable = prot == PAGE_READONLY || prot == PAGE_READWRITE || prot == PAGE_WRITECOPY ||
                    prot == PAGE_EXECUTE_READ || prot == PAGE_EXECUTE_READWRITE || prot == PAGE_EXECUTE_WRITECOPY;
                const bool writable = prot == PAGE_READWRITE || prot == PAGE_WRITECOPY ||
                    prot == PAGE_EXECUTE_READWRITE || prot == PAGE_EXECUTE_WRITECOPY;
                if (!readable || (access == ExactMemoryAccess::kWrite && !writable)) return false;
                const auto regionBegin = reinterpret_cast<std::uintptr_t>(mbi.BaseAddress);
                if (mbi.RegionSize == 0 ||
                    regionBegin > (std::numeric_limits<std::uintptr_t>::max)() - mbi.RegionSize) {
                    return false;
                }
                const auto regionEnd = regionBegin + mbi.RegionSize;
                if (regionEnd <= cursor) return false;
                cursor = std::min(regionEnd, end);
            }
            return true;
        }

        struct GeneratorOutputTrackMasterHeaderView
        {
            std::int32_t numBytes;
            std::int32_t numTracks;
            std::byte unused[8];
        };
        static_assert(sizeof(GeneratorOutputTrackMasterHeaderView) == 0x10);

        struct GeneratorOutputTrackHeaderView
        {
            std::int16_t capacity;
            std::int16_t numData;
            std::int16_t dataOffset;
            std::int16_t elementSizeBytes;
            float onFraction;
            std::int8_t flags;
            std::int8_t type;
            std::uint16_t pad;
        };
        static_assert(sizeof(GeneratorOutputTrackHeaderView) == 0x10);

        struct GeneratorOutputView
        {
            const std::byte* tracks;
            bool deleteTracks;
        };
        static_assert(offsetof(GeneratorOutputView, tracks) == 0x0);

        struct FinalBehaviorYawCandidate
        {
            bool valid{ false };
            float extractedYawDegrees{ 0.0f };
            float onFraction{ 0.0f };
        };

        struct FinalBehaviorYawProbe
        {
            bool graphReadable{ false };
            bool ambiguous{ false };
            std::int32_t selectedOutput{ -1 };
            float extractedYawDegrees{ 0.0f };
            float physicalYawDegrees{ 0.0f };
            float interpolationAmount{ 0.0f };
            float onFraction{ 0.0f };
            FinalBehaviorYawCandidate candidates[2]{};
        };

        [[nodiscard]] float QuaternionYawDegrees(
            float qx, float qy, float qz, float qw) noexcept
        {
            if (!std::isfinite(qx) || !std::isfinite(qy) ||
                !std::isfinite(qz) || !std::isfinite(qw)) {
                return 0.0f;
            }
            const float lenSq = qx * qx + qy * qy + qz * qz + qw * qw;
            if (!(lenSq > 1.0e-12f) || !std::isfinite(lenSq)) return 0.0f;
            const float invLen = 1.0f / std::sqrt(lenSq);
            qx *= invLen;
            qy *= invLen;
            qz *= invLen;
            qw *= invLen;
            const float yaw = std::atan2(
                2.0f * (qw * qz + qx * qy),
                1.0f - 2.0f * (qy * qy + qz * qz));
            return yaw * kRadToDeg;
        }

        [[nodiscard]] FinalBehaviorYawCandidate ProbeFinalBehaviorExtractedYaw(
            const void* opaqueOutput) noexcept
        {
            FinalBehaviorYawCandidate result{};
            if (!opaqueOutput ||
                !IsExactRangeAccessible(opaqueOutput, sizeof(GeneratorOutputView), ExactMemoryAccess::kRead)) {
                return result;
            }

            GeneratorOutputView output{};
            std::memcpy(std::addressof(output), opaqueOutput, sizeof(output));
            if (!output.tracks ||
                !IsExactRangeAccessible(output.tracks, sizeof(GeneratorOutputTrackMasterHeaderView), ExactMemoryAccess::kRead)) {
                return result;
            }

            GeneratorOutputTrackMasterHeaderView master{};
            std::memcpy(std::addressof(master), output.tracks, sizeof(master));
            constexpr std::int32_t kMaxTracks = 64;
            constexpr std::int32_t kMaxTrackBytes = 1024 * 1024;
            if (master.numTracks <= 1 || master.numTracks > kMaxTracks ||
                master.numBytes < static_cast<std::int32_t>(sizeof(master)) ||
                master.numBytes > kMaxTrackBytes) {
                return result;
            }

            constexpr std::int32_t kExtractedMotionTrack = 1;
            const std::int64_t headerOffset =
                static_cast<std::int64_t>(sizeof(GeneratorOutputTrackMasterHeaderView)) +
                static_cast<std::int64_t>(kExtractedMotionTrack) *
                    static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView));
            if (headerOffset < 0 ||
                headerOffset + static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView)) > master.numBytes) {
                return result;
            }

            const auto* headerAddress = output.tracks + headerOffset;
            if (!IsExactRangeAccessible(headerAddress, sizeof(GeneratorOutputTrackHeaderView), ExactMemoryAccess::kRead)) {
                return result;
            }
            GeneratorOutputTrackHeaderView header{};
            std::memcpy(std::addressof(header), headerAddress, sizeof(header));

            // Havok QSTRANSFORM track type == 1. onFraction == 0 means this output buffer's
            // track is not active; this is how the two generatorOutputs can be distinguished
            // without assuming which pointer was swapped into slot 0 or 1 this frame.
            if (!std::isfinite(header.onFraction) || header.onFraction <= 0.0f ||
                header.type != 1 || header.numData <= 0 || header.capacity < header.numData ||
                header.dataOffset < 0 || header.elementSizeBytes < 0x20) {
                return result;
            }
            if (static_cast<std::int64_t>(header.dataOffset) + 0x20 > master.numBytes) {
                return result;
            }

            const auto* transformAddress = output.tracks + header.dataOffset;
            if (!IsExactRangeAccessible(transformAddress, 0x20, ExactMemoryAccess::kRead)) {
                return result;
            }
            std::array<float, 4> rotation{};
            std::memcpy(rotation.data(), transformAddress + 0x10, sizeof(rotation));
            const float yawDegrees = QuaternionYawDegrees(
                rotation[0], rotation[1], rotation[2], rotation[3]);
            if (!std::isfinite(yawDegrees)) return result;

            result.valid = true;
            result.extractedYawDegrees = yawDegrees;
            result.onFraction = header.onFraction;
            return result;
        }

        [[nodiscard]] FinalBehaviorYawProbe ProbeFinalBehaviorYaw(RE::Actor* actor) noexcept
        {
            FinalBehaviorYawProbe result{};
            if (!actor) return result;

            RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
            if (!actor->GetAnimationGraphManager(manager) || !manager) return result;
            const auto activeGraph = manager->GetRuntimeData().activeGraph;
            if (activeGraph >= manager->graphs.size()) return result;
            auto* graph = manager->graphs[activeGraph].get();
            if (!graph) return result;
            result.graphReadable = true;

            for (std::int32_t i = 0; i < 2; ++i) {
                result.candidates[i] = ProbeFinalBehaviorExtractedYaw(graph->generatorOutputs[i]);
            }

            const bool v0 = result.candidates[0].valid;
            const bool v1 = result.candidates[1].valid;
            std::int32_t selected = -1;
            if (v0 != v1) {
                selected = v0 ? 0 : 1;
            } else if (v0 && v1) {
                const float a0 = graph->interpolationAmounts[0];
                const float a1 = graph->interpolationAmounts[1];
                const bool amountsFinite = std::isfinite(a0) && std::isfinite(a1);
                if (amountsFinite && std::abs(a0 - a1) > 1.0e-4f) {
                    selected = a0 > a1 ? 0 : 1;
                } else {
                    const float f0 = result.candidates[0].onFraction;
                    const float f1 = result.candidates[1].onFraction;
                    if (std::abs(f0 - f1) > 1.0e-4f) {
                        selected = f0 > f1 ? 0 : 1;
                    } else {
                        result.ambiguous = true;
                        return result;
                    }
                }
            } else {
                return result;
            }

            result.selectedOutput = selected;
            result.extractedYawDegrees = result.candidates[selected].extractedYawDegrees;
            // Skyrim/Havok extracted-motion yaw is opposite the bhkCharacterController world-yaw
            // convention. R104 leaf telemetry established +authored yaw for left while physical
            // left turns decreased world yaw, and vice versa for right.
            result.physicalYawDegrees = -result.extractedYawDegrees;
            result.onFraction = result.candidates[selected].onFraction;
            result.interpolationAmount = graph->interpolationAmounts[selected];
            return result;
        }

        struct OwnerState
        {
            RE::Character* actor{ nullptr };
            RE::FormID formID{ 0 };
            bool enabled{ false };

            std::uint64_t frameSerial{ 0 };
            std::uint32_t callsThisFrame{ 0 };
            std::uint32_t previousFrameCalls{ 0 };
            float frameDelta{ kFallbackDeltaTime };
            float lastValidDelta{ kFallbackDeltaTime };
            float frameStartYaw{ 0.0f };
            bool frameStartValid{ false };

            float responsePerSecond{ 4.0f };
            float maxYawRateDegreesPerSecond{ 75.0f };
            float minErrorDegrees{ 0.10f };
            std::uint32_t telemetryEveryFrames{ 60 };
            std::uint32_t telemetryBurstRemaining{ 24 };

            // R28 frame-boundary policy snapshot. All profile/graph/injector queries
            // happen in Character::Update, never inside the physical V13 callback.
            bool animationDrivenAtFrameStart{ false };
            bool actionBypassAtFrameStart{ false };
            bool authoredTurnAtFrameStart{ false };
            bool runtimeCarrierAtFrameStart{ false };
            bool universalNativeCarrierAtFrameStart{ false };
            bool profilelessUniversalAtFrameStart{ false };
            bool locomotionGraphWitnessReadableAtFrameStart{ false };
            bool locomotionGraphActiveAtFrameStart{ false };
            LocomotionWitnessSource locomotionGraphWitnessSourceAtFrameStart{ LocomotionWitnessSource::kNone };

            // R104 diagnostic-only StablePivot context. No policy reads these fields.
            bool externalWitnessActiveAtFrameStart{ false };
            bool externalYawPassAtFrameStart{ false };
            std::uint8_t stablePivotPhaseAtFrameStart{ 0 };
            std::uint32_t stablePivotRouteDegreesAtFrameStart{ 0 };
            bool stablePivotNativeHkbWitnessAtFrameStart{ false };
            float stablePivotNativeHkbWitnessAgeSecondsAtFrameStart{ -1.0f };
            std::string stablePivotNativeHkbKeyAtFrameStart;
            bool stablePivotTurnDeltaReadAtFrameStart{ false };
            float stablePivotTurnDeltaAtFrameStart{ 0.0f };
            float stablePivotPhysicalYawRateDegreesPerSecondAtFrameStart{ 0.0f };

            // R115: profile-disabled CapsuleSlerp may wake only for the serialized native
            // standing-turn phase. The turn blender remains Bethesda-owned; this context
            // carries only the requested physical cadence and read-only TurnDelta witness.
            bool nativeStandingTurnRateActiveAtFrameStart{ false };
            float nativeStandingTurnPhysicalYawRateDegreesPerSecondAtFrameStart{ 0.0f };
            bool nativeStandingTurnTurnDeltaReadAtFrameStart{ false };
            float nativeStandingTurnTurnDeltaAtFrameStart{ 0.0f };

            std::uint64_t totalV13Calls{ 0 };
            std::uint64_t nativeTargetSamples{ 0 };
            std::uint64_t targetLimited{ 0 };
            std::uint64_t targetUnchanged{ 0 };
            std::uint64_t turnBypasses{ 0 };
            std::uint64_t actionBypasses{ 0 };
            std::uint64_t noAnimationDrivenBypasses{ 0 };
            std::uint64_t noLocomotionCarrierBypasses{ 0 };
            std::uint64_t proceduralCarrierCalls{ 0 };
            std::uint64_t commandSamples{ 0 };
            std::uint64_t commandClamps{ 0 };
            std::uint64_t missingMiddleHigh{ 0 };
            std::uint64_t invalidCommands{ 0 };
            std::uint64_t realizedSamples{ 0 };
            std::uint64_t realizedViolations{ 0 };
            std::uint64_t standingYawBlocked{ 0 };
            std::uint64_t standingYawPassAuthoredTurn{ 0 };
            std::uint64_t standingYawPassAction{ 0 };

            float lastNativeTargetYaw{ 0.0f };
            float lastNativeError{ 0.0f };
            float lastLimitedStep{ 0.0f };
            float lastCommandInput{ 0.0f };
            float lastCommandOutput{ 0.0f };
        };

        struct OwnerSnapshot
        {
            RE::Character* actor{ nullptr };
            RE::FormID formID{ 0 };
            bool enabled{ false };
            std::uint64_t frameSerial{ 0 };
            std::uint32_t callIndex{ 0 };
            float deltaTime{ kFallbackDeltaTime };
            float frameStartYaw{ 0.0f };
            bool frameStartValid{ false };
            float responsePerSecond{ 4.0f };
            float maxYawRateDegreesPerSecond{ 75.0f };
            float minErrorDegrees{ 0.10f };
            std::uint32_t telemetryEveryFrames{ 60 };
            bool burstLog{ false };
            bool animationDriven{ false };
            bool actionBypass{ false };
            bool recentTurnWitness{ false };
            bool recentLocomotionWitness{ false };
            bool universalNativeCarrier{ false };
            bool profilelessUniversal{ false };
            bool locomotionGraphWitnessReadable{ false };
            bool locomotionGraphActive{ false };
            LocomotionWitnessSource locomotionGraphWitnessSource{ LocomotionWitnessSource::kNone };

            bool externalWitnessActive{ false };
            bool externalYawPass{ false };
            std::uint8_t stablePivotPhase{ 0 };
            std::uint32_t stablePivotRouteDegrees{ 0 };
            bool stablePivotNativeHkbWitness{ false };
            float stablePivotNativeHkbWitnessAgeSeconds{ -1.0f };
            std::string stablePivotNativeHkbKey;
            bool stablePivotTurnDeltaRead{ false };
            float stablePivotTurnDelta{ 0.0f };
            float stablePivotPhysicalYawRateDegreesPerSecond{ 0.0f };

            bool nativeStandingTurnRateActive{ false };
            float nativeStandingTurnPhysicalYawRateDegreesPerSecond{ 0.0f };
            bool nativeStandingTurnTurnDeltaRead{ false };
            float nativeStandingTurnTurnDelta{ 0.0f };
        };

        struct CommandClampResult
        {
            bool available{ false };
            bool finite{ false };
            bool clamped{ false };
            float inputRadPerSecond{ 0.0f };
            float outputRadPerSecond{ 0.0f };
        };

        std::unordered_map<std::uintptr_t*, ControllerHook> g_hooks;
        std::unordered_map<RE::bhkCharacterController*, OwnerState> g_owners;
        std::unordered_map<RE::FormID, RE::bhkCharacterController*> g_actorControllers;
        // R103: narrow external witness published by Hooks::StablePivot. It exists because
        // native Troll turn clips are valid behavior clips but are not MotionDB-backed, so
        // the R74 standing firewall otherwise cannot distinguish them from idle yaw.
        struct ExternalAuthoredTurnWitness
        {
            bool active{ false };
            bool allowNativeStandingYaw{ false };
        };
        std::unordered_map<RE::FormID, ExternalAuthoredTurnWitness> g_externalAuthoredTurnWitnesses;

        struct NativeStandingTurnRateContext
        {
            bool active{ false };
            float signedYawRateDegreesPerSecond{ 0.0f };
            bool turnDeltaRead{ false };
            float turnDelta{ 0.0f };
        };
        std::unordered_map<RE::FormID, NativeStandingTurnRateContext> g_nativeStandingTurnRateContexts;

        struct StablePivotDiagnosticContext
        {
            std::uint8_t phase{ 0 };
            std::uint32_t routeDegrees{ 0 };
            bool nativeHkbWitness{ false };
            float nativeHkbWitnessAgeSeconds{ -1.0f };
            std::string nativeHkbKey;
            bool turnDeltaRead{ false };
            float turnDelta{ 0.0f };
            float physicalYawRateDegreesPerSecond{ 0.0f };
        };
        std::unordered_map<RE::FormID, StablePivotDiagnosticContext> g_stablePivotDiagnostics;
        std::mutex g_mutex;

        [[nodiscard]] float NormalizeRadians(float value) noexcept
        {
            if (!std::isfinite(value)) {
                return 0.0f;
            }
            value = std::fmod(value, kTwoPi);
            if (value > kPi) value -= kTwoPi;
            if (value < -kPi) value += kTwoPi;
            return value;
        }

        [[nodiscard]] float SanitizeDeltaTime(float candidate, float fallback) noexcept
        {
            if (std::isfinite(candidate) && candidate >= kMinDeltaTime && candidate <= kMaxDeltaTime) {
                return candidate;
            }
            if (std::isfinite(fallback) && fallback >= kMinDeltaTime && fallback <= kMaxDeltaTime) {
                return fallback;
            }
            return kFallbackDeltaTime;
        }

        [[nodiscard]] float ComputeAlpha(float responsePerSecond, float deltaTime) noexcept
        {
            if (!std::isfinite(responsePerSecond) || responsePerSecond <= 0.0f ||
                !std::isfinite(deltaTime) || deltaTime <= 0.0f) {
                return 0.0f;
            }
            return std::clamp(1.0f - std::exp(-responsePerSecond * deltaTime), 0.0f, 1.0f);
        }

        [[nodiscard]] bool RuntimeEligible(RE::Character* actor) noexcept
        {
            return actor && !actor->IsPlayerRef() && actor->GetParentCell() && actor->Is3DLoaded() &&
                   !actor->IsDisabled() && !actor->IsDead() &&
                   !actor->IsInRagdollState() && !actor->IsInKillMove() &&
                   !actor->IsInMidair() && !actor->IsInWater() &&
                   !actor->IsBeingRidden();
        }

        [[nodiscard]] float ReadYaw(const RE::hkTransform& transform) noexcept
        {
            alignas(16) float c0[4]{};
            _mm_store_ps(c0, transform.rotation.col0.quad);
            return NormalizeRadians(std::atan2(c0[1], c0[0]));
        }

        void RotateTargetAroundWorldZ(RE::hkTransform& transform, float deltaYaw) noexcept
        {
            if (!std::isfinite(deltaYaw) || std::abs(deltaYaw) < 1.0e-7f) {
                return;
            }

            const float c = std::cos(deltaYaw);
            const float s = std::sin(deltaYaw);
            const auto rotateColumn = [c, s](RE::hkVector4& column) {
                alignas(16) float v[4]{};
                _mm_store_ps(v, column.quad);
                const float x = v[0];
                const float y = v[1];
                v[0] = c * x - s * y;
                v[1] = s * x + c * y;
                column.quad = _mm_load_ps(v);
            };

            // Yaw only. Translation and terrain pitch/roll are preserved byte-for-byte.
            rotateColumn(transform.rotation.col0);
            rotateColumn(transform.rotation.col1);
            rotateColumn(transform.rotation.col2);
        }

        [[nodiscard]] bool IsActionBypassActive(RE::Character* actor) noexcept
        {
            if (!actor) return false;
            const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
            if (!profile) return false;

            for (const auto& variable : profile->rotation.actionBypassVariables) {
                if (variable.empty()) continue;
                bool active = false;
                if (actor->GetGraphVariableBool(variable.c_str(), active) && active) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool IsUniversalActionBypassActive(RE::Character* actor) noexcept
        {
            if (!actor) return false;
            for (const char* variable : { "IsAttacking", "IsStaggering", "IsRecoiling" }) {
                bool active = false;
                if (actor->GetGraphVariableBool(variable, active) && active) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] LocomotionGraphWitness ReadLocomotionGraphWitness(
            RE::Character* actor,
            const MotionProfileDatabase::Profile* profile) noexcept
        {
            LocomotionGraphWitness out{};
            if (!actor) return out;
            if (!profile) {
                if (SyntheticMotionInjector::IsUniversalCreatureEligible(actor)) {
                    out.readable = true;
                    out.active = SpeedIntentTranslator::HasUniversalMovementDemand(actor);
                    out.source = LocomotionWitnessSource::kUniversalMovementDemand;
                }
                return out;
            }

            // R71 already treats GraphLocomotionActiveVariable as the authoritative
            // Standing-vs-Locomotion witness for pivot dispatch. Reuse that exact
            // profile-level contract here even when AnimationPriority itself is disabled.
            if (!profile->animationPriority.graphLocomotionActiveVariable.empty()) {
                bool active = false;
                if (actor->GetGraphVariableBool(
                        profile->animationPriority.graphLocomotionActiveVariable.c_str(), active)) {
                    out.readable = true;
                    out.active = active;
                    out.source = LocomotionWitnessSource::kAnimationPriority;
                    return out;
                }
            }

            // Fallback for profiles that expose only the SpeedIntent movement witness.
            if (!profile->speed.graphMoving.empty()) {
                bool active = false;
                if (actor->GetGraphVariableBool(profile->speed.graphMoving.c_str(), active)) {
                    out.readable = true;
                    out.active = active;
                    out.source = LocomotionWitnessSource::kSpeedIntent;
                }
            }
            return out;
        }

        [[nodiscard]] const char* LocomotionWitnessSourceName(LocomotionWitnessSource source) noexcept
        {
            switch (source) {
            case LocomotionWitnessSource::kAnimationPriority:
                return "ANIMATION_PRIORITY_GRAPH_LOCOMOTION";
            case LocomotionWitnessSource::kSpeedIntent:
                return "SPEED_INTENT_GRAPH_MOVING";
            case LocomotionWitnessSource::kUniversalMovementDemand:
                return "UNIVERSAL_MOVEMENT_DEMAND";
            default:
                return "NONE";
            }
        }

        [[nodiscard]] const char* StablePivotDiagnosticPhaseName(std::uint8_t phase) noexcept
        {
            switch (phase) {
            case 1: return "STOPPING";
            case 2: return "PENDING";
            case 3: return "ACTIVE";
            default: return "IDLE";
            }
        }

        [[nodiscard]] ControllerHook LookupHook(RE::bhkCharacterController* controller) noexcept
        {
            ControllerHook out{};
            if (!controller) return out;

            auto** vtableSlot = reinterpret_cast<std::uintptr_t**>(controller);
            auto* vtable = vtableSlot ? *vtableSlot : nullptr;
            if (!vtable) return out;

            std::scoped_lock lock{ g_mutex };
            const auto it = g_hooks.find(vtable);
            if (it != g_hooks.end()) out = it->second;
            return out;
        }

        [[nodiscard]] OwnerSnapshot SnapshotOwnerForCall(RE::bhkCharacterController* controller)
        {
            OwnerSnapshot out{};
            if (!controller) return out;

            std::scoped_lock lock{ g_mutex };
            const auto it = g_owners.find(controller);
            if (it == g_owners.end()) return out;

            auto& state = it->second;
            ++state.callsThisFrame;
            ++state.totalV13Calls;

            out.actor = state.actor;
            out.formID = state.formID;
            out.enabled = state.enabled;
            out.frameSerial = state.frameSerial;
            out.callIndex = state.callsThisFrame;
            out.deltaTime = state.frameDelta;
            out.frameStartYaw = state.frameStartYaw;
            out.frameStartValid = state.frameStartValid;
            out.responsePerSecond = state.responsePerSecond;
            out.maxYawRateDegreesPerSecond = state.maxYawRateDegreesPerSecond;
            out.minErrorDegrees = state.minErrorDegrees;
            out.telemetryEveryFrames = state.telemetryEveryFrames;
            out.burstLog = state.telemetryBurstRemaining > 0;
            out.animationDriven = state.animationDrivenAtFrameStart;
            out.actionBypass = state.actionBypassAtFrameStart;
            out.recentTurnWitness = state.authoredTurnAtFrameStart;
            out.recentLocomotionWitness = state.runtimeCarrierAtFrameStart;
            out.universalNativeCarrier = state.universalNativeCarrierAtFrameStart;
            out.profilelessUniversal = state.profilelessUniversalAtFrameStart;
            out.locomotionGraphWitnessReadable = state.locomotionGraphWitnessReadableAtFrameStart;
            out.locomotionGraphActive = state.locomotionGraphActiveAtFrameStart;
            out.locomotionGraphWitnessSource = state.locomotionGraphWitnessSourceAtFrameStart;
            out.externalWitnessActive = state.externalWitnessActiveAtFrameStart;
            out.externalYawPass = state.externalYawPassAtFrameStart;
            out.stablePivotPhase = state.stablePivotPhaseAtFrameStart;
            out.stablePivotRouteDegrees = state.stablePivotRouteDegreesAtFrameStart;
            out.stablePivotNativeHkbWitness = state.stablePivotNativeHkbWitnessAtFrameStart;
            out.stablePivotNativeHkbWitnessAgeSeconds = state.stablePivotNativeHkbWitnessAgeSecondsAtFrameStart;
            out.stablePivotNativeHkbKey = state.stablePivotNativeHkbKeyAtFrameStart;
            out.stablePivotTurnDeltaRead = state.stablePivotTurnDeltaReadAtFrameStart;
            out.stablePivotTurnDelta = state.stablePivotTurnDeltaAtFrameStart;
            out.stablePivotPhysicalYawRateDegreesPerSecond =
                state.stablePivotPhysicalYawRateDegreesPerSecondAtFrameStart;
            out.nativeStandingTurnRateActive = state.nativeStandingTurnRateActiveAtFrameStart;
            out.nativeStandingTurnPhysicalYawRateDegreesPerSecond =
                state.nativeStandingTurnPhysicalYawRateDegreesPerSecondAtFrameStart;
            out.nativeStandingTurnTurnDeltaRead = state.nativeStandingTurnTurnDeltaReadAtFrameStart;
            out.nativeStandingTurnTurnDelta = state.nativeStandingTurnTurnDeltaAtFrameStart;

            if (state.telemetryBurstRemaining > 0) {
                --state.telemetryBurstRemaining;
            }
            return out;
        }

        [[nodiscard]] CommandClampResult ClampProcessRotationSpeed(
            RE::Character* actor,
            float maxYawRateDegreesPerSecond,
            bool emitTelemetry)
        {
            CommandClampResult result{};
            if (!actor) return result;

            auto* process = actor->GetActorRuntimeData().currentProcess;
            auto* middleHigh = process ? process->middleHigh : nullptr;
            if (!middleHigh) {
                std::scoped_lock lock{ g_mutex };
                const auto controllerIt = g_actorControllers.find(actor->GetFormID());
                if (controllerIt != g_actorControllers.end()) {
                    const auto ownerIt = g_owners.find(controllerIt->second);
                    if (ownerIt != g_owners.end()) ++ownerIt->second.missingMiddleHigh;
                }
                return result;
            }

            result.available = true;
            result.inputRadPerSecond = middleHigh->rotationSpeed.z;
            result.finite = std::isfinite(result.inputRadPerSecond);

            const float maxRate = std::max(0.0f, maxYawRateDegreesPerSecond) * kDegToRad;
            if (!result.finite) {
                result.outputRadPerSecond = 0.0f;
                result.clamped = true;
                middleHigh->rotationSpeed.z = 0.0f;
            } else {
                result.outputRadPerSecond = std::clamp(result.inputRadPerSecond, -maxRate, maxRate);
                result.clamped =
                    std::abs(result.outputRadPerSecond - result.inputRadPerSecond) > 1.0e-7f;
                middleHigh->rotationSpeed.z = result.outputRadPerSecond;
            }

            bool shouldLog = emitTelemetry && result.clamped;
            {
                std::scoped_lock lock{ g_mutex };
                const auto controllerIt = g_actorControllers.find(actor->GetFormID());
                if (controllerIt != g_actorControllers.end()) {
                    const auto ownerIt = g_owners.find(controllerIt->second);
                    if (ownerIt != g_owners.end()) {
                        auto& state = ownerIt->second;
                        ++state.commandSamples;
                        state.lastCommandInput = result.inputRadPerSecond;
                        state.lastCommandOutput = result.outputRadPerSecond;
                        if (result.clamped) ++state.commandClamps;
                        if (!result.finite) ++state.invalidCommands;
                        shouldLog = shouldLog ||
                            (emitTelemetry && state.telemetryEveryFrames > 0 &&
                             state.frameSerial % state.telemetryEveryFrames == 0);
                    }
                }
            }

            if (shouldLog) {
                SKSE::log::info(
                    "[CapsuleSlerp][R35_RATE_BARRIER] form={:08X} input={:+.3f}deg/s output={:+.3f}deg/s max={:.2f}deg/s clamped={} role=SAFETY_ONLY_NOT_STEERING_SOURCE",
                    actor->GetFormID(),
                    result.inputRadPerSecond * kRadToDeg,
                    result.outputRadPerSecond * kRadToDeg,
                    maxYawRateDegreesPerSecond,
                    result.clamped);
            }
            return result;
        }

        void AngularTransformThunk(RE::bhkCharacterController* controller, RE::hkTransform& vanillaTarget)
        {
            const auto hook = LookupHook(controller);
            if (!hook.original) {
                return;
            }

            const auto owner = SnapshotOwnerForCall(controller);
            if (!owner.formID || !owner.enabled || !owner.actor) {
                hook.original(controller, vanillaTarget);
                return;
            }

            // Capture the native physical steering command BEFORE ALR modifies either the
            // target or rotationSpeed.z. This is the exact diagnostic surface recovered
            // from the R16.4/R16.5 branch.
            RE::hkTransform current{};
            controller->GetTransformImpl(current);
            const float currentYaw = ReadYaw(current);
            const float nativeTargetYaw = ReadYaw(vanillaTarget);
            const float referenceYaw = owner.frameStartValid ? owner.frameStartYaw : currentYaw;
            const float nativeError = NormalizeRadians(nativeTargetYaw - referenceYaw);
            const float nativeErrorDegrees = nativeError * kRadToDeg;
            const bool animationDriven = owner.animationDriven;
            const bool actionBypass = owner.actionBypass;

            float rawRotationSpeed = 0.0f;
            bool rawRotationSpeedValid = false;
            if (auto* process = owner.actor->GetActorRuntimeData().currentProcess) {
                if (auto* middleHigh = process->middleHigh) {
                    rawRotationSpeed = middleHigh->rotationSpeed.z;
                    rawRotationSpeedValid = std::isfinite(rawRotationSpeed);
                }
            }

            {
                std::scoped_lock lock{ g_mutex };
                const auto it = g_owners.find(controller);
                if (it != g_owners.end()) {
                    auto& state = it->second;
                    ++state.nativeTargetSamples;
                    state.lastNativeTargetYaw = nativeTargetYaw;
                    state.lastNativeError = nativeError;
                }
            }

            const bool periodic =
                owner.callIndex == 1 && owner.telemetryEveryFrames > 0 &&
                owner.frameSerial % owner.telemetryEveryFrames == 0;
            if (owner.burstLog || periodic || std::abs(nativeErrorDegrees) >= 5.0f) {
                SKSE::log::debug(
                    "[CapsuleSlerp][R35_V13_PRE_NATIVE_TARGET] form={:08X} frame={} call={} physical={:+.2f}deg frameStart={:+.2f}deg nativeTarget={:+.2f}deg error={:+.3f}deg animationDriven={} locomotionCarrier={} turnWitness={} actionBypass={} rotationSpeedRaw={:+.3f}deg/s rotationSpeedValid={} source=NATIVE_CONTROLLER_V13_BEFORE_ALR",
                    owner.formID,
                    owner.frameSerial,
                    owner.callIndex,
                    currentYaw * kRadToDeg,
                    referenceYaw * kRadToDeg,
                    nativeTargetYaw * kRadToDeg,
                    nativeErrorDegrees,
                    animationDriven,
                    owner.recentLocomotionWitness,
                    owner.recentTurnWitness,
                    actionBypass,
                    rawRotationSpeed * kRadToDeg,
                    rawRotationSpeedValid);
            }

            const bool r104PivotTrace = owner.stablePivotPhase != 0;
            if (r104PivotTrace) {
                SKSE::log::info(
                    "[R104][YAW_CHAIN_PRE] form={:08X} frame={} call={} phase={} route={} "
                    "external[active={} yawPass={}] hkb[seen={} ageMs={:.2f} key=\"{}\"] "
                    "TurnDeltaPreNative[read={} value={:+.3f}] graph[read={} moving={} source={}] "
                    "policy[anim={} carrier={} combinedTurnWitness={} action={}] "
                    "yaw[physical={:+.3f} frameStart={:+.3f} nativeTarget={:+.3f} error={:+.3f}] "
                    "rotationSpeedRaw={:+.3f}deg/s valid={} action=OBSERVE_ONLY_PRE_POLICY",
                    owner.formID, owner.frameSerial, owner.callIndex,
                    StablePivotDiagnosticPhaseName(owner.stablePivotPhase),
                    owner.stablePivotRouteDegrees,
                    owner.externalWitnessActive, owner.externalYawPass,
                    owner.stablePivotNativeHkbWitness,
                    owner.stablePivotNativeHkbWitnessAgeSeconds >= 0.0f
                        ? owner.stablePivotNativeHkbWitnessAgeSeconds * 1000.0f : -1.0f,
                    owner.stablePivotNativeHkbKey,
                    owner.stablePivotTurnDeltaRead, owner.stablePivotTurnDelta,
                    owner.locomotionGraphWitnessReadable, owner.locomotionGraphActive,
                    LocomotionWitnessSourceName(owner.locomotionGraphWitnessSource),
                    animationDriven, owner.recentLocomotionWitness, owner.recentTurnWitness, actionBypass,
                    currentYaw * kRadToDeg, referenceYaw * kRadToDeg,
                    nativeTargetYaw * kRadToDeg, nativeErrorDegrees,
                    rawRotationSpeed * kRadToDeg, rawRotationSpeedValid);
            }

            // R109: route-90 physical yaw comes ONLY from the FINAL hkbBehaviorGraph
            // TRACK_EXTRACTED_MOTION. R107/R108's synthetic 90 deg/s servo and leaf-HKB
            // witness are removed. STOPPING remains a hard hold. PENDING/ACTIVE may rotate
            // only when the final composed behavior output itself contains a sane yaw delta
            // in the latched physical direction. If the output is absent/ambiguous/zero,
            // wrong-sided, or turnStop already closed yawPass, physical yaw is held.
            const bool r109Pivot90 = owner.stablePivotRouteDegrees == 90;
            if (r109Pivot90 && owner.stablePivotPhase == 1 && !actionBypass) {
                const float holdCorrection = NormalizeRadians(currentYaw - nativeTargetYaw);
                if (std::abs(holdCorrection) > 1.0e-7f) {
                    RotateTargetAroundWorldZ(vanillaTarget, holdCorrection);
                }
                SKSE::log::info(
                    "[CapsuleSlerp][R109_FINAL_BEHAVIOR_HOLD] form={:08X} frame={} call={} phase=STOPPING physical={:+.3f}deg nativeTarget={:+.3f}deg reason=WAIT_FOR_NATIVE_STANDING_BEFORE_TURN_EVENT action=HOLD_PHYSICAL_YAW",
                    owner.formID, owner.frameSerial, owner.callIndex,
                    currentYaw * kRadToDeg, nativeTargetYaw * kRadToDeg);
                hook.original(controller, vanillaTarget);
                return;
            }

            if (r109Pivot90 &&
                (owner.stablePivotPhase == 2 || owner.stablePivotPhase == 3) &&
                !actionBypass) {
                const auto finalBehavior = ProbeFinalBehaviorYaw(owner.actor);
                const float expectedPhysicalSign = owner.stablePivotPhysicalYawRateDegreesPerSecond;
                const bool expectedSignValid = std::isfinite(expectedPhysicalSign) &&
                    std::abs(expectedPhysicalSign) > 1.0e-3f;
                const float physicalDeltaDegrees = finalBehavior.physicalYawDegrees;
                const float absDeltaDegrees = std::abs(physicalDeltaDegrees);
                const float corruptionLimitDegrees = std::max(5.0f, 720.0f * owner.deltaTime);
                const bool outputSane = finalBehavior.graphReadable && !finalBehavior.ambiguous &&
                    finalBehavior.selectedOutput >= 0 && std::isfinite(physicalDeltaDegrees) &&
                    absDeltaDegrees > 0.005f && absDeltaDegrees <= corruptionLimitDegrees;
                const bool directionMatches = outputSane && expectedSignValid &&
                    physicalDeltaDegrees * expectedPhysicalSign > 0.0f;
                const bool yawPassOpen = owner.stablePivotPhase == 2 || owner.externalYawPass;
                const bool useFinalBehaviorYaw = outputSane && directionMatches && yawPassOpen;

                if (useFinalBehaviorYaw) {
                    const float authoredTargetYaw = NormalizeRadians(
                        referenceYaw + physicalDeltaDegrees * kDegToRad);
                    const float correction = NormalizeRadians(authoredTargetYaw - nativeTargetYaw);
                    if (std::abs(correction) > 1.0e-7f) {
                        RotateTargetAroundWorldZ(vanillaTarget, correction);
                    }
                    SKSE::log::info(
                        "[CapsuleSlerp][R109_FINAL_BEHAVIOR_YAW] form={:08X} frame={} call={} phase={} output={} interpolation={:.4f} onFraction={:.4f} extractedYaw={:+.4f}deg physicalDelta={:+.4f}deg frameStart={:+.3f}deg nativeTarget={:+.3f}deg passedTarget={:+.3f}deg expectedPhysicalSign={:+.0f} action=USE_FINAL_COMPOSED_BEHAVIOR_ROOT_NO_SYNTHETIC_RATE",
                        owner.formID, owner.frameSerial, owner.callIndex,
                        StablePivotDiagnosticPhaseName(owner.stablePivotPhase),
                        finalBehavior.selectedOutput,
                        finalBehavior.interpolationAmount,
                        finalBehavior.onFraction,
                        finalBehavior.extractedYawDegrees,
                        physicalDeltaDegrees,
                        referenceYaw * kRadToDeg,
                        nativeTargetYaw * kRadToDeg,
                        authoredTargetYaw * kRadToDeg,
                        expectedPhysicalSign);
                    hook.original(controller, vanillaTarget);
                    return;
                }

                const float holdCorrection = NormalizeRadians(currentYaw - nativeTargetYaw);
                if (std::abs(holdCorrection) > 1.0e-7f) {
                    RotateTargetAroundWorldZ(vanillaTarget, holdCorrection);
                }
                const char* reason = !yawPassOpen ? "TURNSTOP_BLENDOUT" :
                    (!finalBehavior.graphReadable ? "NO_ACTIVE_GRAPH" :
                    (finalBehavior.ambiguous ? "AMBIGUOUS_GENERATOR_OUTPUT" :
                    (finalBehavior.selectedOutput < 0 ? "NO_ACTIVE_EXTRACTED_MOTION_TRACK" :
                    (!outputSane ? "ZERO_OR_CORRUPT_FINAL_YAW" :
                    (!expectedSignValid ? "NO_LATCHED_DIRECTION" : "WRONG_SIDE_FINAL_YAW")))));
                SKSE::log::info(
                    "[CapsuleSlerp][R109_FINAL_BEHAVIOR_HOLD] form={:08X} frame={} call={} phase={} physical={:+.3f}deg nativeTarget={:+.3f}deg output={} extractedYaw={:+.4f}deg physicalDelta={:+.4f}deg expectedPhysicalSign={:+.0f} reason={} action=HOLD_NO_GHOST_ROTATION",
                    owner.formID, owner.frameSerial, owner.callIndex,
                    StablePivotDiagnosticPhaseName(owner.stablePivotPhase),
                    currentYaw * kRadToDeg, nativeTargetYaw * kRadToDeg,
                    finalBehavior.selectedOutput,
                    finalBehavior.extractedYawDegrees,
                    finalBehavior.physicalYawDegrees,
                    expectedPhysicalSign,
                    reason);
                hook.original(controller, vanillaTarget);
                return;
            }

            // R115: the regression from R93-R114 proved that clip clock normalization and
            // final physical yaw cadence are independent. During the R114 TURN_READY window,
            // Bethesda still selects/blends the 90-degree turn and owns TurnDelta. Once that
            // final blender input is non-zero, V13 advances the capsule from the actor-frame
            // physical start at the profile's authored 90 deg/s rate. This is the one useful
            // part of R107, transplanted without its invalid child-weight/HKB authority.
            const float r115StandingRate = owner.nativeStandingTurnPhysicalYawRateDegreesPerSecond;
            const bool r115StandingRateValid =
                owner.nativeStandingTurnRateActive &&
                owner.externalWitnessActive && owner.externalYawPass &&
                owner.nativeStandingTurnTurnDeltaRead &&
                std::isfinite(owner.nativeStandingTurnTurnDelta) &&
                std::abs(owner.nativeStandingTurnTurnDelta) > 0.5f &&
                std::isfinite(r115StandingRate) &&
                std::abs(r115StandingRate) > 1.0e-3f;
            if (r115StandingRateValid && !actionBypass) {
                const float physicalStepDegrees = r115StandingRate * owner.deltaTime;
                const float authoredTargetYaw = NormalizeRadians(
                    referenceYaw + physicalStepDegrees * kDegToRad);
                const float authoredCorrection = NormalizeRadians(authoredTargetYaw - nativeTargetYaw);
                if (std::abs(authoredCorrection) > 1.0e-7f) {
                    RotateTargetAroundWorldZ(vanillaTarget, authoredCorrection);
                }
                SKSE::log::info(
                    "[CapsuleSlerp][R115_NATIVE_TURN_PHYSICAL_RATE] form={:08X} frame={} call={} physical={:+.3f}deg frameStart={:+.3f}deg nativeTarget={:+.3f}deg passedTarget={:+.3f}deg TurnDelta={:+.3f} rate={:+.2f}deg/s step={:+.3f}deg discardedNativeDebt={:+.3f}deg action=R114_LIFECYCLE_PLUS_R107_RATE_NO_CHILD_WEIGHT_NO_TURNDELTA_WRITE",
                    owner.formID, owner.frameSerial, owner.callIndex,
                    currentYaw * kRadToDeg, referenceYaw * kRadToDeg,
                    nativeTargetYaw * kRadToDeg, authoredTargetYaw * kRadToDeg,
                    owner.nativeStandingTurnTurnDelta, r115StandingRate, physicalStepDegrees,
                    NormalizeRadians(nativeTargetYaw - authoredTargetYaw) * kRadToDeg);
                hook.original(controller, vanillaTarget);
                return;
            }

            // C1 profileless V13 firewall. Procedural locomotion yaw is authored upstream
            // exclusively by ProcessMotionData R1/R2. V13 contributes ZERO additional yaw
            // in both moving and standing states. Genuine non-locomotion authored rotation
            // and finite actions fail open to their native target.
            const bool c1Profileless = owner.profilelessUniversal;
            if (c1Profileless && !owner.recentTurnWitness && !actionBypass) {
                const float holdCorrection = NormalizeRadians(currentYaw - nativeTargetYaw);
                if (std::abs(holdCorrection) > 1.0e-7f) {
                    RotateTargetAroundWorldZ(vanillaTarget, holdCorrection);
                }
                float savedRotationSpeedZ = 0.0f;
                bool restoreRotationSpeed = false;
                if (auto* process = owner.actor->GetActorRuntimeData().currentProcess; process && process->middleHigh) {
                    savedRotationSpeedZ = process->middleHigh->rotationSpeed.z;
                    restoreRotationSpeed = std::isfinite(savedRotationSpeedZ);
                    process->middleHigh->rotationSpeed.z = 0.0f;
                }
                if (owner.burstLog || periodic || std::abs(nativeErrorDegrees) >= 1.0f) {
                    SKSE::log::info(
                        "[C1_V13_FIREWALL][HOLD] form={:08X} frame={} call={} movementDemand={} physical={:+.2f}deg nativeTarget={:+.2f}deg correction={:+.3f}deg rotationSpeedZ=ZERO_DURING_V13 action=ZERO_NATIVE_CAPSULE_YAW_PMD_R1R2_IS_SOLE_PROCEDURAL_OWNER",
                        owner.formID, owner.frameSerial, owner.callIndex,
                        owner.locomotionGraphActive, currentYaw * kRadToDeg,
                        nativeTargetYaw * kRadToDeg, holdCorrection * kRadToDeg);
                }
                hook.original(controller, vanillaTarget);
                if (restoreRotationSpeed) {
                    if (auto* process = owner.actor->GetActorRuntimeData().currentProcess; process && process->middleHigh) {
                        process->middleHigh->rotationSpeed.z = savedRotationSpeedZ;
                    }
                }
                return;
            }

            // R74 standing physical firewall. A readable profile graph witness saying
            // STANDING is stronger than a stale/current-or-previous RuntimeYaw carrier.
            // While standing, the capsule may rotate only when an authored RotationClip
            // or an explicit action bypass owns the frame. Otherwise neutralize ONLY the
            // target yaw to the controller's current physical yaw. Pitch/roll/translation
            // remain untouched. Unknown witness fails open; moving locomotion is unchanged.
            if (owner.locomotionGraphWitnessReadable && !owner.locomotionGraphActive) {
                if (!owner.recentTurnWitness && !actionBypass) {
                    const float standingCorrection = NormalizeRadians(currentYaw - nativeTargetYaw);
                    const bool changed = std::abs(standingCorrection) > 1.0e-7f;
                    if (changed) {
                        RotateTargetAroundWorldZ(vanillaTarget, standingCorrection);
                    }

                    std::uint64_t blockedIndex = 0;
                    {
                        std::scoped_lock lock{ g_mutex };
                        const auto it = g_owners.find(controller);
                        if (it != g_owners.end()) {
                            blockedIndex = ++it->second.standingYawBlocked;
                            if (changed) ++it->second.targetLimited;
                            else ++it->second.targetUnchanged;
                        }
                    }

                    const bool logStanding = changed &&
                        (r104PivotTrace || blockedIndex <= 12 ||
                         (blockedIndex > 0 && blockedIndex % 120 == 0));
                    if (logStanding) {
                        SKSE::log::info(
                            "[CapsuleSlerp][R74_STANDING_YAW_BLOCK] form={:08X} frame={} call={} physical={:+.2f}deg nativeTarget={:+.2f}deg correction={:+.3f}deg graphWitness={} graphMoving=false authoredTurn=false actionBypass=false action=KEEP_CAPSULE_PHYSICAL_YAW_UNTIL_AUTHORED_TURN",
                            owner.formID,
                            owner.frameSerial,
                            owner.callIndex,
                            currentYaw * kRadToDeg,
                            nativeTargetYaw * kRadToDeg,
                            standingCorrection * kRadToDeg,
                            LocomotionWitnessSourceName(owner.locomotionGraphWitnessSource));
                    }

                    if (r104PivotTrace) {
                        SKSE::log::info(
                            "[R104][YAW_CHAIN_OUT] form={:08X} frame={} call={} phase={} route={} "
                            "path=STANDING_FIREWALL nativeTarget={:+.3f} passedTarget={:+.3f} physicalBefore={:+.3f} "
                            "correction={:+.3f}deg action=OBSERVE_ONLY_TARGET_AFTER_POLICY",
                            owner.formID, owner.frameSerial, owner.callIndex,
                            StablePivotDiagnosticPhaseName(owner.stablePivotPhase),
                            owner.stablePivotRouteDegrees,
                            nativeTargetYaw * kRadToDeg, ReadYaw(vanillaTarget) * kRadToDeg,
                            currentYaw * kRadToDeg, standingCorrection * kRadToDeg);
                    }
                    hook.original(controller, vanillaTarget);
                    return;
                }

                {
                    std::scoped_lock lock{ g_mutex };
                    const auto it = g_owners.find(controller);
                    if (it != g_owners.end()) {
                        if (owner.recentTurnWitness) ++it->second.standingYawPassAuthoredTurn;
                        if (actionBypass) ++it->second.standingYawPassAction;
                    }
                }
            }

            // Procedural capsule yaw is legal ONLY for a real animation-driven
            // RuntimeYawClip carrier. Explicit RotationClips/turns and action-owned motion
            // always retain their native/authored target. Outside the carrier contract we
            // are read-only: no invented heading, no CapsuleSlerp correction.
            const char* bypassReason = nullptr;
            if (owner.recentTurnWitness) {
                bypassReason = "PROFILE_ROTATION_CLIP_WITNESS";
            } else if (actionBypass) {
                bypassReason = "ACTION_BYPASS";
            } else if (!animationDriven) {
                bypassReason = "BANIMATIONDRIVEN_FALSE";
            } else if (!owner.recentLocomotionWitness) {
                bypassReason = "NO_RUNTIME_LOCOMOTION_CARRIER";
            }

            if (bypassReason) {
                {
                    std::scoped_lock lock{ g_mutex };
                    const auto it = g_owners.find(controller);
                    if (it != g_owners.end()) {
                        if (owner.recentTurnWitness) ++it->second.turnBypasses;
                        else if (actionBypass) ++it->second.actionBypasses;
                        else if (!animationDriven) ++it->second.noAnimationDrivenBypasses;
                        else ++it->second.noLocomotionCarrierBypasses;
                    }
                }

                if (owner.burstLog || periodic) {
                    SKSE::log::debug(
                        "[CapsuleSlerp][R35_V13_BYPASS] form={:08X} frame={} call={} reason={} animationDriven={} locomotionCarrier={} turnWitness={} action=PASS_NATIVE_TARGET_UNMODIFIED",
                        owner.formID,
                        owner.frameSerial,
                        owner.callIndex,
                        bypassReason,
                        animationDriven,
                        owner.recentLocomotionWitness,
                        owner.recentTurnWitness);
                }
                if (r104PivotTrace) {
                    SKSE::log::info(
                        "[R104][YAW_CHAIN_OUT] form={:08X} frame={} call={} phase={} route={} "
                        "path=BYPASS reason={} nativeTarget={:+.3f} passedTarget={:+.3f} physicalBefore={:+.3f} "
                        "action=OBSERVE_ONLY_TARGET_AFTER_POLICY",
                        owner.formID, owner.frameSerial, owner.callIndex,
                        StablePivotDiagnosticPhaseName(owner.stablePivotPhase),
                        owner.stablePivotRouteDegrees, bypassReason,
                        nativeTargetYaw * kRadToDeg, ReadYaw(vanillaTarget) * kRadToDeg,
                        currentYaw * kRadToDeg);
                }
                hook.original(controller, vanillaTarget);
                return;
            }

            {
                std::scoped_lock lock{ g_mutex };
                const auto it = g_owners.find(controller);
                if (it != g_owners.end()) {
                    ++it->second.proceduralCarrierCalls;
                }
            }

            // B8 profileless yaw uses the proven ProcessMotionData R1/R2 transport,
            // exactly like the functional profile lane. V13 is only the downstream physical
            // sink and must never add a second procedural correction.
            if (owner.universalNativeCarrier && (owner.burstLog || periodic)) {
                SKSE::log::debug(
                    "[C1_UNIVERSAL_YAW][V13_PASS_LEGACY_UNREACHABLE_PROFILELESS] form={:08X} frame={} call={} physical={:+.2f}deg nativeTarget={:+.2f}deg action=PASS_NATIVE_R1R2_IS_SOLE_PROCEDURAL_OWNER",
                    owner.formID, owner.frameSerial, owner.callIndex,
                    currentYaw * kRadToDeg, nativeTargetYaw * kRadToDeg);
            }

            // R35 physical policy: DO NOT modify the V13 target at all.
            // The AI steering yaw lives exclusively in ProcessMotionData R1/R2.
            // Passing the native target through unchanged avoids introducing a second
            // procedural yaw authority or accumulate a frozen physical-heading debt.
            if (r104PivotTrace) {
                SKSE::log::info(
                    "[R104][YAW_CHAIN_OUT] form={:08X} frame={} call={} phase={} route={} "
                    "path=PROCEDURAL_NATIVE_PASS nativeTarget={:+.3f} passedTarget={:+.3f} physicalBefore={:+.3f} "
                    "action=OBSERVE_ONLY_TARGET_AFTER_POLICY",
                    owner.formID, owner.frameSerial, owner.callIndex,
                    StablePivotDiagnosticPhaseName(owner.stablePivotPhase),
                    owner.stablePivotRouteDegrees,
                    nativeTargetYaw * kRadToDeg, ReadYaw(vanillaTarget) * kRadToDeg,
                    currentYaw * kRadToDeg);
            }
            hook.original(controller, vanillaTarget);

            {
                std::scoped_lock lock{ g_mutex };
                const auto it = g_owners.find(controller);
                if (it != g_owners.end()) {
                    ++it->second.targetUnchanged;
                    it->second.lastLimitedStep = 0.0f;
                }
            }

            if (owner.burstLog || periodic || std::abs(nativeErrorDegrees) >= 5.0f) {
                SKSE::log::debug(
                    "[CapsuleSlerp][R35_V13_NATIVE_PASS] form={:08X} frame={} call={} physical={:+.2f}deg nativeTarget={:+.2f}deg nativeError={:+.3f}deg action=PASS_NATIVE_UNMODIFIED yawSource=R35_AI_STEERING_PROCESS_MOTION_DATA_R1_R2",
                    owner.formID,
                    owner.frameSerial,
                    owner.callIndex,
                    currentYaw * kRadToDeg,
                    nativeTargetYaw * kRadToDeg,
                    nativeErrorDegrees);
            }
        }

        void RegisterController(RE::Character* actor, const MotionProfileDatabase::CapsuleSlerpConfig& config)
        {
            if (!RuntimeEligible(actor)) return;

            auto* controller = actor->GetCharController();
            if (!controller) return;

            auto** vtableSlot = reinterpret_cast<std::uintptr_t**>(controller);
            auto* vtable = vtableSlot ? *vtableSlot : nullptr;
            if (!vtable) return;

            bool installLogged = false;
            void* originalAddress = nullptr;
            {
                std::scoped_lock lock{ g_mutex };
                const auto formID = actor->GetFormID();
                if (const auto old = g_actorControllers.find(formID);
                    old != g_actorControllers.end() && old->second != controller) {
                    g_owners.erase(old->second);
                }

                g_actorControllers[formID] = controller;
                auto& owner = g_owners[controller];
                owner.actor = actor;
                owner.formID = formID;
                owner.enabled = config.enabled;
                owner.responsePerSecond = config.responsePerSecond;
                owner.maxYawRateDegreesPerSecond = config.maxYawRateDegreesPerSecond;
                owner.minErrorDegrees = config.minErrorDegrees;
                owner.telemetryEveryFrames = std::max<std::uint32_t>(1, config.telemetryEveryFrames);

                if (!g_hooks.contains(vtable)) {
                    ControllerHook hook{};
                    hook.vtable = vtable;
                    hook.original = reinterpret_cast<AngularTransformFn>(vtable[kAngularTransformVFunc]);
                    if (hook.original && hook.original != &AngularTransformThunk) {
                        originalAddress = reinterpret_cast<void*>(hook.original);
                        g_hooks.emplace(vtable, hook);
                        REL::safe_write(
                            reinterpret_cast<std::uintptr_t>(std::addressof(vtable[kAngularTransformVFunc])),
                            reinterpret_cast<std::uintptr_t>(&AngularTransformThunk));
                        installLogged = true;
                    }
                }
            }

            if (installLogged) {
                SKSE::log::info(
                    "[CapsuleSlerp][R35_V13_INSTALL] form={:08X} controller={} vtable={} priorV13={} source=NATIVE_V13_PRE_ALR targetPolicy=PROFILE_PASS_NATIVE_PROFILELESS_HOLD_NATIVE_CAPSULE_YAW_AI_STEERING_R1_R2_IS_UPSTREAM max={:.2f}deg/s response={:.2f}/s bAnimationDrivenGate=REQUIRED RuntimeYawCarrier=CERTIFIED_INJECTOR_WITNESS_REQUIRED CapsuleR2YawWrites=ZERO externalPMDR2Writer=SYNTHETIC_INJECTOR",
                    actor->GetFormID(),
                    static_cast<void*>(controller),
                    static_cast<void*>(vtable),
                    originalAddress,
                    config.maxYawRateDegreesPerSecond,
                    config.responsePerSecond);
            }
        }

        void BeginActorFrame(
            RE::Character* actor,
            const MotionProfileDatabase::CapsuleSlerpConfig& config,
            float hookDeltaTime)
        {
            if (!actor) return;
            auto* controller = actor->GetCharController();
            if (!controller) return;

            // Character::Update boundary. These queries are intentionally outside
            // V13/physics. SyntheticMotionInjector::BeginActorFrame has already run, so
            // bAnimationDriven/root authority reflect this frame's proven carrier state.
            const bool animationDriven = actor->IsAnimationDriven();
            const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
            const bool universalNativeCarrier = !profile &&
                SyntheticMotionInjector::IsUniversalCreatureEligible(actor) &&
                SpeedIntentTranslator::HasUniversalMovementDemand(actor);
            const bool actionBypass = profile ? IsActionBypassActive(actor) :
                IsUniversalActionBypassActive(actor);
            bool externalAuthoredTurnActive = false;
            bool externalAuthoredTurnYawPass = false;
            StablePivotDiagnosticContext pivotDiagnostic{};
            NativeStandingTurnRateContext nativeStandingTurnRate{};
            {
                std::scoped_lock lock{ g_mutex };
                const auto externalIt = g_externalAuthoredTurnWitnesses.find(actor->GetFormID());
                if (externalIt != g_externalAuthoredTurnWitnesses.end()) {
                    externalAuthoredTurnActive = externalIt->second.active;
                    externalAuthoredTurnYawPass = externalIt->second.active &&
                        externalIt->second.allowNativeStandingYaw;
                }
                const auto diagnosticIt = g_stablePivotDiagnostics.find(actor->GetFormID());
                if (diagnosticIt != g_stablePivotDiagnostics.end()) {
                    pivotDiagnostic = diagnosticIt->second;
                }
                const auto nativeRateIt = g_nativeStandingTurnRateContexts.find(actor->GetFormID());
                if (nativeRateIt != g_nativeStandingTurnRateContexts.end()) {
                    nativeStandingTurnRate = nativeRateIt->second;
                }
            }
            const bool motionDbAuthoredTurn =
                SyntheticMotionInjector::HasAuthoredRotationClipForCurrentOrPreviousFrame(actor);
            const bool nativeAuthoredTurn = !profile &&
                SyntheticMotionInjector::HasNativeAuthoredTurnForCurrentOrPreviousFrame(actor);
            // Profileless native authored turns suppress universal procedural yaw just as
            // profile RotationClips suppress the older R1/R2 lane.
            const bool authoredTurn = motionDbAuthoredTurn || nativeAuthoredTurn || externalAuthoredTurnYawPass;
            const bool runtimeCarrier = profile ?
                SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(actor) :
                universalNativeCarrier;
            const auto locomotionGraph = ReadLocomotionGraphWitness(actor, profile);

            RE::hkTransform current{};
            controller->GetTransformImpl(current);
            const float currentYaw = ReadYaw(current);

            std::uint32_t previousCalls = 0;
            std::uint64_t previousFrame = 0;
            float previousStartYaw = 0.0f;
            float previousDt = kFallbackDeltaTime;
            float previousMaxRate = config.maxYawRateDegreesPerSecond;
            float previousStablePivotPhysicalYawRate = 0.0f;
            float previousNativeStandingTurnPhysicalYawRate = 0.0f;
            bool previousValid = false;
            bool previousAnimationDriven = false;
            bool previousRuntimeCarrier = false;
            bool previousAuthoredTurn = false;
            bool previousActionBypass = false;
            bool previousLocomotionGraphReadable = false;
            bool previousLocomotionGraphActive = false;
            LocomotionWitnessSource previousLocomotionGraphSource = LocomotionWitnessSource::kNone;
            std::uint64_t realizedIndex = 0;
            bool policyChanged = false;
            std::uint64_t currentFrame = 0;

            {
                std::scoped_lock lock{ g_mutex };
                const auto it = g_owners.find(controller);
                if (it == g_owners.end()) return;

                auto& state = it->second;
                previousCalls = state.callsThisFrame;
                previousFrame = state.frameSerial;
                previousStartYaw = state.frameStartYaw;
                previousDt = state.frameDelta;
                previousMaxRate = state.maxYawRateDegreesPerSecond;
                previousStablePivotPhysicalYawRate =
                    state.stablePivotPhysicalYawRateDegreesPerSecondAtFrameStart;
                previousNativeStandingTurnPhysicalYawRate =
                    state.nativeStandingTurnPhysicalYawRateDegreesPerSecondAtFrameStart;
                previousValid = state.frameSerial > 0 && state.frameStartValid;
                previousAnimationDriven = state.animationDrivenAtFrameStart;
                previousRuntimeCarrier = state.runtimeCarrierAtFrameStart;
                previousAuthoredTurn = state.authoredTurnAtFrameStart;
                previousActionBypass = state.actionBypassAtFrameStart;
                previousLocomotionGraphReadable = state.locomotionGraphWitnessReadableAtFrameStart;
                previousLocomotionGraphActive = state.locomotionGraphActiveAtFrameStart;
                previousLocomotionGraphSource = state.locomotionGraphWitnessSourceAtFrameStart;

                const float dt = SanitizeDeltaTime(
                    hookDeltaTime,
                    SanitizeDeltaTime(controller->stepInfo.deltaTime, state.lastValidDelta));
                state.lastValidDelta = dt;

                policyChanged =
                    state.frameSerial == 0 ||
                    state.animationDrivenAtFrameStart != animationDriven ||
                    state.actionBypassAtFrameStart != actionBypass ||
                    state.authoredTurnAtFrameStart != authoredTurn ||
                    state.runtimeCarrierAtFrameStart != runtimeCarrier ||
                    state.universalNativeCarrierAtFrameStart != universalNativeCarrier ||
                    state.locomotionGraphWitnessReadableAtFrameStart != locomotionGraph.readable ||
                    state.locomotionGraphActiveAtFrameStart != locomotionGraph.active ||
                    state.locomotionGraphWitnessSourceAtFrameStart != locomotionGraph.source ||
                    state.nativeStandingTurnRateActiveAtFrameStart != nativeStandingTurnRate.active;

                ++state.frameSerial;
                currentFrame = state.frameSerial;
                state.previousFrameCalls = previousCalls;
                state.callsThisFrame = 0;
                state.frameDelta = dt;
                state.frameStartYaw = currentYaw;
                state.frameStartValid = true;
                state.actor = actor;
                // R115 may temporarily wake V13 for a profile that otherwise keeps
                // CapsuleSlerp disabled. Outside this exact TURN_READY context it remains dormant.
                state.enabled = config.enabled || nativeStandingTurnRate.active;
                state.responsePerSecond = config.responsePerSecond;
                state.maxYawRateDegreesPerSecond = config.maxYawRateDegreesPerSecond;
                state.minErrorDegrees = config.minErrorDegrees;
                state.telemetryEveryFrames = std::max<std::uint32_t>(1, config.telemetryEveryFrames);
                state.animationDrivenAtFrameStart = animationDriven;
                state.actionBypassAtFrameStart = actionBypass;
                state.authoredTurnAtFrameStart = authoredTurn;
                state.runtimeCarrierAtFrameStart = runtimeCarrier;
                state.universalNativeCarrierAtFrameStart = universalNativeCarrier;
                state.profilelessUniversalAtFrameStart = !profile &&
                    SyntheticMotionInjector::IsUniversalCreatureEligible(actor);
                state.locomotionGraphWitnessReadableAtFrameStart = locomotionGraph.readable;
                state.locomotionGraphActiveAtFrameStart = locomotionGraph.active;
                state.locomotionGraphWitnessSourceAtFrameStart = locomotionGraph.source;
                state.externalWitnessActiveAtFrameStart = externalAuthoredTurnActive;
                state.externalYawPassAtFrameStart = externalAuthoredTurnYawPass;
                state.stablePivotPhaseAtFrameStart = pivotDiagnostic.phase;
                state.stablePivotRouteDegreesAtFrameStart = pivotDiagnostic.routeDegrees;
                state.stablePivotNativeHkbWitnessAtFrameStart = pivotDiagnostic.nativeHkbWitness;
                state.stablePivotNativeHkbWitnessAgeSecondsAtFrameStart = pivotDiagnostic.nativeHkbWitnessAgeSeconds;
                state.stablePivotNativeHkbKeyAtFrameStart = pivotDiagnostic.nativeHkbKey;
                state.stablePivotTurnDeltaReadAtFrameStart = pivotDiagnostic.turnDeltaRead;
                state.stablePivotTurnDeltaAtFrameStart = pivotDiagnostic.turnDelta;
                state.stablePivotPhysicalYawRateDegreesPerSecondAtFrameStart =
                    pivotDiagnostic.physicalYawRateDegreesPerSecond;
                state.nativeStandingTurnRateActiveAtFrameStart = nativeStandingTurnRate.active;
                state.nativeStandingTurnPhysicalYawRateDegreesPerSecondAtFrameStart =
                    nativeStandingTurnRate.active && std::isfinite(nativeStandingTurnRate.signedYawRateDegreesPerSecond)
                        ? nativeStandingTurnRate.signedYawRateDegreesPerSecond : 0.0f;
                state.nativeStandingTurnTurnDeltaReadAtFrameStart =
                    nativeStandingTurnRate.active && nativeStandingTurnRate.turnDeltaRead;
                state.nativeStandingTurnTurnDeltaAtFrameStart =
                    state.nativeStandingTurnTurnDeltaReadAtFrameStart && std::isfinite(nativeStandingTurnRate.turnDelta)
                        ? nativeStandingTurnRate.turnDelta : 0.0f;

                if (previousValid) {
                    realizedIndex = ++state.realizedSamples;
                }
            }

            if (policyChanged || currentFrame <= 8) {
                SKSE::log::debug(
                    "[CapsuleSlerp][R74_POLICY] form={:08X} frame={} bAnimationDriven={} runtimeYawCarrier={} authoredTurn={} actionBypass={} graphWitness[readable={} moving={} source={}] action={} source=CHARACTER_UPDATE_SNAPSHOT",
                    actor->GetFormID(),
                    currentFrame,
                    animationDriven,
                    runtimeCarrier,
                    authoredTurn,
                    actionBypass,
                    locomotionGraph.readable,
                    locomotionGraph.active,
                    LocomotionWitnessSourceName(locomotionGraph.source),
                    (locomotionGraph.readable && !locomotionGraph.active && !authoredTurn && !actionBypass) ?
                        "STANDING_YAW_BLOCK" :
                        ((animationDriven && runtimeCarrier && !authoredTurn && !actionBypass) ?
                            "AI_STEERING_R1_R2_ACTIVE_V13_NATIVE_PASS" : "PASS_NATIVE"));
            }

            if (!previousValid) return;

            // IMPORTANT: unlike old FRAME_RESULT telemetry, this is the real physical capsule
            // transform read through bhkCharacterController::GetTransformImpl.
            const float realizedDelta = NormalizeRadians(currentYaw - previousStartYaw);
            const float realizedDegrees = realizedDelta * kRadToDeg;
            const float realizedRate = previousDt > kMinDeltaTime ? realizedDegrees / previousDt : 0.0f;
            const float effectiveMaxRate = std::max(
                previousMaxRate,
                std::max(std::abs(previousStablePivotPhysicalYawRate),
                         std::abs(previousNativeStandingTurnPhysicalYawRate)));
            const float tolerance = std::max(2.0f, effectiveMaxRate * 0.15f);
            const bool violated = std::abs(realizedRate) > effectiveMaxRate + tolerance;

            std::uint64_t violationIndex = 0;
            if (violated) {
                std::scoped_lock lock{ g_mutex };
                const auto it = g_owners.find(controller);
                if (it != g_owners.end()) {
                    violationIndex = ++it->second.realizedViolations;
                }
            }

            const bool logViolation = violated &&
                (violationIndex <= 8 || (violationIndex > 0 && violationIndex % 120 == 0));
            if (logViolation) {
                SKSE::log::warn(
                    "[CapsuleSlerp][R35_PHYSICAL_VIOLATION] form={:08X} frame={} v13Calls={} capsuleStart={:+.3f}deg nextCapsule={:+.3f}deg physicalDelta={:+.4f}deg dt={:.6f} physicalRate={:+.2f}deg/s max={:.2f}deg/s violation={} prevPolicy[anim={} carrier={} turn={} action={} graphReadable={} graphMoving={} graphSource={}] source=GetTransformImpl",
                    actor->GetFormID(),
                    previousFrame,
                    previousCalls,
                    previousStartYaw * kRadToDeg,
                    currentYaw * kRadToDeg,
                    realizedDegrees,
                    previousDt,
                    realizedRate,
                    effectiveMaxRate,
                    violated,
                    previousAnimationDriven,
                    previousRuntimeCarrier,
                    previousAuthoredTurn,
                    previousActionBypass,
                    previousLocomotionGraphReadable,
                    previousLocomotionGraphActive,
                    LocomotionWitnessSourceName(previousLocomotionGraphSource));
            } else if (realizedIndex <= 24 ||
                (config.telemetryEveryFrames > 0 && previousFrame % config.telemetryEveryFrames == 0)) {
                SKSE::log::debug(
                    "[CapsuleSlerp][R35_PHYSICAL_RESULT] form={:08X} frame={} v13Calls={} capsuleStart={:+.3f}deg nextCapsule={:+.3f}deg physicalDelta={:+.4f}deg dt={:.6f} physicalRate={:+.2f}deg/s max={:.2f}deg/s violation={} prevPolicy[anim={} carrier={} turn={} action={} graphReadable={} graphMoving={} graphSource={}] source=GetTransformImpl",
                    actor->GetFormID(),
                    previousFrame,
                    previousCalls,
                    previousStartYaw * kRadToDeg,
                    currentYaw * kRadToDeg,
                    realizedDegrees,
                    previousDt,
                    realizedRate,
                    effectiveMaxRate,
                    violated,
                    previousAnimationDriven,
                    previousRuntimeCarrier,
                    previousAuthoredTurn,
                    previousActionBypass,
                    previousLocomotionGraphReadable,
                    previousLocomotionGraphActive,
                    LocomotionWitnessSourceName(previousLocomotionGraphSource));
            }
        }

        void UnregisterActor(RE::FormID formID)
        {
            if (!formID) return;
            std::scoped_lock lock{ g_mutex };
            g_externalAuthoredTurnWitnesses.erase(formID);
            g_nativeStandingTurnRateContexts.erase(formID);
            g_stablePivotDiagnostics.erase(formID);
            const auto it = g_actorControllers.find(formID);
            if (it == g_actorControllers.end()) return;
            g_owners.erase(it->second);
            g_actorControllers.erase(it);
        }

    }


    void BeginRuntimeYawFrame(RE::Character* actor, bool enabled, float deltaTime)
    {
        if (!actor || actor->IsPlayerRef()) return;

        const auto formID = actor->GetFormID();
        if (!enabled || !RuntimeEligible(actor)) {
            UnregisterActor(formID);
            return;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        bool nativeStandingTurnRateActive = false;
        {
            std::scoped_lock lock{ g_mutex };
            const auto it = g_nativeStandingTurnRateContexts.find(formID);
            nativeStandingTurnRateActive =
                it != g_nativeStandingTurnRateContexts.end() && it->second.active;
        }

        if (profile) {
            if (!profile->capsuleSlerp.enabled && !nativeStandingTurnRateActive) {
                UnregisterActor(formID);
                return;
            }
            RegisterController(actor, profile->capsuleSlerp);
            BeginActorFrame(actor, profile->capsuleSlerp, deltaTime);
            return;
        }

        // C1 profileless V13 is a FIREWALL, not the steering source. Register for the
        // actor lifetime so vanilla capsule yaw cannot escape while standing or moving.
        if (!SyntheticMotionInjector::IsUniversalCreatureEligible(actor)) {
            UnregisterActor(formID);
            return;
        }

        MotionProfileDatabase::CapsuleSlerpConfig universal{};
        universal.enabled = true;
        universal.responsePerSecond = 4.0f;
        universal.maxYawRateDegreesPerSecond = 60.0f;
        universal.minErrorDegrees = 0.10f;
        universal.telemetryEveryFrames = 60;
        RegisterController(actor, universal);
        BeginActorFrame(actor, universal, deltaTime);
    }

    bool HasAuthoredTurnWitnessForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!actor) return false;
        if (SyntheticMotionInjector::HasAuthoredRotationClipForCurrentOrPreviousFrame(actor)) {
            return true;
        }
        std::scoped_lock lock{ g_mutex };
        const auto it = g_externalAuthoredTurnWitnesses.find(actor->GetFormID());
        return it != g_externalAuthoredTurnWitnesses.end() && it->second.active;
    }


    bool TryGetFinalBehaviorPhysicalYawDeltaDegrees(
        RE::Actor* actor,
        float& outPhysicalYawDeltaDegrees,
        std::int32_t* outGeneratorOutputIndex)
    {
        outPhysicalYawDeltaDegrees = 0.0f;
        if (outGeneratorOutputIndex) *outGeneratorOutputIndex = -1;
        if (!actor || actor->IsPlayerRef()) return false;

        const auto probe = ProbeFinalBehaviorYaw(actor);
        if (!probe.graphReadable || probe.ambiguous || probe.selectedOutput < 0 ||
            !std::isfinite(probe.physicalYawDegrees)) {
            return false;
        }
        outPhysicalYawDeltaDegrees = probe.physicalYawDegrees;
        if (outGeneratorOutputIndex) *outGeneratorOutputIndex = probe.selectedOutput;
        return true;
    }

    void SetExternalAuthoredTurnWitness(RE::Actor* actor, bool active, bool allowNativeStandingYaw)
    {
        if (!actor || actor->IsPlayerRef()) return;
        const auto formID = actor->GetFormID();
        bool changed = false;
        bool priorActive = false;
        bool priorYawPass = false;
        {
            std::scoped_lock lock{ g_mutex };
            if (const auto it = g_externalAuthoredTurnWitnesses.find(formID);
                it != g_externalAuthoredTurnWitnesses.end()) {
                priorActive = it->second.active;
                priorYawPass = it->second.allowNativeStandingYaw;
            }
            if (active) {
                g_externalAuthoredTurnWitnesses[formID] = { true, allowNativeStandingYaw };
            } else {
                g_externalAuthoredTurnWitnesses.erase(formID);
            }
            changed = priorActive != active ||
                (active && priorYawPass != allowNativeStandingYaw);
        }
        if (changed) {
            SKSE::log::info(
                "[CapsuleSlerp][R103_EXTERNAL_AUTHORED_TURN] form={:08X} active={} yawPass={} source=STABLE_PIVOT_LIFECYCLE action={}",
                formID, active, active && allowNativeStandingYaw,
                !active ? "RESTORE_R74_STANDING_FIREWALL" :
                    (allowNativeStandingYaw ? "PASS_NATIVE_STANDING_YAW" : "HOLD_PHYSICAL_YAW_DURING_TURNSTOP_BLENDOUT"));
        }
    }

    void SetNativeStandingTurnPhysicalYawRate(
        RE::Actor* actor,
        bool active,
        float signedYawRateDegreesPerSecond,
        bool turnDeltaRead,
        float turnDelta)
    {
        if (!actor || actor->IsPlayerRef()) return;
        const auto formID = actor->GetFormID();
        const bool validRate = active && std::isfinite(signedYawRateDegreesPerSecond) &&
            std::abs(signedYawRateDegreesPerSecond) > 1.0e-3f;
        bool changed = false;
        NativeStandingTurnRateContext prior{};
        {
            std::scoped_lock lock{ g_mutex };
            if (const auto it = g_nativeStandingTurnRateContexts.find(formID);
                it != g_nativeStandingTurnRateContexts.end()) {
                prior = it->second;
            }
            if (validRate) {
                auto& context = g_nativeStandingTurnRateContexts[formID];
                context.active = true;
                context.signedYawRateDegreesPerSecond = signedYawRateDegreesPerSecond;
                context.turnDeltaRead = turnDeltaRead && std::isfinite(turnDelta);
                context.turnDelta = context.turnDeltaRead ? turnDelta : 0.0f;
                changed = !prior.active ||
                    std::abs(prior.signedYawRateDegreesPerSecond - context.signedYawRateDegreesPerSecond) > 1.0e-4f ||
                    prior.turnDeltaRead != context.turnDeltaRead;
            } else {
                changed = prior.active;
                g_nativeStandingTurnRateContexts.erase(formID);
            }
        }
        if (changed) {
            SKSE::log::info(
                "[CapsuleSlerp][R115_NATIVE_TURN_RATE_CONTEXT] form={:08X} active={} rate={:+.2f}deg/s TurnDelta[read={} value={:+.3f}] action={}",
                formID, validRate, validRate ? signedYawRateDegreesPerSecond : 0.0f,
                validRate && turnDeltaRead && std::isfinite(turnDelta),
                validRate && turnDeltaRead && std::isfinite(turnDelta) ? turnDelta : 0.0f,
                validRate ? "WAKE_V13_ONLY_FOR_R114_TURN_READY" : "RETURN_TO_PROFILE_NATIVE_POLICY");
        }
    }

    void SetStablePivotDiagnosticContext(
        RE::Actor* actor,
        std::uint8_t phase,
        std::uint32_t routeDegrees,
        bool nativeHkbWitness,
        float nativeHkbWitnessAgeSeconds,
        const char* nativeHkbKey,
        bool turnDeltaRead,
        float turnDelta,
        float physicalYawRateDegreesPerSecond)
    {
        if (!actor || actor->IsPlayerRef()) return;
        const auto formID = actor->GetFormID();
        std::scoped_lock lock{ g_mutex };
        if (phase == 0) {
            g_stablePivotDiagnostics.erase(formID);
            return;
        }
        auto& context = g_stablePivotDiagnostics[formID];
        context.phase = phase;
        context.routeDegrees = routeDegrees;
        context.nativeHkbWitness = nativeHkbWitness;
        context.nativeHkbWitnessAgeSeconds = nativeHkbWitnessAgeSeconds;
        context.nativeHkbKey = nativeHkbKey ? nativeHkbKey : "";
        context.turnDeltaRead = turnDeltaRead;
        context.turnDelta = turnDeltaRead && std::isfinite(turnDelta) ? turnDelta : 0.0f;
        context.physicalYawRateDegreesPerSecond = std::isfinite(physicalYawRateDegreesPerSecond)
            ? physicalYawRateDegreesPerSecond : 0.0f;
    }

    bool TryGetPhysicalYawRadians(RE::Actor* actor, float& outYawRadians)
    {
        outYawRadians = 0.0f;
        if (!actor || actor->IsPlayerRef()) return false;
        auto* controller = actor->GetCharController();
        if (!controller) return false;

        RE::hkTransform current{};
        controller->GetTransformImpl(current);
        const float yaw = ReadYaw(current);
        if (!std::isfinite(yaw)) return false;
        outYawRadians = yaw;
        return true;
    }

    bool InstallRuntimeYawBridge()
    {
        // R35 adds no CapsuleSlerp-side yaw. AI/path steering yaw is injected by
        // SyntheticMotionInjector through ProcessMotionData R1/R2; V13 is observed/chained
        // and its native target is passed through unchanged because V13 is the downstream physical sink used by the proven R30 transport.
        SKSE::log::info(
            "[CapsuleSlerp][R115_INSTALL] legacyStablePivot90=R109_FINAL_OUTPUT_DORMANT_FOR_TROLL; nativeStanding90=R114_LIFECYCLE_PLUS_PROFILE_PHYSICAL_RATE; gate=TURN_READY+READ_ONLY_NONZERO_TURNDELTA; syntheticChildWitness=NONE; TurnDeltaWrites=ZERO; rotationSpeed.z=NOT_SOURCE; localTimeWrites=ZERO; locomotionYaw=PROCESS_MOTION_DATA_R1_R2; profileDisabledCapsule=WAKES_ONLY_DURING_NATIVE_TURN_RATE_CONTEXT");
        return true;
    }

    void UpdateActor(RE::Character* actor, bool enabled)
    {
        // Compatibility path retained for surrounding project code. The normal R35 owner is
        // Hooks::CharacterUpdateHook -> BeginRuntimeYawFrame, which supplies the real frame dt.
        BeginRuntimeYawFrame(actor, enabled, kFallbackDeltaTime);
    }

    void Clear()
    {
        {
            std::scoped_lock lock{ g_mutex };
            for (const auto& [controller, state] : g_owners) {
                SKSE::log::info(
                    "[CapsuleSlerp][R74_SUMMARY] form={:08X} frames={} v13Calls={} nativeTargetSamples={} target[limited={} unchanged={}] bypass[turn={} action={} noAnimDriven={} noCarrier={}] proceduralCarrierCalls={} standing[blocked={} passTurn={} passAction={}] rateBarrier[samples={} clamps={} missingMiddleHigh={} invalid={}] physical[samples={} violations={}] last[nativeTarget={:+.2f}deg error={:+.3f}deg limitedStep={:+.3f}deg rawRate={:+.2f}deg/s limitedRate={:+.2f}deg/s]",
                    state.formID,
                    state.frameSerial,
                    state.totalV13Calls,
                    state.nativeTargetSamples,
                    state.targetLimited,
                    state.targetUnchanged,
                    state.turnBypasses,
                    state.actionBypasses,
                    state.noAnimationDrivenBypasses,
                    state.noLocomotionCarrierBypasses,
                    state.proceduralCarrierCalls,
                    state.standingYawBlocked,
                    state.standingYawPassAuthoredTurn,
                    state.standingYawPassAction,
                    state.commandSamples,
                    state.commandClamps,
                    state.missingMiddleHigh,
                    state.invalidCommands,
                    state.realizedSamples,
                    state.realizedViolations,
                    state.lastNativeTargetYaw * kRadToDeg,
                    state.lastNativeError * kRadToDeg,
                    state.lastLimitedStep * kRadToDeg,
                    state.lastCommandInput * kRadToDeg,
                    state.lastCommandOutput * kRadToDeg);
            }

            for (const auto& [vtable, hook] : g_hooks) {
                if (!vtable || !hook.original) continue;
                if (vtable[kAngularTransformVFunc] == reinterpret_cast<std::uintptr_t>(&AngularTransformThunk)) {
                    REL::safe_write(
                        reinterpret_cast<std::uintptr_t>(std::addressof(vtable[kAngularTransformVFunc])),
                        reinterpret_cast<std::uintptr_t>(hook.original));
                }
            }

            g_hooks.clear();
            g_owners.clear();
            g_actorControllers.clear();
            g_externalAuthoredTurnWitnesses.clear();
            g_nativeStandingTurnRateContexts.clear();
            g_stablePivotDiagnostics.clear();
        }

        SKSE::log::info(
            "[CapsuleSlerp][R74_CLEAR] v13OwnersCleared=true movingPolicy=R35_NATIVE_PASS standingPolicy=AUTHORED_TURN_REQUIRED");
    }
}

