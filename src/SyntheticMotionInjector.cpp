// BUILD_ID: INCESSUS-C1-NATIVE-EXTRACTED-ROOT-PMD-YAW-2026-10-03
// R94 is a diagnostic-only probe layered on the R85/R93 transport contract.
// It does not change graph selection, WFM injection, root authority, playback, yaw, or warping.
// Its sole purpose is to distinguish generated aliases from distinct MotionDB keys/phases before
// any corrective change is attempted for the remaining micro-tremors.
// R85 intentionally restores the pre-Troll R69.7 physical transport contract.
// Physical translation authority is the canonical MotionDB-backed HKX sampled at its
// clip local time and injected into that clip leaf's WORLD_FROM_MODEL. Behavior node
// names/aliases are observation only; ProcessMotionData T1/T2 never transport creature XY.
// R65 preserves the proven R62/R59 root lifecycle and transport. The CTD fix is
// deliberately narrow: raw ABI pointers are range-checked only at the exact bytes
// about to be dereferenced. No zero-dt timeout, no carrier-policy rewrite, no yaw freeze.
// BASE: proven R30 ProcessMotionData R1/R2 yaw transport preserved.
// R35 preserves the R34 SteeringIntentController + restored DB lifecycle; V13 policy changes only in CapsuleSlerp.
// R33 replaced Bethesda controller-rate sourcing with SteeringIntentController's
// AI/path-derived, filtered, acceleration-limited yaw command. Authored RotationClips
// remain MotionDB-owned; all other creature DB clips preserve native R1/R2.
#include "PCH.h"
#include "SyntheticMotionInjector.h"
#include "AnimationMotionDatabase.h"
#include "AnimationPriority.h"
#include "CapsuleSlerp.h"
#include "MotionProfileDatabase.h"
#include "TranslationAuthority.h"
#include "SteeringIntentController.h"
#include "SpeedIntentTranslator.h"
#include "SKSE/Trampoline.h"

#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <Windows.h>
#include "RE/A/AIProcess.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BShkbAnimationGraph.h"
#include "RE/H/HighProcessData.h"
#include "RE/H/hkbGenerator.h"
#include "RE/M/MiddleHighProcessData.h"
#include "RE/T/TESPackage.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cstddef>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <sstream>
#include <limits>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace SyntheticMotionInjector
{
    using Clock = std::chrono::steady_clock;

    namespace detail
    {
        constexpr REL::RelocationID kProcessMotionDataID{ 31949, 32703 };
        constexpr REL::RelocationID kProcessTranslationDataID{ 31812, 32582 };
        constexpr REL::RelocationID kProcessRotationDataID{ 31813, 32583 };

        constexpr std::size_t kStubCapacity = 128;
        constexpr std::size_t kRequiredTrampolineBytes = 4 * kStubCapacity + 64;
        constexpr std::size_t kDefaultTrampolineBytes = 1024;
        constexpr std::uint64_t kCallLogLimit = 0;
        constexpr std::uint64_t kPairLogLimit = 0;
        constexpr float kCyclePassRelativeError = 0.05f;
        constexpr bool kEnablePlayerCycleAudit = false;

        // B3 universal creature locomotion session. ActorState direction pre-arms
        // bAnimationDriven BEFORE vanilla capsule translation can escape. The actual root
        // authority still requires a real vanilla T1->T2 translation delta. The pre-arm is
        // bounded and action states cancel it immediately. Playable races are excluded.
        constexpr std::uint32_t kNativeLocomotionContinuityGraceFrames = 3;
        constexpr std::uint32_t kNativeLocomotionProfilelessContinuityGraceFrames = 12;
        constexpr std::uint32_t kNativeLocomotionEntryTimeoutFrames = 8;

        // R35: keep the R30-proven ProcessMotionData transport; source is now the AI/path controller.
        // Legacy R32 rate constants remain only for dormant diagnostics in this branch.
        // for the frame. The historical R16.8 sign convention is preserved exactly:
        //      PMD yaw step = -preFrameRotationSpeed.z * dt
        // The 720deg/s clamp is a corruption/ABI safety barrier, not gameplay tuning.
        constexpr float kVanillaYawFallbackDeltaTime = 1.0f / 60.0f;
        constexpr float kRadiansToDegrees = 57.295779513082320876f;
        constexpr float kMaxSaneVanillaYawRateRadPerSecond = 12.566370614359172953f;  // 720 deg/s

        // R65 surgical CTD firewall: validate only bytes immediately dereferenced.
        enum class ExactMemoryAccess : std::uint8_t { kRead, kWrite };

        [[nodiscard]] bool IsExactRangeAccessible(const void* pointer, std::size_t bytes, ExactMemoryAccess access) noexcept
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
                    mbi.State != MEM_COMMIT || (mbi.Protect & (PAGE_GUARD | PAGE_NOACCESS)) != 0) return false;
                const DWORD prot = mbi.Protect & 0xFFu;
                const bool readable = prot == PAGE_READONLY || prot == PAGE_READWRITE || prot == PAGE_WRITECOPY ||
                    prot == PAGE_EXECUTE_READ || prot == PAGE_EXECUTE_READWRITE || prot == PAGE_EXECUTE_WRITECOPY;
                const bool writable = prot == PAGE_READWRITE || prot == PAGE_WRITECOPY ||
                    prot == PAGE_EXECUTE_READWRITE || prot == PAGE_EXECUTE_WRITECOPY;
                if (!readable || (access == ExactMemoryAccess::kWrite && !writable)) return false;
                const auto regionBegin = reinterpret_cast<std::uintptr_t>(mbi.BaseAddress);
                if (mbi.RegionSize == 0 || regionBegin > (std::numeric_limits<std::uintptr_t>::max)() - mbi.RegionSize) return false;
                const auto regionEnd = regionBegin + mbi.RegionSize;
                if (regionEnd <= cursor) return false;
                cursor = std::min(regionEnd, end);
            }
            return true;
        }

        void LogR65PointerReject(std::string_view site, const void* pointer, std::size_t bytes) noexcept
        {
            static std::atomic<std::uint32_t> logs{0};
            const auto n = logs.fetch_add(1, std::memory_order_relaxed);
            if (n < 12) {
                SKSE::log::warn("[WFM][R65_CTD_POINTER_REJECT] site={} pointer=0x{:X} bytes={} action=SKIP_ONLY_THIS_RAW_DEREFERENCE",
                    site, reinterpret_cast<std::uintptr_t>(pointer), bytes);
            }
        }

        enum class Site : std::uint8_t
        {
            kTranslation1,
            kTranslation2,
            kRotation1,
            kRotation2
        };

        // D1 diagnostic-only mirror of the native MotionDataContainer layout used by
        // Character::ProcessMotionData. This candidate NEVER samples, mutates, replaces,
        // or owns these buffers. It only records the container metadata already passed to
        // the four verified T1/T2/R1/R2 sampler call sites.
        //
        // Layout is intentionally local instead of adding a new runtime dependency:
        //   +0x00 translationDataPtr
        //   +0x08 translationSegCount
        //   +0x10 rotationDataPtr
        //   +0x18 rotationSegCount
        struct NativeMotionDataContainerView
        {
            std::uintptr_t translationDataPtr{ 0 };
            std::uint32_t translationSegCount{ 0 };
            std::uint32_t pad0C{ 0 };
            std::uintptr_t rotationDataPtr{ 0 };
            std::uint32_t rotationSegCount{ 0 };
            std::uint32_t pad1C{ 0 };
        };
        static_assert(offsetof(NativeMotionDataContainerView, translationDataPtr) == 0x00);
        static_assert(offsetof(NativeMotionDataContainerView, translationSegCount) == 0x08);
        static_assert(offsetof(NativeMotionDataContainerView, rotationDataPtr) == 0x10);
        static_assert(offsetof(NativeMotionDataContainerView, rotationSegCount) == 0x18);
        static_assert(sizeof(NativeMotionDataContainerView) == 0x20);

        struct NativeMotionContainerSnapshot
        {
            bool readable{ false };
            std::uintptr_t translationDataPtr{ 0 };
            std::uint32_t translationSegCount{ 0 };
            std::uintptr_t rotationDataPtr{ 0 };
            std::uint32_t rotationSegCount{ 0 };
        };

        [[nodiscard]] NativeMotionContainerSnapshot ReadNativeMotionContainer(const void* raw) noexcept
        {
            NativeMotionContainerSnapshot out{};
            if (!IsExactRangeAccessible(raw, sizeof(NativeMotionDataContainerView), ExactMemoryAccess::kRead)) {
                return out;
            }
            const auto* view = static_cast<const NativeMotionDataContainerView*>(raw);
            out.readable = true;
            out.translationDataPtr = view->translationDataPtr;
            out.translationSegCount = view->translationSegCount;
            out.rotationDataPtr = view->rotationDataPtr;
            out.rotationSegCount = view->rotationSegCount;
            return out;
        }

        [[nodiscard]] constexpr const char* SiteName(Site site) noexcept
        {
            switch (site) {
            case Site::kTranslation1: return "T1";
            case Site::kTranslation2: return "T2";
            case Site::kRotation1: return "R1";
            case Site::kRotation2: return "R2";
            default: return "?";
            }
        }

        struct NativeMotionAuditKey
        {
            RE::FormID formID{ 0 };
            Site site{ Site::kTranslation1 };
            std::string clipName;

            bool operator==(const NativeMotionAuditKey& other) const noexcept
            {
                return formID == other.formID && site == other.site && clipName == other.clipName;
            }
        };

        struct NativeMotionAuditKeyHash
        {
            std::size_t operator()(const NativeMotionAuditKey& key) const noexcept
            {
                std::size_t h = std::hash<RE::FormID>{}(key.formID);
                const auto hs = std::hash<std::uint8_t>{}(static_cast<std::uint8_t>(key.site));
                const auto hc = std::hash<std::string>{}(key.clipName);
                h ^= hs + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
                h ^= hc + 0x9E3779B97F4A7C15ull + (h << 6) + (h >> 2);
                return h;
            }
        };

        [[nodiscard]] std::string FixedString(const RE::BSFixedString* value);
        [[nodiscard]] float QuaternionYaw(const RE::NiQuaternion& q) noexcept;

        struct NativeMotionAuditState
        {
            bool initialized{ false };
            NativeMotionContainerSnapshot container{};
            std::uint64_t calls{ 0 };
        };

        std::mutex g_nativeMotionAuditMutex;
        std::unordered_map<NativeMotionAuditKey, NativeMotionAuditState, NativeMotionAuditKeyHash> g_nativeMotionAudit;
        constexpr std::uint64_t kNativeMotionAuditPeriodicCalls = 60;

        [[nodiscard]] bool SameNativeMotionContainer(
            const NativeMotionContainerSnapshot& a,
            const NativeMotionContainerSnapshot& b) noexcept
        {
            return a.readable == b.readable &&
                   a.translationDataPtr == b.translationDataPtr &&
                   a.translationSegCount == b.translationSegCount &&
                   a.rotationDataPtr == b.rotationDataPtr &&
                   a.rotationSegCount == b.rotationSegCount;
        }

        void AuditNativeMotionContainer(
            Site site,
            const void* motionData,
            float motionTime,
            const RE::BSFixedString* clipNameValue,
            RE::Character* character,
            const RE::NiPoint3* translation,
            const RE::NiQuaternion* rotation)
        {
            // Diagnostic is intentionally broader than the current profile opt-in so one
            // unprofiled creature can be tested in the same build. Player remains excluded.
            if (!character || character->IsPlayerRef()) {
                return;
            }

            const auto container = ReadNativeMotionContainer(motionData);
            const std::string clipName = FixedString(clipNameValue);

            bool emit = false;
            const char* reason = "PERIODIC";
            std::uint64_t calls = 0;
            {
                std::scoped_lock lock{ g_nativeMotionAuditMutex };
                NativeMotionAuditKey key{ character->GetFormID(), site, clipName };
                auto& state = g_nativeMotionAudit[key];
                ++state.calls;
                calls = state.calls;

                const bool changed = state.initialized &&
                    !SameNativeMotionContainer(state.container, container);

                if (!state.initialized) {
                    emit = true;
                    reason = "FIRST";
                } else if (changed) {
                    emit = true;
                    reason = "STATE_CHANGE";
                } else if ((state.calls % kNativeMotionAuditPeriodicCalls) == 0) {
                    emit = true;
                    reason = "PERIODIC";
                }

                state.initialized = true;
                state.container = container;
            }

            if (!emit) {
                return;
            }

            // Expensive graph/native state reads are performed only when a diagnostic line
            // will actually be emitted (first observation, container change, or periodic).
            bool graphDriven = false;
            const bool graphDrivenRead = character->GetGraphVariableBool("bAnimationDriven", graphDriven);
            bool allowRotation = false;
            const bool allowRotationRead = character->GetGraphVariableBool("bAllowRotation", allowRotation);
            const bool animationDriven = character->IsAnimationDriven();
            bool moving = false;
            character->GetGraphVariableBool("isMoving", moving);

            const bool translationAligned = (container.translationDataPtr & 1u) == 0;
            const bool rotationAligned = (container.rotationDataPtr & 1u) == 0;
            // Mirrors the vanilla-list presence test used by AMR. The low pointer bit is
            // part of Bethesda's representation; we report it, never alter it.
            const bool hasVanillaTranslationList = container.readable &&
                container.translationSegCount > static_cast<std::uint32_t>(translationAligned);
            const bool hasVanillaRotationList = container.readable &&
                container.rotationSegCount > static_cast<std::uint32_t>(rotationAligned);

            const auto* profile = MotionProfileDatabase::ResolveProfile(character);
            const bool hasProfile = profile != nullptr;
            const std::string profileName = profile ? profile->name : std::string{ "<none>" };
            if (translation) {
                SKSE::log::info(
                    R"([D1_NATIVE_MOTION] reason={} call={} form={:08X} site={} clip="{}" motionTime={:+.6f} container=0x{:X} readable={} T[ptr=0x{:X} segs={} lowBit={} vanillaList={}] R[ptr=0x{:X} segs={} lowBit={} vanillaList={}] sampleT=({:+.5f},{:+.5f},{:+.5f}) graph[bAnimationDrivenRead={} bAnimationDriven={} IsAnimationDriven={} bAllowRotationRead={} bAllowRotation={} moving={}] profile="{}" hasProfile={} writes=ZERO)",
                    reason, calls, character->GetFormID(), SiteName(site), clipName, motionTime,
                    reinterpret_cast<std::uintptr_t>(motionData), container.readable,
                    container.translationDataPtr, container.translationSegCount,
                    static_cast<unsigned>(container.translationDataPtr & 1u), hasVanillaTranslationList,
                    container.rotationDataPtr, container.rotationSegCount,
                    static_cast<unsigned>(container.rotationDataPtr & 1u), hasVanillaRotationList,
                    translation->x, translation->y, translation->z,
                    graphDrivenRead, graphDriven, animationDriven,
                    allowRotationRead, allowRotation, moving,
                    profileName, hasProfile);
            } else if (rotation) {
                SKSE::log::info(
                    R"([D1_NATIVE_MOTION] reason={} call={} form={:08X} site={} clip="{}" motionTime={:+.6f} container=0x{:X} readable={} T[ptr=0x{:X} segs={} lowBit={} vanillaList={}] R[ptr=0x{:X} segs={} lowBit={} vanillaList={}] sampleR=({:+.6f},{:+.6f},{:+.6f},{:+.6f}) sampleYaw={:+.6f}rad/{:+.3f}deg graph[bAnimationDrivenRead={} bAnimationDriven={} IsAnimationDriven={} bAllowRotationRead={} bAllowRotation={} moving={}] profile="{}" hasProfile={} writes=ZERO)",
                    reason, calls, character->GetFormID(), SiteName(site), clipName, motionTime,
                    reinterpret_cast<std::uintptr_t>(motionData), container.readable,
                    container.translationDataPtr, container.translationSegCount,
                    static_cast<unsigned>(container.translationDataPtr & 1u), hasVanillaTranslationList,
                    container.rotationDataPtr, container.rotationSegCount,
                    static_cast<unsigned>(container.rotationDataPtr & 1u), hasVanillaRotationList,
                    rotation->w, rotation->x, rotation->y, rotation->z,
                    QuaternionYaw(*rotation), QuaternionYaw(*rotation) * kRadiansToDegrees,
                    graphDrivenRead, graphDriven, animationDriven,
                    allowRotationRead, allowRotation, moving,
                    profileName, hasProfile);
            }
        }

        struct RuntimeSites
        {
            std::uintptr_t translation1{ 0 };
            std::uintptr_t translation2{ 0 };
            std::uintptr_t rotation1{ 0 };
            std::uintptr_t rotation2{ 0 };
        };

        struct ClipKey
        {
            const RE::hkbCharacter* character{ nullptr };
            std::string clipName;

            bool operator==(const ClipKey& other) const noexcept
            {
                return character == other.character && clipName == other.clipName;
            }
        };

        struct ClipKeyHash
        {
            std::size_t operator()(const ClipKey& key) const noexcept
            {
                const auto h1 = std::hash<const void*>{}(key.character);
                const auto h2 = std::hash<std::string>{}(key.clipName);
                return h1 ^ (h2 + 0x9E3779B97F4A7C15ull + (h1 << 6) + (h1 >> 2));
            }
        };

        struct ClipInfo
        {
            std::string profile;
            std::string animationName;
            std::string databaseKey;
            float clipLocalTime{ 0.0f };
            float controlLocalTime{ 0.0f };
            float weight{ 0.0f };
            float motionTrackWeight{ 0.0f };
            float futureLocalTime{ 0.0f };
            std::int32_t futureLoops{ 0 };
            float duration{ 0.0f };
            bool hasControl{ false };
            bool futureTimeValid{ false };
            bool databaseBacked{ false };
            std::uint64_t seenFrameSerial{ 0 };
            Clock::time_point seen{};
        };

        struct TranslationHalf
        {
            bool valid{ false };
            float motionTime{ 0.0f };
            RE::NiPoint3 existing{};
            bool externalValid{ false };
            RE::NiPoint3 external{};
            std::string animationName;
            std::string key;
            float mapAge{ std::numeric_limits<float>::infinity() };
            float controlLocalTime{ 0.0f };
            float weight{ 0.0f };
            float motionTrackWeight{ 0.0f };
            float futureLocalTime{ 0.0f };
            std::int32_t futureLoops{ 0 };
        };

        struct RotationHalf
        {
            bool valid{ false };
            float motionTime{ 0.0f };
            RE::NiQuaternion existing{};
        };

        struct MotionPairState
        {
            TranslationHalf t1{};
            RotationHalf r1{};
        };

        struct PlayerCycleAudit
        {
            bool localTimeValid{ false };
            float lastLocalTime{ 0.0f };
            bool cycleStarted{ false };
            float physicalPathXY{ 0.0f };
            std::uint64_t attributedFrames{ 0 };
            bool ambiguousBlend{ false };
            std::uint64_t completedCycles{ 0 };
            std::uint64_t tightPasses{ 0 };
            std::uint64_t driftCycles{ 0 };
            std::uint64_t ambiguousCycles{ 0 };
        };

        struct RootMotionWitness
        {
            const RE::hkbClipGenerator* clip{ nullptr };
            std::string nodeName;
            std::string animationName;
            bool motionDbBacked{ false };
            bool clipMemberDeltaObserved{ false };
            float extractedX{ 0.0f };
            float extractedY{ 0.0f };
            float extractedZ{ 0.0f };
            float extractedYawDeg{ 0.0f };
        };

        struct RootContributionWitness
        {
            const RE::hkbClipGenerator* clip{ nullptr };
            std::string nodeName;
            std::string animationName;
            std::string databaseKey;
            bool databaseBacked{ false };
            bool controlReadable{ false };
            bool deltaValid{ false };
            bool yawValid{ false };
            float clipLocalTime{ 0.0f };
            float controlWeight{ 0.0f };
            float masterWeight{ 0.0f };
            float motionTrackWeight{ 0.0f };
            float poseWeight{ 0.0f };
            float deltaX{ 0.0f };
            float deltaY{ 0.0f };
            float deltaZ{ 0.0f };
            float deltaYaw{ 0.0f };
        };

        struct RootContributionSummary
        {
            std::size_t clips{ 0 };
            std::size_t dbClips{ 0 };
            float poseWeight{ 0.0f };
            float dbRootWeight{ 0.0f };
            float dbRootShare{ 0.0f };
            float mixedX{ 0.0f };
            float mixedY{ 0.0f };
            float mixedZ{ 0.0f };
            float mixedYaw{ 0.0f };
            bool mixedTranslationValid{ false };
            bool mixedYawValid{ false };
        };

        // R94 diagnostic-only topology probe. IMPORTANT: dbClips counts generated leaf
        // callbacks, not necessarily distinct physical animations. The same authored HKX
        // can appear through several aliases. We therefore group generated DB contributors
        // by MotionDB key and separately detect same-key phase/delta divergence. This probe
        // never changes weights or suppresses any leaf.
        struct PhysicalCompositionProbe
        {
            std::size_t dbLeaves{ 0 };
            std::size_t uniqueKeys{ 0 };
            std::size_t phaseSplitKeys{ 0 };
            std::size_t maxAliasesForOneKey{ 0 };
            std::string signature;
            std::string detail;
        };

        struct SyntheticClipPhaseState
        {
            std::string animationName;
            float previousLocalTime{ 0.0f };
            std::uint64_t lastSeenFrame{ 0 };
            bool valid{ false };
        };

        struct SyntheticWfmInjectionWitness
        {
            const RE::hkbClipGenerator* clip{ nullptr };
            const void* output{ nullptr };
            std::string animationName;
            std::string databaseKey;
            float localDx{ 0.0f };
            float localDy{ 0.0f };
            float worldDx{ 0.0f };
            float worldDy{ 0.0f };
            float beforeX{ 0.0f };
            float beforeY{ 0.0f };
            float afterX{ 0.0f };
            float afterY{ 0.0f };
            std::uint64_t frame{ 0 };
            bool applied{ false };
        };

        struct SyntheticWfmSideProbe
        {
            bool readable{ false };
            bool syntheticRoot{ false };
            float offsetX{ 0.0f };
            float offsetY{ 0.0f };
            float distanceXY{ 0.0f };
        };

        enum class TransitionSideMapping : std::uint8_t
        {
            kUnknown = 0,
            kChild0FromChild1To,
            kChild1FromChild0To
        };

        struct TransitionWfmAnchors
        {
            bool readable{ false };
            std::int32_t size{ 0 };
            float fromX{ 0.0f };
            float fromY{ 0.0f };
            float toX{ 0.0f };
            float toY{ 0.0f };
        };

        struct TransitionSideMapProbe
        {
            TransitionSideMapping mapping{ TransitionSideMapping::kUnknown };
            bool anchorsReadable{ false };
            float directScore{ std::numeric_limits<float>::infinity() };
            float swappedScore{ std::numeric_limits<float>::infinity() };
            float confidence{ 0.0f };
        };

        struct TransitionSideMapState
        {
            TransitionSideMapping mapping{ TransitionSideMapping::kUnknown };
            std::uint64_t lastSeenFrame{ 0 };
            const RE::hkbGenerator* fromGenerator{ nullptr };
            const RE::hkbGenerator* toGenerator{ nullptr };
        };

        enum class RuntimeWfmPolicy : std::uint8_t
        {
            kNone = 0,
            kIgnoreFrom,
            kIgnoreTo,
            kRootToRoot
        };

        struct RuntimeWfmPolicyState
        {
            RuntimeWfmPolicy policy{ RuntimeWfmPolicy::kNone };
            std::uint64_t createdFrame{ 0 };
            std::uint64_t lastSeenFrame{ 0 };
            const RE::hkbGenerator* fromGenerator{ nullptr };
            const RE::hkbGenerator* toGenerator{ nullptr };
        };


        struct PhaseSyncGroupConfig
        {
            std::array<std::string, 8> keys{};
            std::size_t keyCount{ 0 };
        };

        struct PhaseSyncConfig
        {
            bool loaded{ false };
            bool enabled{ false };
            std::uint32_t stallFrames{ 3 };
            float stallDeltaSeconds{ 0.001f };
            float snapThresholdCycles{ 0.08f };
            float releaseThresholdCycles{ 0.04f };
            std::uint32_t releaseHealthyFrames{ 4 };
            std::array<PhaseSyncGroupConfig, 8> groups{};
            std::size_t groupCount{ 0 };
        };

        struct DirectionalPhaseAnchorState
        {
            bool valid{ false };
            std::uint64_t frame{ 0 };
            float phase{ 0.0f };
            float lastDeltaCycles{ 0.0f };
            bool moving{ false };
        };

        struct DirectionalPhaseClipState
        {
            bool rawValid{ false };
            float lastRawLocalTime{ 0.0f };
            std::uint64_t lastFrame{ 0 };
            std::uint32_t stalledFrames{ 0 };
            std::uint32_t healthyFrames{ 0 };
            bool locked{ false };
            std::uint64_t lockFrame{ 0 };
            std::uint64_t corrections{ 0 };
        };

        struct NativeTranslationPairSample
        {
            RE::NiPoint3 value{};
            float motionTime{ 0.0f };
            std::uint64_t frame{ 0 };
            std::string clipName;
            bool valid{ false };
        };

        struct NativeRotationPairSample
        {
            RE::NiQuaternion value{};
            float motionTime{ 0.0f };
            std::uint64_t frame{ 0 };
            std::string clipName;
            bool valid{ false };
        };

        struct NativeExtractedDeltaSample
        {
            RE::NiPoint3 delta{};
            std::uint64_t frame{ 0 };
            std::string clipName;
            bool valid{ false };
        };

        struct NativeActorState
        {
            std::string profile;
            std::uint64_t frameSerial{ 0 };
            const RE::hkbCharacter* hkbCharacter{ nullptr };
            std::array<const RE::hkbCharacter*, 8> hkbCharacters{};
            std::size_t hkbCharacterCount{ 0 };
            bool isPlayer{ false };
            std::uint64_t translationCalls{ 0 };
            std::uint64_t rotationCalls{ 0 };
            std::uint64_t translationPairs{ 0 };
            std::uint64_t rotationPairs{ 0 };
            std::uint64_t callLogs{ 0 };
            std::uint64_t pairLogs{ 0 };
            std::uint64_t externalTranslationWriteFrame{ 0 };
            std::uint64_t externalRotationWriteFrame{ 0 };
            std::uint64_t externalTranslationWrites{ 0 };
            std::uint64_t externalRotationWrites{ 0 };
            std::uint64_t externalPassThroughs{ 0 };

            // B1 universal native-motion authority. T1/T2 and R1/R2 are sampled by
            // Skyrim before our callbacks. A non-zero native pair delta is therefore a
            // direct runtime witness that this animation already carries vanilla motion.
            // These fields replace MotionDB membership as the bootstrap proof; they do not
            // alter the sampler outputs and do not classify animation names.
            std::unordered_map<std::uintptr_t, NativeTranslationPairSample> nativeTranslationPairs;
            std::unordered_map<std::uintptr_t, NativeRotationPairSample> nativeRotationPairs;
            std::uint64_t nativeMotionSeenFrame{ 0 };
            std::string nativeMotionSeenClip;
            bool nativeMotionSeenTranslation{ false };
            bool nativeMotionSeenRotation{ false };
            float nativeMotionSeenMagnitude{ 0.0f };
            std::uint64_t nativeMotionWitnesses{ 0 };

            // B2: separate universal locomotion lifecycle. This is intentionally NOT
            // creatureDbSessionActive: native Skyrim motion must not inherit MotionDB
            // continuity/ownership semantics. The list witness is metadata-only and may
            // bootstrap bAnimationDriven before T1/T2 begin producing a non-zero delta.
            std::uint64_t nativeTranslationListSeenFrame{ 0 };
            std::string nativeTranslationListSeenClip;
            std::uint64_t nativeTranslationDeltaSeenFrame{ 0 };
            std::string nativeTranslationDeltaSeenClip;
            float nativeTranslationDeltaSeenMagnitude{ 0.0f };
            std::uint64_t nativeRotationDeltaSeenFrame{ 0 };
            std::string nativeRotationDeltaSeenClip;
            float nativeRotationDeltaSeenMagnitude{ 0.0f };

            // C1 native-extracted transport. ProcessMotionData is DATA SOURCE ONLY.
            // A T1->T2 delta is cached per canonical clip and, on the next Generate,
            // written into that leaf's TRACK_EXTRACTED_MOTION. Havok then composes
            // the same root through the same tree/blends as the pose.
            std::unordered_map<std::string, NativeExtractedDeltaSample> nativeExtractedDeltas;
            std::uint64_t universalExtractedLeafSeenFrame{ 0 };
            std::uint64_t universalExtractedLeafWrites{ 0 };
            std::uint64_t universalExtractedNativePasses{ 0 };
            bool universalExtractedAuthorityEstablished{ false };
            std::uint64_t universalFinalRootFrame{ 0 };
            RE::NiPoint3 universalFinalRootDelta{};
            RE::NiPoint3 universalLastVelocity{};
            std::uint32_t universalRootGapFrames{ 0 };
            std::uint64_t universalRootExactFrames{ 0 };
            std::uint64_t universalRootBridgeFrames{ 0 };
            std::uint64_t universalRootReleaseCount{ 0 };
            std::uint64_t universalRootLogs{ 0 };
            std::uint64_t universalDeltaWaitLogs{ 0 };
            std::uint64_t universalTrackRejectLogs{ 0 };
            std::uint64_t universalFinalWaitLogs{ 0 };

            bool nativeLocomotionSessionActive{ false };
            bool nativeLocomotionAuthorityEstablished{ false };
            bool nativeLocomotionDrivenLatched{ false };
            bool nativeLocomotionOriginalDriven{ false };
            std::uint64_t nativeLocomotionBootstrapDrivenFrame{ 0 };
            std::uint32_t nativeLocomotionContinuityMissFrames{ 0 };
            std::uint32_t nativeLocomotionEntryAgeFrames{ 0 };
            std::uint64_t nativeLocomotionSessionStarts{ 0 };
            std::uint64_t nativeLocomotionSessionStops{ 0 };
            std::uint64_t nativeLocomotionArms{ 0 };
            std::uint64_t nativeLocomotionHolds{ 0 };
            std::uint64_t nativeLocomotionRestores{ 0 };

            // R15.3 diagnostic root-contribution ledger. hkbClipGenerator::Generate is the
            // observation point so only clips that actually participate in generation
            // are counted. DB root deltas are blended by final animation-control pose
            // weights; non-DB clips contribute zero root but remain in the denominator.
            float frameDeltaTime{ 0.0f };
            std::array<RootContributionWitness, 64> rootContributions{};
            std::size_t rootContributionCount{ 0 };

            // R15.6 synthetic WFM transport. Each MotionDB clip keeps its own local-time
            // history so we can derive a true frame delta, inject that delta into the
            // clip's TRACK_WORLD_FROM_MODEL, and let Havok blend the resulting transforms.
            std::unordered_map<const RE::hkbClipGenerator*, SyntheticClipPhaseState> syntheticClipPhase;
            // R52 configured directional phase reacquire. The C++ does not know species
            // or gait clip names; Data/MotionDB/<profile>/GaitRouter.ini declares phase
            // groups as MotionDB keys with the first key acting as the phase anchor.
            std::unordered_map<const RE::hkbClipGenerator*, DirectionalPhaseClipState> directionalPhaseClips;
            std::array<DirectionalPhaseAnchorState, 8> directionalPhaseAnchors{};
            std::uint64_t directionalPhaseCorrections{ 0 };
            std::uint64_t directionalPhaseLockAcquires{ 0 };
            std::array<SyntheticWfmInjectionWitness, 64> syntheticWfmInjections{};
            std::size_t syntheticWfmInjectionCount{ 0 };
            std::uint64_t syntheticWfmTranslationSeenFrame{ 0 };
            std::uint64_t syntheticWfmAppliedTotal{ 0 };
            std::uint64_t syntheticWfmWaitTotal{ 0 };
            std::uint64_t syntheticWfmBypassTxTotal{ 0 };
            std::uint64_t syntheticWfmClipLogs{ 0 };
            std::uint64_t syntheticWfmSideLogs{ 0 };

            // R32 pre-frame vanilla yaw snapshot + proven R30 ProcessMotionData transport.
            // The sample is captured once before native Character::Update and shared by every
            // generated W/T/R leaf so later PMD timing cannot erase the controller request.
            std::uint64_t vanillaYawPreFrameSampleFrame{ 0 };
            bool vanillaYawPreFrameValid{ false };
            float vanillaYawPreFrameRateRadPerSecond{ 0.0f };
            float vanillaYawPreFramePathDesiredRateRadPerSecond{ 0.0f };
            float vanillaYawPreFramePathCurrentRateRadPerSecond{ 0.0f };
            float vanillaYawPreFrameMiddleRotationZ{ 0.0f };
            float vanillaYawPreFrameDesiredMoveX{ 0.0f };
            float vanillaYawPreFrameDesiredMoveY{ 0.0f };
            std::uint64_t vanillaYawSnapshotLogs{ 0 };
            bool vanillaYawLastSnapshotValid{ false };
            float vanillaYawLastSnapshotRateRadPerSecond{ 0.0f };
            std::uint64_t vanillaYawTransportAppliedTotal{ 0 };
            std::uint64_t vanillaYawTransportLogs{ 0 };
            std::uint64_t vanillaYawLastLogFrame{ 0 };
            bool vanillaYawLastLoggedValid{ false };
            float vanillaYawLastLoggedRateRadPerSecond{ 0.0f };

            // R35 AI-goal controller transport telemetry. The R32 fields above are retained
            // only for layout/source continuity in this branch and are no longer authoritative.
            std::uint64_t aiSteeringTransportAppliedTotal{ 0 };
            std::uint64_t aiSteeringTransportLogs{ 0 };
            std::uint64_t aiSteeringLastLogFrame{ 0 };
            bool aiSteeringLastLoggedValid{ false };
            float aiSteeringLastLoggedRateRadPerSecond{ 0.0f };
            std::uint8_t aiSteeringLastLoggedSource{ 0 };
            RootContributionSummary previousRootMix{};
            RootContributionSummary completedRootMix{};
            std::uint64_t rootMixerFrames{ 0 };
            std::uint64_t rootMixerDbFrames{ 0 };
            std::uint64_t rootMixerClipLogs{ 0 };
            std::uint64_t rootMixerFrameLogs{ 0 };
            bool rootMixerLastDbActive{ false };
            float rootMixerLastLoggedShare{ 0.0f };

            // R94 diagnostic-only physical composition topology. These fields only rate-limit
            // telemetry; they never participate in root authority or selection.
            std::string physicalCompositionLastSignature;
            std::uint64_t physicalCompositionLastLogFrame{ 0 };
            std::uint64_t physicalCompositionLogs{ 0 };

            std::uint64_t previousTransitionCalls{ 0 };

            // Runtime WFM policy is latched per transition instance for the lifetime
            // of that blend. Serialized vanilla policy always wins. Runtime policy
            // only changes IGNORE_FROM/IGNORE_TO and is restored immediately after
            // Generate; SYNC, IGNORE_TO_ROTATION and unknown bits are untouched.
            std::unordered_map<const void*, RuntimeWfmPolicyState> runtimeWfmPolicies;
            std::unordered_map<const void*, TransitionSideMapState> transitionSideMaps;
            std::uint64_t wfmSideMapResolvedDirectTotal{ 0 };
            std::uint64_t wfmSideMapResolvedSwappedTotal{ 0 };
            std::uint64_t wfmSideMapCacheUsedTotal{ 0 };
            std::uint64_t wfmSideMapAmbiguousTotal{ 0 };
            std::uint64_t wfmSideMapPostCalibratedTotal{ 0 };
            std::uint64_t wfmSideMapDisagreementTotal{ 0 };
            std::uint64_t wfmRuntimeIgnoreFromSeenFrame{ 0 };
            std::uint64_t wfmRuntimeIgnoreToSeenFrame{ 0 };
            std::uint64_t wfmRuntimeInjectedTotal{ 0 };
            std::uint64_t wfmRuntimeIgnoreFromTotal{ 0 };
            std::uint64_t wfmRuntimeIgnoreToTotal{ 0 };

            // R15.8 creature MotionDB authority lifecycle. Strong generated DB root,
            // transition-local provenance, or a committed DB TX may START a pending
            // session. After ACQUIRED, matching PMD/Generate/root evidence refreshes
            // continuity; short callback gaps are tolerated until explicit release or
            // bounded timeout. Generic DB observation is never authority.
            bool creatureDbSessionActive{ false };
            float creatureDbSessionZeroRootSeconds{ 0.0f };
            std::uint64_t creatureDbSessionStarts{ 0 };
            std::uint64_t creatureDbSessionStops{ 0 };

            // R15.8 PMD is a continuity witness only AFTER strong DB ownership
            // acquisition. It can never start ownership. The acquired key set binds
            // PMD continuity to the generators that actually participated in the
            // acquired DB root lane instead of accepting arbitrary DB leaves.
            std::uint64_t creatureDbPmdSourceFrame{ 0 };
            std::string creatureDbPmdSourceKey;
            std::uint64_t creatureDbPmdSourceHits{ 0 };
            std::uint64_t creatureDbPmdHeartbeatLogs{ 0 };
            std::uint64_t creatureDbPmdContinuityFrame{ 0 };
            std::uint64_t creatureDbGenerateContinuityFrame{ 0 };
            std::uint64_t creatureDbContinuityHolds{ 0 };
            std::uint64_t creatureDbContinuityMisses{ 0 };
            std::uint64_t creatureDbExplicitReleases{ 0 };
            std::array<std::string, 8> previousDbRootKeys{};
            std::size_t previousDbRootKeyCount{ 0 };

            // R40 authored-turn progress proof. RotationClip observation/root validity is
            // not sufficient: the authored clip must have produced a meaningful yaw
            // delta in the completed Generate frame. This prevents a TurnLoop frozen at
            // nearly constant local time from monopolizing capsule yaw just because a
            // new DB entry is repeatedly pending.
            std::array<std::string, 8> previousDbYawProgressKeys{};
            std::size_t previousDbYawProgressKeyCount{ 0 };
            std::array<std::string, 8> creatureDbOwnershipKeys{};
            std::size_t creatureDbOwnershipKeyCount{ 0 };

            // DB ownership is certified by one of two proof paths:
            // 1) transition-local INPLACE -> ROOT provenance writes IGNORE_FROM, or
            // 2) stable DB composition with no transition in the previous/current
            //    evaluation. In either case the movement gate must then observe
            // non-zero authored DB WFM translation downstream.
            bool creatureDbOwnershipEstablished{ false };
            bool creatureDbEntryPending{ false };
            float creatureDbEntryAgeSeconds{ 0.0f };
            std::uint64_t creatureDbEntryStartFrame{ 0 };
            std::uint64_t creatureDbEntryPolicyWrites{ 0 };
            std::uint64_t creatureDbEntryConfirms{ 0 };
            std::uint64_t creatureDbEntryTimeouts{ 0 };
            std::uint32_t creatureDbEntryAgeFrames{ 0 };

            // R15.6 transition-local root provenance. Runtime WFM flags are no
            // longer armed actor-globally. A transition is eligible only when its
            // own generated children prove which side currently carries MotionDB
            // WFM. Skyrim's hkbBlendingTransitionEffect child ABI is child0=TO,
            // child1=FROM; this mapping is cross-checked against any runtime WFM
            // anchors that become available and is rejected on disagreement.
            std::uint64_t creatureDbProvenanceSeenFrame{ 0 };
            std::uint64_t creatureDbEntryProvenanceFrame{ 0 };
            std::uint64_t creatureDbProvenancePolicyWrites{ 0 };
            std::uint64_t creatureDbProvenanceRootToRoot{ 0 };
            std::uint64_t creatureDbProvenanceAmbiguous{ 0 };

            // Legacy bootstrap counters retained for R15.6 session diagnostics. A DB-backed clip observed during
            // frame N may request bAnimationDriven=true for frame N+1. This is
            // transport setup only; it never opens XY and PMD evaluation can never
            // certify root ownership.
            std::uint64_t creatureDbObservedFrame{ 0 };
            std::string creatureDbObservedKey;

            // R28 safe clip-carrier witness set. The already-proven clip Update path
            // only records DB keys; profile classification happens later on the normal
            // actor/V13 path. No new ProcessMotionData/Havok ABI hook is introduced.
            std::array<std::string, 16> creatureDbObservedKeys{};
            std::size_t creatureDbObservedKeyCount{ 0 };

            bool creatureDrivenLatched{ false };
            bool creatureOriginalDriven{ false };
            std::uint64_t creatureBootstrapDrivenFrame{ 0 };
            std::uint64_t creatureArmCount{ 0 };
            std::uint64_t creatureHoldCount{ 0 };
            std::uint64_t creatureRestoreCount{ 0 };
            std::uint64_t creatureArmFailCount{ 0 };
            std::string creatureLastArmLoggedKey;

            // R14 runtime worldFromModel policy. Current/previous root-capable
            // leaves are captured from the live graph; no animation family/name
            // whitelist is used to classify motion.
            std::array<RootMotionWitness, 32> rootWitnesses{};
            std::size_t rootWitnessCount{ 0 };
            std::uint64_t wfmTransitionCallsFrame{ 0 };
            std::uint64_t wfmTransitionChangesFrame{ 0 };
            std::uint64_t wfmTransitionCallsTotal{ 0 };
            std::uint64_t wfmTransitionChangesTotal{ 0 };
            std::uint64_t wfmOutputClassifiedTotal{ 0 };
            std::uint64_t wfmOutputUnknownTotal{ 0 };
            std::uint64_t wfmRootRootTotal{ 0 };
            std::uint64_t wfmInplaceRootTotal{ 0 };
            std::uint64_t wfmRootInplaceTotal{ 0 };
            std::uint64_t wfmInplaceInplaceTotal{ 0 };
            // Exact runtime root-output evidence. Set by hkbClipGenerator::Generate
            // and transition outputs only after TRACK_EXTRACTED_MOTION is valid.
            std::uint64_t wfmRootOutputSeenFrame{ 0 };
            // Raw serialized WFM tags observed on live transitions. These are
            // Native serialized tag context only; runtime-injected tags use separate markers.
            std::uint64_t wfmIgnoreFromSeenFrame{ 0 };
            std::uint64_t wfmIgnoreToSeenFrame{ 0 };
            std::uint64_t nativeClipGenerateCallsTotal{ 0 };
            std::uint64_t nativeClipRootOutputsTotal{ 0 };
            std::uint64_t nativeClipRootLogs{ 0 };
            RE::NiPoint3 rootDebugPosition{};
            float rootDebugYaw{ 0.0f };
            bool rootDebugSnapshotValid{ false };
            bool rootDebugNativeDriven{ false };
            std::uint64_t rootDebugTranslationWrites{ 0 };
            std::uint64_t rootDebugRotationWrites{ 0 };
            std::uint64_t unexplainedPhysicalFrames{ 0 };
            float unexplainedPhysicalMaxXY{ 0.0f };

            // R1-77 universal MotionDB diagnostics. No animation-name/family
            // assumptions: any DB-backed creature clip receives the same trace.
            std::unordered_set<std::string> motionDiagObservedSignatures;
            std::unordered_set<std::string> motionDiagPmdSignatures;
            std::unordered_set<std::string> motionDiagTxSignatures;
            std::unordered_set<std::string> motionDiagRotSignatures;

            // Player frame-scoped bAnimationDriven write. In phase 2 this is no
            // longer armed by clip presence; only an explicit committed locomotion
            // transaction may request it. The previous graph value is restored on
            // the following PlayerCharacter::Update.
            bool playerDrivenLatched{ false };
            bool playerOriginalDriven{ false };
            std::uint64_t playerDatabaseClipFrame{ 0 };
            std::uint64_t playerArmCount{ 0 };
            std::uint64_t playerRestoreCount{ 0 };
            std::uint64_t playerLastArmLoggedTransaction{ 0 };
            std::uint64_t playerClipWitnesses{ 0 };
            bool playerClipAnnounced{ false };
            bool playerFrameAnnounced{ false };
            bool playerOverrideAnnounced{ false };
            bool playerOwnershipAnnounced{ false };
            std::uint64_t playerFrameCount{ 0 };
            std::uint64_t playerClipCallbacks{ 0 };
            std::uint64_t playerClipDbMisses{ 0 };
            std::uint64_t playerClipCallbackLogs{ 0 };

            // Preserved R1-55 audit storage. Runtime cycle logging is disabled in
            // R1-57 so phase-2 action/AI transaction logs stay readable.
            std::array<std::string, 8> playerFrameDbKeys{};
            std::size_t playerFrameDbKeyCount{ 0 };
            RE::NiPoint3 playerPreviousPosition{};
            bool playerPreviousPositionValid{ false };
            float playerPhysicalPathTotalXY{ 0.0f };
            std::uint64_t playerSingleKeyFrames{ 0 };
            std::uint64_t playerAmbiguousFrames{ 0 };
            std::unordered_map<std::string, PlayerCycleAudit> playerCycleAudit;

            std::unordered_map<std::string, MotionPairState> pairs;
        };

        struct InvocationCache
        {
            RE::Character* character{ nullptr };
            std::string profile;
            const char* clipName{ nullptr };
            std::uint64_t frameSerial{ 0 };
            std::string animationName;
            bool mapped{ false };
            bool t1Valid{ false };
            bool t2Valid{ false };
            float t1Time{ 0.0f };
            float t2Time{ 0.0f };
            AnimationMotionDatabase::CumulativeSample t1{};
            AnimationMotionDatabase::CumulativeSample t2{};
        };

        using TranslationFn = void (*)(void*, float, RE::NiPoint3&);
        using RotationFn = void (*)(void*, float, RE::NiQuaternion&);

        bool g_installed{ false };
        std::unordered_map<ClipKey, ClipInfo, ClipKeyHash> g_clipInfo;
        std::unordered_map<RE::FormID, NativeActorState> g_actorState;
        struct NativeRootExitWitnessState
        {
            bool pending{ false };
            std::uintptr_t transitionPtr{ 0 };
            std::uint64_t lastSeenFrame{ 0 };
            std::uint64_t serial{ 0 };
        };
        std::unordered_map<RE::FormID, NativeRootExitWitnessState> g_nativeRootExitWitnesses;

        // R46 second structural witness: an actual WFM transition from the parent
        // LocomotionBehavior to StandingBehavior. The producer collapses short repeated
        // transition pulses into one episode. The consumer will not release it while the
        // transition is still fresh and expires it quickly if current runtime evidence no
        // longer supports reentry. This is structural provenance, not a carrier-loss watchdog.
        struct LocomotionStandingWitnessState
        {
            bool pending{ false };
            std::uintptr_t transitionPtr{ 0 };
            std::uint64_t firstSeenFrame{ 0 };
            std::uint64_t lastSeenFrame{ 0 };
            std::uint64_t serial{ 0 };
        };
        std::unordered_map<RE::FormID, LocomotionStandingWitnessState> g_locomotionStandingWitnesses;
        constexpr std::uint64_t kLocomotionStandingEpisodeGapFrames = 4;
        constexpr std::uint64_t kLocomotionStandingWitnessMinAgeFrames = 5;
        constexpr std::uint64_t kLocomotionStandingWitnessMaxAgeFrames = 18;
        std::mutex g_mutex;
        std::mutex g_phaseSyncConfigMutex;
        std::unordered_map<std::string, PhaseSyncConfig> g_phaseSyncConfigs;
        thread_local InvocationCache g_invocationCache{};
        constexpr std::size_t kEvaluationStackCapacity = 16;
        thread_local std::array<RE::Character*, kEvaluationStackCapacity> g_evaluationStack{};
        thread_local std::size_t g_evaluationDepth{ 0 };

        enum class MotionDiagKind
        {
            kObserved,
            kPmd,
            kTranslation,
            kRotation
        };

        [[nodiscard]] bool MarkMotionDiagnosticOnce(
            RE::FormID formID,
            MotionDiagKind kind,
            std::string signature)
        {
            std::scoped_lock lock{ g_mutex };
            const auto it = g_actorState.find(formID);
            if (it == g_actorState.end()) {
                return false;
            }
            auto& state = it->second;
            switch (kind) {
            case MotionDiagKind::kObserved:
                return state.motionDiagObservedSignatures.insert(std::move(signature)).second;
            case MotionDiagKind::kPmd:
                return state.motionDiagPmdSignatures.insert(std::move(signature)).second;
            case MotionDiagKind::kTranslation:
                return state.motionDiagTxSignatures.insert(std::move(signature)).second;
            case MotionDiagKind::kRotation:
                return state.motionDiagRotSignatures.insert(std::move(signature)).second;
            }
            return false;
        }

        [[nodiscard]] bool FindClipInfoForActorStateLocked(
            const NativeActorState& state,
            const std::string& clipName,
            ClipInfo& out);

        [[nodiscard]] float ClipAgeSeconds(const ClipInfo& info) noexcept
        {
            if (info.seen == Clock::time_point{}) {
                return std::numeric_limits<float>::infinity();
            }
            return std::chrono::duration<float>(Clock::now() - info.seen).count();
        }

        // Player remains on its already-proven frame transaction. For the creature test
        // actor, an exact (hkbCharacter*, PMD node name) -> MotionDB animation mapping
        // remains authoritative until that exact graph/node mapping is overwritten or
        // the actor graph identity changes. Do not expire a valid mapping by wall time:
        // sparse clip callbacks are normal and previously created root-motion holes.
        // This policy is animation-type agnostic and is the intended universal model.
        [[nodiscard]] bool IsClipMappingValidForInjection(
            const RE::Character* character,
            const NativeActorState& actorState,
            const ClipInfo& info) noexcept
        {
            if (!info.databaseBacked) {
                return false;
            }
            if (character && character->IsPlayerRef()) {
                return info.profile == "Player" && info.seenFrameSerial != 0 &&
                    info.seenFrameSerial <= actorState.frameSerial &&
                    actorState.frameSerial - info.seenFrameSerial <= 1;
            }
            return !actorState.profile.empty() && info.profile == actorState.profile &&
                info.seen != Clock::time_point{} && info.seenFrameSerial != 0;
        }

        // Universal profile-scoped diagnostic mirror of the real T1 lookup. No animation
        // family/type whitelist exists: the PMD clip either resolves to a fresh
        // MotionDB-backed animation or it does not. Negative native motionTime is
        // intentionally diagnostic/samplable because MotionDB owns timeline clamp.
        void ProbeMotionPmdT1(
            RE::Character* character,
            float motionTime,
            std::string_view clipName)
        {
            if (!character || character->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(character) || clipName.empty()) {
                return;
            }

            ClipInfo info{};
            std::uint64_t actorFrameSerial = 0;
            float ageSeconds = std::numeric_limits<float>::infinity();
            std::string_view result = "NO_ACTOR_STATE";
            bool hasInfo = false;
            {
                std::scoped_lock lock{ g_mutex };
                const auto actorIt = g_actorState.find(character->GetFormID());
                if (actorIt != g_actorState.end() && actorIt->second.hkbCharacter &&
                    actorIt->second.frameSerial != 0) {
                    const auto& actorState = actorIt->second;
                    actorFrameSerial = actorState.frameSerial;
                    if (!FindClipInfoForActorStateLocked(actorState, std::string{ clipName }, info)) {
                        result = "NO_CLIPINFO_FOR_PMD_NAME";
                    } else {
                        hasInfo = true;
                        ageSeconds = ClipAgeSeconds(info);
                        if (!info.databaseBacked) {
                            result = "CLIPINFO_NOT_DB_BACKED";
                        } else if (info.seen == Clock::time_point{} || info.seenFrameSerial == 0) {
                            result = "DB_CLIP_NEVER_SEEN";
                        } else {
                            result = "ELIGIBLE";
                        }
                    }
                }
            }

            if (result == "ELIGIBLE") {
                const auto sample = AnimationMotionDatabase::SampleCumulativeSeconds(info.profile, info.animationName, motionTime);
                result = sample.valid ? "READY_SAMPLE" : "SAMPLE_INVALID";
            }

            const std::string key = hasInfo ? info.databaseKey : std::string{};
            const std::string anim = hasInfo ? info.animationName : std::string{};
            std::string signature;
            signature.reserve(clipName.size() + key.size() + anim.size() + result.size() + 4);
            signature.append(clipName);
            signature.push_back('|');
            signature.append(result);
            signature.push_back('|');
            signature.append(key);
            signature.push_back('|');
            signature.append(anim);
            if (!MarkMotionDiagnosticOnce(character->GetFormID(), MotionDiagKind::kPmd, std::move(signature))) {
                return;
            }

            SKSE::log::info(
                R"([MotionDB][PMD_T1] form={:08X} frame={} clip="{}" motionTime={:.6f} mapped={} dbBacked={} key="{}" anim="{}" seenFrame={} age={:.6f}s mappingPolicy=PERSISTENT_EXACT_HKB_NODE result={})",
                character->GetFormID(), actorFrameSerial, clipName, motionTime,
                hasInfo, hasInfo ? info.databaseBacked : false, key, anim,
                hasInfo ? info.seenFrameSerial : 0, ageSeconds, result);
        }

        void AddPlayerFrameDbKeyLocked(NativeActorState& state, std::string_view key)
        {
            if (key.empty()) {
                return;
            }
            for (std::size_t i = 0; i < state.playerFrameDbKeyCount; ++i) {
                if (state.playerFrameDbKeys[i] == key) {
                    return;
                }
            }
            if (state.playerFrameDbKeyCount < state.playerFrameDbKeys.size()) {
                state.playerFrameDbKeys[state.playerFrameDbKeyCount++] = std::string{ key };
            } else {
                // Saturating at capacity is itself ambiguous; preserve that fact.
                state.playerFrameDbKeyCount = state.playerFrameDbKeys.size();
            }
        }

        [[nodiscard]] RE::Character* CurrentEvaluationActor() noexcept
        {
            if (g_evaluationDepth == 0) {
                return nullptr;
            }
            return g_evaluationStack[g_evaluationDepth - 1];
        }

        [[nodiscard]] bool IsUniversalCreatureActor(RE::Actor* actor) noexcept
        {
            if (!actor || actor->IsPlayerRef()) {
                return false;
            }
            const auto* race = actor->GetRace();
            // B3 hard safety boundary: playable races are humanoid/player-class actors
            // and are never eligible for universal Incessus authority. This excludes
            // bandits and the standard playable races without maintaining EditorID lists.
            return race && !race->GetPlayable();
        }

        [[nodiscard]] bool IsExternalMotionActor(RE::Actor* actor) noexcept
        {
            // Legacy profile/MotionDB lane remains profile-scoped, but B3 applies the
            // same hard creature-only boundary used by the universal native lane.
            return IsUniversalCreatureActor(actor) && MotionProfileDatabase::SupportsActor(actor);
        }

        // R69.7: creature-only rename of the old Player-root eligibility helper.
        // The predicate is intentionally byte-for-byte equivalent in semantics to the
        // R67 gate used by the proven DB ownership/session lifecycle.
        [[nodiscard]] bool IsRuntimeEligibleForExternalRoot(RE::Actor* actor) noexcept
        {
            return actor && actor->GetParentCell() && actor->Is3DLoaded() &&
                !actor->IsDisabled() && !actor->IsDead() &&
                !actor->IsInRagdollState() && !actor->IsInKillMove() &&
                !actor->IsInMidair() && !actor->IsInWater() &&
                !actor->IsBeingRidden();
        }

        [[nodiscard]] bool IsHardInvalidForNativeLocomotion(RE::Actor* actor) noexcept
        {
            // Once native root has been acquired, transient IsInMidair/IsInWater can
            // flicker during controller/root reconciliation. They must not tear down a
            // proven locomotion session in one frame. Hard safety states still release
            // immediately; ordinary continuity handles transient carrier gaps.
            return !actor || !actor->GetParentCell() || !actor->Is3DLoaded() ||
                actor->IsDisabled() || actor->IsDead() || actor->IsInRagdollState() ||
                actor->IsInKillMove() || actor->IsBeingRidden();
        }

        [[nodiscard]] bool CanApplyExternalMotion(RE::Character* character) noexcept
        {
            // Creature authority remains exactly profile-driven. No Player transaction layer exists in Incessus.
            return IsExternalMotionActor(character);
        }

        [[nodiscard]] char LowerASCII(char value) noexcept
        {
            if (value >= 'A' && value <= 'Z') {
                return static_cast<char>(value - 'A' + 'a');
            }
            return value;
        }

        [[nodiscard]] bool ContainsInsensitive(std::string_view text, std::string_view token) noexcept
        {
            if (token.empty() || token.size() > text.size()) {
                return false;
            }
            for (std::size_t i = 0; i + token.size() <= text.size(); ++i) {
                bool match = true;
                for (std::size_t j = 0; j < token.size(); ++j) {
                    if (LowerASCII(text[i + j]) != LowerASCII(token[j])) {
                        match = false;
                        break;
                    }
                }
                if (match) {
                    return true;
                }
            }
            return false;
        }


        [[nodiscard]] std::string TrimAscii(std::string value)
        {
            const auto isSpace = [](unsigned char c) {
                return c == ' ' || c == '\t' || c == '\r' || c == '\n';
            };
            while (!value.empty() && isSpace(static_cast<unsigned char>(value.front()))) {
                value.erase(value.begin());
            }
            while (!value.empty() && isSpace(static_cast<unsigned char>(value.back()))) {
                value.pop_back();
            }
            return value;
        }

        [[nodiscard]] std::string LowerAsciiCopy(std::string value)
        {
            for (auto& c : value) {
                c = LowerASCII(c);
            }
            return value;
        }

        [[nodiscard]] std::string CanonicalMotionName(std::string_view raw)
        {
            std::string value{ raw };
            const auto slash = value.find_last_of("\\/");
            if (slash != std::string::npos) value.erase(0, slash + 1);
            const auto dot = value.find_last_of('.');
            if (dot != std::string::npos) value.erase(dot);
            std::string out;
            out.reserve(value.size());
            for (const unsigned char c : value) {
                if ((c >= 'A' && c <= 'Z') || (c >= 'a' && c <= 'z')) {
                    out.push_back(LowerASCII(static_cast<char>(c)));
                }
            }
            return out;
        }

        [[nodiscard]] bool ParsePhaseSyncBool(std::string_view value, bool fallback) noexcept
        {
            std::string lowered{ value };
            for (auto& c : lowered) {
                c = LowerASCII(c);
            }
            if (lowered == "true" || lowered == "1" || lowered == "yes" || lowered == "on") {
                return true;
            }
            if (lowered == "false" || lowered == "0" || lowered == "no" || lowered == "off") {
                return false;
            }
            return fallback;
        }

        [[nodiscard]] PhaseSyncConfig LoadPhaseSyncConfig(std::string_view profile)
        {
            PhaseSyncConfig config{};
            config.loaded = true;
            if (profile.empty()) {
                return config;
            }

            const std::string path =
                std::string{ "Data/MotionDB/" } + std::string{ profile } + "/GaitRouter.ini";
            std::ifstream input{ path };
            if (!input.is_open()) {
                return config;
            }

            bool inPhaseSync = false;
            std::string line;
            while (std::getline(input, line)) {
                const auto semicolon = line.find(';');
                if (semicolon != std::string::npos) {
                    line.erase(semicolon);
                }
                line = TrimAscii(std::move(line));
                if (line.empty()) {
                    continue;
                }
                if (line.front() == '[' && line.back() == ']') {
                    const auto section =
                        LowerAsciiCopy(TrimAscii(line.substr(1, line.size() - 2)));
                    inPhaseSync = section == "phasesync";
                    continue;
                }
                if (!inPhaseSync) {
                    continue;
                }

                const auto equal = line.find('=');
                if (equal == std::string::npos) {
                    continue;
                }
                auto key = LowerAsciiCopy(TrimAscii(line.substr(0, equal)));
                auto value = TrimAscii(line.substr(equal + 1));

                if (key == "enabled") {
                    config.enabled = ParsePhaseSyncBool(value, config.enabled);
                    continue;
                }
                if (key == "stallframes") {
                    try {
                        config.stallFrames =
                            static_cast<std::uint32_t>(std::clamp(std::stoi(value), 1, 30));
                    } catch (...) {}
                    continue;
                }
                if (key == "stalldeltaseconds") {
                    try {
                        config.stallDeltaSeconds =
                            std::clamp(std::stof(value), 0.00001f, 0.050f);
                    } catch (...) {}
                    continue;
                }
                if (key == "snapthresholdcycles") {
                    try {
                        config.snapThresholdCycles =
                            std::clamp(std::stof(value), 0.01f, 0.49f);
                    } catch (...) {}
                    continue;
                }
                if (key == "releasethresholdcycles") {
                    try {
                        config.releaseThresholdCycles =
                            std::clamp(std::stof(value), 0.005f, 0.25f);
                    } catch (...) {}
                    continue;
                }
                if (key == "releasehealthyframes") {
                    try {
                        config.releaseHealthyFrames =
                            static_cast<std::uint32_t>(std::clamp(std::stoi(value), 1, 30));
                    } catch (...) {}
                    continue;
                }
                if (key.rfind("group", 0) == 0 && config.groupCount < config.groups.size()) {
                    PhaseSyncGroupConfig group{};
                    std::stringstream stream{ value };
                    std::string token;
                    while (std::getline(stream, token, ',') &&
                           group.keyCount < group.keys.size()) {
                        token = LowerAsciiCopy(TrimAscii(std::move(token)));
                        if (!token.empty()) {
                            group.keys[group.keyCount++] = std::move(token);
                        }
                    }
                    if (group.keyCount >= 2) {
                        config.groups[config.groupCount++] = std::move(group);
                    }
                }
            }
            return config;
        }

        [[nodiscard]] PhaseSyncConfig GetPhaseSyncConfig(std::string_view profile)
        {
            const std::string key = LowerAsciiCopy(std::string{ profile });
            {
                std::scoped_lock lock{ g_phaseSyncConfigMutex };
                const auto it = g_phaseSyncConfigs.find(key);
                if (it != g_phaseSyncConfigs.end()) {
                    return it->second;
                }
            }

            auto loaded = LoadPhaseSyncConfig(key);
            {
                std::scoped_lock lock{ g_phaseSyncConfigMutex };
                g_phaseSyncConfigs[key] = loaded;
            }
            return loaded;
        }

        [[nodiscard]] float CircularPhaseDistance(float a, float b) noexcept
        {
            float delta = std::fabs(a - b);
            if (delta > 0.5f) {
                delta = 1.0f - delta;
            }
            return std::clamp(delta, 0.0f, 0.5f);
        }

        [[nodiscard]] float ForwardPhaseDelta(float current, float previous) noexcept
        {
            float delta = current - previous;
            if (delta < -0.5f) {
                delta += 1.0f;
            }
            return delta;
        }

        [[nodiscard]] bool IsRelevantClip(std::string_view node, std::string_view anim) noexcept
        {
            const auto relevant = [&](std::string_view token) {
                return ContainsInsensitive(node, token) || ContainsInsensitive(anim, token);
            };
            return relevant("walk") || relevant("trot") || relevant("run") ||
                   relevant("turn") || relevant("pivot") || relevant("backward");
        }

        [[nodiscard]] const RE::hkbCharacter* ResolveHkbCharacter(RE::Character* character)
        {
            if (!character) {
                return nullptr;
            }
            RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
            if (!character->GetAnimationGraphManager(manager) || !manager) {
                return nullptr;
            }
            const auto activeGraph = manager->GetRuntimeData().activeGraph;
            if (activeGraph >= manager->graphs.size()) {
                return nullptr;
            }
            auto* graph = manager->graphs[activeGraph].get();
            return graph ? std::addressof(graph->characterInstance) : nullptr;
        }

        [[nodiscard]] std::size_t CollectHkbCharacters(
            RE::Character* character,
            std::array<const RE::hkbCharacter*, 8>& out)
        {
            out.fill(nullptr);
            if (!character) {
                return 0;
            }
            RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
            if (!character->GetAnimationGraphManager(manager) || !manager) {
                return 0;
            }

            std::size_t count = 0;
            for (decltype(manager->graphs.size()) i = 0; i < manager->graphs.size() && count < out.size(); ++i) {
                auto* graph = manager->graphs[i].get();
                if (!graph) {
                    continue;
                }
                const auto* candidate = std::addressof(graph->characterInstance);
                bool duplicate = false;
                for (std::size_t j = 0; j < count; ++j) {
                    if (out[j] == candidate) {
                        duplicate = true;
                        break;
                    }
                }
                if (!duplicate) {
                    out[count++] = candidate;
                }
            }
            return count;
        }

        [[nodiscard]] bool StateOwnsHkbCharacter(
            const NativeActorState& state,
            const RE::hkbCharacter* character) noexcept
        {
            if (!character) {
                return false;
            }
            if (state.hkbCharacter == character) {
                return true;
            }
            for (std::size_t i = 0; i < state.hkbCharacterCount && i < state.hkbCharacters.size(); ++i) {
                if (state.hkbCharacters[i] == character) {
                    return true;
                }
            }
            return false;
        }

        constexpr std::uint16_t kWfmIgnoreFrom = 0x0001;
        constexpr std::uint16_t kWfmSync = 0x0002;
        constexpr std::uint16_t kWfmIgnoreTo = 0x0004;
        constexpr std::uint16_t kWfmIgnoreToRotation = 0x0008;
        constexpr std::uint16_t kWfmKnownMask =
            kWfmIgnoreFrom | kWfmSync | kWfmIgnoreTo | kWfmIgnoreToRotation;

        constexpr std::uint16_t kWfmTranslationPolicyMask =
            kWfmIgnoreFrom | kWfmIgnoreTo;
        constexpr float kRootContributionEpsilon = 1.0e-4f;
        // 0.05 degrees of raw authored yaw per completed clip frame. Normal TurnLoop
        // progress is orders of magnitude larger; sub-threshold wobble near a frozen
        // local time is deliberately not accepted as authored rotation authority.
        constexpr float kAuthoredRotationProgressEpsilonRadians = 0.000872664626f;
        constexpr float kDbSessionReleaseSeconds = 0.25f;
        constexpr float kDbOwnershipEntryTimeoutSeconds = 0.50f;
        constexpr std::uint32_t kDbOwnershipEntryTimeoutFrames = 12;
        constexpr std::uint64_t kRuntimePolicyReuseGapFrames = 3;

        [[nodiscard]] bool HasVanillaTranslationList(
            const NativeMotionContainerSnapshot& container) noexcept
        {
            const bool aligned = (container.translationDataPtr & 1u) == 0;
            return container.readable &&
                container.translationSegCount > static_cast<std::uint32_t>(aligned);
        }

        [[nodiscard]] bool HasVanillaRotationList(
            const NativeMotionContainerSnapshot& container) noexcept
        {
            const bool aligned = (container.rotationDataPtr & 1u) == 0;
            return container.readable &&
                container.rotationSegCount > static_cast<std::uint32_t>(aligned);
        }

        [[nodiscard]] bool IsUniversalNativeMotionActor(RE::Actor* actor) noexcept
        {
            return IsUniversalCreatureActor(actor);
        }

        [[nodiscard]] bool HasUniversalActionBypass(RE::Actor* actor) noexcept
        {
            if (!IsUniversalCreatureActor(actor)) {
                return true;
            }
            constexpr std::array<std::string_view, 3> kActionVariables{
                "IsAttacking", "IsStaggering", "IsRecoiling"
            };
            for (const auto variable : kActionVariables) {
                bool value = false;
                if (actor->GetGraphVariableBool(variable.data(), value) && value) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool IsCertifiedNativeLocomotionCarrierName(std::string_view clipName) noexcept
        {
            if (clipName.empty()) return false;

            // B8 carrier certification is intentionally conservative. Native T1->T2 proves
            // authored translation, but D1/B7 proved that attacks, staggers and special idles
            // also carry translation. Those finite-action carriers must never bootstrap the
            // persistent locomotion authority lane.
            constexpr std::array<std::string_view, 19> kReject{
                "attack", "specialidle", "idle", "stagger", "recoil", "turn", "pivot",
                "bash", "block", "shout", "cast", "spell", "kill", "death", "dodge",
                "jump", "landing", "fall", "getup"
            };
            for (const auto token : kReject) {
                if (ContainsInsensitive(clipName, token)) return false;
            }

            // Locomotion clips across the supplied creature graphs use directional/gait
            // semantics in their generator names (e.g. MTForwardSlow, H2HLeft,
            // WalkSlowForward, Left_Run). The positive token is accepted only after all
            // finite-action tokens above were excluded and a real native T1->T2 delta exists.
            constexpr std::array<std::string_view, 10> kLocomotion{
                "walk", "run", "trot", "forward", "backward", "left", "right", "strafe",
                "locomotion", "swim"
            };
            for (const auto token : kLocomotion) {
                if (ContainsInsensitive(clipName, token)) return true;
            }
            return false;
        }

        [[nodiscard]] bool IsCertifiedNativeLocomotionCarrier(
            RE::Actor* actor,
            std::string_view clipName) noexcept
        {
            return IsUniversalCreatureActor(actor) &&
                !HasUniversalActionBypass(actor) &&
                IsCertifiedNativeLocomotionCarrierName(clipName);
        }

        [[nodiscard]] bool HasDirectionalLocomotionIntent(RE::Actor* actor) noexcept
        {
            if (!actor || actor->IsPlayerRef()) {
                return false;
            }
            if (auto* actorState = actor->AsActorState()) {
                return actorState->actorState1.movingForward != 0 ||
                    actorState->actorState1.movingBack != 0 ||
                    actorState->actorState1.movingLeft != 0 ||
                    actorState->actorState1.movingRight != 0;
            }
            return false;
        }

        [[nodiscard]] bool HasNativeLocomotionDemand(RE::Actor* actor) noexcept
        {
            if (!actor || actor->IsPlayerRef()) return false;
            // Preserve the already-proven profile lane exactly. Profileless creatures
            // use the B6 AI/path MovementDemand sampled before Character::Update.
            if (MotionProfileDatabase::SupportsActor(actor)) {
                return HasDirectionalLocomotionIntent(actor);
            }
            return SpeedIntentTranslator::HasUniversalMovementDemand(actor);
        }

        [[nodiscard]] bool HasNativeLocomotionContract(RE::Actor* actor) noexcept
        {
            if (!actor || actor->IsPlayerRef()) return false;
            if (MotionProfileDatabase::SupportsActor(actor)) return true;
            return SpeedIntentTranslator::HasResolvedUniversalGraphContract(actor);
        }

        void ObserveNativeMotionPair(
            Site site,
            const void* motionData,
            float motionTime,
            const RE::BSFixedString* clipNameValue,
            RE::Character* character,
            const RE::NiPoint3* translation,
            const RE::NiQuaternion* rotation)
        {
            if (!IsUniversalNativeMotionActor(character) || !motionData) {
                return;
            }

            const auto container = ReadNativeMotionContainer(motionData);
            const auto containerKey = reinterpret_cast<std::uintptr_t>(motionData);
            const std::string clipName = FixedString(clipNameValue);

            if ((site == Site::kTranslation1 || site == Site::kTranslation2) &&
                HasVanillaTranslationList(container)) {
                {
                    std::scoped_lock lock{ g_mutex };
                    auto& state = g_actorState[character->GetFormID()];
                    if (state.frameSerial != 0) {
                        state.nativeTranslationListSeenFrame = state.frameSerial;
                        if (!clipName.empty()) {
                            state.nativeTranslationListSeenClip = clipName;
                        }
                    }
                }
            }

            if ((site == Site::kTranslation1 || site == Site::kTranslation2) &&
                translation && HasVanillaTranslationList(container) &&
                std::isfinite(translation->x) && std::isfinite(translation->y) &&
                std::isfinite(translation->z) && std::isfinite(motionTime)) {
                std::scoped_lock lock{ g_mutex };
                auto& state = g_actorState[character->GetFormID()];
                if (state.frameSerial == 0) {
                    return;
                }
                auto& pair = state.nativeTranslationPairs[containerKey];
                if (site == Site::kTranslation1) {
                    pair.value = *translation;
                    pair.motionTime = motionTime;
                    pair.frame = state.frameSerial;
                    pair.clipName = clipName;
                    pair.valid = true;
                    return;
                }

                if (!pair.valid || pair.frame != state.frameSerial) {
                    return;
                }
                const float dx = translation->x - pair.value.x;
                const float dy = translation->y - pair.value.y;
                const float planar = std::hypot(dx, dy);
                if (std::isfinite(planar) && planar > kRootContributionEpsilon) {
                    state.nativeMotionSeenFrame = state.frameSerial;
                    state.nativeMotionSeenClip = !clipName.empty() ? clipName : pair.clipName;
                    state.nativeMotionSeenTranslation = true;
                    state.nativeMotionSeenRotation = false;
                    state.nativeMotionSeenMagnitude = planar;
                    state.nativeTranslationDeltaSeenFrame = state.frameSerial;
                    state.nativeTranslationDeltaSeenClip = state.nativeMotionSeenClip;
                    state.nativeTranslationDeltaSeenMagnitude = planar;
                    const auto canonical = CanonicalMotionName(state.nativeMotionSeenClip);
                    if (!canonical.empty()) {
                        auto& sample = state.nativeExtractedDeltas[canonical];
                        sample.delta = { dx, dy, translation->z - pair.value.z };
                        sample.frame = state.frameSerial;
                        sample.clipName = state.nativeMotionSeenClip;
                        sample.valid = std::isfinite(sample.delta.x) &&
                            std::isfinite(sample.delta.y) && std::isfinite(sample.delta.z);
                    }
                    ++state.nativeMotionWitnesses;
                }
                pair.valid = false;
                return;
            }

            if ((site == Site::kRotation1 || site == Site::kRotation2) &&
                rotation && HasVanillaRotationList(container) &&
                std::isfinite(rotation->w) && std::isfinite(rotation->x) &&
                std::isfinite(rotation->y) && std::isfinite(rotation->z) &&
                std::isfinite(motionTime)) {
                std::scoped_lock lock{ g_mutex };
                auto& state = g_actorState[character->GetFormID()];
                if (state.frameSerial == 0) {
                    return;
                }
                auto& pair = state.nativeRotationPairs[containerKey];
                if (site == Site::kRotation1) {
                    pair.value = *rotation;
                    pair.motionTime = motionTime;
                    pair.frame = state.frameSerial;
                    pair.clipName = clipName;
                    pair.valid = true;
                    return;
                }

                if (!pair.valid || pair.frame != state.frameSerial) {
                    return;
                }
                float dyaw = QuaternionYaw(*rotation) - QuaternionYaw(pair.value);
                constexpr float pi = 3.14159265358979323846f;
                constexpr float twoPi = 2.0f * pi;
                while (dyaw > pi) dyaw -= twoPi;
                while (dyaw < -pi) dyaw += twoPi;
                const float magnitude = std::fabs(dyaw);
                if (std::isfinite(magnitude) &&
                    magnitude > kAuthoredRotationProgressEpsilonRadians) {
                    state.nativeMotionSeenFrame = state.frameSerial;
                    state.nativeMotionSeenClip = !clipName.empty() ? clipName : pair.clipName;
                    state.nativeMotionSeenTranslation = false;
                    state.nativeMotionSeenRotation = true;
                    state.nativeMotionSeenMagnitude = magnitude;
                    state.nativeRotationDeltaSeenFrame = state.frameSerial;
                    state.nativeRotationDeltaSeenClip = state.nativeMotionSeenClip;
                    state.nativeRotationDeltaSeenMagnitude = magnitude;
                    ++state.nativeMotionWitnesses;
                }
                pair.valid = false;
            }
        }

        [[nodiscard]] const char* RuntimeWfmPolicyName(RuntimeWfmPolicy policy) noexcept
        {
            switch (policy) {
            case RuntimeWfmPolicy::kIgnoreFrom:
                return "INPLACE_TO_DB_ROOT";
            case RuntimeWfmPolicy::kIgnoreTo:
                return "DB_ROOT_TO_INPLACE";
            case RuntimeWfmPolicy::kRootToRoot:
                return "DB_ROOT_TO_DB_ROOT";
            default:
                return "INPLACE_TO_INPLACE";
            }
        }

        template <std::size_t N>
        void AddUniqueDbKey(std::array<std::string, N>& keys, std::size_t& count, const std::string& key)
        {
            if (key.empty()) {
                return;
            }
            for (std::size_t i = 0; i < count && i < keys.size(); ++i) {
                if (keys[i] == key) {
                    return;
                }
            }
            if (count < keys.size()) {
                keys[count++] = key;
            }
        }

        [[nodiscard]] bool OwnershipContainsDbKeyLocked(
            const NativeActorState& state, const std::string& key)
        {
            if (key.empty()) {
                return false;
            }
            for (std::size_t i = 0; i < state.creatureDbOwnershipKeyCount &&
                 i < state.creatureDbOwnershipKeys.size(); ++i) {
                if (state.creatureDbOwnershipKeys[i] == key) {
                    return true;
                }
            }
            return false;
        }

        void CaptureCurrentDbRootKeysLocked(NativeActorState& state)
        {
            state.previousDbRootKeyCount = 0;
            for (auto& key : state.previousDbRootKeys) {
                key.clear();
            }
            state.previousDbYawProgressKeyCount = 0;
            for (auto& key : state.previousDbYawProgressKeys) {
                key.clear();
            }

            for (std::size_t i = 0; i < state.rootContributionCount &&
                 i < state.rootContributions.size(); ++i) {
                const auto& witness = state.rootContributions[i];
                if (!witness.databaseBacked ||
                    !(witness.poseWeight > kRootContributionEpsilon)) {
                    continue;
                }

                if (witness.deltaValid) {
                    AddUniqueDbKey(
                        state.previousDbRootKeys,
                        state.previousDbRootKeyCount,
                        witness.databaseKey);
                }

                // R40: authored rotation health is key-specific. Directional locomotion
                // yaw elsewhere in the blend cannot make a stalled TurnLoop healthy.
                if (witness.yawValid && std::isfinite(witness.deltaYaw) &&
                    std::fabs(witness.deltaYaw) >= kAuthoredRotationProgressEpsilonRadians) {
                    AddUniqueDbKey(
                        state.previousDbYawProgressKeys,
                        state.previousDbYawProgressKeyCount,
                        witness.databaseKey);
                }
            }
        }

        void ClearDbOwnershipKeysLocked(NativeActorState& state)
        {
            state.creatureDbOwnershipKeyCount = 0;
            for (auto& key : state.creatureDbOwnershipKeys) {
                key.clear();
            }
        }

        [[nodiscard]] RootContributionSummary ComputeRootContributionSummaryLocked(
            const NativeActorState& state)
        {
            RootContributionSummary summary{};
            float weightedX = 0.0f;
            float weightedY = 0.0f;
            float weightedZ = 0.0f;
            float translationWeight = 0.0f;
            float yawSin = 0.0f;
            float yawCos = 0.0f;
            bool anyDbYaw = false;

            for (std::size_t i = 0; i < state.rootContributionCount &&
                 i < state.rootContributions.size(); ++i) {
                const auto& witness = state.rootContributions[i];
                const float poseWeight = std::max(0.0f, witness.poseWeight);
                if (!(poseWeight > kRootContributionEpsilon)) {
                    continue;
                }

                ++summary.clips;
                summary.poseWeight += poseWeight;
                if (!witness.databaseBacked) {
                    // In-place/non-DB contributors represent zero synthetic yaw.
                    yawCos += poseWeight;
                    continue;
                }

                ++summary.dbClips;
                summary.dbRootWeight += poseWeight;
                if (witness.deltaValid) {
                    weightedX += poseWeight * witness.deltaX;
                    weightedY += poseWeight * witness.deltaY;
                    weightedZ += poseWeight * witness.deltaZ;
                    translationWeight += poseWeight;
                }
                if (witness.yawValid) {
                    yawSin += poseWeight * std::sin(witness.deltaYaw);
                    yawCos += poseWeight * std::cos(witness.deltaYaw);
                    anyDbYaw = true;
                } else {
                    yawCos += poseWeight;
                }
            }

            if (summary.poseWeight > kRootContributionEpsilon) {
                summary.dbRootShare = std::clamp(
                    summary.dbRootWeight / summary.poseWeight, 0.0f, 1.0f);
                summary.mixedX = weightedX / summary.poseWeight;
                summary.mixedY = weightedY / summary.poseWeight;
                summary.mixedZ = weightedZ / summary.poseWeight;
                summary.mixedTranslationValid = translationWeight > kRootContributionEpsilon;
                if (anyDbYaw && (std::fabs(yawSin) > kRootContributionEpsilon ||
                    std::fabs(yawCos) > kRootContributionEpsilon)) {
                    summary.mixedYaw = std::atan2(yawSin, yawCos);
                    summary.mixedYawValid = true;
                }
            }
            return summary;
        }

        [[nodiscard]] PhysicalCompositionProbe BuildPhysicalCompositionProbeLocked(
            const NativeActorState& state)
        {
            struct Group
            {
                std::string key;
                std::size_t aliases{ 0 };
                bool localTimeSeen{ false };
                float minLocalTime{ 0.0f };
                float maxLocalTime{ 0.0f };
                bool deltaSeen{ false };
                float minDx{ 0.0f };
                float maxDx{ 0.0f };
                float minDy{ 0.0f };
                float maxDy{ 0.0f };
                float minDz{ 0.0f };
                float maxDz{ 0.0f };
            };

            PhysicalCompositionProbe probe{};
            std::vector<Group> groups;
            groups.reserve(8);

            for (std::size_t i = 0; i < state.rootContributionCount &&
                 i < state.rootContributions.size(); ++i) {
                const auto& witness = state.rootContributions[i];
                const float poseWeight = std::max(0.0f, witness.poseWeight);
                if (!witness.databaseBacked || witness.databaseKey.empty() ||
                    !(poseWeight > kRootContributionEpsilon)) {
                    continue;
                }

                ++probe.dbLeaves;
                auto it = std::find_if(groups.begin(), groups.end(),
                    [&](const Group& group) { return group.key == witness.databaseKey; });
                if (it == groups.end()) {
                    Group group{};
                    group.key = witness.databaseKey;
                    groups.push_back(std::move(group));
                    it = std::prev(groups.end());
                }

                auto& group = *it;
                ++group.aliases;
                if (std::isfinite(witness.clipLocalTime)) {
                    if (!group.localTimeSeen) {
                        group.localTimeSeen = true;
                        group.minLocalTime = witness.clipLocalTime;
                        group.maxLocalTime = witness.clipLocalTime;
                    } else {
                        group.minLocalTime = std::min(group.minLocalTime, witness.clipLocalTime);
                        group.maxLocalTime = std::max(group.maxLocalTime, witness.clipLocalTime);
                    }
                }
                if (witness.deltaValid &&
                    std::isfinite(witness.deltaX) && std::isfinite(witness.deltaY) &&
                    std::isfinite(witness.deltaZ)) {
                    if (!group.deltaSeen) {
                        group.deltaSeen = true;
                        group.minDx = group.maxDx = witness.deltaX;
                        group.minDy = group.maxDy = witness.deltaY;
                        group.minDz = group.maxDz = witness.deltaZ;
                    } else {
                        group.minDx = std::min(group.minDx, witness.deltaX);
                        group.maxDx = std::max(group.maxDx, witness.deltaX);
                        group.minDy = std::min(group.minDy, witness.deltaY);
                        group.maxDy = std::max(group.maxDy, witness.deltaY);
                        group.minDz = std::min(group.minDz, witness.deltaZ);
                        group.maxDz = std::max(group.maxDz, witness.deltaZ);
                    }
                }
            }

            std::sort(groups.begin(), groups.end(),
                [](const Group& a, const Group& b) { return a.key < b.key; });

            probe.uniqueKeys = groups.size();
            std::ostringstream signature;
            std::ostringstream detail;
            for (std::size_t i = 0; i < groups.size(); ++i) {
                const auto& group = groups[i];
                probe.maxAliasesForOneKey = std::max(probe.maxAliasesForOneKey, group.aliases);

                const float localSpread = group.localTimeSeen
                    ? std::max(0.0f, group.maxLocalTime - group.minLocalTime)
                    : 0.0f;
                const float deltaSpread = group.deltaSeen
                    ? std::sqrt(
                        std::pow(group.maxDx - group.minDx, 2.0f) +
                        std::pow(group.maxDy - group.minDy, 2.0f) +
                        std::pow(group.maxDz - group.minDz, 2.0f))
                    : 0.0f;

                // The thresholds are deliberately diagnostic, not policy. A key with several
                // aliases at the same phase/delta is treated as one authored motion witness;
                // a measurable phase or frame-delta spread means the same HKX is being sampled
                // in genuinely different phases within this generated frame.
                const bool phaseSplit = group.aliases > 1 &&
                    (localSpread > 0.0025f || deltaSpread > 0.0025f);
                if (phaseSplit) {
                    ++probe.phaseSplitKeys;
                }

                if (i != 0) {
                    signature << '|';
                    detail << " | ";
                }
                signature << group.key << ':' << group.aliases << ':' << (phaseSplit ? 'S' : 'A');
                detail << group.key
                       << " aliases=" << group.aliases
                       << " localSpread=" << localSpread
                       << " deltaSpread=" << deltaSpread
                       << " phaseSplit=" << (phaseSplit ? "true" : "false");
            }
            probe.signature = signature.str();
            probe.detail = detail.str();
            return probe;
        }

        [[nodiscard]] RootContributionWitness BuildRootContributionWitness(
            const RE::hkbClipGenerator* clip,
            std::string nodeName,
            std::string animationName,
            std::string_view profile,
            float frameDeltaTime,
            const AnimationMotionDatabase::ClipMetrics& metrics)
        {
            RootContributionWitness witness{};
            if (!clip) {
                return witness;
            }

            witness.clip = clip;
            witness.nodeName = std::move(nodeName);
            witness.animationName = std::move(animationName);
            witness.databaseBacked = metrics.valid;
            witness.databaseKey = metrics.valid ? metrics.key : std::string{};
            witness.clipLocalTime = clip->localTime;

            if (auto* control = clip->animationControl.get()) {
                witness.controlReadable = true;
                witness.controlWeight = std::isfinite(control->weight) ? control->weight : 0.0f;
                witness.masterWeight = std::isfinite(control->masterWeight) ? control->masterWeight : 0.0f;
                witness.motionTrackWeight = std::isfinite(control->motionTrackWeight) ?
                    control->motionTrackWeight : 0.0f;
                witness.poseWeight = std::max(0.0f, witness.controlWeight) *
                    std::max(0.0f, witness.masterWeight);
            } else {
                // A generated clip without a readable control still participates.
                // Use unit weight as a conservative fallback and expose it in logs.
                witness.poseWeight = 1.0f;
            }

            if (metrics.valid && metrics.duration > 1.0e-6f &&
                frameDeltaTime > 0.0f && std::isfinite(clip->localTime)) {
                float playbackSpeed = clip->playbackSpeed;
                if (auto* control = clip->animationControl.get();
                    control && std::isfinite(control->playbackSpeed)) {
                    playbackSpeed = control->playbackSpeed;
                }
                if (!std::isfinite(playbackSpeed)) {
                    playbackSpeed = 1.0f;
                }

                const float deltaSeconds = frameDeltaTime * playbackSpeed;
                if (deltaSeconds >= 0.0f) {
                    const float startLocalTime = clip->localTime - deltaSeconds;
                    float startPhase = std::fmod(startLocalTime, metrics.duration) / metrics.duration;
                    if (startPhase < 0.0f) {
                        startPhase += 1.0f;
                    }

                    const auto delta = AnimationMotionDatabase::SampleDeltaSeconds(
                        profile, witness.animationName, startPhase, deltaSeconds);
                    if (delta.valid) {
                        witness.deltaValid = true;
                        witness.deltaX = delta.dx;
                        witness.deltaY = delta.dy;
                        witness.deltaZ = delta.dz;
                        witness.yawValid = delta.yawValid;
                        witness.deltaYaw = delta.deltaYaw;
                    }
                }
            }
            return witness;
        }

        void RegisterRootContributionLocked(
            NativeActorState& state,
            RootContributionWitness witness)
        {
            if (!witness.clip) {
                return;
            }
            for (std::size_t i = 0; i < state.rootContributionCount &&
                 i < state.rootContributions.size(); ++i) {
                if (state.rootContributions[i].clip == witness.clip) {
                    state.rootContributions[i] = std::move(witness);
                    return;
                }
            }
            if (state.rootContributionCount < state.rootContributions.size()) {
                state.rootContributions[state.rootContributionCount++] = std::move(witness);
            }
        }

        // Havok 2013 hkbGeneratorOutput raw layout used by Skyrim.
        // Source contract:
        //   track 0 = TRACK_WORLD_FROM_MODEL
        //   track 1 = TRACK_EXTRACTED_MOTION
        //   TrackHeader::onFraction == 0 means the data is invalid.
        // We keep this view local and read-only; no generator output memory is modified.
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

        enum class RootOutputClass : std::uint8_t
        {
            kUnknown,
            kInPlace,
            kRootMotion
        };

        struct QsTrackProbe
        {
            bool headerReadable{ false };
            bool valid{ false };
            bool qsTransform{ false };
            std::int16_t capacity{ 0 };
            std::int16_t numData{ 0 };
            std::int16_t dataOffset{ 0 };
            std::int16_t elementSizeBytes{ 0 };
            float onFraction{ 0.0f };
            std::int8_t flags{ 0 };
            std::int8_t type{ 0 };
            float x{ 0.0f };
            float y{ 0.0f };
            float z{ 0.0f };
            float yawDeg{ 0.0f };
            bool transformFinite{ false };
            bool nonIdentity{ false };
        };

        struct GeneratorOutputProbe
        {
            bool outputPresent{ false };
            bool layoutReadable{ false };
            std::int32_t numBytes{ 0 };
            std::int32_t numTracks{ 0 };
            QsTrackProbe worldFromModel{};
            QsTrackProbe extractedMotion{};
            RootOutputClass rootClass{ RootOutputClass::kUnknown };
        };

        [[nodiscard]] const char* RootOutputClassName(RootOutputClass value) noexcept
        {
            switch (value) {
            case RootOutputClass::kInPlace:
                return "IDENTITY_OR_OFF";
            case RootOutputClass::kRootMotion:
                return "NONZERO_EXTRACTED";
            default:
                return "UNKNOWN";
            }
        }

        [[nodiscard]] float QuaternionYawDeg(
            float qx, float qy, float qz, float qw) noexcept
        {
            if (!std::isfinite(qx) || !std::isfinite(qy) ||
                !std::isfinite(qz) || !std::isfinite(qw)) {
                return 0.0f;
            }
            const float lenSq = qx * qx + qy * qy + qz * qz + qw * qw;
            if (!(lenSq > 1.0e-12f) || !std::isfinite(lenSq)) {
                return 0.0f;
            }
            const float invLen = 1.0f / std::sqrt(lenSq);
            qx *= invLen;
            qy *= invLen;
            qz *= invLen;
            qw *= invLen;
            const float yaw = std::atan2(
                2.0f * (qw * qz + qx * qy),
                1.0f - 2.0f * (qy * qy + qz * qz));
            return yaw * 57.29577951308232f;
        }

        [[nodiscard]] QsTrackProbe ProbeQsTrack(
            const std::byte* tracks,
            const GeneratorOutputTrackMasterHeaderView& master,
            std::int32_t trackIndex) noexcept
        {
            QsTrackProbe probe{};
            if (!tracks || trackIndex < 0 || trackIndex >= master.numTracks) {
                return probe;
            }

            constexpr std::int32_t kHeaderBytes = 0x10;
            const std::int64_t headerOffset =
                static_cast<std::int64_t>(kHeaderBytes) +
                static_cast<std::int64_t>(trackIndex) *
                    static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView));
            if (headerOffset < 0 ||
                headerOffset + static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView)) >
                    master.numBytes) {
                return probe;
            }

            GeneratorOutputTrackHeaderView header{};
            const auto* headerAddress = tracks + headerOffset;
            if (!IsExactRangeAccessible(headerAddress, sizeof(header), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("TRACK_HEADER", headerAddress, sizeof(header));
                return probe;
            }
            std::memcpy(std::addressof(header), headerAddress, sizeof(header));
            probe.headerReadable = true;
            probe.capacity = header.capacity;
            probe.numData = header.numData;
            probe.dataOffset = header.dataOffset;
            probe.elementSizeBytes = header.elementSizeBytes;
            probe.onFraction = header.onFraction;
            probe.flags = header.flags;
            probe.type = header.type;

            // Havok defines QSTRANSFORM as TrackTypes value 1.
            probe.qsTransform = header.type == 1;
            probe.valid =
                std::isfinite(header.onFraction) &&
                header.onFraction > 0.0f &&
                header.numData > 0 &&
                header.capacity >= header.numData &&
                header.dataOffset >= 0 &&
                header.elementSizeBytes > 0;

            if (!probe.valid || !probe.qsTransform ||
                header.elementSizeBytes < 0x20) {
                return probe;
            }

            const std::int64_t dataStart = header.dataOffset;
            const std::int64_t requiredBytes =
                std::min<std::int64_t>(header.elementSizeBytes, 0x30);
            if (dataStart < 0 ||
                dataStart + requiredBytes > master.numBytes) {
                probe.valid = false;
                return probe;
            }

            // hkQsTransform begins with hkVector4 translation (16 bytes), then
            // hkQuaternion rotation (16 bytes). Scale is irrelevant here.
            if (!IsExactRangeAccessible(tracks + dataStart, 0x20, ExactMemoryAccess::kRead)) {
                LogR65PointerReject("QS_TRANSFORM_READ", tracks + dataStart, 0x20);
                probe.valid = false;
                return probe;
            }
            std::array<float, 4> translation{};
            std::array<float, 4> rotation{};
            std::memcpy(translation.data(), tracks + dataStart, sizeof(translation));
            std::memcpy(rotation.data(), tracks + dataStart + 0x10, sizeof(rotation));

            probe.x = translation[0];
            probe.y = translation[1];
            probe.z = translation[2];
            probe.yawDeg = QuaternionYawDeg(
                rotation[0], rotation[1], rotation[2], rotation[3]);
            probe.transformFinite =
                std::isfinite(probe.x) &&
                std::isfinite(probe.y) &&
                std::isfinite(probe.z) &&
                std::isfinite(probe.yawDeg);
            if (probe.transformFinite) {
                probe.nonIdentity =
                    std::hypot(probe.x, probe.y) > 1.0e-5f ||
                    std::fabs(probe.z) > 1.0e-5f ||
                    std::fabs(probe.yawDeg) > 1.0e-4f;
            }
            return probe;
        }

        [[nodiscard]] GeneratorOutputProbe ProbeGeneratorOutput(
            const void* opaqueOutput) noexcept
        {
            GeneratorOutputProbe probe{};
            if (!opaqueOutput) {
                return probe;
            }
            probe.outputPresent = true;

            if (!IsExactRangeAccessible(opaqueOutput, sizeof(GeneratorOutputView), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("GENERATOR_OUTPUT", opaqueOutput, sizeof(GeneratorOutputView));
                return probe;
            }
            GeneratorOutputView outputView{};
            std::memcpy(std::addressof(outputView), opaqueOutput, sizeof(outputView));
            if (!outputView.tracks) {
                return probe;
            }
            if (!IsExactRangeAccessible(outputView.tracks, sizeof(GeneratorOutputTrackMasterHeaderView), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("TRACK_MASTER", outputView.tracks, sizeof(GeneratorOutputTrackMasterHeaderView));
                return probe;
            }

            GeneratorOutputTrackMasterHeaderView master{};
            std::memcpy(std::addressof(master), outputView.tracks, sizeof(master));

            // Standard Skyrim behavior outputs are small. Reject impossible layouts
            // rather than gambling on an invalid pointer/offset.
            constexpr std::int32_t kMaxTracks = 64;
            constexpr std::int32_t kMaxTrackBytes = 1024 * 1024;
            if (master.numTracks <= 0 || master.numTracks > kMaxTracks ||
                master.numBytes < static_cast<std::int32_t>(sizeof(master)) ||
                master.numBytes > kMaxTrackBytes) {
                return probe;
            }
            const std::int64_t minimumHeaderBytes =
                static_cast<std::int64_t>(sizeof(master)) +
                static_cast<std::int64_t>(master.numTracks) *
                    static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView));
            if (minimumHeaderBytes > master.numBytes) {
                return probe;
            }

            probe.layoutReadable = true;
            probe.numBytes = master.numBytes;
            probe.numTracks = master.numTracks;
            probe.worldFromModel = ProbeQsTrack(outputView.tracks, master, 0);
            if (master.numTracks > 1) {
                probe.extractedMotion = ProbeQsTrack(outputView.tracks, master, 1);

                // R15.4: this decoder is TELEMETRY ONLY. A valid identity
                // TRACK_EXTRACTED_MOTION is NOT proof of root-motion capability;
                // R14.2 incorrectly classified every such track as ROOT. We label
                // only a non-identity finite delta as ROOT/NONZERO here. This value
                // never opens the translation gate and never changes WFM flags.
                if (!probe.extractedMotion.headerReadable) {
                    probe.rootClass = RootOutputClass::kUnknown;
                } else if (!probe.extractedMotion.valid) {
                    probe.rootClass = RootOutputClass::kInPlace;
                } else if (probe.extractedMotion.qsTransform &&
                           probe.extractedMotion.transformFinite) {
                    probe.rootClass = probe.extractedMotion.nonIdentity ?
                        RootOutputClass::kRootMotion :
                        RootOutputClass::kInPlace;
                } else {
                    probe.rootClass = RootOutputClass::kUnknown;
                }
            }
            return probe;
        }

        struct FinalBehaviorTranslationProbe
        {
            bool graphReadable{ false };
            bool ambiguous{ false };
            std::int32_t selectedOutput{ -1 };
            RE::NiPoint3 delta{};
            float onFraction{ 0.0f };
            float interpolationAmount{ 0.0f };
        };

        [[nodiscard]] FinalBehaviorTranslationProbe ProbeFinalBehaviorTranslation(RE::Actor* actor) noexcept
        {
            FinalBehaviorTranslationProbe result{};
            if (!actor) return result;
            RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
            if (!actor->GetAnimationGraphManager(manager) || !manager) return result;
            const auto activeGraph = manager->GetRuntimeData().activeGraph;
            if (activeGraph >= manager->graphs.size()) return result;
            auto* graph = manager->graphs[activeGraph].get();
            if (!graph) return result;
            result.graphReadable = true;

            std::array<GeneratorOutputProbe, 2> probes{};
            for (std::int32_t i = 0; i < 2; ++i) {
                probes[i] = ProbeGeneratorOutput(graph->generatorOutputs[i]);
            }
            const bool v0 = probes[0].extractedMotion.valid && probes[0].extractedMotion.transformFinite;
            const bool v1 = probes[1].extractedMotion.valid && probes[1].extractedMotion.transformFinite;
            std::int32_t selected = -1;
            if (v0 != v1) {
                selected = v0 ? 0 : 1;
            } else if (v0 && v1) {
                const float a0 = graph->interpolationAmounts[0];
                const float a1 = graph->interpolationAmounts[1];
                if (std::isfinite(a0) && std::isfinite(a1) && std::abs(a0 - a1) > 1.0e-4f) {
                    selected = a0 > a1 ? 0 : 1;
                } else {
                    const float f0 = probes[0].extractedMotion.onFraction;
                    const float f1 = probes[1].extractedMotion.onFraction;
                    if (std::abs(f0 - f1) > 1.0e-4f) {
                        selected = f0 > f1 ? 0 : 1;
                    } else {
                        const float dx = probes[0].extractedMotion.x - probes[1].extractedMotion.x;
                        const float dy = probes[0].extractedMotion.y - probes[1].extractedMotion.y;
                        if (std::hypot(dx, dy) <= 1.0e-4f) selected = 0;
                        else { result.ambiguous = true; return result; }
                    }
                }
            } else {
                return result;
            }
            result.selectedOutput = selected;
            result.delta = {
                probes[selected].extractedMotion.x,
                probes[selected].extractedMotion.y,
                probes[selected].extractedMotion.z
            };
            result.onFraction = probes[selected].extractedMotion.onFraction;
            result.interpolationAmount = graph->interpolationAmounts[selected];
            return result;
        }

        struct SyntheticWfmWriteResult
        {
            bool layoutReadable{ false };
            bool trackWritable{ false };
            bool applied{ false };
            float beforeX{ 0.0f };
            float beforeY{ 0.0f };
            float afterX{ 0.0f };
            float afterY{ 0.0f };
            float worldDx{ 0.0f };
            float worldDy{ 0.0f };
        };

        [[nodiscard]] float NormalizeClipTime(float value, float duration) noexcept
        {
            if (!std::isfinite(value) || !std::isfinite(duration) || duration <= 1.0e-6f) {
                return 0.0f;
            }
            float result = std::fmod(value, duration);
            if (result < 0.0f) {
                result += duration;
            }
            return result;
        }

        [[nodiscard]] AnimationMotionDatabase::DeltaSample ResolveSyntheticFrameDeltaLocked(
            NativeActorState& state,
            const RE::hkbClipGenerator* clip,
            std::string_view profile,
            std::string_view animationName,
            const AnimationMotionDatabase::ClipMetrics& metrics,
            float timeOffset)
        {
            AnimationMotionDatabase::DeltaSample out{};
            if (!clip || !metrics.valid || metrics.duration <= 1.0e-6f ||
                !std::isfinite(clip->localTime)) {
                return out;
            }

            float playbackSpeed = clip->playbackSpeed;
            if (auto* control = clip->animationControl.get();
                control && std::isfinite(control->playbackSpeed)) {
                playbackSpeed = control->playbackSpeed;
            }
            if (!std::isfinite(playbackSpeed)) {
                playbackSpeed = 1.0f;
            }

            const float effectiveTime = clip->localTime +
                (std::isfinite(timeOffset) ? timeOffset * playbackSpeed : 0.0f);
            const float currentLocal = NormalizeClipTime(effectiveTime, metrics.duration);

            auto& phase = state.syntheticClipPhase[clip];
            const bool identityChanged = phase.animationName != std::string{ animationName };
            const bool stale = phase.valid && state.frameSerial > phase.lastSeenFrame + 4;
            if (!phase.valid || identityChanged || stale) {
                phase.animationName = std::string{ animationName };
                phase.previousLocalTime = currentLocal;
                phase.lastSeenFrame = state.frameSerial;
                phase.valid = true;
                return out;
            }

            float deltaSeconds = currentLocal - phase.previousLocalTime;
            if (deltaSeconds < -0.5f * metrics.duration) {
                deltaSeconds += metrics.duration;
            } else if (deltaSeconds < 0.0f) {
                // Small backward seeks happen during sync/transition retiming. Rebase
                // instead of synthesizing a backwards teleport.
                phase.previousLocalTime = currentLocal;
                phase.lastSeenFrame = state.frameSerial;
                return out;
            }

            const float expected = std::max(
                0.0f,
                state.frameDeltaTime * std::max(0.0f, playbackSpeed));
            const float maxReasonableDelta = std::max(0.10f, expected * 4.0f + 0.02f);
            if (!std::isfinite(deltaSeconds) || deltaSeconds < 0.0f ||
                deltaSeconds > maxReasonableDelta) {
                phase.previousLocalTime = currentLocal;
                phase.lastSeenFrame = state.frameSerial;
                return out;
            }

            const float startPhase = phase.previousLocalTime / metrics.duration;
            out = AnimationMotionDatabase::SampleDeltaSeconds(
                profile, animationName, startPhase, deltaSeconds);

            phase.previousLocalTime = currentLocal;
            phase.lastSeenFrame = state.frameSerial;
            return out;
        }

        [[nodiscard]] SyntheticWfmWriteResult InjectSyntheticTranslationIntoWfm(
            void* opaqueOutput,
            float localDx,
            float localDy) noexcept
        {
            SyntheticWfmWriteResult result{};
            if (!opaqueOutput || !std::isfinite(localDx) || !std::isfinite(localDy)) {
                return result;
            }

            if (!IsExactRangeAccessible(opaqueOutput, sizeof(GeneratorOutputView), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("WFM_OUTPUT", opaqueOutput, sizeof(GeneratorOutputView));
                return result;
            }
            GeneratorOutputView outputView{};
            std::memcpy(std::addressof(outputView), opaqueOutput, sizeof(outputView));
            if (!outputView.tracks) {
                return result;
            }
            if (!IsExactRangeAccessible(outputView.tracks, sizeof(GeneratorOutputTrackMasterHeaderView), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("WFM_TRACK_MASTER", outputView.tracks, sizeof(GeneratorOutputTrackMasterHeaderView));
                return result;
            }

            GeneratorOutputTrackMasterHeaderView master{};
            std::memcpy(std::addressof(master), outputView.tracks, sizeof(master));
            constexpr std::int32_t kMaxTracks = 64;
            constexpr std::int32_t kMaxTrackBytes = 1024 * 1024;
            if (master.numTracks <= 0 || master.numTracks > kMaxTracks ||
                master.numBytes < static_cast<std::int32_t>(sizeof(master)) ||
                master.numBytes > kMaxTrackBytes) {
                return result;
            }
            result.layoutReadable = true;

            constexpr std::int32_t kHeaderOffset = 0x10;  // TRACK_WORLD_FROM_MODEL = 0
            if (kHeaderOffset + static_cast<std::int32_t>(sizeof(GeneratorOutputTrackHeaderView)) >
                master.numBytes) {
                return result;
            }

            GeneratorOutputTrackHeaderView header{};
            const auto* headerAddress = outputView.tracks + kHeaderOffset;
            if (!IsExactRangeAccessible(headerAddress, sizeof(header), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("WFM_HEADER", headerAddress, sizeof(header));
                return result;
            }
            std::memcpy(std::addressof(header), headerAddress, sizeof(header));
            if (!std::isfinite(header.onFraction) || header.onFraction <= 0.0f ||
                header.numData <= 0 || header.capacity < header.numData ||
                header.dataOffset < 0 || header.elementSizeBytes < 0x20 ||
                header.type != 1) {
                return result;
            }
            if (static_cast<std::int64_t>(header.dataOffset) + 0x20 > master.numBytes) {
                return result;
            }
            const auto* transformAddress = outputView.tracks + header.dataOffset;
            if (!IsExactRangeAccessible(transformAddress, 0x20, ExactMemoryAccess::kRead) ||
                !IsExactRangeAccessible(transformAddress, 0x10, ExactMemoryAccess::kWrite)) {
                LogR65PointerReject("WFM_TRANSFORM", transformAddress, 0x20);
                return result;
            }
            result.trackWritable = true;

            std::array<float, 4> translation{};
            std::array<float, 4> rotation{};
            std::memcpy(
                translation.data(),
                outputView.tracks + header.dataOffset,
                sizeof(translation));
            std::memcpy(
                rotation.data(),
                outputView.tracks + header.dataOffset + 0x10,
                sizeof(rotation));
            if (!std::isfinite(translation[0]) || !std::isfinite(translation[1])) {
                return result;
            }

            float qx = rotation[0];
            float qy = rotation[1];
            float qz = rotation[2];
            float qw = rotation[3];
            const float qLenSq = qx * qx + qy * qy + qz * qz + qw * qw;
            if (!(qLenSq > 1.0e-10f) || !std::isfinite(qLenSq)) {
                return result;
            }
            const float invLen = 1.0f / std::sqrt(qLenSq);
            qx *= invLen;
            qy *= invLen;
            qz *= invLen;
            qw *= invLen;

            // Rotate the local MotionDB planar delta by the clip's current WFM
            // orientation. R32 leaves this translation pass rotation-neutral;
            // vanilla pathing yaw is transported through ProcessMotionData R1/R2.
            const float xx = qx * qx;
            const float yy = qy * qy;
            const float zz = qz * qz;
            const float xy = qx * qy;
            const float wz = qw * qz;
            result.worldDx =
                (1.0f - 2.0f * (yy + zz)) * localDx +
                2.0f * (xy - wz) * localDy;
            result.worldDy =
                2.0f * (xy + wz) * localDx +
                (1.0f - 2.0f * (xx + zz)) * localDy;
            if (!std::isfinite(result.worldDx) || !std::isfinite(result.worldDy)) {
                return result;
            }

            result.beforeX = translation[0];
            result.beforeY = translation[1];
            translation[0] += result.worldDx;
            translation[1] += result.worldDy;
            result.afterX = translation[0];
            result.afterY = translation[1];

            auto* mutableTracks = const_cast<std::byte*>(outputView.tracks);
            std::memcpy(
                mutableTracks + header.dataOffset,
                translation.data(),
                sizeof(translation));
            result.applied = true;
            return result;
        }

        struct NativeExtractedWriteResult
        {
            bool layoutReadable{ false };
            bool trackWritable{ false };
            bool nativeAlreadyPresent{ false };
            bool applied{ false };
            float beforeX{ 0.0f };
            float beforeY{ 0.0f };
            float afterX{ 0.0f };
            float afterY{ 0.0f };
            float onFraction{ 0.0f };
        };

        [[nodiscard]] NativeExtractedWriteResult InjectNativeTranslationIntoExtractedMotion(
            void* opaqueOutput,
            float localDx,
            float localDy) noexcept
        {
            NativeExtractedWriteResult result{};
            if (!opaqueOutput || !std::isfinite(localDx) || !std::isfinite(localDy)) return result;
            if (!IsExactRangeAccessible(opaqueOutput, sizeof(GeneratorOutputView), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("C1_EXTRACTED_OUTPUT", opaqueOutput, sizeof(GeneratorOutputView));
                return result;
            }
            GeneratorOutputView outputView{};
            std::memcpy(std::addressof(outputView), opaqueOutput, sizeof(outputView));
            if (!outputView.tracks ||
                !IsExactRangeAccessible(outputView.tracks, sizeof(GeneratorOutputTrackMasterHeaderView), ExactMemoryAccess::kRead)) {
                return result;
            }
            GeneratorOutputTrackMasterHeaderView master{};
            std::memcpy(std::addressof(master), outputView.tracks, sizeof(master));
            if (master.numTracks <= 1 || master.numTracks > 64 || master.numBytes <= 0 || master.numBytes > 1024 * 1024) {
                return result;
            }
            result.layoutReadable = true;

            constexpr std::int32_t kExtractedMotionTrack = 1;
            const std::int64_t headerOffset =
                static_cast<std::int64_t>(sizeof(GeneratorOutputTrackMasterHeaderView)) +
                static_cast<std::int64_t>(kExtractedMotionTrack) *
                    static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView));
            if (headerOffset < 0 ||
                headerOffset + static_cast<std::int64_t>(sizeof(GeneratorOutputTrackHeaderView)) > master.numBytes) return result;

            GeneratorOutputTrackHeaderView header{};
            const auto* headerAddress = outputView.tracks + headerOffset;
            if (!IsExactRangeAccessible(headerAddress, sizeof(header), ExactMemoryAccess::kRead)) {
                LogR65PointerReject("C1_EXTRACTED_HEADER", headerAddress, sizeof(header));
                return result;
            }
            std::memcpy(std::addressof(header), headerAddress, sizeof(header));
            result.onFraction = header.onFraction;
            if (!std::isfinite(header.onFraction) || header.onFraction <= 0.0f ||
                header.type != 1 || header.numData <= 0 || header.capacity < header.numData ||
                header.dataOffset < 0 || header.elementSizeBytes < 0x20 ||
                static_cast<std::int64_t>(header.dataOffset) + 0x20 > master.numBytes) return result;

            const auto* transformAddress = outputView.tracks + header.dataOffset;
            if (!IsExactRangeAccessible(transformAddress, 0x20, ExactMemoryAccess::kRead) ||
                !IsExactRangeAccessible(transformAddress, 0x10, ExactMemoryAccess::kWrite)) {
                LogR65PointerReject("C1_EXTRACTED_TRANSFORM", transformAddress, 0x20);
                return result;
            }
            result.trackWritable = true;
            std::array<float, 4> translation{};
            std::memcpy(translation.data(), transformAddress, sizeof(translation));
            if (!std::isfinite(translation[0]) || !std::isfinite(translation[1]) || !std::isfinite(translation[2])) return result;

            result.beforeX = translation[0];
            result.beforeY = translation[1];
            const float nativePlanar = std::hypot(translation[0], translation[1]);
            if (std::isfinite(nativePlanar) && nativePlanar > kRootContributionEpsilon) {
                result.nativeAlreadyPresent = true;
                result.afterX = translation[0];
                result.afterY = translation[1];
                return result;
            }

            translation[0] = localDx;
            translation[1] = localDy;
            result.afterX = translation[0];
            result.afterY = translation[1];
            auto* mutableTracks = const_cast<std::byte*>(outputView.tracks);
            std::memcpy(mutableTracks + header.dataOffset, translation.data(), sizeof(translation));
            result.applied = true;
            return result;
        }

        [[nodiscard]] bool FindNativeExtractedDeltaForClipLocked(
            const NativeActorState& state,
            std::string_view node,
            std::string_view animation,
            RE::NiPoint3& outDelta,
            std::string& outSourceClip,
            std::uint64_t& outSourceFrame) noexcept
        {
            const std::array<std::string, 2> keys{
                CanonicalMotionName(node), CanonicalMotionName(animation)
            };
            for (const auto& key : keys) {
                if (key.empty()) continue;
                const auto it = state.nativeExtractedDeltas.find(key);
                if (it == state.nativeExtractedDeltas.end()) continue;
                const auto& sample = it->second;
                if (!sample.valid || sample.frame == 0 || state.frameSerial == 0 ||
                    sample.frame + 2 < state.frameSerial || sample.frame > state.frameSerial) continue;
                outDelta = sample.delta;
                outSourceClip = sample.clipName;
                outSourceFrame = sample.frame;
                return true;
            }
            return false;
        }

        [[nodiscard]] SyntheticWfmSideProbe ProbeSyntheticWfmSideLocked(
            const NativeActorState& state,
            const GeneratorOutputProbe& probe) noexcept
        {
            SyntheticWfmSideProbe side{};
            if (!state.rootDebugSnapshotValid ||
                state.syntheticWfmTranslationSeenFrame != state.frameSerial ||
                !probe.worldFromModel.valid || !probe.worldFromModel.transformFinite) {
                return side;
            }

            side.readable = true;
            side.offsetX = probe.worldFromModel.x - state.rootDebugPosition.x;
            side.offsetY = probe.worldFromModel.y - state.rootDebugPosition.y;
            side.distanceXY = std::hypot(side.offsetX, side.offsetY);
            if (!std::isfinite(side.distanceXY)) {
                side.distanceXY = 0.0f;
                return side;
            }

            // R15.6: non-zero WFM is NOT root provenance. A child is DB-root-bearing
            // only when its WFM lands on a transform that was actually produced by a
            // MotionDB leaf injection in THIS evaluation. This rejects native/root
            // transforms and stale capsule displacement that merely happen to be non-zero.
            // Parent transition outputs normally preserve the chosen child's WFM exactly;
            // a small tolerance covers floating-point blend/copy noise.
            for (std::size_t i = 0; i < state.syntheticWfmInjectionCount; ++i) {
                const auto& witness = state.syntheticWfmInjections[i];
                if (!witness.applied || witness.frame != state.frameSerial) {
                    continue;
                }
                const float injectionDistance = std::hypot(witness.worldDx, witness.worldDy);
                if (!std::isfinite(injectionDistance) || injectionDistance <= 1.0e-5f) {
                    continue;
                }
                const float matchDistance = std::hypot(
                    probe.worldFromModel.x - witness.afterX,
                    probe.worldFromModel.y - witness.afterY);
                const float tolerance = std::max(0.05f, injectionDistance * 0.03f);
                if (std::isfinite(matchDistance) && matchDistance <= tolerance) {
                    side.syntheticRoot = true;
                    break;
                }
            }
            return side;
        }

        [[nodiscard]] const char* TransitionSideMappingName(TransitionSideMapping mapping) noexcept
        {
            switch (mapping) {
            case TransitionSideMapping::kChild0FromChild1To:
                return "0_FROM_1_TO";
            case TransitionSideMapping::kChild1FromChild0To:
                return "1_FROM_0_TO";
            default:
                return "UNKNOWN";
            }
        }

        [[nodiscard]] float WfmDistanceXY(
            float ax, float ay, float bx, float by) noexcept
        {
            const float distance = std::hypot(ax - bx, ay - by);
            return std::isfinite(distance) ? distance : std::numeric_limits<float>::infinity();
        }

        [[nodiscard]] TransitionSideMapProbe ResolveTransitionSideMapping(
            const GeneratorOutputProbe& child0,
            const GeneratorOutputProbe& child1,
            const TransitionWfmAnchors& anchors) noexcept
        {
            TransitionSideMapProbe result{};
            result.anchorsReadable = anchors.readable;
            if (!anchors.readable ||
                !child0.worldFromModel.valid || !child0.worldFromModel.transformFinite ||
                !child1.worldFromModel.valid || !child1.worldFromModel.transformFinite) {
                return result;
            }

            result.directScore =
                WfmDistanceXY(child0.worldFromModel.x, child0.worldFromModel.y,
                    anchors.fromX, anchors.fromY) +
                WfmDistanceXY(child1.worldFromModel.x, child1.worldFromModel.y,
                    anchors.toX, anchors.toY);
            result.swappedScore =
                WfmDistanceXY(child1.worldFromModel.x, child1.worldFromModel.y,
                    anchors.fromX, anchors.fromY) +
                WfmDistanceXY(child0.worldFromModel.x, child0.worldFromModel.y,
                    anchors.toX, anchors.toY);

            if (!std::isfinite(result.directScore) || !std::isfinite(result.swappedScore)) {
                return result;
            }

            result.confidence = std::fabs(result.directScore - result.swappedScore);
            const float best = std::min(result.directScore, result.swappedScore);
            const float requiredMargin = std::max(1.0e-4f, best * 0.05f);
            if (result.confidence <= requiredMargin) {
                return result;
            }

            result.mapping = result.directScore < result.swappedScore ?
                TransitionSideMapping::kChild0FromChild1To :
                TransitionSideMapping::kChild1FromChild0To;
            return result;
        }

        struct ExtractedMotionProbe
        {
            float x{ 0.0f };
            float y{ 0.0f };
            float z{ 0.0f };
            float yawDeg{ 0.0f };
            bool finite{ false };
            bool nonIdentity{ false };
        };

        [[nodiscard]] ExtractedMotionProbe ProbeExtractedMotion(
            const RE::hkbClipGenerator* clip) noexcept
        {
            ExtractedMotionProbe result{};
            if (!clip) {
                return result;
            }

            static_assert(sizeof(RE::hkVector4) == 0x10);
            static_assert(sizeof(RE::hkQuaternion) == 0x10);

            std::array<float, 4> translation{};
            std::array<float, 4> rotation{};
            std::memcpy(
                translation.data(),
                std::addressof(clip->extractedMotion.translation),
                sizeof(translation));
            std::memcpy(
                rotation.data(),
                std::addressof(clip->extractedMotion.rotation),
                sizeof(rotation));

            result.x = translation[0];
            result.y = translation[1];
            result.z = translation[2];

            const float qx = rotation[0];
            const float qy = rotation[1];
            const float qz = rotation[2];
            const float qw = rotation[3];
            result.finite =
                std::isfinite(result.x) && std::isfinite(result.y) && std::isfinite(result.z) &&
                std::isfinite(qx) && std::isfinite(qy) && std::isfinite(qz) && std::isfinite(qw);
            if (!result.finite) {
                return result;
            }

            const float qLenSq = qx * qx + qy * qy + qz * qz + qw * qw;
            float yaw = 0.0f;
            if (qLenSq > 1.0e-10f && std::isfinite(qLenSq)) {
                const float invLen = 1.0f / std::sqrt(qLenSq);
                const float nx = qx * invLen;
                const float ny = qy * invLen;
                const float nz = qz * invLen;
                const float nw = qw * invLen;
                yaw = std::atan2(
                    2.0f * (nw * nz + nx * ny),
                    1.0f - 2.0f * (ny * ny + nz * nz));
            }
            result.yawDeg = yaw * 57.29577951308232f;

            const float planar = std::hypot(result.x, result.y);
            result.nonIdentity =
                planar > 1.0e-5f ||
                std::fabs(result.z) > 1.0e-5f ||
                std::fabs(yaw) > 1.0e-5f;
            return result;
        }


        [[nodiscard]] RE::FormID ResolveActorFormIDForContext(
            const RE::hkbContext& context) noexcept
        {
            if (!context.character) {
                return 0;
            }

            std::scoped_lock lock{ g_mutex };
            for (const auto& [formID, state] : g_actorState) {
                if (state.frameSerial != 0 && StateOwnsHkbCharacter(state, context.character)) {
                    return formID;
                }
            }
            return 0;
        }

        [[nodiscard]] bool RegisterRootMotionWitnessLocked(
            NativeActorState& state,
            const RE::hkbClipGenerator* clip,
            std::string_view nodeName,
            std::string_view animationName,
            bool motionDbBacked,
            const ExtractedMotionProbe& extracted)
        {
            if (!clip || animationName.empty()) {
                return false;
            }

            // Legacy clip/member telemetry only. MotionDB membership or the
            // clip object's cached extractedMotion can help diagnose lifecycle,
            // but neither is root authority in R14.2.
            const bool telemetryRelevant = motionDbBacked || extracted.nonIdentity;
            if (!telemetryRelevant) {
                return false;
            }

            for (std::size_t i = 0; i < state.rootWitnessCount && i < state.rootWitnesses.size(); ++i) {
                if (state.rootWitnesses[i].clip == clip) {
                    auto& existing = state.rootWitnesses[i];
                    existing.motionDbBacked = existing.motionDbBacked || motionDbBacked;
                    existing.clipMemberDeltaObserved =
                        existing.clipMemberDeltaObserved || extracted.nonIdentity;
                    existing.extractedX = extracted.x;
                    existing.extractedY = extracted.y;
                    existing.extractedZ = extracted.z;
                    existing.extractedYawDeg = extracted.yawDeg;
                    return false;
                }
            }

            if (state.rootWitnessCount >= state.rootWitnesses.size()) {
                return false;
            }

            auto& witness = state.rootWitnesses[state.rootWitnessCount++];
            witness.clip = clip;
            witness.nodeName = std::string{ nodeName };
            witness.animationName = std::string{ animationName };
            witness.motionDbBacked = motionDbBacked;
            witness.clipMemberDeltaObserved = extracted.nonIdentity;
            witness.extractedX = extracted.x;
            witness.extractedY = extracted.y;
            witness.extractedZ = extracted.z;
            witness.extractedYawDeg = extracted.yawDeg;
            return true;
        }

        struct RawHkArrayView
        {
            const void* data{ nullptr };
            std::int32_t size{ 0 };
            std::int32_t capacityAndFlags{ 0 };
        };
        static_assert(sizeof(RawHkArrayView) == 0x10);

        struct RawQsTransformView
        {
            std::array<float, 4> translation{};
            std::array<float, 4> rotation{};
            std::array<float, 4> scale{};
        };
        static_assert(sizeof(RawQsTransformView) == 0x30);

        struct BlendingTransitionEffectView
        {
            std::byte pad00[0x50];
            float duration;                       // 50
            float toGeneratorStartTimeFraction;   // 54
            std::uint16_t flags;                  // 58
            std::uint8_t endMode;                 // 5A
            std::uint8_t blendCurve;              // 5B
            std::uint16_t alignmentBone;          // 5C
            std::uint16_t pad5E;                  // 5E
            RE::hkbGenerator* fromGenerator;       // 60
            RE::hkbGenerator* toGenerator;         // 68
            RawHkArrayView worldFromModelTransforms; // 70: Havok cache [from, to, ...]
            float timeRemaining;                  // 80
            float timeInTransition;               // 84
            bool applySelfTransition;              // 88
            bool initializeCharacterPose;          // 89
        };
        static_assert(offsetof(BlendingTransitionEffectView, flags) == 0x58);
        static_assert(offsetof(BlendingTransitionEffectView, fromGenerator) == 0x60);
        static_assert(offsetof(BlendingTransitionEffectView, toGenerator) == 0x68);
        static_assert(offsetof(BlendingTransitionEffectView, worldFromModelTransforms) == 0x70);
        static_assert(offsetof(BlendingTransitionEffectView, timeRemaining) == 0x80);

        [[nodiscard]] TransitionWfmAnchors ProbeTransitionWfmAnchors(
            const BlendingTransitionEffectView* transition) noexcept
        {
            TransitionWfmAnchors anchors{};
            if (!transition) {
                return anchors;
            }

            const auto& array = transition->worldFromModelTransforms;
            anchors.size = array.size;
            const std::int32_t capacity = array.capacityAndFlags & 0x3FFFFFFF;
            const auto dataAddress = reinterpret_cast<std::uintptr_t>(array.data);
            if (!array.data || array.size < 2 || array.size > 8 ||
                capacity < array.size || capacity > 32 ||
                dataAddress < 0x10000 || (dataAddress & 0x0F) != 0 ||
                !IsExactRangeAccessible(array.data, 2 * sizeof(RawQsTransformView), ExactMemoryAccess::kRead)) {
                if (array.data && dataAddress >= 0x10000) {
                    LogR65PointerReject("TRANSITION_WFM_ANCHORS", array.data, 2 * sizeof(RawQsTransformView));
                }
                return anchors;
            }

            RawQsTransformView from{};
            RawQsTransformView to{};
            const auto* bytes = static_cast<const std::byte*>(array.data);
            std::memcpy(std::addressof(from), bytes, sizeof(from));
            std::memcpy(std::addressof(to), bytes + sizeof(RawQsTransformView), sizeof(to));

            anchors.fromX = from.translation[0];
            anchors.fromY = from.translation[1];
            anchors.toX = to.translation[0];
            anchors.toY = to.translation[1];
            anchors.readable =
                std::isfinite(anchors.fromX) && std::isfinite(anchors.fromY) &&
                std::isfinite(anchors.toX) && std::isfinite(anchors.toY);
            return anchors;
        }



        struct ClipGeneratorGenerateHook
        {
            // Native hkbGenerator::Generate ABI. Owner runs first, then R15.3
            // injects the MotionDB frame delta into this leaf's WORLD_FROM_MODEL.
            // Parent generators therefore blend the synthetic root with the exact
            // same graph weights used for the final pose.
            static void Thunk(
                RE::hkbClipGenerator* clip,
                const RE::hkbContext& context,
                const void* const* activeChildrenOutput,
                void* output,
                float timeOffset)
            {
                Func(clip, context, activeChildrenOutput, output, timeOffset);

                if (!clip || !context.character || !output) {
                    return;
                }

                const RE::FormID formID = ResolveActorFormIDForContext(context);
                if (formID == 0) {
                    return;
                }

                const char* nodeRaw = clip->name.c_str();
                const char* animRaw = clip->animationName.c_str();
                const std::string node = nodeRaw ? nodeRaw : "";
                const std::string anim = animRaw ? animRaw : "";

                std::string profile;
                float frameDeltaTime = 0.0f;
                bool isPlayer = false;
                bool nativeLocomotionSession = false;
                std::uint64_t frame = 0;
                {
                    std::scoped_lock lock{ g_mutex };
                    const auto it = g_actorState.find(formID);
                    if (it == g_actorState.end()) {
                        return;
                    }
                    profile = it->second.profile;
                    frameDeltaTime = it->second.frameDeltaTime;
                    isPlayer = it->second.isPlayer;
                    nativeLocomotionSession = it->second.nativeLocomotionSessionActive;
                    frame = it->second.frameSerial;
                }

                const auto metrics = AnimationMotionDatabase::GetClipMetrics(profile, anim);
                auto contribution = BuildRootContributionWitness(
                    clip, node, anim, profile, frameDeltaTime, metrics);

                AnimationMotionDatabase::DeltaSample exactDelta{};
                if (metrics.valid && !isPlayer) {
                    std::scoped_lock lock{ g_mutex };
                    const auto it = g_actorState.find(formID);
                    if (it != g_actorState.end()) {
                        exactDelta = ResolveSyntheticFrameDeltaLocked(
                            it->second, clip, profile, anim, metrics, timeOffset);
                    }
                }

                // Replace the R15.0 frameDeltaTime estimate with the clip-instance
                // local-time delta. This handles loops and transition retiming without
                // teleporting on first acquisition.
                if (metrics.valid && !isPlayer) {
                    contribution.deltaValid = exactDelta.valid;
                    contribution.deltaX = exactDelta.valid ? exactDelta.dx : 0.0f;
                    contribution.deltaY = exactDelta.valid ? exactDelta.dy : 0.0f;
                    contribution.deltaZ = exactDelta.valid ? exactDelta.dz : 0.0f;
                    contribution.yawValid = exactDelta.valid && exactDelta.yawValid;
                    contribution.deltaYaw = contribution.yawValid ? exactDelta.deltaYaw : 0.0f;
                }

                // B2 native locomotion owns planar translation exclusively while active.
                // Keep DB membership/yaw telemetry intact, but do not let the legacy baked
                // translation become a root-acquisition witness after this native session exits.
                if (nativeLocomotionSession) {
                    contribution.deltaValid = false;
                    contribution.deltaX = 0.0f;
                    contribution.deltaY = 0.0f;
                    contribution.deltaZ = 0.0f;
                }

                SyntheticWfmWriteResult injection{};
                const float planarDelta = exactDelta.valid ?
                    std::hypot(exactDelta.dx, exactDelta.dy) : 0.0f;
                if (metrics.valid && !isPlayer && !nativeLocomotionSession && exactDelta.valid &&
                    std::isfinite(planarDelta) && planarDelta > 1.0e-6f) {
                    injection = InjectSyntheticTranslationIntoWfm(
                        output, exactDelta.dx, exactDelta.dy);
                }

                // C1 profileless transport: replace MotionDB as the leaf-delta SOURCE only.
                // The native PMD T1->T2 vector from the previous/current frame is written
                // into TRACK_EXTRACTED_MOTION on the matching safe locomotion leaf. The
                // parent Havok graph remains the sole blender/composer; nothing here touches
                // capsule translation, bAnimationDriven, WFM translation, or rotation.
                NativeExtractedWriteResult universalInjection{};
                RE::NiPoint3 universalDelta{};
                std::string universalSourceClip;
                std::uint64_t universalSourceFrame = 0;
                bool universalCandidate = false;
                if (!isPlayer && profile.empty()) {
                    if (auto* actor = RE::TESForm::LookupByID<RE::Actor>(formID);
                        actor && IsUniversalCreatureActor(actor) &&
                        IsRuntimeEligibleForExternalRoot(actor) &&
                        HasNativeLocomotionDemand(actor) && !HasUniversalActionBypass(actor)) {
                        const bool namedLocomotion =
                            IsCertifiedNativeLocomotionCarrierName(node) ||
                            IsCertifiedNativeLocomotionCarrierName(anim);
                        if (namedLocomotion) {
                            {
                                std::scoped_lock lock{ g_mutex };
                                const auto it = g_actorState.find(formID);
                                if (it != g_actorState.end()) {
                                    universalCandidate = FindNativeExtractedDeltaForClipLocked(
                                        it->second, node, anim, universalDelta,
                                        universalSourceClip, universalSourceFrame);
                                }
                            }
                            const float universalPlanar = std::hypot(universalDelta.x, universalDelta.y);
                            if (universalCandidate &&
                                IsCertifiedNativeLocomotionCarrierName(universalSourceClip) &&
                                std::isfinite(universalPlanar) && universalPlanar > kRootContributionEpsilon) {
                                universalInjection = InjectNativeTranslationIntoExtractedMotion(
                                    output, universalDelta.x, universalDelta.y);
                                if (universalInjection.applied || universalInjection.nativeAlreadyPresent) {
                                    std::scoped_lock lock{ g_mutex };
                                    auto it = g_actorState.find(formID);
                                    if (it != g_actorState.end()) {
                                        auto& state = it->second;
                                        state.universalExtractedLeafSeenFrame = state.frameSerial;
                                        if (universalInjection.applied) ++state.universalExtractedLeafWrites;
                                        else ++state.universalExtractedNativePasses;
                                        const bool emit = state.universalRootLogs < 32 || (state.frameSerial % 120) == 0;
                                        if (emit) {
                                            ++state.universalRootLogs;
                                            SKSE::log::info(
                                                "[C1_NATIVE_EXTRACTED][LEAF] form={:08X} frame={} sourceFrame={} node=\"{}\" anim=\"{}\" sourceClip=\"{}\" delta=({:+.5f},{:+.5f}) trackBefore=({:+.5f},{:+.5f}) trackAfter=({:+.5f},{:+.5f}) on={:.3f} action={}",
                                                formID, state.frameSerial, universalSourceFrame, node, anim, universalSourceClip,
                                                universalDelta.x, universalDelta.y,
                                                universalInjection.beforeX, universalInjection.beforeY,
                                                universalInjection.afterX, universalInjection.afterY,
                                                universalInjection.onFraction,
                                                universalInjection.applied ? "WRITE_TRACK_EXTRACTED_MOTION" : "KEEP_NATIVE_TRACK_EXTRACTED_MOTION");
                                        }
                                    }
                                }
                                else {
                                    std::scoped_lock lock{ g_mutex };
                                    auto it = g_actorState.find(formID);
                                    if (it != g_actorState.end() && it->second.universalTrackRejectLogs < 16) {
                                        ++it->second.universalTrackRejectLogs;
                                        SKSE::log::warn(
                                            "[C1_NATIVE_EXTRACTED][TRACK_REJECT] form={:08X} frame={} node=\"{}\" anim=\"{}\" sourceClip=\"{}\" layoutReadable={} trackWritable={} on={:.3f} action=FAIL_OPEN_NO_PHYSICAL_ROOT",
                                            formID, it->second.frameSerial, node, anim, universalSourceClip,
                                            universalInjection.layoutReadable, universalInjection.trackWritable, universalInjection.onFraction);
                                    }
                                }
                            } else if (!universalCandidate) {
                                std::scoped_lock lock{ g_mutex };
                                auto it = g_actorState.find(formID);
                                if (it != g_actorState.end() && it->second.universalDeltaWaitLogs < 16) {
                                    ++it->second.universalDeltaWaitLogs;
                                    SKSE::log::debug(
                                        "[C1_NATIVE_EXTRACTED][DELTA_WAIT] form={:08X} frame={} node=\"{}\" anim=\"{}\" action=WAIT_FOR_MATCHING_NATIVE_T1_T2_NO_CAPSULE_AUTHORITY_CHANGE",
                                        formID, it->second.frameSerial, node, anim);
                                }
                            }
                        }
                    }
                }

                // R35: WORLD_FROM_MODEL rotation is intentionally left untouched.
                // R29 proved that the memory write succeeds, but a per-Generate absolute
                // offset did not become physical root yaw. The R35 transport moves AI-goal steering
                // yaw to the verified ProcessMotionData R1/R2 delta pair instead.

                const GeneratorOutputProbe probe = ProbeGeneratorOutput(output);
                std::uint64_t rootCount = 0;
                bool emitNative = false;
                bool emitContribution = false;
                bool emitInjection = false;
                bool emitWait = false;
                RootContributionWitness contributionLog{};
                RootContributionSummary mix{};
                {
                    std::scoped_lock lock{ g_mutex };
                    const auto it = g_actorState.find(formID);
                    if (it == g_actorState.end()) {
                        return;
                    }
                    auto& state = it->second;
                    frame = state.frameSerial;
                    ++state.nativeClipGenerateCallsTotal;

                    RegisterRootContributionLocked(state, std::move(contribution));
                    mix = ComputeRootContributionSummaryLocked(state);
                    if (metrics.valid) {
                        for (std::size_t i = 0; i < state.rootContributionCount; ++i) {
                            if (state.rootContributions[i].clip == clip) {
                                contributionLog = state.rootContributions[i];
                                break;
                            }
                        }
                        emitContribution =
                            state.rootMixerClipLogs < 24 ||
                            (state.frameSerial % 240) == 0;
                        if (emitContribution) {
                            ++state.rootMixerClipLogs;
                        }
                    }

                    if (metrics.valid && !isPlayer) {
                        if (injection.applied) {
                            state.syntheticWfmTranslationSeenFrame = state.frameSerial;
                            ++state.syntheticWfmAppliedTotal;
                            SyntheticWfmInjectionWitness witness{};
                            witness.clip = clip;
                            witness.output = output;
                            witness.animationName = anim;
                            witness.databaseKey = metrics.key;
                            witness.localDx = exactDelta.dx;
                            witness.localDy = exactDelta.dy;
                            witness.worldDx = injection.worldDx;
                            witness.worldDy = injection.worldDy;
                            witness.beforeX = injection.beforeX;
                            witness.beforeY = injection.beforeY;
                            witness.afterX = injection.afterX;
                            witness.afterY = injection.afterY;
                            witness.frame = state.frameSerial;
                            witness.applied = true;
                            if (state.syntheticWfmInjectionCount < state.syntheticWfmInjections.size()) {
                                state.syntheticWfmInjections[state.syntheticWfmInjectionCount++] = std::move(witness);
                            }
                            if (state.creatureDbOwnershipEstablished &&
                                OwnershipContainsDbKeyLocked(state, metrics.key)) {
                                state.creatureDbGenerateContinuityFrame = state.frameSerial;
                            }
                            emitInjection =
                                state.syntheticWfmClipLogs < 32 ||
                                (state.syntheticWfmAppliedTotal % 240) == 0;
                            if (emitInjection) {
                                ++state.syntheticWfmClipLogs;
                            }
                        } else if (!exactDelta.valid) {
                            ++state.syntheticWfmWaitTotal;
                            emitWait = state.syntheticWfmWaitTotal <= 16 ||
                                (state.syntheticWfmWaitTotal % 240) == 0;
                        }
                    }

                    // Native TRACK_EXTRACTED_MOTION remains diagnostic only.
                    if (probe.rootClass == RootOutputClass::kRootMotion) {
                        state.wfmRootOutputSeenFrame = state.frameSerial;
                        rootCount = ++state.nativeClipRootOutputsTotal;
                        emitNative = state.nativeClipRootLogs < 32 ||
                                     (rootCount % 240 == 0) ||
                                     probe.extractedMotion.nonIdentity;
                        if (emitNative) {
                            ++state.nativeClipRootLogs;
                        }
                    }
                }

                if (emitInjection) {
                    SKSE::log::debug(R"([SYNTH_WFM][CLIP_INJECT] form={:08X} frame={} clipPtr=0x{:X} node="{}" anim="{}" key="{}" localDelta=({:+.5f},{:+.5f}) worldDelta=({:+.5f},{:+.5f}) WFM[before=({:+.5f},{:+.5f}) after=({:+.5f},{:+.5f})] layoutReadable={} trackWritable={} note="translation only; R35 leaves WFM rotation untouched; AI steering yaw transport is ProcessMotionData R1/R2")",
                        formID,
                        frame,
                        reinterpret_cast<std::uintptr_t>(clip),
                        node,
                        anim,
                        metrics.key,
                        exactDelta.dx,
                        exactDelta.dy,
                        injection.worldDx,
                        injection.worldDy,
                        injection.beforeX,
                        injection.beforeY,
                        injection.afterX,
                        injection.afterY,
                        injection.layoutReadable,
                        injection.trackWritable);
                }

                if (!emitInjection && emitWait) {
                    SKSE::log::debug(R"([SYNTH_WFM][CLIP_WAIT] form={:08X} frame={} clipPtr=0x{:X} node="{}" anim="{}" key="{}" localTime={:.6f} reason=NEED_PREVIOUS_CLIP_TIME_OR_REBASE note="no translation injected on acquisition/seek frame; R35 AI steering yaw is independent in ProcessMotionData R1/R2")",
                        formID,
                        frame,
                        reinterpret_cast<std::uintptr_t>(clip),
                        node,
                        anim,
                        metrics.key,
                        clip->localTime);
                }

                if (emitContribution) {
                    SKSE::log::debug(R"([ROOT_MIXER][CLIP] form={:08X} frame={} clipPtr=0x{:X} node="{}" anim="{}" key="{}" control={} weights[control={:.5f} master={:.5f} motionTrack={:.5f} pose={:.5f}] dbDelta[valid={} xyz=({:+.5f},{:+.5f},{:+.5f}) yawValid={} yaw={:+.5f}rad] mixNow[clips={} dbClips={} poseWeight={:.5f} dbWeight={:.5f} share={:.5f} xyz=({:+.5f},{:+.5f},{:+.5f}) yaw={:+.5f}rad] note="R15.4 diagnostic only; physical blend is the Havok WFM tree")",
                        formID,
                        frame,
                        reinterpret_cast<std::uintptr_t>(clip),
                        contributionLog.nodeName,
                        contributionLog.animationName,
                        contributionLog.databaseKey,
                        contributionLog.controlReadable,
                        contributionLog.controlWeight,
                        contributionLog.masterWeight,
                        contributionLog.motionTrackWeight,
                        contributionLog.poseWeight,
                        contributionLog.deltaValid,
                        contributionLog.deltaX,
                        contributionLog.deltaY,
                        contributionLog.deltaZ,
                        contributionLog.yawValid,
                        contributionLog.deltaYaw,
                        mix.clips,
                        mix.dbClips,
                        mix.poseWeight,
                        mix.dbRootWeight,
                        mix.dbRootShare,
                        mix.mixedX,
                        mix.mixedY,
                        mix.mixedZ,
                        mix.mixedYaw);
                }

                if (emitNative) {
                    SKSE::log::debug(R"([ROOT_OUTPUT][CLIP] form={:08X} frame={} sample={} clipPtr=0x{:X} node="{}" anim="{}" timeOffset={:.6f} class={} WFM[valid={} on={:.4f} xyz=({:+.5f},{:+.5f},{:+.5f}) yaw={:+.3f}] EXTRACTED[valid={} on={:.4f} xyz=({:+.5f},{:+.5f},{:+.5f}) yaw={:+.3f} nonIdentity={}] note="native extracted track telemetry only; synthetic translation is injected into clip WFM")",
                        formID,
                        frame,
                        rootCount,
                        reinterpret_cast<std::uintptr_t>(clip),
                        node,
                        anim,
                        timeOffset,
                        RootOutputClassName(probe.rootClass),
                        probe.worldFromModel.valid,
                        probe.worldFromModel.onFraction,
                        probe.worldFromModel.x,
                        probe.worldFromModel.y,
                        probe.worldFromModel.z,
                        probe.worldFromModel.yawDeg,
                        probe.extractedMotion.valid,
                        probe.extractedMotion.onFraction,
                        probe.extractedMotion.x,
                        probe.extractedMotion.y,
                        probe.extractedMotion.z,
                        probe.extractedMotion.yawDeg,
                        probe.extractedMotion.nonIdentity);
                }
            }

            static inline REL::Relocation<decltype(Thunk)> Func;
        };

        struct BlendingTransitionGenerateHook
        {
            // R15.6 transition-local root provenance authority.
            // No actor-global prearm exists. The generated child outputs of THIS
            // hkbBlendingTransitionEffect decide the policy. Runtime evidence across
            // R15.4/R15.5 establishes the Skyrim ABI as child0=TO, child1=FROM.
            // A MotionDB WFM signature on only TO => IGNORE_FROM; only FROM =>
            // IGNORE_TO; both => ROOT_TO_ROOT/no mutation; neither => preserve vanilla.
            static void Thunk(
                void* self,
                const RE::hkbContext& context,
                const void* const* activeChildrenOutput,
                void* output,
                float timeOffset)
            {
                auto* transition = static_cast<BlendingTransitionEffectView*>(self);
                if (!transition) {
                    Func(self, context, activeChildrenOutput, output, timeOffset);
                    return;
                }

                const std::uint16_t flagsBefore = transition->flags;
                const bool runtimeSidesReady =
                    transition->fromGenerator != nullptr &&
                    transition->toGenerator != nullptr;

                const bool childrenReadable = activeChildrenOutput &&
                    IsExactRangeAccessible(activeChildrenOutput, 2 * sizeof(void*), ExactMemoryAccess::kRead);
                if (activeChildrenOutput && !childrenReadable) {
                    LogR65PointerReject("ACTIVE_CHILDREN_ARRAY", activeChildrenOutput, 2 * sizeof(void*));
                }
                const void* child0Output = childrenReadable ? activeChildrenOutput[0] : nullptr;
                const void* child1Output = childrenReadable ? activeChildrenOutput[1] : nullptr;
                const GeneratorOutputProbe child0Probe = ProbeGeneratorOutput(child0Output);
                const GeneratorOutputProbe child1Probe = ProbeGeneratorOutput(child1Output);

                const TransitionWfmAnchors anchorsBefore =
                    ProbeTransitionWfmAnchors(transition);
                const TransitionSideMapProbe preMap =
                    ResolveTransitionSideMapping(child0Probe, child1Probe, anchorsBefore);

                const RE::FormID formID = ResolveActorFormIDForContext(context);
                std::uint64_t frame = 0;
                RuntimeWfmPolicy runtimePolicy = RuntimeWfmPolicy::kNone;
                bool runtimeEligible = false;
                bool serializedPolicy = false;
                bool dbSessionActive = false;
                bool syntheticInjectedCurrent = false;
                bool mapAmbiguous = false;
                bool usedCachedMap = false;
                bool dbEntryPending = false;
                bool dbOwnershipEstablished = false;
                bool fromDbRoot = false;
                bool toDbRoot = false;
                SyntheticWfmSideProbe rawChild0Side{};
                SyntheticWfmSideProbe rawChild1Side{};
                // Skyrim/Behavior child ABI validated by the R15.4/R15.5 corpus:
                // activeChildrenOutput[0] is TO and [1] is FROM.
                TransitionSideMapping semanticMapping =
                    TransitionSideMapping::kChild1FromChild0To;
                const char* mapSource = "ABI_CHILD0_TO_CHILD1_FROM";
                SyntheticWfmSideProbe fromSide{};
                SyntheticWfmSideProbe toSide{};
                std::uint16_t flagsApplied = flagsBefore;

                if (formID != 0) {
                    std::scoped_lock lock{ g_mutex };
                    const auto actorIt = g_actorState.find(formID);
                    if (actorIt != g_actorState.end()) {
                        auto& state = actorIt->second;
                        frame = state.frameSerial;
                        ++state.wfmTransitionCallsFrame;
                        ++state.wfmTransitionCallsTotal;

                        // Only serialized/native bits present before our write may
                        // participate in OPEN_NATIVE admission/continuity.
                        if ((flagsBefore & kWfmIgnoreFrom) != 0) {
                            state.wfmIgnoreFromSeenFrame = state.frameSerial;
                        }
                        if ((flagsBefore & kWfmIgnoreTo) != 0) {
                            state.wfmIgnoreToSeenFrame = state.frameSerial;
                        }

                        serializedPolicy =
                            (flagsBefore & kWfmTranslationPolicyMask) != 0;
                        dbSessionActive = state.creatureDbSessionActive;
                        syntheticInjectedCurrent =
                            state.syntheticWfmTranslationSeenFrame == state.frameSerial;

                        // Anchor mapping remains a runtime ABI cross-check. We do
                        // not need anchors to operate, but if Skyrim ever exposes them
                        // and they disagree with child0=TO/child1=FROM, fail CLOSED.
                        if (preMap.mapping != TransitionSideMapping::kUnknown) {
                            auto& cached = state.transitionSideMaps[self];
                            cached.mapping = preMap.mapping;
                            cached.lastSeenFrame = state.frameSerial;
                            cached.fromGenerator = transition->fromGenerator;
                            cached.toGenerator = transition->toGenerator;
                            if (preMap.mapping == TransitionSideMapping::kChild0FromChild1To) {
                                ++state.wfmSideMapResolvedDirectTotal;
                            } else {
                                ++state.wfmSideMapResolvedSwappedTotal;
                            }
                            if (preMap.mapping != semanticMapping) {
                                mapAmbiguous = true;
                                ++state.wfmSideMapDisagreementTotal;
                                ++state.creatureDbProvenanceAmbiguous;
                                mapSource = "ABI_ANCHOR_DISAGREEMENT";
                            } else {
                                mapSource = "ABI_CONFIRMED_BY_ANCHOR";
                            }
                        }

                        rawChild0Side = ProbeSyntheticWfmSideLocked(state, child0Probe);
                        rawChild1Side = ProbeSyntheticWfmSideLocked(state, child1Probe);

                        // ABI: child0=TO, child1=FROM. These are TRANSITION-LOCAL
                        // provenance facts, not actor-global root observations.
                        toSide = rawChild0Side;
                        fromSide = rawChild1Side;
                        toDbRoot = toSide.syntheticRoot;
                        fromDbRoot = fromSide.syntheticRoot;

                        dbEntryPending = state.creatureDbEntryPending;
                        dbOwnershipEstablished = state.creatureDbOwnershipEstablished;
                        runtimeEligible =
                            !state.isPlayer && runtimeSidesReady && !serializedPolicy &&
                            syntheticInjectedCurrent && !mapAmbiguous &&
                            (fromDbRoot || toDbRoot);

                        if (runtimeEligible) {
                            state.creatureDbProvenanceSeenFrame = state.frameSerial;

                            if (!fromDbRoot && toDbRoot) {
                                // INPLACE -> DB ROOT: ignore FROM WFM so TO owns root.
                                runtimePolicy = RuntimeWfmPolicy::kIgnoreFrom;
                                ++state.wfmInplaceRootTotal;
                                flagsApplied = static_cast<std::uint16_t>(
                                    (flagsBefore & ~kWfmTranslationPolicyMask) |
                                    kWfmIgnoreFrom);
                            } else if (fromDbRoot && !toDbRoot) {
                                // DB ROOT -> INPLACE: ignore TO WFM so FROM owns root
                                // for the duration of the blend-out.
                                runtimePolicy = RuntimeWfmPolicy::kIgnoreTo;
                                ++state.wfmRootInplaceTotal;
                                flagsApplied = static_cast<std::uint16_t>(
                                    (flagsBefore & ~kWfmTranslationPolicyMask) |
                                    kWfmIgnoreTo);
                            } else {
                                // DB ROOT -> DB ROOT: let Havok blend both roots.
                                runtimePolicy = RuntimeWfmPolicy::kRootToRoot;
                                ++state.wfmRootRootTotal;
                                ++state.creatureDbProvenanceRootToRoot;
                            }

                            if (flagsApplied != flagsBefore) {
                                ++state.wfmTransitionChangesFrame;
                                ++state.wfmTransitionChangesTotal;
                                ++state.wfmRuntimeInjectedTotal;
                                ++state.creatureDbProvenancePolicyWrites;
                                if (runtimePolicy == RuntimeWfmPolicy::kIgnoreFrom) {
                                    state.wfmRuntimeIgnoreFromSeenFrame = state.frameSerial;
                                    ++state.wfmRuntimeIgnoreFromTotal;
                                    // Only a proven INPLACE -> DB ROOT transition may
                                    // satisfy acquisition. ROOT -> INPLACE is exit
                                    // continuity and must never certify entry ownership.
                                    if (state.creatureDbEntryProvenanceFrame != state.frameSerial) {
                                        state.creatureDbEntryPolicyWrites = 0;
                                    }
                                    state.creatureDbEntryProvenanceFrame = state.frameSerial;
                                    ++state.creatureDbEntryPolicyWrites;
                                } else if (runtimePolicy == RuntimeWfmPolicy::kIgnoreTo) {
                                    state.wfmRuntimeIgnoreToSeenFrame = state.frameSerial;
                                    ++state.wfmRuntimeIgnoreToTotal;
                                }
                            }
                        }

                        if (child0Probe.rootClass == RootOutputClass::kUnknown ||
                            child1Probe.rootClass == RootOutputClass::kUnknown) {
                            ++state.wfmOutputUnknownTotal;
                        } else {
                            ++state.wfmOutputClassifiedTotal;
                        }
                    }
                }

                const auto* generator = reinterpret_cast<RE::hkbGenerator*>(self);
                const std::string effectName =
                    generator && generator->name.c_str() ? generator->name.c_str() : "";
                const std::string fromName =
                    transition->fromGenerator && transition->fromGenerator->name.c_str() ?
                    transition->fromGenerator->name.c_str() : "";
                const std::string toName =
                    transition->toGenerator && transition->toGenerator->name.c_str() ?
                    transition->toGenerator->name.c_str() : "";

                const bool nativeRootActionExit =
                    formID != 0 &&
                    ContainsInsensitive(toName, "DefaultBehavior") &&
                    (ContainsInsensitive(fromName, "Attack") ||
                     ContainsInsensitive(fromName, "Stagger") ||
                     ContainsInsensitive(fromName, "Recoil"));
                if (nativeRootActionExit) {
                    bool newExitEpisode = false;
                    std::uint64_t witnessSerial = 0;
                    {
                        std::scoped_lock lock{ g_mutex };
                        auto& witness = g_nativeRootExitWitnesses[formID];
                        const auto transitionPtr = reinterpret_cast<std::uintptr_t>(self);
                        newExitEpisode =
                            witness.lastSeenFrame == 0 ||
                            witness.transitionPtr != transitionPtr ||
                            frame > witness.lastSeenFrame + 1;
                        witness.transitionPtr = transitionPtr;
                        witness.lastSeenFrame = frame;
                        if (newExitEpisode) {
                            witness.pending = true;
                            witnessSerial = ++witness.serial;
                        }
                    }
                    if (newExitEpisode) {
                        SKSE::log::info(
                            R"([NativeRootExit][R42_WITNESS] form={:08X} frame={} serial={} effect="{}" from="{}" to="{}" transition=0x{:X} action=ARM_GAIT_REENTRY_NEXT_CHARACTER_UPDATE)",
                            formID, frame, witnessSerial, effectName, fromName, toName,
                            reinterpret_cast<std::uintptr_t>(self));
                    }
                }

                const bool locomotionToStandingExit =
                    formID != 0 &&
                    fromName == "LocomotionBehavior" &&
                    toName == "StandingBehavior";
                if (locomotionToStandingExit) {
                    bool newExitEpisode = false;
                    std::uint64_t witnessSerial = 0;
                    std::uint64_t firstSeenFrame = frame;
                    {
                        std::scoped_lock lock{ g_mutex };
                        auto& witness = g_locomotionStandingWitnesses[formID];
                        const auto transitionPtr = reinterpret_cast<std::uintptr_t>(self);
                        newExitEpisode =
                            witness.lastSeenFrame == 0 ||
                            frame > witness.lastSeenFrame + kLocomotionStandingEpisodeGapFrames;
                        if (newExitEpisode) {
                            witness.pending = true;
                            witness.firstSeenFrame = frame;
                            witnessSerial = ++witness.serial;
                        }
                        witness.transitionPtr = transitionPtr;
                        witness.lastSeenFrame = frame;
                        firstSeenFrame = witness.firstSeenFrame;
                    }
                    if (newExitEpisode) {
                        SKSE::log::info(
                            R"([LocomotionDrop][R46_WITNESS] form={:08X} frame={} serial={} firstSeenFrame={} effect="{}" from="{}" to="{}" transition=0x{:X} action=ARM_RECENT_STANDING_DROP_WITNESS)",
                            formID, frame, witnessSerial, firstSeenFrame, effectName, fromName, toName,
                            reinterpret_cast<std::uintptr_t>(self));
                    }
                }

                // Runtime mutation exists only while the transition owner Generate
                // executes. SYNC, IGNORE_TO_ROTATION and unknown bits are preserved.
                const bool runtimeWrite = flagsApplied != flagsBefore;
                if (runtimeWrite) {
                    transition->flags = flagsApplied;
                }
                Func(self, context, activeChildrenOutput, output, timeOffset);
                const std::uint16_t flagsSeenAfterOwner = transition->flags;
                if (runtimeWrite) {
                    transition->flags = flagsBefore;
                }

                const GeneratorOutputProbe blendedProbe = ProbeGeneratorOutput(output);
                const TransitionWfmAnchors anchorsAfter =
                    ProbeTransitionWfmAnchors(transition);
                const TransitionSideMapProbe postMap =
                    ResolveTransitionSideMapping(child0Probe, child1Probe, anchorsAfter);

                bool postCalibrated = false;
                bool mapDisagreement = false;
                if (formID != 0 && postMap.mapping != TransitionSideMapping::kUnknown) {
                    std::scoped_lock lock{ g_mutex };
                    const auto actorIt = g_actorState.find(formID);
                    if (actorIt != g_actorState.end()) {
                        auto& state = actorIt->second;
                        auto& cached = state.transitionSideMaps[self];
                        if (semanticMapping != TransitionSideMapping::kUnknown &&
                            semanticMapping != postMap.mapping) {
                            mapDisagreement = true;
                            ++state.wfmSideMapDisagreementTotal;
                        }
                        cached.mapping = postMap.mapping;
                        cached.lastSeenFrame = state.frameSerial;
                        cached.fromGenerator = transition->fromGenerator;
                        cached.toGenerator = transition->toGenerator;
                        ++state.wfmSideMapPostCalibratedTotal;
                        postCalibrated = true;
                    }
                }

                if (formID != 0) {
                    const std::uint16_t unknownBefore =
                        static_cast<std::uint16_t>(flagsBefore & ~kWfmKnownMask);
                    const bool nativeIgnoreFrom = (flagsBefore & kWfmIgnoreFrom) != 0;
                    const bool nativeIgnoreTo = (flagsBefore & kWfmIgnoreTo) != 0;
                    const bool runtimeChanged = flagsApplied != flagsBefore;
                    const bool emit =
                        runtimeChanged || serializedPolicy || frame <= 5 ||
                        (frame != 0 && (frame % 240) == 0);

                    if (runtimeChanged) {
                        SKSE::log::debug(R"([WFM][DB_PROVENANCE_POLICY] form={:08X} frame={} effect="{}" ptr=0x{:X} from="{}" to="{}" policy={} proof={} provenance[FROM_DB={} TO_DB={}] rawChildABI[child0_TO_root={} child1_FROM_root={}] session[active={} entryPending={} owned={}] flags[before=0x{:04X} applied=0x{:04X} ownerAfter=0x{:04X} restored=0x{:04X}] note="transition-local MotionDB root provenance; no actor-global prearm")",
                            formID,
                            frame,
                            effectName,
                            reinterpret_cast<std::uintptr_t>(self),
                            fromName,
                            toName,
                            RuntimeWfmPolicyName(runtimePolicy),
                            mapSource,
                            fromDbRoot,
                            toDbRoot,
                            rawChild0Side.syntheticRoot,
                            rawChild1Side.syntheticRoot,
                            dbSessionActive,
                            dbEntryPending,
                            dbOwnershipEstablished,
                            flagsBefore,
                            flagsApplied,
                            flagsSeenAfterOwner,
                            transition->flags);
                    }

                    if (mapAmbiguous) {
                        SKSE::log::info(
                            R"([WFM][SIDE_MAP_AMBIGUOUS] form={:08X} frame={} effect="{}" ptr=0x{:X} from="{}" to="{}" preAnchors[readable={} size={} from=({:+.5f},{:+.5f}) to=({:+.5f},{:+.5f})] scores[direct={:.6f} swapped={:.6f} confidence={:.6f}] action=FAIL_CLOSED_PRESERVE_VANILLA_FLAGS)",
                            formID,
                            frame,
                            effectName,
                            reinterpret_cast<std::uintptr_t>(self),
                            fromName,
                            toName,
                            anchorsBefore.readable,
                            anchorsBefore.size,
                            anchorsBefore.fromX,
                            anchorsBefore.fromY,
                            anchorsBefore.toX,
                            anchorsBefore.toY,
                            preMap.directScore,
                            preMap.swappedScore,
                            preMap.confidence);
                    }

                    if (emit) {
                        SKSE::log::debug(R"([WFM][SIDE_MAP] form={:08X} frame={} effect="{}" ptr=0x{:X} selected={} source={} usedCache={} pre[anchors={} size={} from=({:+.5f},{:+.5f}) to=({:+.5f},{:+.5f}) direct={:.6f} swapped={:.6f} confidence={:.6f}] post[selected={} calibrated={} disagreement={} anchors={} size={} from=({:+.5f},{:+.5f}) to=({:+.5f},{:+.5f}) direct={:.6f} swapped={:.6f} confidence={:.6f}])",
                            formID,
                            frame,
                            effectName,
                            reinterpret_cast<std::uintptr_t>(self),
                            TransitionSideMappingName(semanticMapping),
                            mapSource,
                            usedCachedMap,
                            anchorsBefore.readable,
                            anchorsBefore.size,
                            anchorsBefore.fromX,
                            anchorsBefore.fromY,
                            anchorsBefore.toX,
                            anchorsBefore.toY,
                            preMap.directScore,
                            preMap.swappedScore,
                            preMap.confidence,
                            TransitionSideMappingName(postMap.mapping),
                            postCalibrated,
                            mapDisagreement,
                            anchorsAfter.readable,
                            anchorsAfter.size,
                            anchorsAfter.fromX,
                            anchorsAfter.fromY,
                            anchorsAfter.toX,
                            anchorsAfter.toY,
                            postMap.directScore,
                            postMap.swappedScore,
                            postMap.confidence);

                        SKSE::log::debug(R"([WFM][TRANSITION] form={:08X} frame={} effect="{}" ptr=0x{:X} from="{}" fromPtr=0x{:X} to="{}" toPtr=0x{:X} sidesReady={} abi[children=0x{:X} child0Out=0x{:X} child1Out=0x{:X} output=0x{:X} timeOffset={:.6f}] duration={:.6f} toStart={:.6f} flagsBefore=0x{:04X}[IGNORE_FROM={} SYNC={} IGNORE_TO={} IGNORE_TO_ROT={} UNKNOWN=0x{:04X}] flagsApplied=0x{:04X} flagsOwnerAfter=0x{:04X} flagsRestored=0x{:04X} policy={} serializedPolicy={} syntheticInjectedCurrent={} semanticMap={} childWfmOffset[FROM=({:+.5f},{:+.5f}) root={} TO=({:+.5f},{:+.5f}) root={}] rawChildWfm[0=({:+.5f},{:+.5f}) 1=({:+.5f},{:+.5f})] childDelta[0={} 1={}] blendedDelta={} action={})",
                            formID,
                            frame,
                            effectName,
                            reinterpret_cast<std::uintptr_t>(self),
                            fromName,
                            reinterpret_cast<std::uintptr_t>(transition->fromGenerator),
                            toName,
                            reinterpret_cast<std::uintptr_t>(transition->toGenerator),
                            runtimeSidesReady,
                            reinterpret_cast<std::uintptr_t>(activeChildrenOutput),
                            reinterpret_cast<std::uintptr_t>(child0Output),
                            reinterpret_cast<std::uintptr_t>(child1Output),
                            reinterpret_cast<std::uintptr_t>(output),
                            timeOffset,
                            transition->duration,
                            transition->toGeneratorStartTimeFraction,
                            flagsBefore,
                            nativeIgnoreFrom,
                            (flagsBefore & kWfmSync) != 0,
                            nativeIgnoreTo,
                            (flagsBefore & kWfmIgnoreToRotation) != 0,
                            unknownBefore,
                            flagsApplied,
                            flagsSeenAfterOwner,
                            transition->flags,
                            RuntimeWfmPolicyName(runtimePolicy),
                            serializedPolicy,
                            syntheticInjectedCurrent,
                            TransitionSideMappingName(semanticMapping),
                            fromSide.offsetX,
                            fromSide.offsetY,
                            fromSide.syntheticRoot,
                            toSide.offsetX,
                            toSide.offsetY,
                            toSide.syntheticRoot,
                            child0Probe.worldFromModel.x,
                            child0Probe.worldFromModel.y,
                            child1Probe.worldFromModel.x,
                            child1Probe.worldFromModel.y,
                            RootOutputClassName(child0Probe.rootClass),
                            RootOutputClassName(child1Probe.rootClass),
                            RootOutputClassName(blendedProbe.rootClass),
                            runtimeChanged ? "RUNTIME_WFM_INJECTED" :
                                (serializedPolicy ? "SERIALIZED_WFM_PRESERVED" :
                                    (mapAmbiguous ? "AMBIGUOUS_NO_POLICY_CHANGE" : "NO_POLICY_CHANGE")));
                    }
                }
            }

            static inline REL::Relocation<decltype(Thunk)> Func;
        };

        [[nodiscard]] std::string FixedString(const RE::BSFixedString* value)
        {
            if (!value) {
                return {};
            }
            const char* raw = value->c_str();
            return raw ? std::string{ raw } : std::string{};
        }

        [[nodiscard]] float QuaternionYaw(const RE::NiQuaternion& q) noexcept
        {
            const float siny = 2.0f * (q.w * q.z + q.x * q.y);
            const float cosy = 1.0f - 2.0f * (q.y * q.y + q.z * q.z);
            return std::atan2(siny, cosy);
        }

        [[nodiscard]] std::uintptr_t DecodeCallTarget(std::uintptr_t callSite) noexcept
        {
            if (!callSite || *reinterpret_cast<const std::uint8_t*>(callSite) != 0xE8) {
                return 0;
            }
            std::int32_t displacement = 0;
            std::memcpy(&displacement, reinterpret_cast<const void*>(callSite + 1), sizeof(displacement));
            return callSite + 5 + static_cast<std::intptr_t>(displacement);
        }

        [[nodiscard]] RuntimeSites HistoricalSites(std::uintptr_t base) noexcept
        {
            RuntimeSites out{};
            if (!base) {
                return out;
            }
            if (REL::Module::IsAE()) {
                out.translation1 = base + 0x298;
                out.translation2 = base + 0x2AA;
                out.rotation1 = base + 0x35C;
                out.rotation2 = base + 0x36D;
            } else {
                out.translation1 = base + 0x28D;
                out.translation2 = base + 0x2A1;
                out.rotation1 = base + 0x355;
                out.rotation2 = base + 0x368;
            }
            return out;
        }

        template <class Fn>
        [[nodiscard]] void* BuildChainedDispatchStub(
            SKSE::Trampoline& trampoline,
            std::uintptr_t currentTarget,
            Fn afterHook,
            bool ae)
        {
            std::array<std::uint8_t, kStubCapacity> code{};
            std::size_t n = 0;
            auto emit8 = [&](std::uint8_t value) {
                code[n++] = value;
            };
            auto emit64 = [&](std::uint64_t value) {
                std::memcpy(code.data() + n, &value, sizeof(value));
                n += sizeof(value);
            };

            // Entry still has the exact Character::ProcessMotionData register state.
            // Preserve the original first 3 sampler arguments, call whatever target
            // is currently installed at the call site (vanilla, AMR, or another
            // compatible wrapper), restore the arguments, then call our 5-argument
            // post-sampler hook. This is a chain, not a replacement.
            emit8(0x48); emit8(0x83); emit8(0xEC); emit8(0x58);                         // sub rsp,58h
            emit8(0x48); emit8(0x89); emit8(0x4C); emit8(0x24); emit8(0x30);           // mov [rsp+30h],rcx
            emit8(0x4C); emit8(0x89); emit8(0x44); emit8(0x24); emit8(0x38);           // mov [rsp+38h],r8
            emit8(0xF3); emit8(0x0F); emit8(0x11); emit8(0x4C); emit8(0x24); emit8(0x40); // movss [rsp+40h],xmm1
            emit8(0x48); emit8(0x89); emit8(0x5C); emit8(0x24); emit8(0x48);           // mov [rsp+48h],rbx
            emit8(0x4C); emit8(0x89); emit8(0x6C); emit8(0x24); emit8(0x50);           // mov [rsp+50h],r13

            emit8(0x48); emit8(0xB8); emit64(static_cast<std::uint64_t>(currentTarget)); // mov rax,target
            emit8(0xFF); emit8(0xD0);                                                   // call rax

            emit8(0x48); emit8(0x8B); emit8(0x4C); emit8(0x24); emit8(0x30);           // mov rcx,[rsp+30h]
            emit8(0x4C); emit8(0x8B); emit8(0x44); emit8(0x24); emit8(0x38);           // mov r8,[rsp+38h]
            emit8(0xF3); emit8(0x0F); emit8(0x10); emit8(0x4C); emit8(0x24); emit8(0x40); // movss xmm1,[rsp+40h]
            emit8(0x48); emit8(0x8B); emit8(0x5C); emit8(0x24); emit8(0x48);           // mov rbx,[rsp+48h]
            emit8(0x4C); emit8(0x8B); emit8(0x6C); emit8(0x24); emit8(0x50);           // mov r13,[rsp+50h]

            emit8(0x4C); emit8(0x89); emit8(0x6C); emit8(0x24); emit8(0x20);           // mov [rsp+20h],r13
            emit8(0x49); emit8(0x89); emit8(0xD9);                                     // mov r9,rbx
            if (ae) {
                emit8(0x49); emit8(0x83); emit8(0xE9); emit8(0x14);                     // sub r9,14h
            }
            emit8(0x48); emit8(0xB8);
            emit64(static_cast<std::uint64_t>(reinterpret_cast<std::uintptr_t>(afterHook)));
            emit8(0xFF); emit8(0xD0);                                                   // call rax
            emit8(0x48); emit8(0x83); emit8(0xC4); emit8(0x58);                         // add rsp,58h
            emit8(0xC3);                                                                 // ret

            if (n > code.size()) {
                return nullptr;
            }
            void* memory = trampoline.allocate(n);
            std::memcpy(memory, code.data(), n);
            return memory;
        }

        [[nodiscard]] bool LookupClipInfo(
            const RE::hkbCharacter* character,
            const std::string& clipName,
            ClipInfo& out,
            float& ageSeconds)
        {
            if (!character || clipName.empty()) {
                return false;
            }
            const auto now = Clock::now();
            std::scoped_lock lock{ g_mutex };
            const auto it = g_clipInfo.find(ClipKey{ character, clipName });
            if (it == g_clipInfo.end()) {
                return false;
            }
            out = it->second;
            ageSeconds = it->second.seen == Clock::time_point{} ?
                std::numeric_limits<float>::infinity() :
                std::chrono::duration<float>(now - it->second.seen).count();
            return true;
        }

        [[nodiscard]] bool ShouldLogForClip(
            const std::string& clipName,
            const ClipInfo& info,
            bool mapped) noexcept
        {
            return IsRelevantClip(clipName, mapped ? info.animationName : std::string{});
        }

        // Caller must hold g_mutex. PlayerCharacter may own more than one live
        // BShkbAnimationGraph/hkbCharacter (notably separate graph instances), so
        // ProcessMotionData lookup must search every graph character captured for
        // the actor instead of assuming activeGraph is the clip owner.
        [[nodiscard]] bool FindClipInfoForActorStateLocked(
            const NativeActorState& state,
            const std::string& clipName,
            ClipInfo& out)
        {
            const ClipInfo* best = nullptr;
            auto consider = [&](const RE::hkbCharacter* hkbCharacter) {
                if (!hkbCharacter) {
                    return;
                }
                const auto it = g_clipInfo.find(ClipKey{ hkbCharacter, clipName });
                if (it == g_clipInfo.end()) {
                    return;
                }
                if (!best || it->second.seenFrameSerial > best->seenFrameSerial) {
                    best = std::addressof(it->second);
                }
            };

            consider(state.hkbCharacter);
            for (std::size_t i = 0; i < state.hkbCharacterCount && i < state.hkbCharacters.size(); ++i) {
                consider(state.hkbCharacters[i]);
            }
            if (!best) {
                return false;
            }
            out = *best;
            return true;
        }

        [[nodiscard]] bool BuildExternalComparison(
            const ClipInfo& info,
            bool mapped,
            float age,
            float motionTime,
            TranslationHalf& out)
        {
            (void)age;
            if (!mapped || !info.databaseBacked || !std::isfinite(motionTime)) {
                return false;
            }
            const auto sample = AnimationMotionDatabase::SampleCumulativeSeconds(info.profile, info.animationName, motionTime);
            if (!sample.valid) {
                return false;
            }
            out.externalValid = true;
            out.external = { sample.x, sample.y, sample.z };
            out.key = sample.key;
            return true;
        }

        [[nodiscard]] bool ResolveExternalSample(
            RE::Character* character,
            float motionTime,
            const RE::BSFixedString* clipNameValue,
            AnimationMotionDatabase::CumulativeSample& out)
        {
            if (!CanApplyExternalMotion(character) || !std::isfinite(motionTime)) {
                return false;
            }
            // Keep the Player path byte-policy conservative in this creature test build.
            // Non-player MotionDB clips accept Skyrim's native negative pre-roll time.
            if (character->IsPlayerRef() && motionTime < 0.0f) {
                return false;
            }

            const std::string clipName = FixedString(clipNameValue);
            if (clipName.empty()) {
                return false;
            }
            ClipInfo info{};
            {
                std::scoped_lock lock{ g_mutex };
                const auto actorIt = g_actorState.find(character->GetFormID());
                if (actorIt == g_actorState.end() || !actorIt->second.hkbCharacter ||
                    actorIt->second.frameSerial == 0) {
                    return false;
                }
                const auto& actorState = actorIt->second;
                if (!FindClipInfoForActorStateLocked(actorState, clipName, info) ||
                    !IsClipMappingValidForInjection(character, actorState, info)) {
                    return false;
                }
            }

            // Creature clip authority is MotionDB membership itself. Do not gate the
            // authored sample through WALK/TROT/RUN ownership: turns, attacks and
            // every other DB-backed creature animation are equally eligible.

            out = AnimationMotionDatabase::SampleCumulativeSeconds(info.profile, info.animationName, motionTime);
            return out.valid;
        }

        [[nodiscard]] const char* FixedStringData(const RE::BSFixedString* value) noexcept
        {
            return value ? value->c_str() : nullptr;
        }

        [[nodiscard]] bool CacheMatches(
            RE::Character* character,
            const RE::BSFixedString* clipNameValue) noexcept
        {
            const char* raw = FixedStringData(clipNameValue);
            return character && raw && g_invocationCache.character == character &&
                   g_invocationCache.clipName == raw && g_invocationCache.mapped;
        }

        [[nodiscard]] bool ResolveCachedExternalSample(
            Site site,
            RE::Character* character,
            float motionTime,
            const RE::BSFixedString* clipNameValue,
            AnimationMotionDatabase::CumulativeSample& out)
        {
            if (!CanApplyExternalMotion(character) || !std::isfinite(motionTime)) {
                return false;
            }
            // Creature/non-player path accepts native negative pre-roll. MotionDB clamps
            // cumulative samples to the authored timeline; ProcessMotionData owns t0/t1.
            if (character->IsPlayerRef() && motionTime < 0.0f) {
                return false;
            }

            const char* raw = FixedStringData(clipNameValue);
            if (!raw || !*raw) {
                return false;
            }

            if (site == Site::kTranslation1) {
                ProbeMotionPmdT1(character, motionTime, raw);
                g_invocationCache = {};
                g_invocationCache.character = character;
                g_invocationCache.clipName = raw;

                ClipInfo info{};
                {
                    std::scoped_lock lock{ g_mutex };
                    const auto actorIt = g_actorState.find(character->GetFormID());
                    if (actorIt == g_actorState.end() || !actorIt->second.hkbCharacter ||
                        actorIt->second.frameSerial == 0) {
                        return false;
                    }
                    const auto& actorState = actorIt->second;
                    if (!FindClipInfoForActorStateLocked(actorState, std::string{ raw }, info) ||
                        !IsClipMappingValidForInjection(character, actorState, info)) {
                        return false;
                    }
                    g_invocationCache.frameSerial = actorState.frameSerial;
                }

                // Creature clip authority is MotionDB membership itself. The cached
                // ProcessMotionData path therefore accepts every DB-backed creature clip.

                g_invocationCache.profile = info.profile;
                g_invocationCache.animationName = std::move(info.animationName);
                g_invocationCache.mapped = true;
                g_invocationCache.t1 =
                    AnimationMotionDatabase::SampleCumulativeSeconds(g_invocationCache.profile, g_invocationCache.animationName, motionTime);
                g_invocationCache.t1Valid = g_invocationCache.t1.valid;
                g_invocationCache.t1Time = motionTime;

                // R15.8: PMD remains forbidden from acquisition. AFTER ownership
                // is strongly acquired, a READY_DB_T1 for a key that belongs to the
                // acquired root set is a continuity heartbeat for that ownership.
                // Arbitrary DB leaves are telemetry only and cannot sustain it.
                if (!character->IsPlayerRef() && g_invocationCache.t1Valid) {
                    bool logHeartbeat = false;
                    bool continuityMatch = false;
                    std::uint64_t heartbeatFrame = 0;
                    std::uint64_t heartbeatHits = 0;
                    std::size_t ownershipKeyCount = 0;
                    std::string heartbeatKey = g_invocationCache.t1.key;
                    {
                        std::scoped_lock lock{ g_mutex };
                        auto& state = g_actorState[character->GetFormID()];
                        const bool firstThisFrame =
                            state.creatureDbPmdSourceFrame != state.frameSerial;
                        const bool keyChanged =
                            !heartbeatKey.empty() && state.creatureDbPmdSourceKey != heartbeatKey;
                        state.creatureDbPmdSourceFrame = state.frameSerial;
                        if (!heartbeatKey.empty()) {
                            state.creatureDbPmdSourceKey = heartbeatKey;
                        }
                        heartbeatFrame = state.frameSerial;
                        heartbeatHits = ++state.creatureDbPmdSourceHits;
                        ownershipKeyCount = state.creatureDbOwnershipKeyCount;
                        continuityMatch =
                            state.creatureDbOwnershipEstablished &&
                            OwnershipContainsDbKeyLocked(state, heartbeatKey);
                        if (continuityMatch) {
                            state.creatureDbPmdContinuityFrame = state.frameSerial;
                        }
                        logHeartbeat =
                            state.creatureDbOwnershipEstablished &&
                            (state.creatureDbPmdHeartbeatLogs < 16 || keyChanged ||
                             (firstThisFrame && (heartbeatFrame % 240) == 0));
                        if (logHeartbeat) {
                            ++state.creatureDbPmdHeartbeatLogs;
                        }
                    }
                    if (logHeartbeat) {
                        SKSE::log::debug(R"([DB_SOURCE][{}] form={:08X} frame={} clip="{}" key="{}" motionTime={:.6f} hits={} ownershipKeys={} match={} proof=READY_DB_T1 note="PMD can sustain only an already-acquired matching DB ownership; never acquires")",
                            continuityMatch ? "PMD_CONTINUITY" : "PMD_TELEMETRY",
                            character->GetFormID(), heartbeatFrame, raw, heartbeatKey,
                            motionTime, heartbeatHits, ownershipKeyCount, continuityMatch);
                    }
                }

                out = g_invocationCache.t1;
                return g_invocationCache.t1Valid;
            }

            if (CacheMatches(character, clipNameValue)) {
                if (site == Site::kTranslation2) {
                    g_invocationCache.t2 =
                        AnimationMotionDatabase::SampleCumulativeSeconds(g_invocationCache.profile, g_invocationCache.animationName, motionTime);
                    g_invocationCache.t2Valid = g_invocationCache.t2.valid;
                    g_invocationCache.t2Time = motionTime;
                    out = g_invocationCache.t2;
                    return g_invocationCache.t2Valid;
                }
                if (site == Site::kRotation1 && g_invocationCache.t1Valid &&
                    g_invocationCache.t1Time == motionTime) {
                    out = g_invocationCache.t1;
                    return true;
                }
                if (site == Site::kRotation2 && g_invocationCache.t2Valid &&
                    g_invocationCache.t2Time == motionTime) {
                    out = g_invocationCache.t2;
                    return true;
                }
            }

            // Fail soft if another owner changes call ordering: resolve this sample
            // independently and preserve the R1-45 behavior instead of injecting stale data.
            return ResolveExternalSample(character, motionTime, clipNameValue, out);
        }

        void ApplyExternalTranslation(
            Site site,
            RE::Character* character,
            float motionTime,
            const RE::BSFixedString* clipName,
            RE::NiPoint3& translation)
        {
            AnimationMotionDatabase::CumulativeSample sample{};
            if (!ResolveCachedExternalSample(site, character, motionTime, clipName, sample)) {
                return;
            }

            // R85 canonical-HKX contract (restored from the stable pre-Troll path):
            // creature translation has exactly ONE physical transport: the MotionDB trajectory
            // resolved from the clip's real animationName/HKX and injected into that leaf's
            // WORLD_FROM_MODEL. Node names such as H2H.../Slow/Fast are aliases only. Havok
            // decides their actual physical participation through the parent WFM tree/masks.
            // ProcessMotionData T1/T2 must NOT become a second XY transport.
            bool emitBypass = false;
            std::uint64_t frame = 0;
            std::uint64_t bypasses = 0;
            {
                std::scoped_lock lock{ g_mutex };
                auto& state = g_actorState[character->GetFormID()];
                frame = state.frameSerial;
                bypasses = ++state.syntheticWfmBypassTxTotal;
                emitBypass = bypasses <= 16 || (bypasses % 240) == 0;
            }
            if (emitBypass) {
                SKSE::log::debug(R"([MotionDB][TX_BYPASS_WFM] form={:08X} frame={} site={} clip="{}" key="{}" motionTime={:.6f} resolved=({:+.5f},{:+.5f},{:+.5f}) action=PRESERVE_NATIVE_T1_T2 note="creature translation authority is synthetic clip WFM; prevents double transport")",
                    character->GetFormID(),
                    frame,
                    site == Site::kTranslation1 ? "T1" : "T2",
                    FixedString(clipName),
                    sample.key,
                    motionTime,
                    sample.x,
                    sample.y,
                    sample.z);
            }
            (void)translation;
        }

        void ApplyExternalRotation(
            Site site,
            RE::Character* character,
            float motionTime,
            const RE::BSFixedString* clipName,
            RE::NiQuaternion& rotation)
        {
            AnimationMotionDatabase::CumulativeSample sample{};
            const bool resolved = ResolveCachedExternalSample(site, character, motionTime, clipName, sample);

            const bool creatureProfileActor =
                character && !character->IsPlayerRef() && MotionProfileDatabase::SupportsActor(character);

            // R104 diagnostic-only: unresolved native Troll turns are exactly the calls that
            // disappeared from the old telemetry because ResolveCachedExternalSample returned
            // false. Observe their ORIGINAL ProcessMotionData quaternion before any Incessus
            // authority decision. No writes are performed here. Profile rotationClips provide
            // the semantic key; node aliases such as MTTurnRight90Slow and .HKX00 are accepted
            // by prefix so this remains profile-driven rather than species-hardcoded.
            std::string r104RotationKey;
            bool r104SemanticRotationClip = false;
            if (creatureProfileActor && clipName) {
                if (const auto* motionProfile = MotionProfileDatabase::ResolveProfile(character)) {
                    std::string node = FixedString(clipName);
                    std::transform(node.begin(), node.end(), node.begin(), [](char c) { return LowerASCII(c); });
                    for (const auto& key : motionProfile->rotationClips) {
                        if (!key.empty() && node.rfind(key, 0) == 0) {
                            r104SemanticRotationClip = true;
                            r104RotationKey = key;
                            break;
                        }
                    }
                }
            }

            const bool r104PivotTurnTrace = r104SemanticRotationClip &&
                CapsuleSlerp::HasAuthoredTurnWitnessForCurrentOrPreviousFrame(character);

            if (r104PivotTurnTrace) {
                const float yaw = QuaternionYaw(rotation);
                std::uint64_t frame = 0;
                bool hasPair = false;
                float pairYaw1 = 0.0f;
                float pairDeltaYaw = 0.0f;
                float pairT1 = 0.0f;
                {
                    std::scoped_lock lock{ g_mutex };
                    auto& actorState = g_actorState[character->GetFormID()];
                    frame = actorState.frameSerial;
                    const std::string node = FixedString(clipName);
                    auto& pair = actorState.pairs[node];
                    if (site == Site::kRotation1) {
                        pair.r1.valid = true;
                        pair.r1.motionTime = motionTime;
                        pair.r1.existing = rotation;
                    } else if (pair.r1.valid) {
                        pairYaw1 = QuaternionYaw(pair.r1.existing);
                        pairT1 = pair.r1.motionTime;
                        pairDeltaYaw = yaw - pairYaw1;
                        constexpr float r104Pi = 3.14159265358979323846f;
                        constexpr float r104TwoPi = 2.0f * r104Pi;
                        while (pairDeltaYaw > r104Pi) pairDeltaYaw -= r104TwoPi;
                        while (pairDeltaYaw < -r104Pi) pairDeltaYaw += r104TwoPi;
                        pair.r1.valid = false;
                        hasPair = true;
                    }
                }

                SKSE::log::info(
                    "[R104][PMD_NATIVE_TURN] form={:08X} frame={} site={} clip=\"{}\" semanticKey=\"{}\" "
                    "motionTime={:+.6f} resolved={} q=({:+.6f},{:+.6f},{:+.6f},{:+.6f}) yaw={:+.6f}rad/{:+.3f}deg "
                    "pair[valid={} t1={:+.6f} yaw1={:+.6f} deltaYaw={:+.6f}rad/{:+.3f}deg] "
                    "action=OBSERVE_NATIVE_BEFORE_INCESSUS_ROTATION_POLICY",
                    character->GetFormID(), frame,
                    site == Site::kRotation1 ? "R1" : "R2",
                    FixedString(clipName), r104RotationKey, motionTime, resolved,
                    rotation.w, rotation.x, rotation.y, rotation.z, yaw, yaw * kRadiansToDegrees,
                    hasPair, pairT1, pairYaw1, pairDeltaYaw, pairDeltaYaw * kRadiansToDegrees);
            }

            const bool profilelessUniversalActor =
                character && !character->IsPlayerRef() &&
                !MotionProfileDatabase::SupportsActor(character);
            const std::string nativeClipName = FixedString(clipName);
            const bool universalLocomotionYawOwner = profilelessUniversalActor &&
                IsCertifiedNativeLocomotionCarrier(character, nativeClipName);

            // C1: profileless locomotion uses the proven R30/R35 R1/R2 transport independently of XY authority.
            // This REPLACES native locomotion yaw on certified locomotion leaves instead of
            // adding another yaw target at V13. Finite actions/turns are excluded by the
            // carrier certificate and therefore keep their native/authored rotation.
            if (universalLocomotionYawOwner) {
                SteeringIntentController::FrameCommand command{};
                const bool commandAvailable =
                    SpeedIntentTranslator::HasUniversalMovementDemand(character) &&
                    SteeringIntentController::GetFrameCommand(character, command) && command.valid;
                const float actorConventionStep =
                    commandAvailable && std::isfinite(command.yawStepRadians)
                    ? command.yawStepRadians : 0.0f;
                const float rawStep = -actorConventionStep;
                const float stepRadians =
                    std::clamp(rawStep, -0.78539816339f, +0.78539816339f);
                const bool safetyClamped = stepRadians != rawStep;
                const float halfStep = 0.5f * stepRadians;

                if (site == Site::kRotation1) {
                    rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
                } else {
                    rotation = { std::cos(halfStep), 0.0f, 0.0f, std::sin(halfStep) };
                }

                std::uint64_t frame = 0;
                std::uint64_t writeIndex = 0;
                bool emitLog = false;
                {
                    std::scoped_lock lock{ g_mutex };
                    auto& state = g_actorState[character->GetFormID()];
                    frame = state.frameSerial;
                    state.externalRotationWriteFrame = state.frameSerial;
                    ++state.externalRotationWrites;
                    if (site == Site::kRotation2) {
                        writeIndex = ++state.aiSteeringTransportAppliedTotal;
                        emitLog = state.aiSteeringTransportLogs < 36 || safetyClamped ||
                            (state.frameSerial % 120) == 0;
                        if (emitLog) ++state.aiSteeringTransportLogs;
                    }
                }

                if (emitLog) {
                    SKSE::log::debug(
                        "[C1_UNIVERSAL_YAW][R1R2] form={:08X} frame={} clip=\"{}\" valid={} source={} goalDist={:.2f} actorStep={:+.3f}deg forcedPair[R1=IDENTITY R2Yaw={:+.3f}deg] safetyClamped={} writeIndex={} action=REPLACE_NATIVE_LOCOMOTION_YAW",
                        character->GetFormID(), frame, nativeClipName,
                        commandAvailable,
                        commandAvailable ? SteeringIntentController::SourceName(command.source) : "NONE",
                        commandAvailable ? command.goalDistanceXY : 0.0f,
                        actorConventionStep * kRadiansToDegrees,
                        stepRadians * kRadiansToDegrees,
                        safetyClamped, writeIndex);
                }
                return;
            }

            if (!resolved) {
                return;
            }

            bool runtimeYawOwner = false;
            bool explicitRotationOwner = false;
            if (creatureProfileActor) {
                if (const auto* motionProfile = MotionProfileDatabase::ResolveProfile(character)) {
                    runtimeYawOwner = std::find(
                        motionProfile->runtimeYawClips.begin(),
                        motionProfile->runtimeYawClips.end(),
                        sample.key) != motionProfile->runtimeYawClips.end();
                    explicitRotationOwner = std::binary_search(
                        motionProfile->rotationClips.begin(),
                        motionProfile->rotationClips.end(),
                        sample.key);
                }
            }

            // R35 keeps the exact R30 ProcessMotionData transport.  Only the command source changed:
            // SteeringIntentController has already sampled the AI/path goal once at the
            // Character::Update boundary and produced one yawStep for the actor/frame.
            // Every W/T/R leaf receives that same step, so Havok blending cannot create
            // competing per-leaf steering values.
            if (creatureProfileActor && runtimeYawOwner && !explicitRotationOwner) {
                SteeringIntentController::FrameCommand command{};
                const bool commandAvailable = SteeringIntentController::GetFrameCommand(character, command);

                // RotationClips/TurnLoop remain the exclusive authored yaw authority.
                // Locomotion leaves may still be generated during the transition, but they
                // receive identity R1/R2 until the authored-turn witness disappears.
                // R103: native StablePivot turns (notably Troll MTTurn*90) are valid authored
                // yaw owners even though they are not MotionDB-backed. CapsuleSlerp publishes
                // that external witness at Character::Update. Reuse the combined witness here
                // so outgoing RuntimeYaw locomotion leaves receive identity R1/R2 for the entire
                // pivot lifecycle instead of competing with the native turn.
                const bool authoredTurnActive =
                    CapsuleSlerp::HasAuthoredTurnWitnessForCurrentOrPreviousFrame(character);

                bool actionBypassActive = false;
                if (const auto* motionProfile = MotionProfileDatabase::ResolveProfile(character)) {
                    for (const auto& variable : motionProfile->rotation.actionBypassVariables) {
                        if (variable.empty()) continue;
                        bool active = false;
                        if (character->GetGraphVariableBool(variable.c_str(), active) && active) {
                            actionBypassActive = true;
                            break;
                        }
                    }
                }

                // R55: explicit canned pivot staging/active/cleanup must suppress the
                // procedural locomotion yaw lane even while outgoing RuntimeYawClips are
                // still generated. This restores the older proven single-owner contract.
                const bool pivotAuthorityBusy = AnimationPriority::IsBusy(character);
                const bool intentValid = commandAvailable && !authoredTurnActive &&
                    !actionBypassActive && !pivotAuthorityBusy;
                const float actorConventionStep = intentValid && std::isfinite(command.yawStepRadians)
                    ? command.yawStepRadians
                    : 0.0f;
                // R32 telemetry proved bhk/PMD physical yaw has the opposite sign from
                // Actor::data.angle.z / atan2(worldDX, worldDY). Preserve the R30/R32
                // transport convention by negating only at this boundary.
                const float rawStep = -actorConventionStep;
                const float stepRadians = std::clamp(rawStep, -0.78539816339f, +0.78539816339f);
                const bool safetyClamped = stepRadians != rawStep;
                const float halfStep = 0.5f * stepRadians;

                if (site == Site::kRotation1) {
                    rotation = { 1.0f, 0.0f, 0.0f, 0.0f };
                } else {
                    rotation = { std::cos(halfStep), 0.0f, 0.0f, std::sin(halfStep) };
                }

                std::uint64_t frame = 0;
                std::uint64_t writeIndex = 0;
                bool emitLog = false;
                {
                    std::scoped_lock lock{ g_mutex };
                    auto& state = g_actorState[character->GetFormID()];
                    frame = state.frameSerial;
                    state.externalRotationWriteFrame = state.frameSerial;
                    ++state.externalRotationWrites;

                    if (site == Site::kRotation2) {
                        writeIndex = ++state.aiSteeringTransportAppliedTotal;
                        if (state.aiSteeringLastLogFrame != state.frameSerial) {
                            state.aiSteeringLastLogFrame = state.frameSerial;
                            const float rate = intentValid ? command.currentYawRateRadPerSecond : 0.0f;
                            const auto sourceValue = static_cast<std::uint8_t>(command.source);
                            const bool validityChanged = state.aiSteeringLastLoggedValid != intentValid;
                            const bool sourceChanged = state.aiSteeringLastLoggedSource != sourceValue;
                            const bool rateChanged = !state.aiSteeringLastLoggedValid ||
                                std::abs(rate - state.aiSteeringLastLoggedRateRadPerSecond) >=
                                    (10.0f / kRadiansToDegrees);
                            emitLog = state.aiSteeringTransportLogs < 36 || validityChanged ||
                                sourceChanged || rateChanged || safetyClamped ||
                                (state.frameSerial % 120) == 0;
                            if (emitLog) {
                                ++state.aiSteeringTransportLogs;
                                state.aiSteeringLastLoggedValid = intentValid;
                                state.aiSteeringLastLoggedRateRadPerSecond = rate;
                                state.aiSteeringLastLoggedSource = sourceValue;
                            }
                        }
                    }
                }

                if (emitLog) {
                    SKSE::log::debug("[AI_STEERING_YAW][TRANSPORT] form={:08X} frame={} steeringFrame={} clip=\"{}\" key=\"{}\" "
                        "valid={} source={} target={:08X} goalDist={:.2f} error[raw={:+.2f} filtered={:+.2f}]deg "
                        "rate[target={:+.2f} current={:+.2f}]deg/s actorStep={:+.3f}deg forcedPair[R1=IDENTITY R2Yaw={:+.3f}deg] "
                        "suppressed[authoredTurn={} actionBypass={} pivotAuthority={}] safetyClamped={} writeIndex={} sourceAuthority=R35_AI_GOAL_CONTROLLER sink=ProcessMotionData_R1_R2",
                        character->GetFormID(),
                        frame,
                        command.frameSerial,
                        FixedString(clipName),
                        sample.key,
                        intentValid,
                        SteeringIntentController::SourceName(command.source),
                        command.targetFormID,
                        command.goalDistanceXY,
                        command.rawErrorRadians * kRadiansToDegrees,
                        command.filteredErrorRadians * kRadiansToDegrees,
                        command.targetYawRateRadPerSecond * kRadiansToDegrees,
                        command.currentYawRateRadPerSecond * kRadiansToDegrees,
                        actorConventionStep * kRadiansToDegrees,
                        stepRadians * kRadiansToDegrees,
                        authoredTurnActive,
                        actionBypassActive,
                        pivotAuthorityBusy,
                        safetyClamped,
                        writeIndex);
                }
                return;
            }

            if (!sample.yawValid || !std::isfinite(sample.yaw)) {
                if (creatureProfileActor) {
                    const char* siteName = site == Site::kRotation1 ? "R1" : "R2";
                    std::string signature = "SKIP|" + sample.key + "|" + siteName;
                    if (MarkMotionDiagnosticOnce(
                            character->GetFormID(), MotionDiagKind::kRotation, std::move(signature))) {
                        SKSE::log::info(
                            R"([MotionDB][ROT_SKIP] form={:08X} site={} clip="{}" key="{}" motionTime={:.6f} reason=YAW_INVALID)",
                            character->GetFormID(), siteName, FixedString(clipName), sample.key, motionTime);
                    }
                }
                return;
            }

            // Authored MotionDB yaw remains exclusive to Intent.ini::RotationClips.
            // For the wolf profile this is turnloopingl/turnloopingr.
            if (creatureProfileActor && !explicitRotationOwner) {
                const char* siteName = site == Site::kRotation1 ? "R1" : "R2";
                std::string signature = "AUTHORITY_SKIP|" + sample.key + "|" + siteName;
                if (MarkMotionDiagnosticOnce(
                        character->GetFormID(), MotionDiagKind::kRotation, std::move(signature))) {
                    SKSE::log::info(
                        R"([MotionDB][ROT_AUTHORITY_SKIP] form={:08X} frame={} site={} clip="{}" key="{}" motionTime={:.6f} reason=NOT_RUNTIMEYAW_OR_ROTATIONCLIP action=PRESERVE_EXISTING_R1_R2)",
                        character->GetFormID(), g_invocationCache.frameSerial, siteName,
                        FixedString(clipName), sample.key, motionTime);
                }
                return;
            }

            const float halfYaw = sample.yaw * 0.5f;
            rotation = { std::cos(halfYaw), 0.0f, 0.0f, std::sin(halfYaw) };

            if (creatureProfileActor) {
                const char* siteName = site == Site::kRotation1 ? "R1" : "R2";
                std::string signature = "WRITE|" + sample.key + "|" + siteName;
                if (MarkMotionDiagnosticOnce(
                        character->GetFormID(), MotionDiagKind::kRotation, std::move(signature))) {
                    SKSE::log::info(
                        R"([MotionDB][ROT_WRITE] form={:08X} frame={} site={} clip="{}" key="{}" motionTime={:.6f} yaw={:+.6f}rad/{:+.3f}deg writtenQ=({:+.6f},{:+.6f},{:+.6f},{:+.6f}))",
                        character->GetFormID(), g_invocationCache.frameSerial, siteName, FixedString(clipName), sample.key, motionTime,
                        sample.yaw, sample.yaw * 57.29577951308232f,
                        rotation.w, rotation.x, rotation.y, rotation.z);
                }
            }

            std::scoped_lock lock{ g_mutex };
            auto& state = g_actorState[character->GetFormID()];
            state.externalRotationWriteFrame = state.frameSerial;
            ++state.externalRotationWrites;
        }

        void LogTranslation(
            Site site,
            RE::Character* character,
            float motionTime,
            const RE::BSFixedString* clipNameValue,
            const RE::NiPoint3& translation)
        {
            if (!character || character->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(character)) {
                return;
            }

            const std::string clipName = FixedString(clipNameValue);
            const auto* hkbCharacter = ResolveHkbCharacter(character);
            ClipInfo info{};
            float age = std::numeric_limits<float>::infinity();
            const bool mapped = LookupClipInfo(hkbCharacter, clipName, info, age);
            if (!ShouldLogForClip(clipName, info, mapped)) {
                return;
            }

            TranslationHalf half{};
            half.valid = true;
            half.motionTime = motionTime;
            half.existing = translation;
            half.animationName = mapped ? info.animationName : std::string{};
            half.mapAge = age;
            if (mapped) {
                half.controlLocalTime = info.controlLocalTime;
                half.weight = info.weight;
                half.motionTrackWeight = info.motionTrackWeight;
                half.futureLocalTime = info.futureLocalTime;
                half.futureLoops = info.futureLoops;
            }
            [[maybe_unused]] const bool externalComparable =
                BuildExternalComparison(info, mapped, age, motionTime, half);

            std::uint64_t frame = 0;
            bool logCall = false;
            bool logPair = false;
            bool hasPair = false;
            TranslationHalf first{};
            std::uint64_t pairIndex = 0;
            {
                std::scoped_lock lock{ g_mutex };
                auto& actorState = g_actorState[character->GetFormID()];
                frame = actorState.frameSerial;
                ++actorState.translationCalls;
                if (actorState.callLogs < kCallLogLimit) {
                    ++actorState.callLogs;
                    logCall = true;
                }
                auto& pair = actorState.pairs[clipName];
                if (site == Site::kTranslation1) {
                    pair.t1 = half;
                } else if (pair.t1.valid) {
                    first = pair.t1;
                    pair.t1.valid = false;
                    hasPair = true;
                    ++actorState.translationPairs;
                    pairIndex = actorState.translationPairs;
                    if (actorState.pairLogs < kPairLogLimit) {
                        ++actorState.pairLogs;
                        logPair = true;
                    }
                }
            }

            if (logCall) {
                SKSE::log::info(
                    R"([NativeAudit][T] form={:08X} frame={} site={} clip="{}" motionTime={:+.6f} existing=({:+.5f},{:+.5f},{:+.5f}) map[found={} age={:.5f}s anim="{}"] control[local={:.6f} weight={:.6f} motionWeight={:.6f} future={:.6f} loops={}] external[valid={} key="{}" wouldBe=({:+.5f},{:+.5f},{:+.5f})] actorDriven={})",
                    character->GetFormID(), frame,
                    site == Site::kTranslation1 ? "T1" : "T2",
                    clipName, motionTime,
                    translation.x, translation.y, translation.z,
                    mapped, age, half.animationName,
                    half.controlLocalTime, half.weight, half.motionTrackWeight,
                    half.futureLocalTime, half.futureLoops,
                    half.externalValid, half.key,
                    half.external.x, half.external.y, half.external.z,
                    character->IsAnimationDriven());
            }

            if (hasPair) {
                const RE::NiPoint3 existingDelta{
                    half.existing.x - first.existing.x,
                    half.existing.y - first.existing.y,
                    half.existing.z - first.existing.z
                };
                if (logPair) {
                    RE::NiPoint3 externalDelta{};
                    const bool externalPair = first.externalValid && half.externalValid && first.key == half.key;
                    if (externalPair) {
                        externalDelta = {
                            half.external.x - first.external.x,
                            half.external.y - first.external.y,
                            half.external.z - first.external.z
                        };
                    }
                    SKSE::log::info(
                        R"([NativeAudit][TPAIR] form={:08X} frame={} pair={} clip="{}" t={:+.6f}->{:+.6f} dt={:+.6f} existingDelta=({:+.5f},{:+.5f},{:+.5f}) distXY={:.5f} external[valid={} key="{}" delta=({:+.5f},{:+.5f},{:+.5f}) distXY={:.5f}] mapAge={:.5f}s actorDriven={})",
                        character->GetFormID(), frame, pairIndex, clipName,
                        first.motionTime, half.motionTime, half.motionTime - first.motionTime,
                        existingDelta.x, existingDelta.y, existingDelta.z,
                        std::hypot(existingDelta.x, existingDelta.y),
                        externalPair, externalPair ? half.key : std::string{},
                        externalDelta.x, externalDelta.y, externalDelta.z,
                        std::hypot(externalDelta.x, externalDelta.y),
                        half.mapAge, character->IsAnimationDriven());
                }
            }
        }

        void LogRotation(
            Site site,
            RE::Character* character,
            float motionTime,
            const RE::BSFixedString* clipNameValue,
            const RE::NiQuaternion& rotation)
        {
            if (!character || character->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(character)) {
                return;
            }
            const std::string clipName = FixedString(clipNameValue);
            const auto* hkbCharacter = ResolveHkbCharacter(character);
            ClipInfo info{};
            float age = std::numeric_limits<float>::infinity();
            const bool mapped = LookupClipInfo(hkbCharacter, clipName, info, age);
            if (!ShouldLogForClip(clipName, info, mapped)) {
                return;
            }

            std::uint64_t frame = 0;
            bool logCall = false;
            bool logPair = false;
            bool hasPair = false;
            RotationHalf first{};
            std::uint64_t pairIndex = 0;
            {
                std::scoped_lock lock{ g_mutex };
                auto& actorState = g_actorState[character->GetFormID()];
                frame = actorState.frameSerial;
                ++actorState.rotationCalls;
                if (actorState.callLogs < kCallLogLimit) {
                    ++actorState.callLogs;
                    logCall = true;
                }
                auto& pair = actorState.pairs[clipName];
                if (site == Site::kRotation1) {
                    pair.r1.valid = true;
                    pair.r1.motionTime = motionTime;
                    pair.r1.existing = rotation;
                } else if (pair.r1.valid) {
                    first = pair.r1;
                    pair.r1.valid = false;
                    hasPair = true;
                    ++actorState.rotationPairs;
                    pairIndex = actorState.rotationPairs;
                    if (actorState.pairLogs < kPairLogLimit) {
                        ++actorState.pairLogs;
                        logPair = true;
                    }
                }
            }

            if (logCall) {
                SKSE::log::info(
                    R"([NativeAudit][R] form={:08X} frame={} site={} clip="{}" motionTime={:+.6f} existingQ=({:+.6f},{:+.6f},{:+.6f},{:+.6f}) yaw={:+.6f} map[found={} age={:.5f}s anim="{}"] actorDriven={})",
                    character->GetFormID(), frame,
                    site == Site::kRotation1 ? "R1" : "R2",
                    clipName, motionTime,
                    rotation.w, rotation.x, rotation.y, rotation.z, QuaternionYaw(rotation),
                    mapped, age, mapped ? info.animationName : std::string{},
                    character->IsAnimationDriven());
            }

            if (hasPair) {
                const float yaw1 = QuaternionYaw(first.existing);
                const float yaw2 = QuaternionYaw(rotation);
                float dyaw = yaw2 - yaw1;
                constexpr float pi = 3.14159265358979323846f;
                constexpr float twoPi = 2.0f * pi;
                while (dyaw > pi) dyaw -= twoPi;
                while (dyaw < -pi) dyaw += twoPi;

                if (logPair) {
                    SKSE::log::info(
                        R"([NativeAudit][RPAIR] form={:08X} frame={} pair={} clip="{}" t={:+.6f}->{:+.6f} dt={:+.6f} yaw={:+.6f}->{:+.6f} deltaYaw={:+.6f}rad/{:+.3f}deg actorDriven={})",
                        character->GetFormID(), frame, pairIndex, clipName,
                        first.motionTime, motionTime, motionTime - first.motionTime,
                        yaw1, yaw2, dyaw, dyaw * 57.29577951308232f,
                        character->IsAnimationDriven());
                }
            }
        }

        void Translation1After(
            void* motionData, float motionTime, RE::NiPoint3& translation,
            const RE::BSFixedString* clipName, RE::Character* character)
        {
            ObserveNativeMotionPair(Site::kTranslation1, motionData, motionTime, clipName, character, std::addressof(translation), nullptr);
            ApplyExternalTranslation(Site::kTranslation1, character, motionTime, clipName, translation);
        }

        void Translation2After(
            void* motionData, float motionTime, RE::NiPoint3& translation,
            const RE::BSFixedString* clipName, RE::Character* character)
        {
            ObserveNativeMotionPair(Site::kTranslation2, motionData, motionTime, clipName, character, std::addressof(translation), nullptr);
            ApplyExternalTranslation(Site::kTranslation2, character, motionTime, clipName, translation);
        }

        void Rotation1After(
            void* motionData, float motionTime, RE::NiQuaternion& rotation,
            const RE::BSFixedString* clipName, RE::Character* character)
        {
            ObserveNativeMotionPair(Site::kRotation1, motionData, motionTime, clipName, character, nullptr, std::addressof(rotation));
            ApplyExternalRotation(Site::kRotation1, character, motionTime, clipName, rotation);
        }

        void Rotation2After(
            void* motionData, float motionTime, RE::NiQuaternion& rotation,
            const RE::BSFixedString* clipName, RE::Character* character)
        {
            ObserveNativeMotionPair(Site::kRotation2, motionData, motionTime, clipName, character, nullptr, std::addressof(rotation));
            ApplyExternalRotation(Site::kRotation2, character, motionTime, clipName, rotation);
        }
    }

    void BeginActorFrame(RE::Character* character, float deltaTime)
    {
        // B2: native motion observation is universal for non-player actors.
        // Profile-driven steering/gait/pivot systems remain independently gated.
        if (!detail::IsUniversalNativeMotionActor(character)) {
            return;
        }

        // R35: legacy R32 pre-frame controller-rate sampling is intentionally disabled.
        // rotationSpeed/path rotation state is not an authority in this build.
        const float preFrameYawRateCandidate = 0.0f;
        const float preFrameMiddleRotationCandidate = 0.0f;
        const float preFramePathDesiredCandidate = 0.0f;
        const float preFramePathCurrentCandidate = 0.0f;
        const float preFrameDesiredMoveXCandidate = 0.0f;
        const float preFrameDesiredMoveYCandidate = 0.0f;
        const bool preFrameYawRateValid = false;
        const bool preFrameYawEligible = false;

        const auto* hkbCharacter = detail::ResolveHkbCharacter(character);
        std::array<const RE::hkbCharacter*, 8> hkbCharacters{};
        const std::size_t hkbCharacterCount = detail::CollectHkbCharacters(character, hkbCharacters);

        // Close the previous R14 frame before any new graph update. This is the
        // realized world-space result after Havok + ProcessMotionData + controller.
        const RE::NiPoint3 currentRootDebugPosition = character->GetPosition();
        const float currentRootDebugYaw = character->data.angle.z;
        bool logRootFrameResult = false;
        std::uint64_t completedFrame = 0;
        std::size_t completedRootWitnesses = 0;
        std::uint64_t completedTransitionCalls = 0;
        std::uint64_t completedTransitionChanges = 0;
        std::uint64_t completedTranslationWrites = 0;
        std::uint64_t completedRotationWrites = 0;
        bool completedNativeDriven = false;
        bool completedNonzeroExtractedTelemetry = false;
        float completedDx = 0.0f;
        float completedDy = 0.0f;
        float completedDz = 0.0f;
        float completedDistanceXY = 0.0f;
        float completedYawDeg = 0.0f;
        detail::RootContributionSummary completedRootMix{};
        detail::PhysicalCompositionProbe completedPhysicalComposition{};
        bool logPhysicalComposition = false;
        bool physicalCompositionChanged = false;
        bool physicalCompositionNoTransitionMultiKey = false;
        bool logRootMixerFrame = false;
        {
            std::scoped_lock lock{ detail::g_mutex };
            auto& state = detail::g_actorState[character->GetFormID()];

            if (state.rootDebugSnapshotValid && state.frameSerial != 0) {
                completedFrame = state.frameSerial;
                completedRootWitnesses = state.rootWitnessCount;
                completedTransitionCalls = state.wfmTransitionCallsFrame;
                completedTransitionChanges = state.wfmTransitionChangesFrame;
                state.previousTransitionCalls = state.wfmTransitionCallsFrame;
                completedTranslationWrites =
                    state.externalTranslationWrites - state.rootDebugTranslationWrites;
                completedRotationWrites =
                    state.externalRotationWrites - state.rootDebugRotationWrites;
                completedNativeDriven = state.rootDebugNativeDriven;
                completedNonzeroExtractedTelemetry =
                    state.wfmRootOutputSeenFrame == state.frameSerial;

                completedDx = currentRootDebugPosition.x - state.rootDebugPosition.x;
                completedDy = currentRootDebugPosition.y - state.rootDebugPosition.y;
                completedDz = currentRootDebugPosition.z - state.rootDebugPosition.z;
                completedDistanceXY = std::hypot(completedDx, completedDy);

                float yawDelta = currentRootDebugYaw - state.rootDebugYaw;
                constexpr float kPi = 3.14159265358979323846f;
                constexpr float kTwoPi = 2.0f * kPi;
                while (yawDelta > kPi) {
                    yawDelta -= kTwoPi;
                }
                while (yawDelta <= -kPi) {
                    yawDelta += kTwoPi;
                }
                completedYawDeg = yawDelta * 57.29577951308232f;

                const bool physicalMoved =
                    completedDistanceXY > 0.01f ||
                    std::fabs(completedYawDeg) > 0.10f;
                logRootFrameResult =
                    physicalMoved ||
                    completedRootWitnesses > 0 ||
                    completedTransitionCalls > 0 ||
                    completedTranslationWrites > 0 ||
                    completedRotationWrites > 0;

            }

            if (state.frameSerial != 0) {
                completedRootMix = detail::ComputeRootContributionSummaryLocked(state);
                completedPhysicalComposition = detail::BuildPhysicalCompositionProbeLocked(state);
                physicalCompositionNoTransitionMultiKey =
                    completedTransitionCalls == 0 && completedPhysicalComposition.uniqueKeys > 1;
                physicalCompositionChanged =
                    completedPhysicalComposition.signature != state.physicalCompositionLastSignature;
                const bool physicalCompositionInteresting =
                    completedPhysicalComposition.uniqueKeys > 1 ||
                    completedPhysicalComposition.phaseSplitKeys > 0;
                const bool physicalCompositionPeriodic =
                    physicalCompositionInteresting &&
                    (state.physicalCompositionLastLogFrame == 0 ||
                     state.frameSerial - state.physicalCompositionLastLogFrame >= 120);
                logPhysicalComposition =
                    physicalCompositionInteresting &&
                    (physicalCompositionChanged || physicalCompositionPeriodic);
                if (physicalCompositionChanged) {
                    state.physicalCompositionLastSignature = completedPhysicalComposition.signature;
                }
                if (logPhysicalComposition) {
                    state.physicalCompositionLastLogFrame = state.frameSerial;
                    ++state.physicalCompositionLogs;
                }
                state.completedRootMix = completedRootMix;
                state.previousRootMix = completedRootMix;
                detail::CaptureCurrentDbRootKeysLocked(state);
                ++state.rootMixerFrames;
                if (completedRootMix.dbRootWeight > detail::kRootContributionEpsilon) {
                    ++state.rootMixerDbFrames;
                }
                const bool dbActive =
                    completedRootMix.dbRootWeight > detail::kRootContributionEpsilon;
                const bool dbEdge = dbActive != state.rootMixerLastDbActive;
                const bool shareChanged =
                    std::fabs(completedRootMix.dbRootShare - state.rootMixerLastLoggedShare) > 0.15f;
                logRootMixerFrame =
                    state.rootMixerFrameLogs < 24 || dbEdge || shareChanged ||
                    (state.frameSerial % 120) == 0;
                state.rootMixerLastDbActive = dbActive;
                if (logRootMixerFrame) {
                    ++state.rootMixerFrameLogs;
                    state.rootMixerLastLoggedShare = completedRootMix.dbRootShare;
                }
            }

            if (!state.runtimeWfmPolicies.empty()) {
                for (auto it = state.runtimeWfmPolicies.begin();
                     it != state.runtimeWfmPolicies.end();) {
                    const bool stale =
                        state.frameSerial > it->second.lastSeenFrame &&
                        (state.frameSerial - it->second.lastSeenFrame) > 60;
                    if (stale) {
                        it = state.runtimeWfmPolicies.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            if (!state.transitionSideMaps.empty()) {
                for (auto it = state.transitionSideMaps.begin();
                     it != state.transitionSideMaps.end();) {
                    const bool stale =
                        state.frameSerial > it->second.lastSeenFrame &&
                        (state.frameSerial - it->second.lastSeenFrame) > 60;
                    if (stale) {
                        it = state.transitionSideMaps.erase(it);
                    } else {
                        ++it;
                    }
                }
            }

            state.rootContributionCount = 0;
            state.rootWitnessCount = 0;
            state.wfmTransitionCallsFrame = 0;
            state.wfmTransitionChangesFrame = 0;
            state.rootDebugPosition = currentRootDebugPosition;
            state.rootDebugYaw = currentRootDebugYaw;
            state.rootDebugSnapshotValid = true;
            state.rootDebugNativeDriven = character->IsAnimationDriven();
            state.rootDebugTranslationWrites = state.externalTranslationWrites;
            state.rootDebugRotationWrites = state.externalRotationWrites;
        }

        if (logRootFrameResult) {
            SKSE::log::debug(R"([WFM][FRAME_RESULT] form={:08X} frame={} clipTelemetryWitnesses={} transitions={} policyChanges={} pmdWrites[T={} R={}] nativeDriven={} nonzeroExtractedTelemetry={} realizedWorldDelta=({:+.5f},{:+.5f},{:+.5f}) distanceXY={:.5f} realizedYaw={:+.4f}deg note="physical actor result measured at next Character::Update boundary")",
                character->GetFormID(),
                completedFrame,
                completedRootWitnesses,
                completedTransitionCalls,
                completedTransitionChanges,
                completedTranslationWrites,
                completedRotationWrites,
                completedNativeDriven,
                completedNonzeroExtractedTelemetry,
                completedDx,
                completedDy,
                completedDz,
                completedDistanceXY,
                completedYawDeg);
        }
        if (logRootMixerFrame) {
            SKSE::log::debug(R"([ROOT_MIXER][FRAME] form={:08X} frame={} clips={} dbClips={} poseWeight={:.5f} dbRootWeight={:.5f} dbRootShare={:.5f} mixedTranslation[valid={} xyz=({:+.5f},{:+.5f},{:+.5f})] mixedYaw[valid={} yaw={:+.5f}rad/{:+.3f}deg] note="all generated pose contributors; DB root is blended by pose weight")",
                character->GetFormID(),
                completedFrame,
                completedRootMix.clips,
                completedRootMix.dbClips,
                completedRootMix.poseWeight,
                completedRootMix.dbRootWeight,
                completedRootMix.dbRootShare,
                completedRootMix.mixedTranslationValid,
                completedRootMix.mixedX,
                completedRootMix.mixedY,
                completedRootMix.mixedZ,
                completedRootMix.mixedYawValid,
                completedRootMix.mixedYaw,
                completedRootMix.mixedYaw * 57.29577951308232f);
        }
        if (logPhysicalComposition) {
            SKSE::log::info(
                "[PhysicalComposition][R94_PROBE] form={:08X} frame={} transitions={} dbLeaves={} uniqueKeys={} phaseSplitKeys={} maxAliasesForOneKey={} topologyChanged={} noTransitionMultiKey={} signature=\"{}\" detail=\"{}\" note=GENERATED_LEAF_TOPOLOGY_ONLY_NOT_FINAL_PARENT_WEIGHT_AND_NOT_A_BEHAVIOR_CHANGE",
                character->GetFormID(),
                completedFrame,
                completedTransitionCalls,
                completedPhysicalComposition.dbLeaves,
                completedPhysicalComposition.uniqueKeys,
                completedPhysicalComposition.phaseSplitKeys,
                completedPhysicalComposition.maxAliasesForOneKey,
                physicalCompositionChanged,
                physicalCompositionNoTransitionMultiKey,
                completedPhysicalComposition.signature,
                completedPhysicalComposition.detail);
        }

        // R1-55 cycle audit retained but disabled during the R1-57 transaction experiment.
        // Attribution is intentionally conservative: only frames that witnessed one
        // unique DB key contribute to that clip's cycle audit. Multi-key frames are
        // marked ambiguous rather than summed as if leaf callbacks were blend weights.
        if (character->IsPlayerRef() && detail::kEnablePlayerCycleAudit) {
            const RE::NiPoint3 currentPosition = character->GetPosition();
            std::scoped_lock lock{ detail::g_mutex };
            auto& state = detail::g_actorState[character->GetFormID()];
            if (state.playerPreviousPositionValid && state.frameSerial != 0) {
                const float dx = currentPosition.x - state.playerPreviousPosition.x;
                const float dy = currentPosition.y - state.playerPreviousPosition.y;
                const float physicalXY = std::hypot(dx, dy);
                if (std::isfinite(physicalXY)) {
                    state.playerPhysicalPathTotalXY += physicalXY;
                    if (state.playerFrameDbKeyCount == 1) {
                        ++state.playerSingleKeyFrames;
                        auto& audit = state.playerCycleAudit[state.playerFrameDbKeys[0]];
                        if (audit.cycleStarted) {
                            audit.physicalPathXY += physicalXY;
                            ++audit.attributedFrames;
                        }
                    } else if (state.playerFrameDbKeyCount > 1) {
                        ++state.playerAmbiguousFrames;
                        for (std::size_t i = 0; i < state.playerFrameDbKeyCount; ++i) {
                            auto& audit = state.playerCycleAudit[state.playerFrameDbKeys[i]];
                            if (audit.cycleStarted) {
                                audit.ambiguousBlend = true;
                                ++audit.attributedFrames;
                            }
                        }
                    } else if (state.externalTranslationWriteFrame == state.frameSerial) {
                        ++state.playerAmbiguousFrames;
                    }
                }
            }
            state.playerPreviousPosition = currentPosition;
            state.playerPreviousPositionValid = true;
            state.playerFrameDbKeyCount = 0;
        }

        // B8 universal native locomotion lifecycle.
        //
        // Profile-backed actors keep the proven legacy/profile bootstrap unchanged.
        // Profileless creatures use a different contract:
        //   1) AutoGraph/vanilla runs with capsule translation fully available.
        //   2) a real native T1->T2 translation delta is observed while vanilla still owns XY.
        //   3) on the NEXT Character::Update boundary, bAnimationDriven + root authority
        //      are handed over atomically. There is no PREARM and therefore no dead frame.
        //   4) authority remains while AI MovementDemand + graph contract remain valid.
        bool nativeLocomotionActiveForFrame = false;
        if (MotionProfileDatabase::SupportsActor(character)) {
            const bool directionalIntent = detail::HasNativeLocomotionDemand(character);
            const bool graphContractReady = detail::HasNativeLocomotionContract(character);
            const bool actionBypass = detail::HasUniversalActionBypass(character);
            const bool profileBacked = MotionProfileDatabase::SupportsActor(character);
            const bool actorInvalidForStart = !detail::IsUniversalCreatureActor(character) ||
                !detail::IsRuntimeEligibleForExternalRoot(character) || !graphContractReady;
            const bool hardInvalid = !detail::IsUniversalCreatureActor(character) ||
                detail::IsHardInvalidForNativeLocomotion(character);
            const bool actorDrivenBefore = character->IsAnimationDriven();

            bool graphMoving = false;
            bool graphMovingRead = character->GetGraphVariableBool("isMoving", graphMoving);
            if (!graphMovingRead) {
                graphMovingRead = character->GetGraphVariableBool("IsMoving", graphMoving);
            }

            bool sessionBefore = false;
            bool ownedBefore = false;
            bool authorityBefore = false;
            bool originalDriven = false;
            bool listFresh = false;
            bool deltaFresh = false;
            bool startNow = false;
            bool stopNow = false;
            bool transferLegacyDbOwnership = false;
            bool provenHandoff = false;
            bool carrierCertified = false;
            std::uint64_t sourceFrame = 0;
            std::uint64_t targetFrame = 0;
            std::string nativeClip;
            std::uint32_t misses = 0;
            std::uint32_t entryAge = 0;

            {
                std::scoped_lock lock{ detail::g_mutex };
                auto& state = detail::g_actorState[character->GetFormID()];
                sourceFrame = state.frameSerial;
                targetFrame = sourceFrame + 1;
                sessionBefore = state.nativeLocomotionSessionActive;
                ownedBefore = state.nativeLocomotionDrivenLatched;
                authorityBefore = state.nativeLocomotionAuthorityEstablished;
                originalDriven = state.nativeLocomotionOriginalDriven;
                listFresh = sourceFrame != 0 &&
                    state.nativeTranslationListSeenFrame == sourceFrame;
                deltaFresh = sourceFrame != 0 &&
                    state.nativeTranslationDeltaSeenFrame == sourceFrame;
                nativeClip = !state.nativeTranslationDeltaSeenClip.empty() ?
                    state.nativeTranslationDeltaSeenClip : state.nativeTranslationListSeenClip;
                carrierCertified = deltaFresh &&
                    detail::IsCertifiedNativeLocomotionCarrierName(nativeClip);

                transferLegacyDbOwnership =
                    profileBacked && !sessionBefore && directionalIntent && !actionBypass &&
                    state.creatureDbSessionActive && state.creatureDrivenLatched;

                if (!profileBacked) {
                    // B8 profileless HANDOFF: a native root delta must already have been
                    // observed while the capsule was still vanilla-authoritative.
                    const bool mayHandoff =
                        !sessionBefore && !actorInvalidForStart && directionalIntent &&
                        !actionBypass && deltaFresh && carrierCertified;

                    if (mayHandoff) {
                        state.nativeLocomotionSessionActive = true;
                        state.nativeLocomotionAuthorityEstablished = true;
                        state.nativeLocomotionContinuityMissFrames = 0;
                        state.nativeLocomotionEntryAgeFrames = 0;
                        state.nativeLocomotionOriginalDriven = actorDrivenBefore;
                        ++state.nativeLocomotionSessionStarts;
                        startNow = true;
                        provenHandoff = true;

                        // A universal native session and legacy DB translation ownership are
                        // mutually exclusive. Profileless actors should not normally have DB
                        // ownership, but clear stale evidence defensively.
                        state.previousDbRootKeyCount = 0;
                        state.creatureDbEntryProvenanceFrame = 0;
                        state.creatureDbPmdContinuityFrame = 0;
                        state.creatureDbGenerateContinuityFrame = 0;
                    } else if (sessionBefore) {
                        const bool carrierSafe = !deltaFresh || carrierCertified;
                        const bool intentSafe =
                            !hardInvalid && directionalIntent && !actionBypass && graphContractReady &&
                            carrierSafe;
                        if (intentSafe) {
                            state.nativeLocomotionContinuityMissFrames = 0;
                            state.nativeLocomotionEntryAgeFrames = 0;
                            ++state.nativeLocomotionHolds;
                        } else {
                            state.nativeLocomotionSessionActive = false;
                            state.nativeLocomotionAuthorityEstablished = false;
                            state.nativeLocomotionContinuityMissFrames = 0;
                            state.nativeLocomotionEntryAgeFrames = 0;
                            ++state.nativeLocomotionSessionStops;
                            stopNow = true;
                        }
                    }
                } else {
                    // Exact B6/profile compatibility lane. The already-validated Wolf and
                    // any explicit profile keep their existing bootstrap semantics.
                    const bool mayStart =
                        !sessionBefore && !actorInvalidForStart && directionalIntent && !actionBypass &&
                        (!actorDrivenBefore || transferLegacyDbOwnership);

                    if (mayStart) {
                        state.nativeLocomotionSessionActive = true;
                        state.nativeLocomotionAuthorityEstablished = false;
                        state.nativeLocomotionContinuityMissFrames = 0;
                        state.nativeLocomotionEntryAgeFrames = 0;
                        state.nativeLocomotionOriginalDriven = transferLegacyDbOwnership ?
                            state.creatureOriginalDriven : actorDrivenBefore;
                        ++state.nativeLocomotionSessionStarts;
                        startNow = true;

                        state.previousDbRootKeyCount = 0;
                        state.creatureDbEntryProvenanceFrame = 0;
                        state.creatureDbPmdContinuityFrame = 0;
                        state.creatureDbGenerateContinuityFrame = 0;

                        if (transferLegacyDbOwnership) {
                            state.creatureDbSessionActive = false;
                            state.creatureDbSessionZeroRootSeconds = 0.0f;
                            state.creatureDbOwnershipEstablished = false;
                            state.creatureDbEntryPending = false;
                            state.creatureDbEntryAgeSeconds = 0.0f;
                            state.creatureDbEntryAgeFrames = 0;
                            state.creatureDrivenLatched = false;
                            state.creatureBootstrapDrivenFrame = 0;
                            detail::ClearDbOwnershipKeysLocked(state);
                        }
                    } else if (sessionBefore) {
                        const bool intentSafe = !hardInvalid && directionalIntent && !actionBypass;

                        if (!state.nativeLocomotionAuthorityEstablished) {
                            state.nativeLocomotionContinuityMissFrames = 0;
                            entryAge = ++state.nativeLocomotionEntryAgeFrames;
                            if (intentSafe) ++state.nativeLocomotionHolds;
                        } else {
                            state.nativeLocomotionEntryAgeFrames = 0;
                            const bool continuity = intentSafe &&
                                (listFresh || deltaFresh || (graphMovingRead && graphMoving));
                            if (continuity) {
                                state.nativeLocomotionContinuityMissFrames = 0;
                                ++state.nativeLocomotionHolds;
                            } else {
                                misses = ++state.nativeLocomotionContinuityMissFrames;
                            }
                        }

                        const bool entryExpired = !state.nativeLocomotionAuthorityEstablished &&
                            state.nativeLocomotionEntryAgeFrames >
                                detail::kNativeLocomotionEntryTimeoutFrames;
                        const bool continuityExpired = state.nativeLocomotionAuthorityEstablished &&
                            state.nativeLocomotionContinuityMissFrames >
                                detail::kNativeLocomotionContinuityGraceFrames;

                        if (!intentSafe || entryExpired || continuityExpired) {
                            state.nativeLocomotionSessionActive = false;
                            state.nativeLocomotionAuthorityEstablished = false;
                            state.nativeLocomotionContinuityMissFrames = 0;
                            state.nativeLocomotionEntryAgeFrames = 0;
                            ++state.nativeLocomotionSessionStops;
                            stopNow = true;
                        }
                    }
                }

                nativeLocomotionActiveForFrame = state.nativeLocomotionSessionActive;
                if (nativeLocomotionActiveForFrame || stopNow) {
                    state.previousDbRootKeyCount = 0;
                    state.creatureDbEntryProvenanceFrame = 0;
                    state.creatureDbPmdContinuityFrame = 0;
                    state.creatureDbGenerateContinuityFrame = 0;
                }
            }

            if (startNow || nativeLocomotionActiveForFrame) {
                bool driven = false;
                const bool readable = character->GetGraphVariableBool("bAnimationDriven", driven);
                bool accepted = readable;
                bool readback = driven;
                const bool needsWrite = readable && !driven;

                if (needsWrite) {
                    accepted = character->SetGraphVariableBool("bAnimationDriven", true);
                    const bool readbackReadable =
                        character->GetGraphVariableBool("bAnimationDriven", readback);
                    accepted = accepted && readbackReadable && readback;
                }

                if (accepted && (readback || transferLegacyDbOwnership)) {
                    std::uint64_t arms = 0;
                    {
                        std::scoped_lock lock{ detail::g_mutex };
                        auto& state = detail::g_actorState[character->GetFormID()];
                        if (state.nativeLocomotionSessionActive) {
                            state.nativeLocomotionDrivenLatched = true;
                            if (needsWrite || transferLegacyDbOwnership) {
                                state.nativeLocomotionBootstrapDrivenFrame = targetFrame;
                                arms = ++state.nativeLocomotionArms;
                            }
                            nativeLocomotionActiveForFrame = true;
                        }
                    }

                    if (startNow) {
                        if (provenHandoff) {
                            SKSE::log::info(
                                "[B8_NATIVE_LOCOMOTION][HANDOFF] form={:08X} sourceFrame={} targetFrame={} clip=\"{}\" movementDemand={} contractReady={} deltaFresh={} carrierCertified={} bAnimationDriven={} arms={} proof=NATIVE_T1_T2_PLUS_LOCOMOTION_CARRIER action=ATOMIC_CAPSULE_TO_ROOT",
                                character->GetFormID(), sourceFrame, targetFrame, nativeClip,
                                directionalIntent, graphContractReady, deltaFresh, carrierCertified, readback, arms);
                        } else {
                            SKSE::log::info(
                                "[B8_NATIVE_LOCOMOTION][PROFILE_PREARM] form={:08X} sourceFrame={} targetFrame={} clip=\"{}\" movementDemand={} listFresh={} deltaFresh={} transferDb={} bAnimationDriven={} arms={} action=PROFILE_COMPATIBILITY",
                                character->GetFormID(), sourceFrame, targetFrame, nativeClip,
                                directionalIntent, listFresh, deltaFresh, transferLegacyDbOwnership,
                                readback, arms);
                        }
                    }
                } else if (startNow) {
                    std::scoped_lock lock{ detail::g_mutex };
                    auto& state = detail::g_actorState[character->GetFormID()];
                    state.nativeLocomotionSessionActive = false;
                    state.nativeLocomotionAuthorityEstablished = false;
                    state.nativeLocomotionDrivenLatched = false;
                    state.nativeLocomotionBootstrapDrivenFrame = 0;
                    nativeLocomotionActiveForFrame = false;
                    SKSE::log::warn(
                        "[B8_NATIVE_LOCOMOTION][HANDOFF_FAIL] form={:08X} sourceFrame={} clip=\"{}\" profileBacked={} readable={} accepted={} readback={} action=FAIL_OPEN_VANILLA",
                        character->GetFormID(), sourceFrame, nativeClip, profileBacked,
                        readable, accepted, readback);
                }
            }

            if (stopNow && ownedBefore) {
                const bool restored = character->SetGraphVariableBool(
                    "bAnimationDriven", originalDriven);
                std::scoped_lock lock{ detail::g_mutex };
                auto& state = detail::g_actorState[character->GetFormID()];
                if (restored) {
                    state.nativeLocomotionDrivenLatched = false;
                    state.nativeLocomotionBootstrapDrivenFrame = 0;
                    ++state.nativeLocomotionRestores;
                } else {
                    state.nativeLocomotionDrivenLatched = true;
                }
                SKSE::log::info(
                    "[B8_NATIVE_LOCOMOTION][STOP] form={:08X} sourceFrame={} clip=\"{}\" profileBacked={} movementDemand={} contractReady={} actionBypass={} hardInvalid={} carrierCertified={} graphMovingRead={} graphMoving={} listFresh={} deltaFresh={} misses={} entryAge={} authorityBefore={} restore={} requested={} action=RETURN_VANILLA",
                    character->GetFormID(), sourceFrame, nativeClip, profileBacked,
                    directionalIntent, graphContractReady, actionBypass, hardInvalid, carrierCertified,
                    graphMovingRead, graphMoving, listFresh, deltaFresh, misses, entryAge,
                    authorityBefore, restored, originalDriven);
            }
        }

        // R15.8 DB root authority lifecycle.
        // Acquisition and continuity are deliberately different concepts:
        // - acquisition: strong generated DB root / transition-local provenance /
        //   legacy committed DB TX only. PMD can NEVER start ownership.
        // - continuity after ACQUIRED: matching PMD, matching generated WFM/root, or
        //   a short bounded grace keeps the already-certified ownership latched.
        // - release: explicit ROOT->INPLACE/native takeover, invalid actor, or a
        //   sustained continuity timeout. A missing Generate callback is not release.
        // R35 BASELINE: R34 already fixed the R33 lifecycle bug; R33 accidentally inherited the R32 preFrameYawEligible guard here.
        // In R33 that diagnostic source was intentionally disabled, so the guard was always
        // false and the entire creature DB session/ownership/bootstrap lifecycle never ran.
        // Restore the proven R30/R28 eligibility: every supported non-player MotionDB actor.
        if (detail::IsExternalMotionActor(character) && !nativeLocomotionActiveForFrame) {
            bool ownedBefore = false;
            bool sessionBefore = false;
            bool ownershipBefore = false;
            bool originalDriven = false;
            bool previousDbRoot = false;
            bool previousDbWrite = false;
            bool previousDbObserved = false;
            bool previousDbProvenance = false;
            bool previousDbTransitionProvenance = false;
            bool previousPmdContinuity = false;
            bool previousGenerateContinuity = false;
            bool nativeTakeover = false;
            bool dbExitTransition = false;
            bool authoredRotationEntryDeferred = false;
            bool bootstrapKeyIsAuthoredRotation = false;
            bool actorInvalid = false;
            bool keepSession = false;
            bool sessionExpired = false;
            bool entryTimedOut = false;
            bool sessionStartedNow = false;
            bool explicitRelease = false;
            std::uint32_t entryAgeFrames = 0;
            float zeroRootSeconds = 0.0f;
            std::uint64_t sourceFrame = 0;
            std::uint64_t nextFrame = 0;
            std::uint64_t continuityHolds = 0;
            std::uint64_t continuityMisses = 0;
            std::string bootstrapKey;
            const float safeDt =
                (std::isfinite(deltaTime) && deltaTime > 0.0f) ? deltaTime : 0.0f;
            const auto* lifecycleProfile = MotionProfileDatabase::ResolveProfile(character);
            {
                std::scoped_lock lock{ detail::g_mutex };
                auto& state = detail::g_actorState[character->GetFormID()];
                sourceFrame = state.frameSerial;
                nextFrame = state.frameSerial + 1;
                ownedBefore = state.creatureDrivenLatched;
                sessionBefore = state.creatureDbSessionActive;
                ownershipBefore = state.creatureDbOwnershipEstablished;
                originalDriven = state.creatureOriginalDriven;
                bootstrapKey = state.creatureDbObservedKey;
                bootstrapKeyIsAuthoredRotation =
                    lifecycleProfile &&
                    std::find(
                        lifecycleProfile->rotationClips.begin(),
                        lifecycleProfile->rotationClips.end(),
                        bootstrapKey) != lifecycleProfile->rotationClips.end();

                const float previousPlanar = std::hypot(
                    state.previousRootMix.mixedX,
                    state.previousRootMix.mixedY);
                previousDbRoot =
                    state.frameSerial != 0 &&
                    state.previousRootMix.dbClips > 0 &&
                    state.previousRootMix.mixedTranslationValid &&
                    std::isfinite(previousPlanar) &&
                    previousPlanar > detail::kRootContributionEpsilon &&
                    state.previousDbRootKeyCount > 0;
                previousDbWrite =
                    state.frameSerial != 0 &&
                    state.externalTranslationWriteFrame == state.frameSerial;
                previousDbObserved =
                    state.frameSerial != 0 &&
                    state.creatureDbObservedFrame == state.frameSerial;
                // R15.12 exit-grace transplant: acquisition provenance is ONLY
                // proven INPLACE -> DB ROOT entry provenance. Generic provenance
                // also includes DB_ROOT -> INPLACE and must not reacquire the gate.
                previousDbProvenance =
                    state.frameSerial != 0 &&
                    state.creatureDbEntryProvenanceFrame == state.frameSerial;
                previousDbTransitionProvenance =
                    state.frameSerial != 0 &&
                    state.creatureDbProvenanceSeenFrame == state.frameSerial;
                previousPmdContinuity =
                    ownershipBefore && state.frameSerial != 0 &&
                    state.creatureDbPmdContinuityFrame == state.frameSerial;
                previousGenerateContinuity =
                    ownershipBefore && state.frameSerial != 0 &&
                    (state.creatureDbGenerateContinuityFrame == state.frameSerial || previousDbRoot);

                // A strongly mixed DB root can legitimately evolve from walk to
                // curve/trot/run while ownership remains DB->DB. Merge only keys
                // that actually participated in a valid mixed root frame.
                if (ownershipBefore && previousDbRoot) {
                    for (std::size_t i = 0; i < state.previousDbRootKeyCount &&
                         i < state.previousDbRootKeys.size(); ++i) {
                        detail::AddUniqueDbKey(
                            state.creatureDbOwnershipKeys,
                            state.creatureDbOwnershipKeyCount,
                            state.previousDbRootKeys[i]);
                    }
                    state.creatureDbGenerateContinuityFrame = state.frameSerial;
                    previousGenerateContinuity = true;
                }

                // R15.12 exit-grace transplant: serialized IGNORE_FROM remains an
                // immediate native/canned takeover. Runtime IGNORE_TO is only an
                // EXIT CANDIDATE because a gait-family handoff can temporarily look
                // ROOT->INPLACE before the new DB gait establishes its first WFM.
                // The existing bounded continuity timer decides whether the exit is real.
                nativeTakeover =
                    ownershipBefore && state.frameSerial != 0 &&
                    state.wfmIgnoreFromSeenFrame == state.frameSerial;
                dbExitTransition =
                    ownershipBefore && state.frameSerial != 0 &&
                    state.wfmRuntimeIgnoreToSeenFrame == state.frameSerial;
                actorInvalid = character->IsDisabled() || character->IsDead();
                explicitRelease = nativeTakeover || actorInvalid;

                // R41: authored RotationClips may not bootstrap a brand-new DB session
                // from residual/root-only evidence. After attack/turn exit, a TurnLoop can
                // remain in the graph for a few frames and still emit tiny WFM/root deltas;
                // using that alone repeatedly re-armed bAnimationDriven and created
                // ENTRY_PENDING -> ENTRY_TIMEOUT churn. A fresh authored-turn session now
                // requires the existing strong INPLACE_TO_DB_ROOT provenance proof.
                //
                // Locomotion RuntimeYawClips keep the proven R28/R35 root-only bootstrap,
                // because that is the Golden Gait path that re-arms walk/trot/run.
                const bool rootOrWriteEvidence = previousDbRoot || previousDbWrite;
                const bool acquisitionEvidence =
                    previousDbProvenance ||
                    (rootOrWriteEvidence && !bootstrapKeyIsAuthoredRotation);
                authoredRotationEntryDeferred =
                    !sessionBefore &&
                    bootstrapKeyIsAuthoredRotation &&
                    rootOrWriteEvidence &&
                    !previousDbProvenance;

                const bool continuityEvidence =
                    ownershipBefore ?
                        (previousDbRoot || previousDbWrite ||
                         (previousDbTransitionProvenance && !dbExitTransition) ||
                         previousPmdContinuity || previousGenerateContinuity) :
                        acquisitionEvidence;

                if (explicitRelease) {
                    state.creatureDbSessionActive = false;
                    state.creatureDbSessionZeroRootSeconds = 0.0f;
                    state.creatureDbEntryPending = false;
                    state.creatureDbOwnershipEstablished = false;
                    state.creatureDbEntryAgeSeconds = 0.0f;
                    state.creatureDbEntryAgeFrames = 0;
                    state.creatureDbPmdContinuityFrame = 0;
                    state.creatureDbGenerateContinuityFrame = 0;
                    detail::ClearDbOwnershipKeysLocked(state);
                    keepSession = false;
                    if (sessionBefore) {
                        ++state.creatureDbSessionStops;
                    }
                    if (ownershipBefore) {
                        ++state.creatureDbExplicitReleases;
                    }
                } else if (!sessionBefore && acquisitionEvidence) {
                    state.creatureDbSessionActive = true;
                    state.creatureDbSessionZeroRootSeconds = 0.0f;
                    keepSession = true;
                    ++state.creatureDbSessionStarts;
                    sessionStartedNow = true;
                    state.creatureDbOwnershipEstablished = false;
                    state.creatureDbEntryPending = true;
                    state.creatureDbEntryAgeSeconds = 0.0f;
                    state.creatureDbEntryAgeFrames = 0;
                    state.creatureDbEntryStartFrame = state.frameSerial + 1;
                    detail::ClearDbOwnershipKeysLocked(state);
                } else if (sessionBefore && continuityEvidence) {
                    state.creatureDbSessionActive = true;
                    state.creatureDbSessionZeroRootSeconds = 0.0f;
                    keepSession = true;
                    if (ownershipBefore) {
                        continuityHolds = ++state.creatureDbContinuityHolds;
                    }
                } else if (sessionBefore) {
                    state.creatureDbSessionZeroRootSeconds += safeDt;
                    zeroRootSeconds = state.creatureDbSessionZeroRootSeconds;
                    keepSession = zeroRootSeconds < detail::kDbSessionReleaseSeconds;
                    if (ownershipBefore) {
                        continuityMisses = ++state.creatureDbContinuityMisses;
                    }
                    if (!keepSession) {
                        state.creatureDbSessionActive = false;
                        state.creatureDbSessionZeroRootSeconds = 0.0f;
                        state.creatureDbEntryPending = false;
                        state.creatureDbOwnershipEstablished = false;
                        state.creatureDbEntryAgeSeconds = 0.0f;
                        state.creatureDbEntryAgeFrames = 0;
                        state.creatureDbPmdContinuityFrame = 0;
                        state.creatureDbGenerateContinuityFrame = 0;
                        detail::ClearDbOwnershipKeysLocked(state);
                        sessionExpired = true;
                        ++state.creatureDbSessionStops;
                    }
                }

                if (!explicitRelease && state.creatureDbEntryPending) {
                    state.creatureDbEntryAgeSeconds += safeDt;
                    ++state.creatureDbEntryAgeFrames;
                    entryAgeFrames = state.creatureDbEntryAgeFrames;
                    if (state.creatureDbEntryAgeSeconds >= detail::kDbOwnershipEntryTimeoutSeconds ||
                        state.creatureDbEntryAgeFrames >= detail::kDbOwnershipEntryTimeoutFrames) {
                        state.creatureDbEntryPending = false;
                        state.creatureDbOwnershipEstablished = false;
                        state.creatureDbEntryAgeSeconds = 0.0f;
                        state.creatureDbEntryAgeFrames = 0;
                        state.creatureDbSessionActive = false;
                        state.creatureDbSessionZeroRootSeconds = 0.0f;
                        state.creatureDbPmdContinuityFrame = 0;
                        state.creatureDbGenerateContinuityFrame = 0;
                        detail::ClearDbOwnershipKeysLocked(state);
                        keepSession = false;
                        sessionExpired = true;
                        entryTimedOut = true;
                        ++state.creatureDbEntryTimeouts;
                        ++state.creatureDbSessionStops;
                    }
                }

                // Upcoming frame ownership is re-established below only if we are
                // still the writer of bAnimationDriven.
                state.creatureDrivenLatched = false;
                state.creatureBootstrapDrivenFrame = 0;
            }

            if (authoredRotationEntryDeferred) {
                SKSE::log::info(
                    "[DB_OWNERSHIP][R41_ROTATION_ENTRY_DEFERRED] form={:08X} sourceFrame={} targetFrame={} key=\"{}\" evidence[root={} tx={} provenance={}] action=WAIT_FOR_INPLACE_TO_DB_ROOT_PROVENANCE noBAnimationDrivenBootstrap=true",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                    previousDbRoot, previousDbWrite, previousDbProvenance);
            }

            if (sessionStartedNow) {
                SKSE::log::info(
                    "[DB_OWNERSHIP][ENTRY_PENDING] form={:08X} sourceFrame={} targetFrame={} key=\"{}\" evidence[root={} tx={} provenance={}] action=WAIT_FOR_STRONG_GATE_PROOF",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                    previousDbRoot, previousDbWrite, previousDbProvenance);
            }

            if (nativeTakeover) {
                SKSE::log::info(
                    "[DB_OWNERSHIP][RELEASE] form={:08X} sourceFrame={} nextFrame={} key=\"{}\" reason=NATIVE_SERIALIZED_IGNORE_FROM action=RELINQUISH_WITHOUT_FALSE_RESTORE",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey);
            } else if (dbExitTransition && ownershipBefore) {
                SKSE::log::debug("[DB_OWNERSHIP][EXIT_CANDIDATE] form={:08X} sourceFrame={} nextFrame={} key=\"{}\" proof=DB_ROOT_TO_INPLACE_CALLBACK action=DEFER_RELEASE graceSeconds={:.3f} note=NEW_DB_GAIT_MAY_BE_IN_CLIP_WAIT_REBASE",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                    detail::kDbSessionReleaseSeconds);
            } else if (actorInvalid && ownershipBefore) {
                SKSE::log::info(
                    "[DB_OWNERSHIP][RELEASE] form={:08X} sourceFrame={} nextFrame={} key=\"{}\" reason=ACTOR_INVALID action=RESTORE_IF_OWNED",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey);
            } else if (entryTimedOut) {
                SKSE::log::warn(
                    "[DB_OWNERSHIP][ENTRY_ABORT] form={:08X} sourceFrame={} nextFrame={} key=\"{}\" ageFrames={} timeoutFrames={} timeoutSeconds={:.3f} action=END_FAILED_SESSION_AND_RESTORE_BANIMATIONDRIVEN",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                    entryAgeFrames, detail::kDbOwnershipEntryTimeoutFrames,
                    detail::kDbOwnershipEntryTimeoutSeconds);
            } else if (sessionExpired && ownershipBefore) {
                SKSE::log::info(
                    "[DB_OWNERSHIP][RELEASE] form={:08X} sourceFrame={} nextFrame={} key=\"{}\" reason=CONTINUITY_TIMEOUT misses={} graceSeconds={:.3f} action=RESTORE_IF_OWNED",
                    character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                    continuityMisses, detail::kDbSessionReleaseSeconds);
            }

            if (!explicitRelease && keepSession && detail::IsRuntimeEligibleForExternalRoot(character)) {
                bool currentDriven = false;
                const bool currentRead =
                    character->GetGraphVariableBool("bAnimationDriven", currentDriven);
                bool accepted = true;
                bool readbackRead = currentRead;
                bool readback = currentDriven;
                bool ownedForNextFrame = false;
                bool newlyArmed = false;

                if (ownedBefore && currentRead && currentDriven) {
                    ownedForNextFrame = true;
                } else if (currentRead && !currentDriven) {
                    accepted = character->SetGraphVariableBool("bAnimationDriven", true);
                    readbackRead = character->GetGraphVariableBool("bAnimationDriven", readback);
                    ownedForNextFrame = accepted && readbackRead && readback;
                    newlyArmed = ownedForNextFrame;
                } else if (!ownedBefore && currentRead && currentDriven) {
                    // Native graph is already animation-driven. DB authority may be
                    // latched without claiming this variable as our property.
                    ownedForNextFrame = false;
                }

                std::uint64_t armCount = 0;
                std::uint64_t holdCount = 0;
                bool logArm = false;
                bool logHold = false;
                if (ownedForNextFrame) {
                    std::scoped_lock lock{ detail::g_mutex };
                    auto& state = detail::g_actorState[character->GetFormID()];
                    state.creatureDrivenLatched = true;
                    if (!ownedBefore) {
                        state.creatureOriginalDriven = currentDriven;
                    } else {
                        state.creatureOriginalDriven = originalDriven;
                    }
                    state.creatureBootstrapDrivenFrame = nextFrame;
                    if (newlyArmed) {
                        armCount = ++state.creatureArmCount;
                        logArm = armCount <= 8 ||
                            state.creatureLastArmLoggedKey != bootstrapKey ||
                            (armCount % 240) == 0;
                        if (logArm) {
                            state.creatureLastArmLoggedKey = bootstrapKey;
                        }
                    } else {
                        holdCount = ++state.creatureHoldCount;
                        logHold = holdCount <= 8 || (holdCount % 240) == 0;
                    }
                } else if (!(currentRead && currentDriven)) {
                    std::scoped_lock lock{ detail::g_mutex };
                    ++detail::g_actorState[character->GetFormID()].creatureArmFailCount;
                }

                if (logArm) {
                    SKSE::log::info(
                        "[DB_SESSION][START] form={:08X} sourceFrame={} targetFrame={} key=\"{}\" evidence[root={} tx={} provenance={} observed={}] accepted={} readback={} arms={} zeroRootSeconds={:.4f}",
                        character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                        previousDbRoot, previousDbWrite, previousDbProvenance, previousDbObserved,
                        accepted, readback, armCount, zeroRootSeconds);
                } else if (logHold) {
                    SKSE::log::info(
                        "[DB_SESSION][HOLD] form={:08X} sourceFrame={} targetFrame={} key=\"{}\" continuity[pmd={} generate={} root={} tx={} provenance={}] holds={} lifecycleHolds={} zeroRootSeconds={:.4f}",
                        character->GetFormID(), sourceFrame, nextFrame, bootstrapKey,
                        previousPmdContinuity, previousGenerateContinuity, previousDbRoot,
                        previousDbWrite, previousDbProvenance,
                        holdCount, continuityHolds, zeroRootSeconds);
                } else if (sessionBefore && ownershipBefore &&
                           !previousDbRoot && !previousDbWrite && !previousDbProvenance &&
                           !previousPmdContinuity && !previousGenerateContinuity &&
                           zeroRootSeconds > 0.0f) {
                    SKSE::log::info(
                        "[DB_SESSION][GRACE] form={:08X} sourceFrame={} targetFrame={} continuityMisses={} zeroRootSeconds={:.4f}/{:.4f} action=KEEP_ACQUIRED_DB_LATCH_DURING_CALLBACK_GAP",
                        character->GetFormID(), sourceFrame, nextFrame,
                        continuityMisses, zeroRootSeconds, detail::kDbSessionReleaseSeconds);
                }
            } else if (!nativeTakeover && ownedBefore && (sessionExpired || actorInvalid)) {
                const bool accepted =
                    character->SetGraphVariableBool("bAnimationDriven", originalDriven);
                std::uint64_t restoreCount = 0;
                {
                    std::scoped_lock lock{ detail::g_mutex };
                    auto& state = detail::g_actorState[character->GetFormID()];
                    restoreCount = ++state.creatureRestoreCount;
                    if (!accepted) {
                        state.creatureDrivenLatched = true;
                        state.creatureOriginalDriven = originalDriven;
                        state.creatureBootstrapDrivenFrame = nextFrame;
                    }
                }
                if (accepted) {
                    SKSE::log::info(
                        "[DB_SESSION][STOP] form={:08X} sourceFrame={} nextFrame={} requested={} restores={} reason={}",
                        character->GetFormID(), sourceFrame, nextFrame,
                        originalDriven, restoreCount,
                        entryTimedOut ? "ENTRY_TIMEOUT" :
                        actorInvalid ? "ACTOR_INVALID" : "CONTINUITY_TIMEOUT");
                } else {
                    SKSE::log::warn(
                        "[DB_SESSION][STOP_FAIL] form={:08X} sourceFrame={} nextFrame={} requested={} restores={} action=RETRY_WITH_OWNERSHIP",
                        character->GetFormID(), sourceFrame, nextFrame,
                        originalDriven, restoreCount);
                }
            }
        }

        bool announcePlayerFrame = false;
        bool emitVanillaYawSnapshot = false;
        std::uint64_t frame = 0;
        float snapshotRate = 0.0f;
        float snapshotPathDesiredRate = 0.0f;
        float snapshotPathCurrentRate = 0.0f;
        float snapshotMiddleRotationZ = 0.0f;
        float snapshotDesiredMoveX = 0.0f;
        float snapshotDesiredMoveY = 0.0f;
        {
            std::scoped_lock lock{ detail::g_mutex };
            auto& state = detail::g_actorState[character->GetFormID()];
            state.profile = MotionProfileDatabase::DatabaseFolderForActor(character);
            state.frameDeltaTime = (std::isfinite(deltaTime) && deltaTime > 0.0f) ? deltaTime : 0.0f;
            ++state.frameSerial;

            if (preFrameYawEligible) {
                state.vanillaYawPreFrameSampleFrame = state.frameSerial;
                state.vanillaYawPreFrameValid = preFrameYawRateValid;
                state.vanillaYawPreFrameRateRadPerSecond =
                    preFrameYawRateValid ? preFrameYawRateCandidate : 0.0f;
                state.vanillaYawPreFramePathDesiredRateRadPerSecond =
                    std::isfinite(preFramePathDesiredCandidate) ? preFramePathDesiredCandidate : 0.0f;
                state.vanillaYawPreFramePathCurrentRateRadPerSecond =
                    std::isfinite(preFramePathCurrentCandidate) ? preFramePathCurrentCandidate : 0.0f;
                state.vanillaYawPreFrameMiddleRotationZ =
                    std::isfinite(preFrameMiddleRotationCandidate) ? preFrameMiddleRotationCandidate : 0.0f;
                state.vanillaYawPreFrameDesiredMoveX =
                    std::isfinite(preFrameDesiredMoveXCandidate) ? preFrameDesiredMoveXCandidate : 0.0f;
                state.vanillaYawPreFrameDesiredMoveY =
                    std::isfinite(preFrameDesiredMoveYCandidate) ? preFrameDesiredMoveYCandidate : 0.0f;

                snapshotRate = state.vanillaYawPreFrameRateRadPerSecond;
                snapshotPathDesiredRate = state.vanillaYawPreFramePathDesiredRateRadPerSecond;
                snapshotPathCurrentRate = state.vanillaYawPreFramePathCurrentRateRadPerSecond;
                snapshotMiddleRotationZ = state.vanillaYawPreFrameMiddleRotationZ;
                snapshotDesiredMoveX = state.vanillaYawPreFrameDesiredMoveX;
                snapshotDesiredMoveY = state.vanillaYawPreFrameDesiredMoveY;

                const float changeThreshold = 10.0f / detail::kRadiansToDegrees;
                const bool validityChanged =
                    state.vanillaYawLastSnapshotValid != preFrameYawRateValid;
                const bool rateChanged =
                    !state.vanillaYawLastSnapshotValid ||
                    std::abs(snapshotRate - state.vanillaYawLastSnapshotRateRadPerSecond) >= changeThreshold;
                emitVanillaYawSnapshot =
                    state.vanillaYawSnapshotLogs < 32 ||
                    validityChanged ||
                    rateChanged ||
                    (state.frameSerial % 120) == 0;
                if (emitVanillaYawSnapshot) {
                    ++state.vanillaYawSnapshotLogs;
                    state.vanillaYawLastSnapshotValid = preFrameYawRateValid;
                    state.vanillaYawLastSnapshotRateRadPerSecond = snapshotRate;
                }
            }

            state.syntheticWfmInjectionCount = 0;
            if (!state.syntheticClipPhase.empty() && (state.frameSerial % 120) == 0) {
                for (auto it = state.syntheticClipPhase.begin(); it != state.syntheticClipPhase.end();) {
                    const bool stale =
                        state.frameSerial > it->second.lastSeenFrame + 240;
                    if (stale) {
                        it = state.syntheticClipPhase.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            if ((state.frameSerial % 240) == 0) {
                for (auto it = state.nativeTranslationPairs.begin();
                     it != state.nativeTranslationPairs.end();) {
                    if (state.frameSerial > it->second.frame + 240) {
                        it = state.nativeTranslationPairs.erase(it);
                    } else {
                        ++it;
                    }
                }
                for (auto it = state.nativeRotationPairs.begin();
                     it != state.nativeRotationPairs.end();) {
                    if (state.frameSerial > it->second.frame + 240) {
                        it = state.nativeRotationPairs.erase(it);
                    } else {
                        ++it;
                    }
                }
                for (auto it = state.nativeExtractedDeltas.begin();
                     it != state.nativeExtractedDeltas.end();) {
                    if (state.frameSerial > it->second.frame + 240) {
                        it = state.nativeExtractedDeltas.erase(it);
                    } else {
                        ++it;
                    }
                }
            }
            state.hkbCharacter = hkbCharacter;
            state.hkbCharacters = hkbCharacters;
            state.hkbCharacterCount = hkbCharacterCount;
            state.isPlayer = character->IsPlayerRef();
            frame = state.frameSerial;
            if (character->IsPlayerRef()) {
                ++state.playerFrameCount;
                if (!state.playerFrameAnnounced) {
                    state.playerFrameAnnounced = true;
                    announcePlayerFrame = true;
                }
            }
        }
        if (emitVanillaYawSnapshot) {
            SKSE::log::info(
                "[VANILLA_YAW_PREFRAME][SNAPSHOT] form={:08X} frame={} valid={} rotationSpeedZ={:+.6f}rad/s/{:+.2f}deg/s pathDesiredRate={:+.6f}rad/s pathCurrentRate={:+.6f}rad/s middleRotationZ={:+.6f}rad desiredMoveXY=({:+.3f},{:+.3f}) source=PRE_CHARACTER_UPDATE_BEFORE_NATIVE_CONSUMPTION",
                character->GetFormID(), frame, preFrameYawRateValid,
                snapshotRate, snapshotRate * detail::kRadiansToDegrees,
                snapshotPathDesiredRate, snapshotPathCurrentRate,
                snapshotMiddleRotationZ, snapshotDesiredMoveX, snapshotDesiredMoveY);
        }
        if (announcePlayerFrame) {
            SKSE::log::info(
                "[PlayerRootMotion][FRAME] form={:08X} frame={} activeHkb=0x{:X} graphCharacters={} Player runtime is disabled in Incessus.",
                character->GetFormID(), frame, reinterpret_cast<std::uintptr_t>(hkbCharacter), hkbCharacterCount);
        }
    }

    void BeginAnimationEvaluation(RE::Character* character)
    {
        RE::Character* value = detail::IsExternalMotionActor(character) ? character : nullptr;
        if (detail::g_evaluationDepth < detail::kEvaluationStackCapacity) {
            detail::g_evaluationStack[detail::g_evaluationDepth++] = value;
        } else {
            detail::g_evaluationStack[detail::kEvaluationStackCapacity - 1] = value;
        }
    }

    void EndAnimationEvaluation(RE::Character*)
    {
        if (detail::g_evaluationDepth == 0) {
            return;
        }
        --detail::g_evaluationDepth;
        detail::g_evaluationStack[detail::g_evaluationDepth] = nullptr;
    }

    [[nodiscard]] bool HasExternalTranslationForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor)) {
            return false;
        }
        bool active = false;
        bool firstPassThrough = false;
        std::uint64_t writes = 0;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(actor->GetFormID());
            if (it == detail::g_actorState.end()) {
                return false;
            }
            auto& state = it->second;
            active = state.frameSerial != 0 &&
                state.externalTranslationWriteFrame == state.frameSerial;
            if (active) {
                firstPassThrough = state.externalPassThroughs++ == 0;
                writes = state.externalTranslationWrites;
            }
        }
        if (firstPassThrough) {
            SKSE::log::info(
                "[TRANSLATION_GATE][DB_AUTHORITY_FIRST] form={:08X} first frame reached ModifyMovementData after a real MotionDB translation write; writes={}.",
                actor->GetFormID(), writes);
        }
        return active;
    }

    [[nodiscard]] bool HasSyntheticWfmTranslationForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        const bool actorDriven = actor->IsAnimationDriven();
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;

        // R15.8: this API is acquisition proof only. Once ownership is ACQUIRED,
        // the post-acquisition DB latch is queried through
        // HasDbMotionSourceAuthorityForCurrentFrame instead of demanding another
        // fresh Generate callback every frame.
        if (state.creatureDbOwnershipEstablished) {
            return false;
        }
        if (state.frameSerial == 0 || state.syntheticWfmTranslationSeenFrame == 0 ||
            !state.creatureDbSessionActive) {
            return false;
        }

        const bool freshSynthetic =
            state.syntheticWfmTranslationSeenFrame == state.frameSerial ||
            state.syntheticWfmTranslationSeenFrame + 1 == state.frameSerial;
        if (!freshSynthetic) {
            return false;
        }

        const bool graphDriven = state.creatureDrivenLatched || actorDriven;
        if (!graphDriven) {
            return false;
        }

        // Path A: transition-local entry proof from R15.6.
        const bool freshEntryProvenance =
            state.creatureDbEntryProvenanceFrame == state.frameSerial ||
            state.creatureDbEntryProvenanceFrame + 1 == state.frameSerial;
        const bool transitionProof =
            state.creatureDbEntryPending &&
            state.creatureDbEntryPolicyWrites > 0 &&
            freshEntryProvenance;

        // Path B: stable no-transition composition. This remains a strong
        // acquisition proof and is not weakened by the lifecycle latch.
        const float stablePlanar = std::hypot(
            state.previousRootMix.mixedX,
            state.previousRootMix.mixedY);
        const bool stableCompositionProof =
            state.creatureDbEntryPending &&
            state.previousTransitionCalls == 0 &&
            state.wfmTransitionCallsFrame == 0 &&
            state.previousRootMix.dbClips > 0 &&
            state.previousRootMix.mixedTranslationValid &&
            state.previousDbRootKeyCount > 0 &&
            std::isfinite(stablePlanar) &&
            stablePlanar > detail::kRootContributionEpsilon;

        return transitionProof || stableCompositionProof;
    }

    [[nodiscard]] bool HasNativeMotionWitnessForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor)) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        if (state.frameSerial == 0 || state.nativeMotionSeenFrame == 0) {
            return false;
        }
        return state.nativeMotionSeenFrame == state.frameSerial ||
            state.nativeMotionSeenFrame + 1 == state.frameSerial;
    }

    [[nodiscard]] bool HasNativeAuthoredRotationForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor)) {
            return false;
        }
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        if (state.frameSerial == 0 || state.nativeRotationDeltaSeenFrame == 0) {
            return false;
        }
        return state.nativeRotationDeltaSeenFrame == state.frameSerial ||
            state.nativeRotationDeltaSeenFrame + 1 == state.frameSerial;
    }

    [[nodiscard]] bool HasNativeAuthoredTurnForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor) || detail::HasUniversalActionBypass(actor)) {
            return false;
        }
        std::string clip;
        bool fresh = false;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(actor->GetFormID());
            if (it == detail::g_actorState.end()) return false;
            const auto& state = it->second;
            if (state.frameSerial == 0 || state.nativeRotationDeltaSeenFrame == 0) return false;
            fresh = state.nativeRotationDeltaSeenFrame == state.frameSerial ||
                state.nativeRotationDeltaSeenFrame + 1 == state.frameSerial;
            clip = state.nativeRotationDeltaSeenClip;
        }
        if (!fresh || clip.empty()) return false;
        // Locomotion clips (including Wolf directional clips with authored cycle yaw)
        // remain procedural-yaw candidates. Only a rotating NON-locomotion carrier is
        // exposed as a native authored-turn witness; actions are handled separately.
        return !detail::IsCertifiedNativeLocomotionCarrierName(clip);
    }

    [[nodiscard]] UniversalRootResult ResolveUniversalExtractedTranslation(
        RE::Actor* actor, float deltaTime, RE::NiPoint3& outTranslation)
    {
        outTranslation = {};
        if (!detail::IsUniversalCreatureActor(actor) || !detail::IsRuntimeEligibleForExternalRoot(actor) ||
            MotionProfileDatabase::SupportsActor(actor)) {
            ReleaseUniversalExtractedRootAuthority(actor);
            return UniversalRootResult::kNone;
        }
        const bool movementDemand = SpeedIntentTranslator::HasUniversalMovementDemand(actor);
        const bool actionBypass = detail::HasUniversalActionBypass(actor);
        if (!movementDemand || actionBypass) {
            ReleaseUniversalExtractedRootAuthority(actor);
            return UniversalRootResult::kNone;
        }

        const auto finalRoot = detail::ProbeFinalBehaviorTranslation(actor);
        const float dt = std::isfinite(deltaTime) && deltaTime > 1.0e-5f ? deltaTime : 0.0f;
        bool exact = false;
        RE::NiPoint3 exactDelta{};
        std::uint64_t frame = 0;
        std::uint64_t leafFrame = 0;
        bool logExact = false;
        bool logBridge = false;
        bool logHold = false;
        bool logWait = false;
        UniversalRootResult result = UniversalRootResult::kNone;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(actor->GetFormID());
            if (it == detail::g_actorState.end()) return UniversalRootResult::kNone;
            auto& state = it->second;
            frame = state.frameSerial;
            leafFrame = state.universalExtractedLeafSeenFrame;
            const bool provenanceFresh = frame != 0 && leafFrame != 0 &&
                (leafFrame == frame || leafFrame + 1 == frame);
            const float planar = std::hypot(finalRoot.delta.x, finalRoot.delta.y);
            exact = provenanceFresh && finalRoot.graphReadable && !finalRoot.ambiguous &&
                finalRoot.selectedOutput >= 0 && std::isfinite(planar) &&
                planar > detail::kRootContributionEpsilon &&
                std::isfinite(finalRoot.delta.x) && std::isfinite(finalRoot.delta.y);

            if (exact) {
                exactDelta = finalRoot.delta;
                outTranslation = exactDelta;
                state.universalExtractedAuthorityEstablished = true;
                state.universalFinalRootFrame = frame;
                state.universalFinalRootDelta = exactDelta;
                state.universalRootGapFrames = 0;
                ++state.universalRootExactFrames;
                if (dt > 0.0f) {
                    state.universalLastVelocity = {
                        exactDelta.x / dt,
                        exactDelta.y / dt,
                        exactDelta.z / dt
                    };
                }
                result = UniversalRootResult::kExact;
                logExact = state.universalRootLogs < 64 || (frame % 120) == 0;
                if (logExact) ++state.universalRootLogs;
            } else if (state.universalExtractedAuthorityEstablished) {
                ++state.universalRootGapFrames;
                const bool bridgeValid = state.universalRootGapFrames <= 4 && dt > 0.0f &&
                    std::isfinite(state.universalLastVelocity.x) &&
                    std::isfinite(state.universalLastVelocity.y);
                if (bridgeValid) {
                    outTranslation = {
                        state.universalLastVelocity.x * dt,
                        state.universalLastVelocity.y * dt,
                        0.0f
                    };
                    ++state.universalRootBridgeFrames;
                    result = UniversalRootResult::kBridge;
                    logBridge = state.universalRootGapFrames == 1 || state.universalRootGapFrames == 4;
                } else {
                    outTranslation = {};
                    result = UniversalRootResult::kHoldZero;
                    logHold = state.universalRootGapFrames == 5 || (frame % 120) == 0;
                }
            } else {
                logWait = state.universalFinalWaitLogs < 16 || (frame % 120) == 0;
                if (logWait) ++state.universalFinalWaitLogs;
            }
        }

        if (logExact) {
            SKSE::log::info(
                "[C1_NATIVE_EXTRACTED][FINAL] form={:08X} frame={} leafFrame={} output={} interpolation={:.4f} on={:.4f} finalDelta=({:+.5f},{:+.5f}) action=OPEN_EXACT_FINAL_HKB_ROOT",
                actor->GetFormID(), frame, leafFrame, finalRoot.selectedOutput,
                finalRoot.interpolationAmount, finalRoot.onFraction,
                exactDelta.x, exactDelta.y);
        } else if (logBridge) {
            SKSE::log::info(
                "[C1_NATIVE_EXTRACTED][BRIDGE] form={:08X} frame={} delta=({:+.5f},{:+.5f}) action=LAST_FINAL_VELOCITY_X_DT",
                actor->GetFormID(), frame, outTranslation.x, outTranslation.y);
        } else if (logHold) {
            SKSE::log::warn(
                "[C1_NATIVE_EXTRACTED][HOLD] form={:08X} frame={} leafFrame={} graphReadable={} ambiguous={} selected={} action=ZERO_XY_AFTER_ESTABLISHED_ROOT_STALE",
                actor->GetFormID(), frame, leafFrame, finalRoot.graphReadable,
                finalRoot.ambiguous, finalRoot.selectedOutput);
        } else if (logWait) {
            SKSE::log::debug(
                "[C1_NATIVE_EXTRACTED][FINAL_WAIT] form={:08X} frame={} leafFrame={} graphReadable={} ambiguous={} selected={} finalDelta=({:+.5f},{:+.5f}) action=FAIL_OPEN_VANILLA_UNTIL_FIRST_FINAL_ROOT",
                actor->GetFormID(), frame, leafFrame, finalRoot.graphReadable,
                finalRoot.ambiguous, finalRoot.selectedOutput, finalRoot.delta.x, finalRoot.delta.y);
        }
        return result;
    }

    [[nodiscard]] bool HasUniversalExtractedRootAuthority(RE::Actor* actor)
    {
        if (!detail::IsUniversalCreatureActor(actor) || MotionProfileDatabase::SupportsActor(actor)) return false;
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        return it != detail::g_actorState.end() && it->second.universalExtractedAuthorityEstablished;
    }

    void ReleaseUniversalExtractedRootAuthority(RE::Actor* actor)
    {
        if (!actor) return;
        bool released = false;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(actor->GetFormID());
            if (it == detail::g_actorState.end()) return;
            auto& state = it->second;
            released = state.universalExtractedAuthorityEstablished;
            state.universalExtractedAuthorityEstablished = false;
            state.universalRootGapFrames = 0;
            state.universalLastVelocity = {};
            state.universalFinalRootDelta = {};
            state.universalFinalRootFrame = 0;
            if (released) ++state.universalRootReleaseCount;
        }
        if (released) {
            SKSE::log::info(
                "[C1_NATIVE_EXTRACTED][RELEASE] form={:08X} reason=NO_MOVEMENT_DEMAND_OR_ACTION_BYPASS action=RETURN_VANILLA",
                actor->GetFormID());
        }
    }

    [[nodiscard]] bool IsUniversalCreatureEligible(RE::Actor* actor)
    {
        return detail::IsUniversalCreatureActor(actor);
    }

    [[nodiscard]] bool HasNativeMotionSessionForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor)) {
            return false;
        }

        const bool actorDriven = actor->IsAnimationDriven();
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        return state.nativeLocomotionSessionActive &&
            (state.nativeLocomotionDrivenLatched || actorDriven);
    }

    void ConfirmNativeLocomotionRootAuthorityAtGate(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor) || !actor->IsAnimationDriven() ||
            !detail::HasNativeLocomotionDemand(actor) || !detail::HasNativeLocomotionContract(actor) ||
            detail::HasUniversalActionBypass(actor)) {
            return;
        }
        bool acquired = false;
        std::uint64_t frame = 0;
        std::string clip;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(actor->GetFormID());
            if (it == detail::g_actorState.end()) {
                return;
            }
            auto& state = it->second;
            frame = state.frameSerial;
            const bool freshTranslationDelta =
                state.nativeTranslationDeltaSeenFrame != 0 &&
                (state.nativeTranslationDeltaSeenFrame == state.frameSerial ||
                 state.nativeTranslationDeltaSeenFrame + 1 == state.frameSerial);
            if (state.nativeLocomotionSessionActive &&
                !state.nativeLocomotionAuthorityEstablished &&
                freshTranslationDelta) {
                state.nativeLocomotionAuthorityEstablished = true;
                state.nativeLocomotionEntryAgeFrames = 0;
                clip = state.nativeTranslationDeltaSeenClip;
                acquired = true;
            }
        }
        if (acquired) {
            SKSE::log::info(
                "[B8_NATIVE_LOCOMOTION][PROFILE_ACQUIRED] form={:08X} frame={} clip=\"{}\" proof=VANILLA_TRANSLATION_T1_T2_PLUS_SESSION_BANIMATIONDRIVEN action=LATCH_NATIVE_ROOT_AUTHORITY",
                actor->GetFormID(), frame, clip);
        }
    }

    [[nodiscard]] bool HasNativeLocomotionRootAuthorityForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor) || !actor->IsAnimationDriven()) {
            return false;
        }
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        return state.nativeLocomotionSessionActive &&
            state.nativeLocomotionAuthorityEstablished;
    }

    [[nodiscard]] bool IsNativeLocomotionBootstrapDrivenForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsUniversalNativeMotionActor(actor)) {
            return false;
        }
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        return state.nativeLocomotionDrivenLatched &&
            state.nativeLocomotionBootstrapDrivenFrame == state.frameSerial;
    }

    [[nodiscard]] bool HasDbMotionSourceAuthorityForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        const bool actorDriven = actor->IsAnimationDriven();
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        if (!state.creatureDbOwnershipEstablished || !state.creatureDbSessionActive) {
            return false;
        }

        const bool graphDriven = state.creatureDrivenLatched || actorDriven;
        if (!graphDriven || state.creatureDbOwnershipKeyCount == 0) {
            return false;
        }

        // Explicit release signals win immediately over an old latch, even before
        // BeginActorFrame has had a chance to retire the session bookkeeping.
        const bool freshNativeTakeover =
            state.wfmIgnoreFromSeenFrame != 0 &&
            (state.wfmIgnoreFromSeenFrame == state.frameSerial ||
             state.wfmIgnoreFromSeenFrame + 1 == state.frameSerial);
        const bool freshDbExit =
            state.wfmRuntimeIgnoreToSeenFrame != 0 &&
            (state.wfmRuntimeIgnoreToSeenFrame == state.frameSerial ||
             state.wfmRuntimeIgnoreToSeenFrame + 1 == state.frameSerial);
        if (freshNativeTakeover || freshDbExit) {
            return false;
        }

        // This is a latched authority state, NOT PMD authority. Matching PMD and
        // generated DB root only refresh creatureDbSessionActive in BeginActorFrame;
        // a bounded callback gap does not destroy a previously certified ownership.
        return true;
    }

    void ConfirmDbRootOwnershipAtGate(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor)) {
            return;
        }

        bool acquired = false;
        bool transitionProof = false;
        bool stableProof = false;
        bool graphDriven = false;
        std::uint64_t frame = 0;
        std::uint64_t writes = 0;
        std::uint64_t confirms = 0;
        std::size_t ownershipKeyCount = 0;
        std::uint64_t previousTransitions = 0;
        std::uint64_t currentTransitions = 0;
        std::uint32_t dbClips = 0;
        float dbShare = 0.0f;
        float mixedX = 0.0f;
        float mixedY = 0.0f;
        std::string key;
        const bool actorDriven = actor->IsAnimationDriven();
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(actor->GetFormID());
            if (it == detail::g_actorState.end()) {
                return;
            }
            auto& state = it->second;
            frame = state.frameSerial;
            writes = state.creatureDbEntryPolicyWrites;
            key = state.creatureDbObservedKey;
            previousTransitions = state.previousTransitionCalls;
            currentTransitions = state.wfmTransitionCallsFrame;
            dbClips = static_cast<std::uint32_t>(state.previousRootMix.dbClips);
            dbShare = state.previousRootMix.dbRootShare;
            mixedX = state.previousRootMix.mixedX;
            mixedY = state.previousRootMix.mixedY;
            graphDriven = state.creatureDrivenLatched || actorDriven;

            const bool freshEntryProvenance =
                state.creatureDbEntryProvenanceFrame == state.frameSerial ||
                state.creatureDbEntryProvenanceFrame + 1 == state.frameSerial;
            transitionProof =
                state.creatureDbEntryPolicyWrites > 0 &&
                freshEntryProvenance;

            const float stablePlanar = std::hypot(mixedX, mixedY);
            stableProof =
                state.previousTransitionCalls == 0 &&
                state.wfmTransitionCallsFrame == 0 &&
                state.previousRootMix.dbClips > 0 &&
                state.previousRootMix.mixedTranslationValid &&
                state.previousDbRootKeyCount > 0 &&
                std::isfinite(stablePlanar) &&
                stablePlanar > detail::kRootContributionEpsilon;

            if (state.creatureDbSessionActive &&
                graphDriven &&
                state.creatureDbEntryPending &&
                !state.creatureDbOwnershipEstablished &&
                (transitionProof || stableProof)) {
                state.creatureDbOwnershipEstablished = true;
                state.creatureDbEntryPending = false;
                state.creatureDbEntryAgeSeconds = 0.0f;
                state.creatureDbEntryAgeFrames = 0;
                detail::ClearDbOwnershipKeysLocked(state);
                for (std::size_t i = 0; i < state.previousDbRootKeyCount &&
                     i < state.previousDbRootKeys.size(); ++i) {
                    detail::AddUniqueDbKey(
                        state.creatureDbOwnershipKeys,
                        state.creatureDbOwnershipKeyCount,
                        state.previousDbRootKeys[i]);
                }
                for (std::size_t i = 0; i < state.syntheticWfmInjectionCount &&
                     i < state.syntheticWfmInjections.size(); ++i) {
                    const auto& witness = state.syntheticWfmInjections[i];
                    if (witness.applied && witness.frame == state.frameSerial) {
                        detail::AddUniqueDbKey(
                            state.creatureDbOwnershipKeys,
                            state.creatureDbOwnershipKeyCount,
                            witness.databaseKey);
                    }
                }
                if (state.creatureDbOwnershipKeyCount == 0) {
                    detail::AddUniqueDbKey(
                        state.creatureDbOwnershipKeys,
                        state.creatureDbOwnershipKeyCount,
                        key);
                }
                state.creatureDbGenerateContinuityFrame = state.frameSerial;
                confirms = ++state.creatureDbEntryConfirms;
                ownershipKeyCount = state.creatureDbOwnershipKeyCount;
                acquired = true;
            }
        }

        if (acquired) {
            if (transitionProof) {
                SKSE::log::info(
                    "[DB_OWNERSHIP][ACQUIRED] form={:08X} frame={} key=\"{}\" mode=TRANSITION_LOCAL provenIgnoreFromWrites={} ownershipKeys={} confirms={} proof=TRANSITION_LOCAL_PROVENANCE_PLUS_OPEN_DB_WFM_NONZERO_XY graphDriven={} action=LATCH_DB_ROOT_AUTHORITY",
                    actor->GetFormID(), frame, key, writes, ownershipKeyCount, confirms, graphDriven);
            } else {
                SKSE::log::info(
                    "[DB_OWNERSHIP][ACQUIRED] form={:08X} frame={} key=\"{}\" mode=STABLE_COMPOSITION previousTransitions={} currentTransitions={} dbClips={} dbShare={:.5f} mixedXY=({:+.5f},{:+.5f}) ownershipKeys={} confirms={} proof=NO_TRANSITION_DB_GENERATE_PLUS_OPEN_DB_WFM_NONZERO_XY graphDriven={} action=LATCH_DB_ROOT_AUTHORITY",
                    actor->GetFormID(), frame, key,
                    previousTransitions, currentTransitions,
                    dbClips, dbShare, mixedX, mixedY,
                    ownershipKeyCount, confirms, graphDriven);
            }
        }
    }

    [[nodiscard]] bool HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->runtimeYawClips.empty()) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;

        // B2: when universal native locomotion owns translation, RuntimeYaw keeps the
        // exact same profile classification/steering algorithm but no longer requires
        // synthetic MotionDB WFM injection as the physical-carrier witness.
        const bool nativeCarrier =
            state.nativeLocomotionSessionActive &&
            state.nativeLocomotionAuthorityEstablished;
        if (state.frameSerial == 0 ||
            state.creatureDbObservedFrame == 0 ||
            state.creatureDbObservedKeyCount == 0) {
            return false;
        }

        const bool recentObservedSet =
            state.creatureDbObservedFrame == state.frameSerial ||
            state.creatureDbObservedFrame + 1 == state.frameSerial;
        if (!recentObservedSet) {
            return false;
        }

        bool recentRuntimeClip = false;
        for (std::size_t i = 0; i < state.creatureDbObservedKeyCount &&
             i < state.creatureDbObservedKeys.size(); ++i) {
            const auto& key = state.creatureDbObservedKeys[i];
            if (std::find(profile->runtimeYawClips.begin(), profile->runtimeYawClips.end(), key) !=
                profile->runtimeYawClips.end()) {
                recentRuntimeClip = true;
                break;
            }
        }
        if (!recentRuntimeClip) {
            return false;
        }
        if (nativeCarrier) {
            return true;
        }
        if (state.previousDbRootKeyCount == 0) {
            return false;
        }

        // previousDbRootKeys are captured from hkbClipGenerator::Generate contributors
        // with valid DB root and non-zero pose weight. This is the physical carrier proof.
        for (std::size_t i = 0; i < state.previousDbRootKeyCount &&
             i < state.previousDbRootKeys.size(); ++i) {
            const auto& key = state.previousDbRootKeys[i];
            if (std::find(profile->runtimeYawClips.begin(), profile->runtimeYawClips.end(), key) !=
                profile->runtimeYawClips.end()) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] std::uint8_t RuntimeYawLocomotionFamilyMaskForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return 0;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->runtimeYawClips.empty()) {
            return 0;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return 0;
        }
        const auto& state = it->second;
        if (state.frameSerial == 0 || state.previousDbRootKeyCount == 0) {
            return 0;
        }

        std::uint8_t mask = 0;
        for (std::size_t i = 0; i < state.previousDbRootKeyCount &&
             i < state.previousDbRootKeys.size(); ++i) {
            const auto& key = state.previousDbRootKeys[i];
            if (std::find(profile->runtimeYawClips.begin(), profile->runtimeYawClips.end(), key) ==
                profile->runtimeYawClips.end()) {
                continue;
            }
            if (key.rfind("walk", 0) == 0) {
                mask |= 0x01;
            } else if (key.rfind("trot", 0) == 0) {
                mask |= 0x02;
            } else if (key.rfind("run", 0) == 0) {
                mask |= 0x04;
            }
        }
        return mask;
    }

    [[nodiscard]] bool HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->rotationClips.empty()) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        if (state.frameSerial == 0 ||
            state.creatureDbObservedFrame == 0 ||
            state.creatureDbObservedKeyCount == 0) {
            return false;
        }

        const bool recentObservedSet =
            state.creatureDbObservedFrame == state.frameSerial ||
            state.creatureDbObservedFrame + 1 == state.frameSerial;
        if (!recentObservedSet) {
            return false;
        }

        for (std::size_t i = 0; i < state.creatureDbObservedKeyCount &&
             i < state.creatureDbObservedKeys.size(); ++i) {
            const auto& key = state.creatureDbObservedKeys[i];
            if (std::find(profile->rotationClips.begin(), profile->rotationClips.end(), key) !=
                profile->rotationClips.end()) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] bool HasAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->rotationClips.empty()) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        if (state.frameSerial == 0 ||
            state.creatureDbObservedFrame == 0 ||
            state.creatureDbObservedKeyCount == 0) {
            return false;
        }

        const bool recentObservedSet =
            state.creatureDbObservedFrame == state.frameSerial ||
            state.creatureDbObservedFrame + 1 == state.frameSerial;
        if (!recentObservedSet) {
            return false;
        }

        // R38 AUTHORED-TURN HEALTH GATE:
        // Observation alone is not enough to monopolize capsule yaw. A RotationClip is
        // exclusive when it either (a) contributed a valid generated DB root in the
        // completed frame, or (b) is still in the bounded strong-entry acquisition phase.
        // This preserves TurnLoop acquisition while preventing an observed-but-frozen
        // clip (zero/invalid root contribution) from suppressing R30 steering forever.
        for (std::size_t i = 0; i < state.creatureDbObservedKeyCount &&
             i < state.creatureDbObservedKeys.size(); ++i) {
            const auto& observedKey = state.creatureDbObservedKeys[i];
            if (std::find(profile->rotationClips.begin(), profile->rotationClips.end(), observedKey) ==
                profile->rotationClips.end()) {
                continue;
            }

            for (std::size_t j = 0; j < state.previousDbRootKeyCount &&
                 j < state.previousDbRootKeys.size(); ++j) {
                if (state.previousDbRootKeys[j] == observedKey) {
                    return true;
                }
            }

            // During a genuine INPLACE->ROOT acquisition the first sample may not yet
            // exist in previousDbRootKeys. Keep authored exclusivity only for that
            // bounded entry phase; once entry is no longer pending, valid root proof is
            // mandatory.
            if (state.creatureDbEntryPending) {
                return true;
            }
        }
        return false;
    }

    [[nodiscard]] bool HasProgressingAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->rotationClips.empty()) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        if (state.frameSerial == 0 ||
            state.creatureDbObservedFrame == 0 ||
            state.creatureDbObservedKeyCount == 0) {
            return false;
        }

        const bool recentObservedSet =
            state.creatureDbObservedFrame == state.frameSerial ||
            state.creatureDbObservedFrame + 1 == state.frameSerial;
        if (!recentObservedSet) {
            return false;
        }

        // R40 health proof only. This does NOT replace the R38 authored-rotation
        // ownership test used by PMD/V13. ENTRY_PENDING can still preserve initial
        // TurnLoop exclusivity there. The stall breaker, however, requires the exact
        // observed RotationClip key to have produced meaningful authored yaw in the
        // completed Generate frame.
        for (std::size_t i = 0; i < state.creatureDbObservedKeyCount &&
             i < state.creatureDbObservedKeys.size(); ++i) {
            const auto& observedKey = state.creatureDbObservedKeys[i];
            if (std::find(profile->rotationClips.begin(), profile->rotationClips.end(), observedKey) ==
                profile->rotationClips.end()) {
                continue;
            }

            for (std::size_t j = 0; j < state.previousDbYawProgressKeyCount &&
                 j < state.previousDbYawProgressKeys.size(); ++j) {
                if (state.previousDbYawProgressKeys[j] == observedKey) {
                    return true;
                }
            }
        }
        return false;
    }

    [[nodiscard]] bool IsDbBootstrapDrivenForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor) || actor->IsPlayerRef()) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        return state.frameSerial != 0 &&
            state.creatureDrivenLatched &&
            state.creatureBootstrapDrivenFrame == state.frameSerial;
    }

    [[nodiscard]] bool HasIgnoreFromTransitionForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor)) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }

        const auto& state = it->second;
        if (state.frameSerial == 0 || state.wfmIgnoreFromSeenFrame == 0) {
            return false;
        }

        // ModifyMovementData may precede this frame's transition Generate, so the
        // immediately previous frame is valid admission context. The tag still
        // cannot open the gate without nativeDriven=true in Hooks.cpp.
        return
            state.wfmIgnoreFromSeenFrame == state.frameSerial ||
            state.wfmIgnoreFromSeenFrame + 1 == state.frameSerial;
    }

    [[nodiscard]] bool HasIgnoreToTransitionForCurrentOrPreviousFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor)) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }

        const auto& state = it->second;
        if (state.frameSerial == 0 || state.wfmIgnoreToSeenFrame == 0) {
            return false;
        }

        // ModifyMovementData can run before the current frame's transition
        // Generate. Accept current OR immediately previous frame so IGNORE_TO can
        // bridge that ordering. This is only a continuity hint; Hooks.cpp also
        // requires that the native gate was already open.
        return
            state.wfmIgnoreToSeenFrame == state.frameSerial ||
            state.wfmIgnoreToSeenFrame + 1 == state.frameSerial;
    }

    [[nodiscard]] float PhysicalFrameDeltaTimeForCurrentActor(RE::Actor* actor)
    {
        if (!actor) {
            return 0.0f;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return 0.0f;
        }

        const float dt = it->second.frameDeltaTime;
        return std::isfinite(dt) && dt > 0.0f ? dt : 0.0f;
    }

    [[nodiscard]] RE::FormID ResolveActorFormIDForHkbContext(const RE::hkbContext& context) noexcept
    {
        return detail::ResolveActorFormIDForContext(context);
    }

    [[nodiscard]] bool HasNativeRootMotionForCurrentFrame(RE::Actor* actor)
    {
        if (!detail::IsExternalMotionActor(actor)) {
            return false;
        }
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_actorState.find(actor->GetFormID());
        if (it == detail::g_actorState.end()) {
            return false;
        }
        const auto& state = it->second;
        return state.frameSerial != 0 &&
            state.wfmRootOutputSeenFrame == state.frameSerial;
    }


    void ApplyConfiguredDirectionalPhaseSyncAfterUpdate(
        RE::hkbClipGenerator* clip,
        const RE::hkbContext& context,
        float inputDeltaTime)
    {
        (void)inputDeltaTime;
        if (!clip || !context.character || !std::isfinite(clip->localTime)) {
            return;
        }

        const RE::FormID formID = detail::ResolveActorFormIDForContext(context);
        if (formID == 0) {
            return;
        }

        std::string profile;
        std::uint64_t frame = 0;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_actorState.find(formID);
            if (it == detail::g_actorState.end() || it->second.isPlayer ||
                it->second.profile.empty() || it->second.frameSerial == 0) {
                return;
            }
            profile = it->second.profile;
            frame = it->second.frameSerial;
        }

        const char* animRaw = clip->animationName.c_str();
        const std::string anim = animRaw ? animRaw : "";
        if (anim.empty()) {
            return;
        }
        const auto metrics = AnimationMotionDatabase::GetClipMetrics(profile, anim);
        if (!metrics.valid || metrics.duration <= 1.0e-6f || metrics.key.empty()) {
            return;
        }

        const auto config = detail::GetPhaseSyncConfig(profile);
        if (!config.enabled || config.groupCount == 0) {
            return;
        }

        std::size_t groupIndex = config.groups.size();
        std::size_t memberIndex = 0;
        const std::string dbKey = detail::LowerAsciiCopy(std::string{ metrics.key });
        for (std::size_t i = 0; i < config.groupCount; ++i) {
            const auto& group = config.groups[i];
            for (std::size_t j = 0; j < group.keyCount; ++j) {
                if (group.keys[j] == dbKey) {
                    groupIndex = i;
                    memberIndex = j;
                    break;
                }
            }
            if (groupIndex != config.groups.size()) {
                break;
            }
        }
        if (groupIndex >= config.groups.size()) {
            return;
        }

        bool emitAcquire = false;
        bool emitCorrection = false;
        bool emitRelease = false;
        float rawLocal = 0.0f;
        float targetLocal = 0.0f;
        float rawPhase = 0.0f;
        float targetPhase = 0.0f;
        float phaseError = 0.0f;
        std::uint32_t stalledFrames = 0;
        std::uint64_t corrections = 0;

        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto actorIt = detail::g_actorState.find(formID);
            if (actorIt == detail::g_actorState.end() ||
                actorIt->second.frameSerial != frame ||
                groupIndex >= actorIt->second.directionalPhaseAnchors.size()) {
                return;
            }

            auto& state = actorIt->second;

            // Only touch clips that actually participated in generated MotionDB root
            // in the immediately preceding/current actor frame. Dormant graph branches
            // remain entirely native.
            const auto phaseIt = state.syntheticClipPhase.find(clip);
            if (phaseIt == state.syntheticClipPhase.end() ||
                phaseIt->second.lastSeenFrame == 0 ||
                phaseIt->second.lastSeenFrame + 2 < state.frameSerial) {
                return;
            }

            // Native action takeover wins. Never phase-lock outgoing locomotion under an
            // IGNORE_FROM / serialized action transition.
            const bool nativeTakeoverFresh =
                state.wfmIgnoreFromSeenFrame != 0 &&
                (state.wfmIgnoreFromSeenFrame == state.frameSerial ||
                 state.wfmIgnoreFromSeenFrame + 1 == state.frameSerial);
            if (nativeTakeoverFresh) {
                auto clipStateIt = state.directionalPhaseClips.find(clip);
                if (clipStateIt != state.directionalPhaseClips.end()) {
                    clipStateIt->second.locked = false;
                    clipStateIt->second.healthyFrames = 0;
                    clipStateIt->second.stalledFrames = 0;
                }
                return;
            }

            rawLocal = detail::NormalizeClipTime(clip->localTime, metrics.duration);
            rawPhase = rawLocal / metrics.duration;
            auto& anchor = state.directionalPhaseAnchors[groupIndex];

            // First configured key is the universal phase anchor for this group.
            // Only the first anchor clip observed in an actor frame establishes the
            // anchor, preventing duplicate center nodes from fighting each other.
            if (memberIndex == 0) {
                if (!anchor.valid || anchor.frame != state.frameSerial) {
                    float deltaCycles = 0.0f;
                    bool moving = false;
                    if (anchor.valid && anchor.frame + 1 == state.frameSerial) {
                        deltaCycles = detail::ForwardPhaseDelta(rawPhase, anchor.phase);
                        moving = std::isfinite(deltaCycles) &&
                            deltaCycles > 0.0005f && deltaCycles < 0.25f;
                    }
                    anchor.valid = true;
                    anchor.frame = state.frameSerial;
                    anchor.phase = rawPhase;
                    anchor.lastDeltaCycles = moving ? deltaCycles : 0.0f;
                    anchor.moving = moving;
                }
                return;
            }

            auto& clipState = state.directionalPhaseClips[clip];
            bool rawProgressing = false;
            if (clipState.rawValid && clipState.lastFrame + 1 == state.frameSerial) {
                float deltaSeconds = rawLocal - clipState.lastRawLocalTime;
                if (deltaSeconds < -0.5f * metrics.duration) {
                    deltaSeconds += metrics.duration;
                }
                rawProgressing = std::isfinite(deltaSeconds) &&
                    deltaSeconds > config.stallDeltaSeconds;
                if (std::fabs(deltaSeconds) <= config.stallDeltaSeconds) {
                    clipState.stalledFrames = std::min<std::uint32_t>(
                        clipState.stalledFrames + 1, 1000000u);
                } else {
                    clipState.stalledFrames = 0;
                }
            } else {
                clipState.stalledFrames = 0;
            }
            clipState.rawValid = true;
            clipState.lastRawLocalTime = rawLocal;
            clipState.lastFrame = state.frameSerial;

            bool anchorFresh = false;
            if (anchor.valid && anchor.frame == state.frameSerial) {
                targetPhase = anchor.phase;
                anchorFresh = true;
            } else if (anchor.valid && anchor.frame + 1 == state.frameSerial && anchor.moving) {
                targetPhase = anchor.phase + anchor.lastDeltaCycles;
                targetPhase -= std::floor(targetPhase);
                anchorFresh = true;
            }

            if (!anchorFresh) {
                clipState.healthyFrames = 0;
                return;
            }

            phaseError = detail::CircularPhaseDistance(rawPhase, targetPhase);
            stalledFrames = clipState.stalledFrames;

            if (!clipState.locked &&
                anchor.moving &&
                clipState.stalledFrames >= config.stallFrames &&
                phaseError >= config.snapThresholdCycles) {
                clipState.locked = true;
                clipState.lockFrame = state.frameSerial;
                clipState.healthyFrames = 0;
                ++state.directionalPhaseLockAcquires;
                emitAcquire = true;
            }

            if (!clipState.locked) {
                return;
            }

            // If native timing has recovered and remained near the anchor for several
            // frames, release the correction and return full phase ownership to Havok.
            if (rawProgressing && phaseError <= config.releaseThresholdCycles) {
                clipState.healthyFrames = std::min<std::uint32_t>(
                    clipState.healthyFrames + 1, config.releaseHealthyFrames);
            } else {
                clipState.healthyFrames = 0;
            }
            if (clipState.healthyFrames >= config.releaseHealthyFrames) {
                clipState.locked = false;
                clipState.healthyFrames = 0;
                clipState.stalledFrames = 0;
                emitRelease = true;
            } else {
                targetLocal = targetPhase * metrics.duration;
                clip->localTime = targetLocal;
                ++clipState.corrections;
                ++state.directionalPhaseCorrections;
                corrections = clipState.corrections;
                emitCorrection =
                    emitAcquire || clipState.corrections <= 4 ||
                    (clipState.corrections % 120) == 0;
            }
        }

        if (emitAcquire) {
            SKSE::log::warn(
                R"([PhaseSync][R52_STALL_LOCK] form={:08X} profile="{}" key="{}" group={} stalledFrames={} rawPhase={:.4f} anchorPhase={:.4f} phaseError={:.4f}cycles proof=CONFIGURED_GROUP_PLUS_RECENT_GENERATED_DB_ROOT_PLUS_DIRECTIONAL_LOCALTIME_STALL action=LOCK_DIRECTIONAL_PHASE_TO_CONFIGURED_CENTER)",
                formID,
                profile,
                dbKey,
                groupIndex,
                stalledFrames,
                rawPhase,
                targetPhase,
                phaseError);
        }
        if (emitCorrection) {
            SKSE::log::info(
                R"([PhaseSync][R52_PHASE_REACQUIRE] form={:08X} profile="{}" key="{}" group={} correction={} rawLocal={:.6f} targetLocal={:.6f} rawPhase={:.4f} targetPhase={:.4f} action=POST_NATIVE_UPDATE_LOCALTIME_REACQUIRE_BEFORE_GENERATE rootTransport=EXISTING_SYNTHETIC_WFM_PHASE_DELTA_UNCHANGED)",
                formID,
                profile,
                dbKey,
                groupIndex,
                corrections,
                rawLocal,
                targetLocal,
                rawPhase,
                targetPhase);
        }
        if (emitRelease) {
            SKSE::log::info(
                R"([PhaseSync][R52_PHASE_RELEASE] form={:08X} profile="{}" key="{}" group={} proof=NATIVE_LOCALTIME_PROGRESSING_AND_WITHIN_CONFIGURED_PHASE_THRESHOLD action=RETURN_PHASE_AUTHORITY_TO_NATIVE_HAVOK)",
                formID,
                profile,
                dbKey,
                groupIndex);
        }
    }

    void ObserveClipUpdate(
        RE::hkbClipGenerator* clip,
        const RE::hkbContext& context,
        float inputDeltaTime,
        RE::FormID actorFormID)
    {
        (void)inputDeltaTime;
        if (!clip || !context.character) {
            return;
        }

        RE::FormID resolvedActorFormID = actorFormID;
        bool playerOwner = false;
        bool motionProfileOwner = false;
        bool ownerResolved = false;
        RE::Character* evaluationActor = detail::CurrentEvaluationActor();
        {
            // Fast path when the callback occurs inside the registered MotionDB evaluation.

            if (detail::IsExternalMotionActor(evaluationActor)) {
                std::array<const RE::hkbCharacter*, 8> evaluationCharacters{};
                const auto count = detail::CollectHkbCharacters(evaluationActor, evaluationCharacters);
                for (std::size_t i = 0; i < count; ++i) {
                    if (evaluationCharacters[i] == context.character) {
                        resolvedActorFormID = evaluationActor->GetFormID();
                        playerOwner = evaluationActor->IsPlayerRef();
                        motionProfileOwner = !evaluationActor->IsPlayerRef() && MotionProfileDatabase::SupportsActor(evaluationActor);
                        ownerResolved = true;
                        break;
                    }
                }
            }

            // Clip callbacks may occur outside Character::Update. Resolve ownership
            // from the persistent graph-character set captured at the creature frame boundary.
            // R4 keeps the proven R2 rule: register every configured MotionDB creature profile.
            if (!ownerResolved) {
                std::scoped_lock lock{ detail::g_mutex };
                for (const auto& [formID, state] : detail::g_actorState) {
                    if (state.frameSerial == 0) {
                        continue;
                    }
                    if (detail::StateOwnsHkbCharacter(state, context.character)) {
                        resolvedActorFormID = formID;
                        playerOwner = state.isPlayer;
                        motionProfileOwner = !state.isPlayer;
                        ownerResolved = true;
                        break;
                    }
                }
            }
        }

        if (!ownerResolved) {
            return;
        }

        const char* nodeRaw = clip->name.c_str();
        const char* animRaw = clip->animationName.c_str();
        const std::string node = nodeRaw ? nodeRaw : "";
        const std::string anim = animRaw ? animRaw : "";
        if (node.empty() || anim.empty()) {
            return;
        }

        detail::ClipInfo info{};
        info.animationName = anim;
        // Snapshot occurs before the owner Update. Do not touch animationControl,
        // bindings, or GetFutureTime/GetExtractedMotion here.
        info.clipLocalTime = clip->localTime;
        std::string databaseProfile;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto actorIt = detail::g_actorState.find(resolvedActorFormID);
            if (actorIt != detail::g_actorState.end()) databaseProfile = actorIt->second.profile;
        }
        if (databaseProfile.empty()) databaseProfile = playerOwner ? "Player" : std::string{};
        info.profile = databaseProfile;
        const auto clipMetrics = AnimationMotionDatabase::GetClipMetrics(databaseProfile, anim);
        info.databaseBacked = clipMetrics.valid;
        info.databaseKey = clipMetrics.valid ? clipMetrics.key : std::string{};

        // R14.2: protected Update remains observation-only. Do NOT infer native
        // root capability from hkaAnimation::extractedMotion here; R14.1 proved
        // that lifecycle probe is not reliable in Skyrim. Native root authority
        // is classified from hkbGeneratorOutput TRACK_EXTRACTED_MOTION instead.
        const auto extractedProbe = detail::ProbeExtractedMotion(clip);

        bool logRootWitness = false;
        detail::RootMotionWitness rootWitnessLog{};
        bool logPlayerCallback = false;
        bool logCycleBegin = false;
        bool logCycleAudit = false;
        bool cycleAmbiguous = false;
        bool cycleTightPass = false;
        std::string cycleKey;
        float cycleDbDistance = 0.0f;
        float cyclePhysicalPath = 0.0f;
        float cycleRatio = 0.0f;
        float cycleRelativeError = 0.0f;
        std::uint64_t cycleFrames = 0;
        std::uint64_t cycleIndex = 0;
        std::uint64_t tightPassCount = 0;
        std::uint64_t driftCount = 0;
        std::uint64_t ambiguousCycleCount = 0;

        bool announcePlayerOwnership = false;
        bool announcePlayerClip = false;
        std::uint64_t frameSerial = 0;
        std::uint64_t callbackIndex = 0;
        std::uint64_t missCount = 0;
        {
            std::scoped_lock lock{ detail::g_mutex };
            auto actorIt = detail::g_actorState.find(resolvedActorFormID);
            if (actorIt == detail::g_actorState.end() || actorIt->second.frameSerial == 0 ||
                !detail::StateOwnsHkbCharacter(actorIt->second, context.character)) {
                return;
            }

            auto& state = actorIt->second;
            frameSerial = state.frameSerial;

            logRootWitness = detail::RegisterRootMotionWitnessLocked(
                state,
                clip,
                node,
                anim,
                info.databaseBacked,
                extractedProbe);
            if (logRootWitness && state.rootWitnessCount > 0) {
                rootWitnessLog = state.rootWitnesses[state.rootWitnessCount - 1];
            }

            if (state.isPlayer || playerOwner) {
                playerOwner = true;
                callbackIndex = ++state.playerClipCallbacks;
                if (!info.databaseBacked) {
                    missCount = ++state.playerClipDbMisses;
                }
                if (state.playerClipCallbackLogs < 16) {
                    ++state.playerClipCallbackLogs;
                    logPlayerCallback = true;
                }
                if (!state.playerOwnershipAnnounced) {
                    state.playerOwnershipAnnounced = true;
                    announcePlayerOwnership = true;
                }
            }

            if (!info.databaseBacked) {
                // The callback is now owned correctly, but no authored file matches
                // this animation. The diagnostic log below preserves the exact name.
            } else {
                info.seenFrameSerial = state.frameSerial;
                info.seen = Clock::now();

                if (motionProfileOwner && !playerOwner) {
                    if (state.creatureDbObservedFrame != state.frameSerial) {
                        state.creatureDbObservedKeyCount = 0;
                        for (auto& observedKey : state.creatureDbObservedKeys) {
                            observedKey.clear();
                        }
                    }
                    state.creatureDbObservedFrame = state.frameSerial;
                    state.creatureDbObservedKey = info.databaseKey;
                    detail::AddUniqueDbKey(
                        state.creatureDbObservedKeys,
                        state.creatureDbObservedKeyCount,
                        info.databaseKey);
                }

                if (playerOwner) {
                    state.playerDatabaseClipFrame = state.frameSerial;
                    ++state.playerClipWitnesses;
                    if (detail::kEnablePlayerCycleAudit) {
                        detail::AddPlayerFrameDbKeyLocked(state, clipMetrics.key);

                        auto& audit = state.playerCycleAudit[clipMetrics.key];
                        if (std::isfinite(info.clipLocalTime) && clipMetrics.duration > 1.0e-6f) {
                        const bool wrapped = audit.localTimeValid &&
                            (audit.lastLocalTime - info.clipLocalTime) > (clipMetrics.duration * 0.50f);
                        if (wrapped) {
                            if (!audit.cycleStarted) {
                                audit.cycleStarted = true;
                                audit.physicalPathXY = 0.0f;
                                audit.attributedFrames = 0;
                                audit.ambiguousBlend = false;
                                logCycleBegin = true;
                                cycleKey = clipMetrics.key;
                                cycleDbDistance = clipMetrics.distanceXY;
                            } else {
                                ++audit.completedCycles;
                                cycleIndex = audit.completedCycles;
                                cycleKey = clipMetrics.key;
                                cycleDbDistance = clipMetrics.distanceXY;
                                cyclePhysicalPath = audit.physicalPathXY;
                                cycleFrames = audit.attributedFrames;
                                cycleAmbiguous = audit.ambiguousBlend || cycleFrames == 0 ||
                                    !(cycleDbDistance > 1.0e-6f);
                                if (!cycleAmbiguous) {
                                    cycleRatio = cyclePhysicalPath / cycleDbDistance;
                                    cycleRelativeError = std::fabs(cyclePhysicalPath - cycleDbDistance) / cycleDbDistance;
                                    cycleTightPass = std::isfinite(cycleRelativeError) &&
                                        cycleRelativeError <= detail::kCyclePassRelativeError;
                                    if (cycleTightPass) {
                                        ++audit.tightPasses;
                                    } else {
                                        ++audit.driftCycles;
                                    }
                                } else {
                                    ++audit.ambiguousCycles;
                                }
                                tightPassCount = audit.tightPasses;
                                driftCount = audit.driftCycles;
                                ambiguousCycleCount = audit.ambiguousCycles;
                                logCycleAudit = true;

                                audit.physicalPathXY = 0.0f;
                                audit.attributedFrames = 0;
                                audit.ambiguousBlend = false;
                            }
                        }
                            audit.lastLocalTime = info.clipLocalTime;
                            audit.localTimeValid = true;
                        }
                    }

                    if (!state.playerClipAnnounced) {
                        state.playerClipAnnounced = true;
                        announcePlayerClip = true;
                    }
                }

                if (detail::g_clipInfo.size() > 4096) {
                    detail::g_clipInfo.clear();
                }
                detail::g_clipInfo[detail::ClipKey{ context.character, node }] = info;
            }
        }

        if (logRootWitness &&
            (frameSerial <= 5 || (frameSerial % 240) == 0 ||
             rootWitnessLog.clipMemberDeltaObserved)) {
            const char* source = rootWitnessLog.motionDbBacked ?
                "MOTIONDB_MEMBERSHIP" : "CLIP_MEMBER_DELTA";
            SKSE::log::debug(R"([WFM][CLIP_TELEMETRY] form={:08X} frame={} clipPtr=0x{:X} node="{}" anim="{}" source={} motionDbMembership={} clipMemberDelta={} memberXYZ=({:+.6f},{:+.6f},{:+.6f}) memberYaw={:+.4f}deg authority=false note="real native authority is ROOT_OUTPUT/CLIP generated TRACK_EXTRACTED_MOTION")",
                resolvedActorFormID,
                frameSerial,
                reinterpret_cast<std::uintptr_t>(rootWitnessLog.clip),
                rootWitnessLog.nodeName,
                rootWitnessLog.animationName,
                source,
                rootWitnessLog.motionDbBacked,
                rootWitnessLog.clipMemberDeltaObserved,
                rootWitnessLog.extractedX,
                rootWitnessLog.extractedY,
                rootWitnessLog.extractedZ,
                rootWitnessLog.extractedYawDeg);
        }

        if (motionProfileOwner && info.databaseBacked) {
            std::string signature = info.databaseKey + "|" + node + "|" + anim;
            if (detail::MarkMotionDiagnosticOnce(
                    resolvedActorFormID, detail::MotionDiagKind::kObserved, std::move(signature))) {
                SKSE::log::info(
                    R"([MotionDB][OBSERVED] form={:08X} frame={} key="{}" node="{}" anim="{}" clipLocalTime={:.6f} source=MOTIONDB_MEMBERSHIP)",
                    resolvedActorFormID, frameSerial, info.databaseKey, node, anim, info.clipLocalTime);
            }
        }
        if (announcePlayerOwnership) {
            SKSE::log::info(
                "[PlayerRootMotion][OWNER_RESOLVED] form={:08X} hkbCharacter=0x{:X} source=PERSISTENT_GRAPH_SET",
                resolvedActorFormID, reinterpret_cast<std::uintptr_t>(context.character));
        }
        if (logPlayerCallback) {
            SKSE::log::info(
                R"([PlayerRootMotion][CLIP_CALLBACK] form={:08X} frame={} index={} hkbCharacter=0x{:X} node="{}" anim="{}" dbBacked={} dbMisses={})",
                resolvedActorFormID, frameSerial, callbackIndex,
                reinterpret_cast<std::uintptr_t>(context.character), node, anim,
                info.databaseBacked, missCount);
        }
        if (announcePlayerClip) {
            SKSE::log::info(
                R"([PlayerRootMotion][DB_CLIP] form={:08X} frame={} node="{}" anim="{}" source=MotionDB/Player)",
                resolvedActorFormID, frameSerial, node, anim);
        }
        if (logCycleBegin) {
            SKSE::log::info(
                R"([PlayerRootMotion][CYCLE_BEGIN] form={:08X} key="{}" dbDistanceXY={:.4f} scale=1.00 note="first complete cycle starts after this seam")",
                resolvedActorFormID, cycleKey, cycleDbDistance);
        }
        if (logCycleAudit) {
            const char* status = cycleAmbiguous ? "BLENDED_AMBIGUOUS" :
                (cycleTightPass ? "PASS_TIGHT" : "DRIFT_OR_CONSTRAINT");
            SKSE::log::info(
                R"([PlayerRootMotion][CYCLE_AUDIT] form={:08X} key="{}" cycle={} status={} frames={} dbDistanceXY={:.4f} physicalPathXY={:.4f} ratio={:.5f} relError={:.3f}% totals[pass={} drift={} ambiguous={}])",
                resolvedActorFormID, cycleKey, cycleIndex, status, cycleFrames,
                cycleDbDistance, cyclePhysicalPath, cycleRatio, cycleRelativeError * 100.0f,
                tightPassCount, driftCount, ambiguousCycleCount);
        }
    }

    bool Install()
    {
        if (detail::g_installed) {
            return true;
        }
        if (REL::Module::IsVR()) {
            SKSE::log::warn("[NativeAudit][INSTALL_SKIP] VR not covered by the verified ProcessMotionData register contract.");
            return false;
        }

        AnimationMotionDatabase::EnsureLoaded();
        SKSE::log::info("[C1_NATIVE_EXTRACTED][INSTALL] profilelessRootSource=NATIVE_PMD_T1_T2_PER_CLIP rootLeafSink=TRACK_EXTRACTED_MOTION rootComposition=FINAL_HKB_BEHAVIOR_GRAPH rootPhysicalSink=MODIFY_MOVEMENT_0x11A exactThenBridge=LAST_FINAL_VELOCITY_X_DT_MAX4 prearm=NONE bAnimationDriven=NOT_ROOT_ADMISSION WFMtranslation=UNTOUCHED yaw=PROCESS_MOTION_DATA_R1_R2 V13=PROFILELESS_FIREWALL_ONLY authoredTurns=NATIVE_NON_LOCOMOTION_ROTATION actionBypass=NATIVE graphControl=B8_AUTOGRAPH_OPTIONAL MotionDB=NOT_REQUIRED_PROFILELESS");

        REL::Relocation<std::uintptr_t> processMotionData{ detail::kProcessMotionDataID };
        REL::Relocation<std::uintptr_t> vanillaTranslation{ detail::kProcessTranslationDataID };
        REL::Relocation<std::uintptr_t> vanillaRotation{ detail::kProcessRotationDataID };
        const auto base = processMotionData.address();
        const auto sites = detail::HistoricalSites(base);
        const std::array<std::uintptr_t, 4> siteArray{
            sites.translation1, sites.translation2, sites.rotation1, sites.rotation2
        };

        for (const auto site : siteArray) {
            if (!site || *reinterpret_cast<const std::uint8_t*>(site) != 0xE8) {
                SKSE::log::warn(
                    "[NativeAudit][INSTALL_SKIP] expected CALL rel32 missing at site=0x{:X}; no partial hook installed.",
                    site);
                return false;
            }
        }

        const std::array<std::uintptr_t, 4> currentTargets{
            detail::DecodeCallTarget(sites.translation1),
            detail::DecodeCallTarget(sites.translation2),
            detail::DecodeCallTarget(sites.rotation1),
            detail::DecodeCallTarget(sites.rotation2)
        };
        for (const auto target : currentTargets) {
            if (!target) {
                SKSE::log::warn("[NativeAudit][INSTALL_SKIP] failed to decode an existing ProcessMotionData call target.");
                return false;
            }
        }

        auto& trampoline = SKSE::GetTrampoline();
        if (trampoline.empty()) {
            trampoline.create(detail::kDefaultTrampolineBytes);
        }
        if (trampoline.free_size() < detail::kRequiredTrampolineBytes) {
            SKSE::log::warn(
                "[NativeAudit][INSTALL_SKIP] trampoline free={} required={}",
                trampoline.free_size(), detail::kRequiredTrampolineBytes);
            return false;
        }

        const bool ae = REL::Module::IsAE();
        void* t1 = detail::BuildChainedDispatchStub(trampoline, currentTargets[0], &detail::Translation1After, ae);
        void* t2 = detail::BuildChainedDispatchStub(trampoline, currentTargets[1], &detail::Translation2After, ae);
        void* r1 = detail::BuildChainedDispatchStub(trampoline, currentTargets[2], &detail::Rotation1After, ae);
        void* r2 = detail::BuildChainedDispatchStub(trampoline, currentTargets[3], &detail::Rotation2After, ae);
        if (!t1 || !t2 || !r1 || !r2) {
            SKSE::log::warn("[NativeAudit][INSTALL_SKIP] failed to build chained dispatch stubs.");
            return false;
        }

        trampoline.write_call<5>(sites.translation1, reinterpret_cast<std::uintptr_t>(t1));
        trampoline.write_call<5>(sites.translation2, reinterpret_cast<std::uintptr_t>(t2));
        trampoline.write_call<5>(sites.rotation1, reinterpret_cast<std::uintptr_t>(r1));
        trampoline.write_call<5>(sites.rotation2, reinterpret_cast<std::uintptr_t>(r2));

        // R15.6: hkbClipGenerator::Update remains owned by Hooks.cpp and is not
        // changed here. Generate now injects MotionDB planar root directly into the
        // leaf WORLD_FROM_MODEL after the existing owner. Parent Havok generators
        // perform the real weighted blend. Transition flags remain temporary.
        REL::Relocation<std::uintptr_t> clipGeneratorVTable{
            RE::VTABLE_hkbClipGenerator[0]
        };
        detail::ClipGeneratorGenerateHook::Func =
            clipGeneratorVTable.write_vfunc(0x17, detail::ClipGeneratorGenerateHook::Thunk);

        REL::Relocation<std::uintptr_t> blendingTransitionVTable{
            RE::VTABLE_hkbBlendingTransitionEffect[0]
        };
        detail::BlendingTransitionGenerateHook::Func =
            blendingTransitionVTable.write_vfunc(0x17, detail::BlendingTransitionGenerateHook::Thunk);

        detail::g_installed = true;

        SKSE::log::info(
            "[NativeAudit][INSTALL] runtime={} processMotion=0x{:X} sites[T1=0x{:X} T2=0x{:X} R1=0x{:X} R2=0x{:X}]",
            ae ? "AE" : "SE", base,
            sites.translation1, sites.translation2, sites.rotation1, sites.rotation2);
        SKSE::log::info(
            "[NativeAudit][CHAIN] existingTargets[T1=0x{:X} T2=0x{:X} R1=0x{:X} R2=0x{:X}] vanilla[T=0x{:X} R=0x{:X}]",
            currentTargets[0], currentTargets[1], currentTargets[2], currentTargets[3],
            vanillaTranslation.address(), vanillaRotation.address());
        SKSE::log::info("[WFM][R65_CTD_FIREWALL_INSTALL] policy=EXACT_DEREFERENCE_RANGE_ONLY noWholeBufferValidation=true catchesLowNonPointerValues=true lifecycle=R62_UNCHANGED");
        SKSE::log::info(
            "[WFM][INSTALL] R85_HKX_CANONICAL_WFM_ROOT_PRE_TROLL_BASELINE: creature XY has ONE physical transport = MotionDB-backed HKX leaf WORLD_FROM_MODEL; Havok parent tree blends the same authored root with the same graph weights/masks as pose; behavior aliases are not independent physical root sources; PMD T1/T2 translation remains native/bypassed; RuntimeYawClips use R1/R2 procedural yaw only.");
        SKSE::log::info(
            "[PhysicalComposition][R94_PROBE_INSTALL] diagnosticOnly=true graphSelection=UNCHANGED WFMInjection=UNCHANGED rootAuthority=UNCHANGED playback=UNCHANGED yaw=UNCHANGED warping=UNCHANGED grouping=MotionDBKey+SameKeyPhaseDeltaSpread purpose=DISTINGUISH_ALIAS_MULTIPLICITY_FROM_DISTINCT_AUTHORED_CONTRIBUTORS_BEFORE_ANY_MICRO_TREMOR_FIX");
        SKSE::log::info(
            "[ExternalMotion][MODE] R85_PRE_TROLL_PHYSICAL_BASELINE: XY=HKX_WFM_ONLY; WFMrotation=UNTOUCHED; PMD_T1_T2_XY=PRESERVE_NATIVE; RuntimeYawClip pair=R1 identity/R2 AI-goal yawStep; authored RotationClips remain MotionDB-owned; R79/R80/R83/R84 composition-transport experiments removed.");
        SKSE::log::info(
            "[MotionDB][ROTATION_POLICY] RuntimeYawClips=R35_AI_GOAL_CONTROLLER_TO_R30_PAIR; RotationClips=AUTHORED_MOTIONDB; otherCreatureDbClips=PRESERVE_EXISTING_R1_R2; playerPath=UNCHANGED.");
        SKSE::log::info(
            "[R104][PMD_NATIVE_TURN_INSTALL] diagnostics=UNRESOLVED_AND_RESOLVED_PROFILE_ROTATIONCLIPS_DURING_AUTHORED_WITNESS; records=ORIGINAL_R1_R2_QUATERNION_AND_PAIR_DELTA; writes=ZERO_NEW.");
        SKSE::log::info(
            "[AI_STEERING_YAW][INSTALL] build=INCESSUS-R85-HKX-CANONICAL-WFM-ROOT-2026-09-03 eligibility=Intent.ini::RuntimeYawClips source=SteeringIntentController::FrameCommand::yawStepRadians pair=R1_IDENTITY_R2_AI_YAW_STEP sink=ProcessMotionData.rotation V13=R74_NATIVE_PASS WFMrotation=UNTOUCHED");
        SKSE::log::info("[AI_STEERING_YAW][R103_EXTERNAL_PIVOT_SUPPRESSION] combinedAuthoredWitness=MOTIONDB_OR_CAPSULE_EXTERNAL_STABLEPIVOT runtimeYawLeaves=IDENTITY_WHILE_PIVOT_ACTIVE rootXY=UNCHANGED");
        SKSE::log::info(
            "[TurnLoopHealth][INSTALL] build=ALR-R52-CONFIGURED-DIRECTIONAL-PHASE-REACQUIRE-R51-R50-R49-R46-R42-R41-R35-AI-R30-2026-08-30 observed=RECENT_ROTATIONCLIP healthy=EXACT_ROTATIONCLIP_KEY_WITH_MEANINGFUL_COMPLETED_DELTA_YAW policy=ENTRY_PENDING_AND_TRANSLATION_ROOT_ARE_NOT_ROTATION_HEALTH_PROOF entryPolicy=ROTATIONCLIP_NEW_SESSION_REQUIRES_INPLACE_TO_DB_ROOT_PROVENANCE");
        SKSE::log::info(
            "[DB_SESSION][R35_R34_REARM_BASELINE] guard=NONPLAYER_SUPPORTED_MOTIONDB_ACTOR previousBrokenGuard=preFrameYawEligible expected=SESSION_BOOTSTRAP_OWNERSHIP_XY_PLUS_R30_YAW");
        SKSE::log::info(
            "[DB_EXIT_GRACE][TEST_MATRIX] 1=EXIT_CALLBACK_LOGS_EXIT_CANDIDATE_NOT_RELEASE 2=NO_CLOSED_XY_DURING_VALID_GAIT_HANDOFF 3=NEW_DB_GAIT_WITHIN_GRACE_KEEPS_OR_REESTABLISHES_DB_LATCH 4=TRUE_INPLACE_EXIT_RELEASES_BY_CONTINUITY_TIMEOUT 5=NATIVE_TAKEOVER_STILL_RELEASES_IMMEDIATELY 6=TURNLOOP_AUTHORED_YAW_REMAINS_EXCLUSIVE_AND_CAPSULE_PROCEDURAL_YAW_IS_EXTERNAL.");
        return true;
    }

    bool ConsumeNativeRootExitToDefaultWitness(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }
        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_nativeRootExitWitnesses.find(actor->GetFormID());
        if (it == detail::g_nativeRootExitWitnesses.end() || !it->second.pending) {
            return false;
        }
        it->second.pending = false;
        return true;
    }

    bool ConsumeRecentLocomotionToStandingWitness(RE::Actor* actor, std::uint64_t& outAgeFrames)
    {
        outAgeFrames = 0;
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }

        const auto formID = actor->GetFormID();
        std::scoped_lock lock{ detail::g_mutex };
        const auto witnessIt = detail::g_locomotionStandingWitnesses.find(formID);
        if (witnessIt == detail::g_locomotionStandingWitnesses.end() || !witnessIt->second.pending) {
            return false;
        }

        const auto actorIt = detail::g_actorState.find(formID);
        if (actorIt == detail::g_actorState.end()) {
            return false;
        }

        auto& witness = witnessIt->second;
        const auto currentFrame = actorIt->second.frameSerial;
        if (currentFrame < witness.lastSeenFrame) {
            witness.pending = false;
            return false;
        }

        const auto ageFrames = currentFrame - witness.lastSeenFrame;
        if (ageFrames > detail::kLocomotionStandingWitnessMaxAgeFrames) {
            witness.pending = false;
            return false;
        }
        if (ageFrames < detail::kLocomotionStandingWitnessMinAgeFrames) {
            return false;
        }

        witness.pending = false;
        outAgeFrames = ageFrames;
        return true;
    }

    void Clear()
    {
        struct DrivenRestore
        {
            RE::FormID formID{ 0 };
            bool value{ false };
        };
        std::vector<DrivenRestore> restores;
        {
            std::scoped_lock lock{ detail::g_mutex };
            restores.reserve(detail::g_actorState.size());
            for (const auto& [formID, state] : detail::g_actorState) {
                if (state.playerDrivenLatched) {
                    restores.push_back({ formID, state.playerOriginalDriven });
                } else if (state.nativeLocomotionDrivenLatched) {
                    restores.push_back({ formID, state.nativeLocomotionOriginalDriven });
                } else if (state.creatureDrivenLatched) {
                    restores.push_back({ formID, state.creatureOriginalDriven });
                }
            }
        }
        for (const auto& restore : restores) {
            if (auto* actor = RE::TESForm::LookupByID<RE::Actor>(restore.formID)) {
                actor->SetGraphVariableBool("bAnimationDriven", restore.value);
            }
        }

        {
            std::scoped_lock lock{ detail::g_mutex };
            for (const auto& [formID, state] : detail::g_actorState) {
                if (state.nativeLocomotionSessionStarts > 0 || state.nativeLocomotionSessionStops > 0 ||
                    state.nativeLocomotionArms > 0) {
                    SKSE::log::info(
                        R"([B3_NATIVE_LOCOMOTION][SUMMARY] form={:08X} starts={} stops={} arms={} holds={} restores={} active={} authority={} listFrame={} listClip="{}" deltaFrame={} deltaClip="{}" deltaMagnitude={:.6f})",
                        formID, state.nativeLocomotionSessionStarts, state.nativeLocomotionSessionStops,
                        state.nativeLocomotionArms, state.nativeLocomotionHolds,
                        state.nativeLocomotionRestores, state.nativeLocomotionSessionActive,
                        state.nativeLocomotionAuthorityEstablished,
                        state.nativeTranslationListSeenFrame, state.nativeTranslationListSeenClip,
                        state.nativeTranslationDeltaSeenFrame, state.nativeTranslationDeltaSeenClip,
                        state.nativeTranslationDeltaSeenMagnitude);
                }
                if (state.creatureDbSessionStarts > 0 || state.creatureDbSessionStops > 0 ||
                    state.creatureArmCount > 0 || state.creatureArmFailCount > 0) {
                    SKSE::log::info(
                        "[DB_SESSION][SUMMARY] form={:08X} starts={} stops={} arms={} holds={} restores={} armFails={} ownership[established={} keys={} entryPending={} entryPolicyWrites={} confirms={} timeouts={}] lifecycle[continuityHolds={} continuityMisses={} explicitReleases={} pmdContinuityFrame={} generateContinuityFrame={}] pmdTelemetry[frame={} key=\"{}\" hits={} logs={}] mixerFrames={} mixerDbFrames={} lastObservedFrame={} lastKey=\"{}\"",
                        formID, state.creatureDbSessionStarts, state.creatureDbSessionStops,
                        state.creatureArmCount, state.creatureHoldCount, state.creatureRestoreCount,
                        state.creatureArmFailCount, state.creatureDbOwnershipEstablished,
                        state.creatureDbOwnershipKeyCount, state.creatureDbEntryPending,
                        state.creatureDbEntryPolicyWrites, state.creatureDbEntryConfirms,
                        state.creatureDbEntryTimeouts, state.creatureDbContinuityHolds,
                        state.creatureDbContinuityMisses, state.creatureDbExplicitReleases,
                        state.creatureDbPmdContinuityFrame, state.creatureDbGenerateContinuityFrame,
                        state.creatureDbPmdSourceFrame, state.creatureDbPmdSourceKey,
                        state.creatureDbPmdSourceHits, state.creatureDbPmdHeartbeatLogs,
                        state.rootMixerFrames, state.rootMixerDbFrames,
                        state.creatureDbObservedFrame, state.creatureDbObservedKey);
                }
                if (state.wfmTransitionCallsTotal > 0) {
                    SKSE::log::info(
                        "[WFM][SUMMARY] form={:08X} transitionCalls={} runtimePolicyChanges={} runtimeInjected={} runtimeIgnoreFrom={} runtimeIgnoreTo={} provenance[policyWrites={} rootToRoot={} ambiguous={}] sideMap[direct={} swapped={} cache={} ambiguous={} postCalibrated={} disagreement={}] synthWfmApplied={} synthWfmWait={} preFrameYawPairs={} creatureTxBypass={} deltaOutputClassified={} deltaOutputUnknown={} clipGenerateCalls={} nonzeroExtractedSamples={} legacyTranslationWrites={} finalRotationWrites={} note=R15_8_ROOT_AUTHORITY_LIFECYCLE_LATCH_ACTIVE",
                        formID,
                        state.wfmTransitionCallsTotal,
                        state.wfmTransitionChangesTotal,
                        state.wfmRuntimeInjectedTotal,
                        state.wfmRuntimeIgnoreFromTotal,
                        state.wfmRuntimeIgnoreToTotal,
                        state.creatureDbProvenancePolicyWrites,
                        state.creatureDbProvenanceRootToRoot,
                        state.creatureDbProvenanceAmbiguous,
                        state.wfmSideMapResolvedDirectTotal,
                        state.wfmSideMapResolvedSwappedTotal,
                        state.wfmSideMapCacheUsedTotal,
                        state.wfmSideMapAmbiguousTotal,
                        state.wfmSideMapPostCalibratedTotal,
                        state.wfmSideMapDisagreementTotal,
                        state.syntheticWfmAppliedTotal,
                        state.syntheticWfmWaitTotal,
                        state.vanillaYawTransportAppliedTotal,
                        state.syntheticWfmBypassTxTotal,
                        state.wfmOutputClassifiedTotal,
                        state.wfmOutputUnknownTotal,
                        state.nativeClipGenerateCallsTotal,
                        state.nativeClipRootOutputsTotal,
                        state.externalTranslationWrites,
                        state.externalRotationWrites);
                }
                if (formID == 0x14 || state.playerFrameCount > 0 || state.playerClipWitnesses > 0) {
                    std::uint64_t cyclePasses = 0;
                    std::uint64_t cycleDrifts = 0;
                    std::uint64_t cycleAmbiguous = 0;
                    std::uint64_t cyclesCompleted = 0;
                    for (const auto& [key, audit] : state.playerCycleAudit) {
                        (void)key;
                        cyclePasses += audit.tightPasses;
                        cycleDrifts += audit.driftCycles;
                        cycleAmbiguous += audit.ambiguousCycles;
                        cyclesCompleted += audit.completedCycles;
                    }
                    SKSE::log::info(
                        "[PlayerRootMotion][SUMMARY] form={:08X} frames={} graphCharacters={} clipCallbacks={} dbMisses={} clipWitnesses={} arms={} restores={} translationWrites={} rotationWrites={} passThroughs={} scale=1.00 physicalPathTotalXY={:.3f} frameAudit[singleKey={} ambiguous={}] cycleAudit[completed={} passTight={} driftOrConstraint={} ambiguous={}]",
                        formID, state.playerFrameCount, state.hkbCharacterCount, state.playerClipCallbacks,
                        state.playerClipDbMisses, state.playerClipWitnesses, state.playerArmCount,
                        state.playerRestoreCount, state.externalTranslationWrites, state.externalRotationWrites,
                        state.externalPassThroughs, state.playerPhysicalPathTotalXY, state.playerSingleKeyFrames,
                        state.playerAmbiguousFrames, cyclesCompleted, cyclePasses, cycleDrifts, cycleAmbiguous);
                }
            }
            detail::g_clipInfo.clear();
            detail::g_actorState.clear();
            detail::g_nativeRootExitWitnesses.clear();
            detail::g_locomotionStandingWitnesses.clear();
        }
        {
            std::scoped_lock lock{ detail::g_nativeMotionAuditMutex };
            detail::g_nativeMotionAudit.clear();
        }
        detail::g_invocationCache = {};
        detail::g_evaluationDepth = 0;
        detail::g_evaluationStack.fill(nullptr);
    }
}

