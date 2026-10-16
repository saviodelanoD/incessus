// BUILD_ID: ALR-UNIVERSAL-ROTATION-INTENT-R60-STEERING-SOURCE-OF-TRUTH-2026-08-31
//
// R60: canned-pivot intent now consumes the same validated/filtered steering error
// used by the runtime capsule-yaw controller. pathingDesiredOrientation.z is retained
// nowhere as a pivot source; invalid steering fails closed and breaks confirmation
// continuity so a stale/zero world heading cannot create a 90/180 request.
#include "PCH.h"
#include "RotationIntentTranslator.h"
#include "MotionProfileDatabase.h"
#include "SyntheticMotionInjector.h"
#include "SteeringIntentController.h"

#include "RE/B/bhkCharacterController.h"

#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

namespace RotationIntentTranslator
{
    namespace
    {
        constexpr float kRadToDeg = 57.29577951308232f;
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 6.28318530717958647692f;

        struct State
        {
            std::string profileName;
            
            float candidateErrorDegrees{ 0.0f };
            float latchedRawErrorDegrees{ 0.0f };
            float latchedErrorDegrees{ 0.0f };

            // Live, unfiltered heading error for the current frame. This is the signal
            // the moving-turn blender consumes. It is deliberately NOT gated by
            // candidateConfirmFrames: a turning actor changes heading error every frame
            // and would never satisfy the confirmation window.
            float liveErrorDegrees{ 0.0f };
            bool liveValid{ false };
            std::uint64_t liveSamples{ 0 };

            std::uint8_t candidateFrames{ 0 };
            bool rawLatched{ false };
            bool sampleFresh{ false };
            std::uint64_t revision{ 0 };
            
            std::uint64_t samples{ 0 };
            std::uint64_t graphWrites{ 0 };
            std::uint64_t capsuleSuppressions{ 0 };
        };

        std::unordered_map<RE::FormID, State> g_states;
        std::mutex g_mutex;

        [[nodiscard]] float NormalizeRadians(float value) noexcept
        {
            if (!std::isfinite(value)) return 0.0f;
            value = std::fmod(value, kTwoPi);
            if (value > kPi) value -= kTwoPi;
            if (value < -kPi) value += kTwoPi;
            return value;
        }

        [[nodiscard]] bool RuntimeEligible(RE::Actor* actor) noexcept
        {
            return actor && actor->GetParentCell() && actor->Is3DLoaded() &&
                   !actor->IsDisabled() && !actor->IsDead() &&
                   !actor->IsInRagdollState() && !actor->IsInKillMove() &&
                   !actor->IsInMidair() && !actor->IsInWater() &&
                   !actor->IsBeingRidden();
        }

        void SuppressControllerYaw(RE::Actor* actor)
        {
            if (!actor) return;
            if (auto* controller = actor->GetCharController()) {
                controller->rotMod = 0.0f;
                controller->rotModTime = 0.0f;
            }
        }

        [[nodiscard]] float SnapToClosestAngle(float errorDegrees, const std::vector<float>& allowedAngles) noexcept
        {
            if (allowedAngles.empty()) return errorDegrees;
            
            float closest = allowedAngles.front();
            float minDiff = std::abs(errorDegrees - closest);
            
            for (float angle : allowedAngles) {
                const float diff = std::abs(errorDegrees - angle);
                if (diff < minDiff) {
                    minDiff = diff;
                    closest = angle;
                }
            }
            return closest;
        }
    }

    bool SupportsActor(RE::Actor* actor)
    {
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        return profile && profile->rotation.enabled;
    }

    void BeforeCharacterUpdate(RE::Character* actor, float)
    {
        if (!actor) return;
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || !profile->rotation.enabled || !RuntimeEligible(actor)) {
            std::scoped_lock lock{ g_mutex };
            g_states.erase(actor->GetFormID());
            return;
        }
        const auto& config = profile->rotation;

        // AnimationPriority is called immediately after this producer. Mark the raw
        // sample stale first so a missing/invalid HighProcess sample cannot reuse an
        // old large turn as a new canned-turn request.
        {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_states[actor->GetFormID()];
            if (!state.profileName.empty() && state.profileName != profile->name) {
                state = {};
            }
            state.profileName = profile->name;
            state.sampleFresh = false;
            state.liveValid = false;
        }

        // A bAnimationDriven=true that THIS mod armed for the DB bootstrap is not
        // evidence that an authored animation owns rotation. Sealing controller yaw on
        // our own flag is self-inflicted: it removes the only remaining yaw authority
        // during walk/trot/run, where the locomotion clips carry no rotation track.
        const bool bootstrapDriven =
            SyntheticMotionInjector::IsDbBootstrapDrivenForCurrentFrame(actor);
        bool shouldSealCapsule = actor->IsAnimationDriven() && !bootstrapDriven;
        if (!shouldSealCapsule) {
            for (const auto& variable : config.actionBypassVariables) {
                if (variable.empty()) continue;
                bool active = false;
                if (actor->GetGraphVariableBool(variable.c_str(), active) && active) {
                    shouldSealCapsule = true;
                    break;
                }
            }
        }

        if (shouldSealCapsule) {
            SuppressControllerYaw(actor);
            std::scoped_lock lock{ g_mutex };
            ++g_states[actor->GetFormID()].capsuleSuppressions;
        }

        // R60 single source of truth: the procedural locomotion yaw and canned-pivot
        // classifier consume the same AI/path steering command. The old
        // pathingDesiredOrientation.z source is proven dead (0.0 in every RAW_LATCH
        // from the R59 capture) and must never create pivot intent.
        SteeringIntentController::FrameCommand steeringCommand{};
        const bool steeringValid =
            SteeringIntentController::GetFrameCommand(actor, steeringCommand) &&
            steeringCommand.valid &&
            std::isfinite(steeringCommand.filteredErrorRadians);

        if (!steeringValid) {
            // Missing steering means "no trustworthy intent", not zero degrees. Break
            // candidate continuity so a future valid sample must earn a full confirmation
            // window before AnimationPriority can consume it. sampleFresh/liveValid were
            // already cleared at the beginning of this actor update.
            std::scoped_lock lock{ g_mutex };
            auto& state = g_states[actor->GetFormID()];
            state.candidateErrorDegrees = 0.0f;
            state.candidateFrames = 0;
            return;
        }

        const float rawErrorDegrees =
            NormalizeRadians(steeringCommand.filteredErrorRadians) * kRadToDeg;

        {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_states[actor->GetFormID()];
            
            if (!state.profileName.empty() && state.profileName != profile->name) {
                state = {};
            }
            state.profileName = profile->name;
            ++state.samples;

            // Live signal now shares the exact filtered steering source used by R35
            // runtime yaw. It remains ungated by candidateConfirmFrames so the moving
            // turn observer can see the current error without waiting for pivot approval.
            state.liveErrorDegrees = rawErrorDegrees;
            state.liveValid = true;
            ++state.liveSamples;

            if (state.liveSamples <= 3 || (state.liveSamples % 240) == 0) {
                SKSE::log::debug(
                    "[RotationIntent][R60_STEERING_SOURCE] form={:08X} n={} valid=true source={} filteredError={:+.2f}deg candidateFrames={} basis=STEERING_INTENT_CONTROLLER_FILTERED_ERROR",
                    actor->GetFormID(),
                    state.liveSamples,
                    SteeringIntentController::SourceName(steeringCommand.source),
                    rawErrorDegrees,
                    state.candidateFrames);
            }

            const float diffRadians = NormalizeRadians((rawErrorDegrees - state.candidateErrorDegrees) / kRadToDeg);
            const float diffDegrees = std::abs(diffRadians * kRadToDeg);

            // Epsilon de 5 graus para filtrar micro-ru�dos do pathfinding
            if (diffDegrees > 5.0f) {
                state.candidateErrorDegrees = rawErrorDegrees;
                state.candidateFrames = 0;
            } else if (state.candidateFrames < 255) {
                ++state.candidateFrames;
            }

            if (state.candidateFrames >= config.candidateConfirmFrames) {
                const float newRaw = state.candidateErrorDegrees;
                const float newSnapped = SnapToClosestAngle(newRaw, config.allowedTurnAngles);
                const bool changed = !state.rawLatched ||
                    std::abs(newRaw - state.latchedRawErrorDegrees) >= 1.0f ||
                    std::abs(newSnapped - state.latchedErrorDegrees) >= 0.1f;

                state.latchedRawErrorDegrees = newRaw;
                state.latchedErrorDegrees = newSnapped;
                state.rawLatched = true;
                // AnimationPriority may consume the raw angle only while the current
                // candidate itself is confirmed. During a re-confirmation window the
                // graph can keep the previous snapped TurnDelta, but priority routing
                // must not act on a stale large-turn decision.
                state.sampleFresh = true;

                if (changed) {
                    ++state.revision;
                    SKSE::log::info(
                        "[RotationIntent][RAW_LATCH] form={:08X} rev={} raw={:+.2f}deg snapped={:+.2f}deg source={} steeringValid=true actorYaw={:+.2f}deg confirmFrames={} basis=STEERING_FILTERED_ERROR",
                        actor->GetFormID(),
                        state.revision,
                        state.latchedRawErrorDegrees,
                        state.latchedErrorDegrees,
                        SteeringIntentController::SourceName(steeringCommand.source),
                        actor->data.angle.z * kRadToDeg,
                        config.candidateConfirmFrames);
                }
            }

            // R17: this module NO LONGER writes TurnDelta.
            //
            // There must be exactly one writer of TurnDelta per frame. The GaitRouter
            // moving-turn blender (Hooks::ApplyMovingTurnBlend) is that writer: it runs
            // in ModifyAnimationUpdateData, which is the correct point in the frame, and
            // it maps the heading error onto the profile's authored L/C/R blend range.
            // Writing the snapped latch here as well produced two writers racing inside
            // the same frame, and the latch resolves to 0.0 whenever the confirmation
            // window has not closed - which is most of the time while actually turning.
            (void)config;
        }
    }

    bool ApplyLatchedGraphGoalAtAnimationInput(RE::Actor* actor)
    {
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!actor || !profile || !profile->rotation.enabled || profile->rotation.turnDeltaVariable.empty()) {
            return false;
        }

        float latched = 0.0f;
        {
            std::scoped_lock lock{ g_mutex };
            const auto it = g_states.find(actor->GetFormID());
            if (it == g_states.end()) return false;
            latched = it->second.latchedErrorDegrees;
        }

        const bool wrote = actor->SetGraphVariableFloat(profile->rotation.turnDeltaVariable.c_str(), latched);
        if (wrote) {
            std::scoped_lock lock{ g_mutex };
            ++g_states[actor->GetFormID()].graphWrites;
        }

        return wrote;
    }

    bool GetLiveTurnError(RE::Actor* actor, float& outDegrees)
    {
        outDegrees = 0.0f;
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }

        std::scoped_lock lock{ g_mutex };
        const auto it = g_states.find(actor->GetFormID());
        if (it == g_states.end() || !it->second.liveValid ||
            !std::isfinite(it->second.liveErrorDegrees)) {
            return false;
        }

        outDegrees = it->second.liveErrorDegrees;
        return true;
    }

    bool GetLatchedRawTurnError(RE::Actor* actor, float& outDegrees)
    {
        outDegrees = 0.0f;
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }

        std::scoped_lock lock{ g_mutex };
        const auto it = g_states.find(actor->GetFormID());
        if (it == g_states.end() || !it->second.rawLatched || !it->second.sampleFresh ||
            !std::isfinite(it->second.latchedRawErrorDegrees)) {
            return false;
        }

        outDegrees = it->second.latchedRawErrorDegrees;
        return true;
    }

    void Clear()
    {
        std::scoped_lock lock{ g_mutex };
        for (const auto& [formID, state] : g_states) {
            SKSE::log::info(
                "[RotationIntent][SUMMARY] form={:08X} profile=\"{}\" revisions={} samples={} liveSamples={} graphWrites={} capsuleSuppressions={} raw={:+.2f}deg snapped={:+.2f}deg live={:+.2f}deg",
                formID, state.profileName, state.revision, state.samples, state.liveSamples,
                state.graphWrites, state.capsuleSuppressions,
                state.latchedRawErrorDegrees, state.latchedErrorDegrees, state.liveErrorDegrees);
        }
        g_states.clear();
    }
}

