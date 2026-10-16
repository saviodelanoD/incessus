// BUILD_ID: INCESSUS-ANIMATION-PRIORITY-R71-CONTINUOUS-FIRST-NATIVE-STANDING-PIVOT-2026-09-01

#include "PCH.h"
#include "AnimationPriority.h"

#include "MotionProfileDatabase.h"
#include "RotationIntentTranslator.h"
#include "SteeringIntentController.h"
#include "RE/B/bhkCharacterController.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>
#include <vector>

namespace AnimationPriority
{
    namespace
    {
        enum class Phase : std::uint8_t
        {
            kIdle = 0,
            kStoppingForCanned,
            kCannedPending,
            kCannedActive,
            kCleanup
        };

        enum class Route : std::uint8_t
        {
            kNone = 0,
            kTurn90,
            kTurn180
        };

        enum class DispatchKind : std::uint8_t
        {
            kNone = 0,
            kPrepare,
            kCanned,
            kCleanup
        };

        constexpr float kMinDeltaTime = 1.0e-5f;
        constexpr float kMaxDeltaTime = 0.100f;

        struct State
        {
            Phase phase{ Phase::kIdle };
            Route route{ Route::kNone };
            std::string profileName;
            std::string acceptedEvent;

            float requestedRawTurn{ 0.0f };
            float phaseElapsed{ 0.0f };
            float cooldownRemaining{ 0.0f };
            std::uint8_t standingFrames{ 0 };
            std::uint8_t activeFalseFrames{ 0 };
            std::uint8_t witnessFailureFrames{ 0 };
            bool witnessFailureLogged{ false };
            bool intermediateRoute{ false };
            bool cleanupFailure{ false };
            std::string cleanupReason;

            // R60: one completed canned pivot consumes one large-turn episode.
            // Re-arm only after a new VALID + CONFIRMED steering sample is below
            // the configured pivot band. Missing intent and locomotion reacquire do
            // not count as neutralization.
            bool pivotRearmRequired{ false };

            // R71: a large combat-facing error while locomotion is active is navigation,
            // not a pivot request. Log/count the continuous episode once, but never stop
            // the actor. Canned pivots are admitted only after native graph Standing has
            // already been observed for the configured confirmation window.
            bool continuousDeferralLogged{ false };

            std::uint64_t requests{ 0 };
            std::uint64_t continuousPivotDeferrals{ 0 };
            std::uint64_t nativeStandingRequests{ 0 };
            std::uint64_t moveStopAttempts{ 0 };
            std::uint64_t moveStopAccepted{ 0 };
            std::uint64_t cannedAttempts{ 0 };
            std::uint64_t cannedAccepted{ 0 };
            std::uint64_t activeConfirmed{ 0 };
            std::uint64_t releases{ 0 };
            std::uint64_t graphAuthorityWrites{ 0 };
            std::uint64_t failures{ 0 };
        };

        struct DispatchPlan
        {
            DispatchKind kind{ DispatchKind::kNone };
            RE::FormID formID{ 0 };
            std::string event;
            std::string preemptEvent;
            std::vector<std::string> events;
            float rawTurn{ 0.0f };
            Route route{ Route::kNone };
            bool valid{ false };
        };

        std::unordered_map<RE::FormID, State> g_states;
        std::mutex g_mutex;

        [[nodiscard]] const char* PhaseName(Phase phase) noexcept
        {
            switch (phase) {
            case Phase::kStoppingForCanned:
                return "STOPPING";
            case Phase::kCannedPending:
                return "PENDING";
            case Phase::kCannedActive:
                return "ACTIVE";
            case Phase::kCleanup:
                return "CLEANUP";
            default:
                return "IDLE";
            }
        }

        [[nodiscard]] const char* RouteName(Route route) noexcept
        {
            switch (route) {
            case Route::kTurn90:
                return "90";
            case Route::kTurn180:
                return "180";
            default:
                return "NONE";
            }
        }

        [[nodiscard]] std::string Lower(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            return value;
        }

        [[nodiscard]] float ResolveDeltaTime(RE::Actor* actor, float hookDeltaTime) noexcept
        {
            if (std::isfinite(hookDeltaTime) && hookDeltaTime >= kMinDeltaTime && hookDeltaTime <= kMaxDeltaTime) {
                return hookDeltaTime;
            }

            if (actor) {
                if (auto* controller = actor->GetCharController()) {
                    const float controllerDelta = controller->stepInfo.deltaTime;
                    if (std::isfinite(controllerDelta) && controllerDelta >= kMinDeltaTime && controllerDelta <= kMaxDeltaTime) {
                        return controllerDelta;
                    }
                }
            }

            return 1.0f / 60.0f;
        }

        [[nodiscard]] bool RuntimeEligible(RE::Actor* actor) noexcept
        {
            return actor && !actor->IsPlayerRef() && actor->GetParentCell() && actor->Is3DLoaded() &&
                   !actor->IsDisabled() && !actor->IsDead() &&
                   !actor->IsInRagdollState() && !actor->IsInKillMove() &&
                   !actor->IsInMidair() && !actor->IsInWater() &&
                   !actor->IsBeingRidden();
        }

        [[nodiscard]] bool HasActionBypass(
            RE::Actor* actor,
            const MotionProfileDatabase::Profile& profile)
        {
            if (!actor) {
                return true;
            }

            for (const auto& variable : profile.rotation.actionBypassVariables) {
                if (variable.empty()) {
                    continue;
                }
                bool active = false;
                if (actor->GetGraphVariableBool(variable.c_str(), active) && active) {
                    return true;
                }
            }
            return false;
        }

        [[nodiscard]] bool ReadAnimationDriven(
            RE::Actor* actor,
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            bool& out)
        {
            out = false;
            return actor && !config.graphAnimationDrivenVariable.empty() &&
                actor->GetGraphVariableBool(config.graphAnimationDrivenVariable.c_str(), out);
        }

        [[nodiscard]] bool ReadLocomotionActive(
            RE::Actor* actor,
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            bool& out)
        {
            out = false;
            return actor && !config.graphLocomotionActiveVariable.empty() &&
                actor->GetGraphVariableBool(config.graphLocomotionActiveVariable.c_str(), out);
        }

        [[nodiscard]] Route ClassifyRoute(
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            float rawTurn,
            bool& outIntermediate)
        {
            outIntermediate = false;
            if (!std::isfinite(rawTurn)) {
                return Route::kNone;
            }

            const float magnitude = std::abs(rawTurn);
            if (magnitude <= config.turn90MinAngle) {
                return Route::kNone;
            }
            if (magnitude < config.turn90MaxAngle) {
                return Route::kTurn90;
            }
            if (magnitude < config.turn180MinAngle) {
                outIntermediate = true;
                return Lower(config.intermediatePolicy) == "turn90thenresidual" ?
                    Route::kTurn90 : Route::kNone;
            }
            return Route::kTurn180;
        }

        [[nodiscard]] bool IsLeftTurn(
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            float rawTurn)
        {
            const bool positiveIsLeft = Lower(config.positiveTurnDirection) != "right";
            return rawTurn > 0.0f ? positiveIsLeft : !positiveIsLeft;
        }

        [[nodiscard]] const char* DirectionName(
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            float rawTurn)
        {
            return IsLeftTurn(config, rawTurn) ? "LEFT" : "RIGHT";
        }

        [[nodiscard]] const std::vector<std::string>* SelectEvents(
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            Route route,
            float rawTurn)
        {
            const bool left = IsLeftTurn(config, rawTurn);
            if (route == Route::kTurn180) {
                return left ? std::addressof(config.turnLeft180Events) :
                              std::addressof(config.turnRight180Events);
            }
            if (route == Route::kTurn90) {
                return left ? std::addressof(config.turnLeft90Events) :
                              std::addressof(config.turnRight90Events);
            }
            return nullptr;
        }

        void WriteSuppressionVariables(
            RE::Actor* actor,
            const MotionProfileDatabase::AnimationPriorityConfig& config,
            bool enabled)
        {
            if (!actor) {
                return;
            }

            const float value = enabled ? 1.0f : 0.0f;
            for (const auto& variable : config.suppressionVariables) {
                if (variable.empty()) {
                    continue;
                }
                (void)actor->SetGraphVariableBool(variable.c_str(), enabled);
                (void)actor->SetGraphVariableFloat(variable.c_str(), value);
            }
        }

        [[nodiscard]] bool ApplyGraphState(
            RE::Actor* actor,
            const MotionProfileDatabase::Profile& profile,
            Phase phase)
        {
            if (!actor || !profile.animationPriority.enabled) {
                return false;
            }

            bool wrote = false;
            if (phase != Phase::kIdle && !profile.rotation.turnDeltaVariable.empty()) {
                // Canned turns are selected explicitly by event. During staging/active,
                // TurnDelta must not feed the normal locomotion/looping turn drivers.
                wrote |= actor->SetGraphVariableFloat(profile.rotation.turnDeltaVariable.c_str(), 0.0f);
            }

            // R71 CONTINUOUS-FIRST: AnimationPriority no longer writes graph Speed.
            // A pivot is subordinate to native AI/graph locomotion: this module may consume
            // an already-established Standing state, but it may never create or prolong one.
            // If native locomotion resumes while a canned event is pending, UpdateActor
            // releases the pivot immediately instead of fighting the gait controller.

            WriteSuppressionVariables(actor, profile.animationPriority, phase != Phase::kIdle);
            return wrote;
        }

        void ReleaseStateLocked(
            RE::Actor* actor,
            State& state,
            const MotionProfileDatabase::Profile* profile,
            const char* reason,
            bool failure)
        {
            const auto previous = state.phase;
            const auto eventName = state.acceptedEvent;
            const auto rawTurn = state.requestedRawTurn;
            const auto route = state.route;
            const bool armPivotRearm = !failure && previous == Phase::kCannedActive;

            state.phase = Phase::kIdle;
            state.route = Route::kNone;
            state.acceptedEvent.clear();
            state.requestedRawTurn = 0.0f;
            state.phaseElapsed = 0.0f;
            state.standingFrames = 0;
            state.activeFalseFrames = 0;
            state.witnessFailureFrames = 0;
            state.witnessFailureLogged = false;
            state.intermediateRoute = false;
            state.cleanupFailure = false;
            state.cleanupReason.clear();
            state.continuousDeferralLogged = false;
            if (armPivotRearm) {
                state.pivotRearmRequired = true;
            }
            ++state.releases;
            if (failure) {
                ++state.failures;
            }
            if (profile) {
                state.cooldownRemaining = profile->animationPriority.retryCooldownSeconds;
            }

            SKSE::log::info(
                "[AnimationPriority][RELEASE] form={:08X} previous={} route={} event=\"{}\" rawTurn={:+.2f} reason={} failure={} authority=CAPSULE",
                actor ? actor->GetFormID() : 0,
                PhaseName(previous),
                RouteName(route),
                eventName,
                rawTurn,
                reason ? reason : "unspecified",
                failure);
            if (armPivotRearm) {
                SKSE::log::info(
                    "[AnimationPriority][REARM_ARMED] form={:08X} route={} completedRaw={:+.2f} policy=WAIT_INTENT_NEUTRAL_OR_NATIVE_LOCOMOTION_REACQUIRE",
                    actor ? actor->GetFormID() : 0, RouteName(route), rawTurn);
            }
        }

        [[nodiscard]] DispatchPlan MakeCannedPlan(
            RE::Actor* actor,
            State& state,
            const MotionProfileDatabase::Profile& profile)
        {
            DispatchPlan plan{};
            const auto* events = SelectEvents(profile.animationPriority, state.route, state.requestedRawTurn);
            if (!events || events->empty()) {
                ReleaseStateLocked(actor, state, std::addressof(profile), "route_has_no_events", true);
                return plan;
            }

            state.phase = Phase::kCannedPending;
            state.phaseElapsed = 0.0f;
            state.acceptedEvent.clear();
            state.activeFalseFrames = 0;
            state.cleanupFailure = false;
            state.cleanupReason.clear();

            plan.kind = DispatchKind::kCanned;
            plan.formID = actor->GetFormID();
            plan.events = *events;
            plan.rawTurn = state.requestedRawTurn;
            plan.route = state.route;
            plan.valid = true;
            return plan;
        }

        [[nodiscard]] DispatchPlan BeginCleanupLocked(
            RE::Actor* actor,
            State& state,
            const MotionProfileDatabase::Profile& profile,
            const char* reason,
            bool failure)
        {
            DispatchPlan plan{};
            state.phase = Phase::kCleanup;
            state.phaseElapsed = 0.0f;
            state.activeFalseFrames = 0;
            state.cleanupFailure = failure;
            state.cleanupReason = reason ? reason : "cleanup";

            plan.kind = DispatchKind::kCleanup;
            plan.formID = actor ? actor->GetFormID() : 0;
            plan.event = profile.animationPriority.cannedTurnStopEvent;
            plan.rawTurn = state.requestedRawTurn;
            plan.route = state.route;
            plan.valid = !plan.event.empty();

            SKSE::log::warn(
                "[AnimationPriority][CLEANUP_BEGIN] form={:08X} previousEvent=\"{}\" route={} rawTurn={:+.2f} reason={} stopEvent=\"{}\" authority=HOLD",
                plan.formID, state.acceptedEvent, RouteName(state.route), state.requestedRawTurn,
                state.cleanupReason, plan.event);
            return plan;
        }

        void CompleteDispatch(RE::Character* actor, const DispatchPlan& plan)
        {
            if (!actor || !plan.valid) {
                return;
            }

            const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
            if (!profile || !profile->animationPriority.enabled) {
                return;
            }

            if (!plan.preemptEvent.empty()) {
                const bool accepted = actor->NotifyAnimationGraph(plan.preemptEvent.c_str());
                SKSE::log::info(
                    "[AnimationPriority][TURN_LOOP_PREEMPT] form={:08X} event=\"{}\" rawTurn={:+.2f} accepted={} note=nonfatal",
                    plan.formID, plan.preemptEvent, plan.rawTurn, accepted);
            }

            if (plan.kind == DispatchKind::kPrepare) {
                // Always give turnStop one graph evaluation before the canned event.
                // If the actor is already standing there is no moveStop to send; the
                // STOPPING phase itself is the one-frame/state-witness barrier.
                if (plan.event.empty()) {
                    SKSE::log::info(
                        "[AnimationPriority][PREPARE_STANDING] form={:08X} rawTurn={:+.2f} moveStop=NOT_NEEDED authority=HOLD",
                        plan.formID, plan.rawTurn);
                    return;
                }

                const bool accepted = actor->NotifyAnimationGraph(plan.event.c_str());
                SKSE::log::info(
                    "[AnimationPriority][MOVE_STOP] form={:08X} event=\"{}\" rawTurn={:+.2f} accepted={} authority=HOLD",
                    plan.formID, plan.event, plan.rawTurn, accepted);

                std::scoped_lock lock{ g_mutex };
                const auto it = g_states.find(plan.formID);
                if (it == g_states.end() || it->second.phase != Phase::kStoppingForCanned) {
                    return;
                }
                auto& state = it->second;
                ++state.moveStopAttempts;
                if (accepted) {
                    ++state.moveStopAccepted;
                } else {
                    ReleaseStateLocked(actor, state, profile, "move_stop_rejected", true);
                }
                return;
            }

            if (plan.kind == DispatchKind::kCleanup) {
                if (plan.event.empty()) {
                    return;
                }
                const bool accepted = actor->NotifyAnimationGraph(plan.event.c_str());
                SKSE::log::info(
                    "[AnimationPriority][CANNED_STOP] form={:08X} event=\"{}\" accepted={} reason=SAFETY_CLEANUP",
                    plan.formID, plan.event, accepted);
                return;
            }

            if (plan.kind != DispatchKind::kCanned) {
                return;
            }

            std::string acceptedEvent;
            for (const auto& eventName : plan.events) {
                if (eventName.empty()) {
                    continue;
                }

                {
                    std::scoped_lock lock{ g_mutex };
                    const auto it = g_states.find(plan.formID);
                    if (it == g_states.end() || it->second.phase != Phase::kCannedPending) {
                        return;
                    }
                    ++it->second.cannedAttempts;
                }

                const bool accepted = actor->NotifyAnimationGraph(eventName.c_str());
                SKSE::log::info(
                    "[AnimationPriority][EVENT_TRY] form={:08X} event=\"{}\" route={} rawTurn={:+.2f} accepted={}",
                    plan.formID, eventName, RouteName(plan.route), plan.rawTurn, accepted);
                if (accepted) {
                    acceptedEvent = eventName;
                    break;
                }
            }

            std::scoped_lock lock{ g_mutex };
            const auto it = g_states.find(plan.formID);
            if (it == g_states.end() || it->second.phase != Phase::kCannedPending) {
                return;
            }
            auto& state = it->second;
            if (acceptedEvent.empty()) {
                ReleaseStateLocked(actor, state, profile, "all_canned_events_rejected", true);
                return;
            }

            state.acceptedEvent = acceptedEvent;
            state.phaseElapsed = 0.0f;
            ++state.cannedAccepted;
            SKSE::log::info(
                "[AnimationPriority][PENDING] form={:08X} event=\"{}\" route={} rawTurn={:+.2f} witness=\"{}\" authority=HOLD",
                plan.formID,
                acceptedEvent,
                RouteName(state.route),
                state.requestedRawTurn,
                profile->animationPriority.graphAnimationDrivenVariable);
        }
    }

    YawPolicy GetYawPolicy(RE::FormID formID) noexcept
    {
        if (!formID) {
            return YawPolicy::kCapsuleSlerp;
        }

        std::scoped_lock lock{ g_mutex };
        const auto it = g_states.find(formID);
        if (it == g_states.end()) {
            return YawPolicy::kCapsuleSlerp;
        }

        switch (it->second.phase) {
        case Phase::kStoppingForCanned:
        case Phase::kCleanup:
            return YawPolicy::kHoldForAnimation;
        case Phase::kCannedActive:
            return YawPolicy::kAnimationOwnsYaw;
        case Phase::kCannedPending:
        default:
            // R71: dispatching a canned event is not yaw ownership. Until the
            // animation-driven witness actually turns true, continuous locomotion
            // and procedural capsule steering remain authoritative.
            return YawPolicy::kCapsuleSlerp;
        }
    }

    bool IsBusy(RE::Actor* actor) noexcept
    {
        return actor && GetYawPolicy(actor->GetFormID()) != YawPolicy::kCapsuleSlerp;
    }

    void UpdateActor(RE::Character* actor, bool enabled, float deltaTime)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        const bool eligible = enabled && profile && profile->animationPriority.enabled &&
            profile->rotation.enabled && RuntimeEligible(actor);

        const auto formID = actor->GetFormID();
        if (!eligible) {
            bool released = false;
            {
                std::scoped_lock lock{ g_mutex };
                const auto it = g_states.find(formID);
                if (it != g_states.end() && it->second.phase != Phase::kIdle) {
                    ReleaseStateLocked(actor, it->second, profile, "actor_not_eligible", true);
                    released = true;
                }
            }
            if (released && profile) {
                WriteSuppressionVariables(actor, profile->animationPriority, false);
            }
            return;
        }

        const float dt = ResolveDeltaTime(actor, deltaTime);
        bool animationDriven = false;
        bool locomotionActive = false;
        const bool animationWitnessValid =
            ReadAnimationDriven(actor, profile->animationPriority, animationDriven);
        const bool locomotionWitnessValid =
            ReadLocomotionActive(actor, profile->animationPriority, locomotionActive);
        const bool witnessesValid = animationWitnessValid && locomotionWitnessValid;

        if (!witnessesValid) {
            bool released = false;
            bool keepAuthority = false;
            bool emitWarning = false;
            std::uint8_t failureFrames = 0;
            {
                std::scoped_lock lock{ g_mutex };
                auto& state = g_states[formID];
                if (!state.profileName.empty() && state.profileName != profile->name) {
                    state = {};
                }
                state.profileName = profile->name;

                if (!state.witnessFailureLogged) {
                    state.witnessFailureLogged = true;
                    emitWarning = true;
                }

                if (state.phase != Phase::kIdle) {
                    if (state.witnessFailureFrames < 255) {
                        ++state.witnessFailureFrames;
                    }
                    failureFrames = state.witnessFailureFrames;
                    if (state.witnessFailureFrames >= 3) {
                        ReleaseStateLocked(actor, state, profile, "graph_witness_unavailable", true);
                        released = true;
                    } else {
                        keepAuthority = true;
                    }
                }
            }

            if (emitWarning) {
                SKSE::log::warn(
                    "[AnimationPriority][WITNESS_UNAVAILABLE] form={:08X} animationVariable=\"{}\" animationReadable={} locomotionVariable=\"{}\" locomotionReadable={} policy=NO_NEW_ROUTE",
                    formID,
                    profile->animationPriority.graphAnimationDrivenVariable,
                    animationWitnessValid,
                    profile->animationPriority.graphLocomotionActiveVariable,
                    locomotionWitnessValid);
            }
            if (released) {
                WriteSuppressionVariables(actor, profile->animationPriority, false);
            } else if (keepAuthority) {
                (void)ApplyGraphAuthority(actor);
                SKSE::log::debug(
                    "[AnimationPriority][WITNESS_RETRY] form={:08X} frameCount={} authority=HOLD",
                    formID, failureFrames);
            }
            return;
        }

        // R66 separates navigation steering from cinematic pivot authority.
        // Path/AI-process/follow targets remain valid inputs for continuous locomotion
        // steering, but they are deliberately NOT allowed to dispatch canned 90/180
        // pivots: navmesh waypoints can jump across obstacles and would otherwise turn
        // every path replanning edge into a stop-and-pivot decision. Combat facing is
        // the only current cinematic source. Existing canned animations are still
        // allowed to finish if the source changes after they become active.
        SteeringIntentController::FrameCommand steeringCommand{};
        const bool steeringCommandValid =
            SteeringIntentController::GetFrameCommand(actor, steeringCommand) && steeringCommand.valid;
        const bool cinematicPivotSource =
            steeringCommandValid && steeringCommand.source == SteeringIntentController::Source::kCombatTarget;

        float rawTurn = 0.0f;
        const bool rawTurnAvailable = RotationIntentTranslator::GetLatchedRawTurnError(actor, rawTurn);
        const bool hasRawTurn = rawTurnAvailable && cinematicPivotSource;
        const bool actionBypass = HasActionBypass(actor, *profile);

        DispatchPlan dispatch{};
        bool applyAuthority = false;
        bool clearSuppression = false;

        {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_states[formID];
            if (!state.profileName.empty() && state.profileName != profile->name) {
                state = {};
            }
            state.profileName = profile->name;
            state.witnessFailureFrames = 0;
            state.witnessFailureLogged = false;
            state.cooldownRemaining = std::max(0.0f, state.cooldownRemaining - dt);

            if (state.phase != Phase::kIdle && actionBypass) {
                ReleaseStateLocked(actor, state, profile, "action_bypass", true);
                clearSuppression = true;
            } else {
                switch (state.phase) {
                case Phase::kStoppingForCanned:
                    state.phaseElapsed += dt;
                    applyAuthority = true;

                    // The AI target may change while we are staging. Re-plan before the
                    // canned event is sent; never commit a stale direction/angle.
                    if (!hasRawTurn || std::abs(rawTurn) < profile->animationPriority.turn90MinAngle) {
                        ReleaseStateLocked(actor, state, profile, "intent_dropped_while_stopping", false);
                        clearSuppression = true;
                        break;
                    }
                    {
                        bool intermediateNow = false;
                        const Route routeNow = ClassifyRoute(profile->animationPriority, rawTurn, intermediateNow);
                        const bool directionChanged = (rawTurn > 0.0f) != (state.requestedRawTurn > 0.0f);
                        if (routeNow == Route::kNone || directionChanged) {
                            ReleaseStateLocked(actor, state, profile, "intent_reversed_while_stopping", false);
                            clearSuppression = true;
                            break;
                        }
                        if (routeNow != state.route || std::abs(rawTurn - state.requestedRawTurn) >= 10.0f) {
                            SKSE::log::info(
                                "[AnimationPriority][REPLAN] form={:08X} oldRaw={:+.2f} newRaw={:+.2f} oldRoute={} newRoute={} direction={}",
                                formID, state.requestedRawTurn, rawTurn, RouteName(state.route), RouteName(routeNow),
                                DirectionName(profile->animationPriority, rawTurn));
                            state.route = routeNow;
                            state.requestedRawTurn = rawTurn;
                            state.intermediateRoute = intermediateNow;
                        }
                    }

                    if (!locomotionActive) {
                        if (state.standingFrames < 255) {
                            ++state.standingFrames;
                        }
                    } else {
                        state.standingFrames = 0;
                    }

                    if (state.standingFrames >= profile->animationPriority.standingConfirmFrames) {
                        SKSE::log::info(
                            "[AnimationPriority][STANDING_CONFIRMED] form={:08X} frames={} rawTurn={:+.2f} witness={} next=CANNED",
                            formID, state.standingFrames, state.requestedRawTurn,
                            profile->animationPriority.graphLocomotionActiveVariable);
                        dispatch = MakeCannedPlan(actor, state, *profile);
                        applyAuthority = state.phase != Phase::kIdle;
                    } else if (state.phaseElapsed >= profile->animationPriority.stopTimeoutSeconds) {
                        ReleaseStateLocked(actor, state, profile, "standing_timeout", true);
                        clearSuppression = true;
                    }
                    break;

                case Phase::kCannedPending:
                    state.phaseElapsed += dt;
                    applyAuthority = false;

                    // R71: the canned event is opportunistic. If Bethesda/AI has already
                    // resumed locomotion before the canned animation actually becomes the
                    // animation-driven owner, movement wins immediately. Never hold a wolf
                    // stationary waiting for a pivot that lost its natural Standing window.
                    if (locomotionActive && !animationDriven) {
                        ReleaseStateLocked(actor, state, profile,
                            "native_locomotion_preempted_pending", false);
                        clearSuppression = true;
                        break;
                    }

                    if (animationDriven) {
                        state.phase = Phase::kCannedActive;
                        state.phaseElapsed = 0.0f;
                        state.activeFalseFrames = 0;
                        ++state.activeConfirmed;
                        applyAuthority = true;
                        SKSE::log::info(
                            "[AnimationPriority][ACTIVE] form={:08X} event=\"{}\" route={} rawTurn={:+.2f} witness={} authority=ANIMATION",
                            formID,
                            state.acceptedEvent,
                            RouteName(state.route),
                            state.requestedRawTurn,
                            profile->animationPriority.graphAnimationDrivenVariable);
                    } else if (state.phaseElapsed >= profile->animationPriority.eventConfirmTimeoutSeconds) {
                        // No animation owner ever appeared, so there is nothing to clean up.
                        // R71 releases immediately rather than entering another HOLD/stop state.
                        ReleaseStateLocked(actor, state, profile,
                            "animation_witness_timeout_before_ownership", true);
                        clearSuppression = true;
                    }
                    break;

                case Phase::kCannedActive:
                    state.phaseElapsed += dt;
                    applyAuthority = true;
                    if (animationDriven) {
                        state.activeFalseFrames = 0;
                    } else if (state.activeFalseFrames < 255) {
                        ++state.activeFalseFrames;
                    }

                    // bAnimationDriven is bound directly to CannedTurnBehavior.isActive.
                    // The first observed false therefore means animation yaw no longer owns
                    // the controller; release immediately so CapsuleSlerp handles residual
                    // instead of allowing one frame of unrestricted vanilla yaw.
                    if (state.activeFalseFrames >= 1) {
                        ReleaseStateLocked(actor, state, profile, "animation_complete", false);
                        clearSuppression = true;
                    } else if (state.phaseElapsed >= profile->animationPriority.activeTimeoutSeconds) {
                        // The graph still claims CannedTurnBehavior owns yaw after its safety
                        // budget. Enter an explicit HOLD cleanup state and request the real
                        // canned stop event; never hand yaw back to the capsule on this tick.
                        dispatch = BeginCleanupLocked(
                            actor, state, *profile, "active_timeout", true);
                        applyAuthority = true;
                    }
                    break;

                case Phase::kCleanup:
                    state.phaseElapsed += dt;
                    applyAuthority = true;

                    // R71: cleanup is a safety state, not a locomotion jail. Once the
                    // animation owner is gone, a native locomotion reacquire wins instantly.
                    if (locomotionActive && !animationDriven) {
                        const std::string reason = state.cleanupReason.empty() ?
                            std::string{ "native_locomotion_reacquired_cleanup" } :
                            state.cleanupReason + "_native_locomotion_reacquired";
                        const bool failure = state.cleanupFailure;
                        ReleaseStateLocked(actor, state, profile, reason.c_str(), failure);
                        clearSuppression = true;
                        break;
                    }

                    if (animationDriven) {
                        state.activeFalseFrames = 0;
                    } else if (state.activeFalseFrames < 255) {
                        ++state.activeFalseFrames;
                    }

                    if (state.activeFalseFrames >= profile->animationPriority.cleanupConfirmFrames) {
                        const std::string reason = state.cleanupReason.empty() ?
                            std::string{ "cleanup_completed" } : state.cleanupReason + "_cleanup_completed";
                        const bool failure = state.cleanupFailure;
                        ReleaseStateLocked(actor, state, profile, reason.c_str(), failure);
                        clearSuppression = true;
                    } else if (state.phaseElapsed >= profile->animationPriority.stopTimeoutSeconds) {
                        ReleaseStateLocked(actor, state, profile, "cleanup_witness_timeout", true);
                        clearSuppression = true;
                    }
                    break;

                case Phase::kIdle:
                default:
                    // R60 strict intent-episode rearm. A completed canned pivot consumes
                    // the entire large-turn episode. Missing/unavailable raw intent is NOT
                    // neutral intent, and merely reacquiring locomotion is NOT permission for
                    // another pivot. Re-arm only after the same validated/confirmed steering
                    // source actually falls below the configured pivot band. While waiting,
                    // this module remains IDLE: procedural locomotion/yaw are free to reduce
                    // the residual naturally.
                    if (state.pivotRearmRequired) {
                        const bool cinematicSourceReleased =
                            steeringCommandValid && !cinematicPivotSource;
                        const bool intentNeutral =
                            hasRawTurn &&
                            std::isfinite(rawTurn) &&
                            std::abs(rawTurn) < profile->animationPriority.turn90MinAngle;

                        if (cinematicSourceReleased) {
                            state.pivotRearmRequired = false;
                            SKSE::log::info(
                                "[AnimationPriority][R66_REARM_SOURCE_RELEASE] form={:08X} source={} target={:08X} action=REARM_CINEMATIC_PIVOT_EPISODE_NAVIGATION_REMAINS_CONTINUOUS",
                                formID,
                                SteeringIntentController::SourceName(steeringCommand.source),
                                steeringCommand.targetFormID);
                        } else if (intentNeutral) {
                            state.pivotRearmRequired = false;
                            SKSE::log::info(
                                "[AnimationPriority][REARM_READY] form={:08X} reason=VALID_COMBAT_FACING_INTENT_NEUTRALIZED rawAvailable=true rawTurn={:+.2f} threshold={:.2f}",
                                formID, rawTurn, profile->animationPriority.turn90MinAngle);
                        } else {
                            break;
                        }
                    }

                    // MotionDB locomotion may legitimately have bAnimationDriven=true. R71
                    // never preempts that locomotion. A different animation-driven standing
                    // state is also not eligible to be stolen by a canned pivot.
                    if (rawTurnAvailable && !cinematicPivotSource &&
                        std::isfinite(rawTurn) &&
                        std::abs(rawTurn) >= profile->animationPriority.turn90MinAngle) {
                        SKSE::log::debug(
                            "[AnimationPriority][R66_NAV_PIVOT_SUPPRESSED] form={:08X} rawTurn={:+.2f} source={} target={:08X} action=KEEP_CONTINUOUS_LOCOMOTION_STEERING_NO_MOVESTOP_NO_CANNED",
                            formID,
                            rawTurn,
                            steeringCommandValid ? SteeringIntentController::SourceName(steeringCommand.source) : "INVALID",
                            steeringCommandValid ? steeringCommand.targetFormID : 0);
                    }

                    const bool drivenWithoutLocomotion = animationDriven && !locomotionActive;
                    if (state.cooldownRemaining > 0.0f || actionBypass || drivenWithoutLocomotion || !hasRawTurn) {
                        state.standingFrames = 0;
                        state.continuousDeferralLogged = false;
                        break;
                    }

                    bool intermediate = false;
                    const Route route = ClassifyRoute(profile->animationPriority, rawTurn, intermediate);
                    if (route == Route::kNone) {
                        state.standingFrames = 0;
                        state.continuousDeferralLogged = false;
                        break;
                    }

                    // R71 CONTINUOUS-FIRST CONTRACT:
                    // A heading error is NOT permission to stop locomotion. While Skyrim's
                    // native graph says locomotion is active, even 90/120/180 degree combat
                    // errors remain continuous steering. The capsule/yaw controller and the
                    // directional locomotion blend are allowed to solve the curve together.
                    // Canned turns are only opportunistic pivots after the graph/AI has
                    // independently entered Standing.
                    if (locomotionActive) {
                        state.standingFrames = 0;
                        if (!state.continuousDeferralLogged) {
                            state.continuousDeferralLogged = true;
                            ++state.continuousPivotDeferrals;
                            SKSE::log::info(
                                "[AnimationPriority][R71_CONTINUOUS_FIRST] form={:08X} rawTurn={:+.2f} direction={} route={} source=COMBAT_TARGET locomotionActive=true action=KEEP_LOCOMOTION_NO_MOVESTOP_NO_TURNSTOP_NO_HOLD",
                                formID,
                                rawTurn,
                                DirectionName(profile->animationPriority, rawTurn),
                                RouteName(route));
                        }
                        break;
                    }

                    state.continuousDeferralLogged = false;
                    if (state.standingFrames < 255) {
                        ++state.standingFrames;
                    }
                    if (state.standingFrames < profile->animationPriority.standingConfirmFrames) {
                        break;
                    }

                    // Native Standing has already been stable for the configured window.
                    // Only now may AnimationPriority request a canned pivot. It sends neither
                    // moveStop nor turnStop and never writes graph Speed. If native locomotion
                    // resumes before the canned animation owns bAnimationDriven, the request
                    // is immediately abandoned.
                    state.route = route;
                    state.requestedRawTurn = rawTurn;
                    state.phaseElapsed = 0.0f;
                    state.activeFalseFrames = 0;
                    state.witnessFailureFrames = 0;
                    state.witnessFailureLogged = false;
                    state.acceptedEvent.clear();
                    state.intermediateRoute = intermediate;
                    state.cleanupFailure = false;
                    state.cleanupReason.clear();
                    ++state.requests;
                    ++state.nativeStandingRequests;

                    SKSE::log::info(
                        "[AnimationPriority][R71_NATIVE_STANDING_PIVOT] form={:08X} profile=\"{}\" rawTurn={:+.2f} direction={} route={} intermediate={} standingFrames={} source=COMBAT_TARGET action=DISPATCH_CANNED_NO_MOVESTOP_NO_TURNSTOP_NO_SPEED_WRITE",
                        formID,
                        profile->name,
                        rawTurn,
                        DirectionName(profile->animationPriority, rawTurn),
                        RouteName(route),
                        intermediate,
                        state.standingFrames);

                    dispatch = MakeCannedPlan(actor, state, *profile);
                    // PENDING is deliberately non-authoritative in R71. The event must
                    // actually establish bAnimationDriven before any graph/yaw suppression.
                    applyAuthority = false;
                    break;
                }
            }
        }

        if (clearSuppression) {
            WriteSuppressionVariables(actor, profile->animationPriority, false);
        }
        if (applyAuthority) {
            (void)ApplyGraphAuthority(actor);
        }
        if (dispatch.valid) {
            CompleteDispatch(actor, dispatch);
        }
    }

    bool ApplyGraphAuthority(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || !profile->animationPriority.enabled) {
            return false;
        }

        Phase phase = Phase::kIdle;
        {
            std::scoped_lock lock{ g_mutex };
            const auto it = g_states.find(actor->GetFormID());
            if (it == g_states.end() || it->second.phase == Phase::kIdle) {
                return false;
            }
            phase = it->second.phase;
        }

        const bool wrote = ApplyGraphState(actor, *profile, phase);
        if (wrote) {
            std::scoped_lock lock{ g_mutex };
            const auto it = g_states.find(actor->GetFormID());
            if (it != g_states.end()) {
                ++it->second.graphAuthorityWrites;
            }
        }
        return wrote;
    }

    void Clear()
    {
        std::scoped_lock lock{ g_mutex };
        for (const auto& [formID, state] : g_states) {
            SKSE::log::info(
                "[AnimationPriority][SUMMARY] form={:08X} profile=\"{}\" requests={} nativeStandingRequests={} continuousDeferrals={} moveStop={}/{} canned={}/{} activeConfirmed={} releases={} failures={} graphWrites={} finalPhase={}",
                formID,
                state.profileName,
                state.requests,
                state.nativeStandingRequests,
                state.continuousPivotDeferrals,
                state.moveStopAccepted,
                state.moveStopAttempts,
                state.cannedAccepted,
                state.cannedAttempts,
                state.activeConfirmed,
                state.releases,
                state.failures,
                state.graphAuthorityWrites,
                PhaseName(state.phase));
        }
        g_states.clear();
    }
}

