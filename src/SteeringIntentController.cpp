// BUILD_ID: ALR-STEERING-INTENT-R33-AI-GOAL-CONTROLLER-2026-08-29
//
// R33 moves ALR one layer above Bethesda's angular controller.  The AI/path system
// remains authoritative for WHERE the creature is trying to go; ALR owns HOW the
// capsule turns toward that goal.  No rotationSpeed.z / V13 target / TurnDelta value
// is used as an input to this controller.
#include "PCH.h"
#include "SteeringIntentController.h"
#include "MotionProfileDatabase.h"

#include "RE/A/AIProcess.h"
#include "RE/B/bhkCharacterController.h"
#include "RE/H/HighProcessData.h"
#include "RE/H/hkTransform.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <immintrin.h>
#include <mutex>
#include <unordered_map>

namespace SteeringIntentController
{
    namespace
    {
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 6.28318530717958647692f;
        constexpr float kRadToDeg = 57.29577951308232f;
        constexpr float kDegToRad = 0.01745329251994329577f;

        constexpr float kFallbackDeltaTime = 1.0f / 60.0f;
        constexpr float kMinDeltaTime = 1.0f / 240.0f;
        constexpr float kMaxDeltaTime = 0.10f;

        // First stable controller candidate.  These deliberately favor a calm animal
        // over Bethesda's aggressive direction-chasing behavior.
        constexpr float kHeadingFilterResponsePerSecond = 8.0f;
        constexpr float kYawProportionalGainPerSecond = 2.5f;
        constexpr float kMaxYawAccelerationDegPerSecond2 = 240.0f;
        constexpr float kMaxYawDecelerationDegPerSecond2 = 360.0f;
        constexpr float kTurnEnterDeadZoneDeg = 3.0f;
        constexpr float kTurnExitDeadZoneDeg = 1.5f;
        constexpr float kReverseHysteresisDeg = 8.0f;
        constexpr float kPathPointMinDistance = 8.0f;
        constexpr float kPathPointMaxDistance = 4096.0f;
        constexpr float kPathPointMaxVerticalDelta = 2048.0f;
        constexpr float kReferenceTargetMinDistance = 8.0f;
        constexpr float kReferenceTargetMaxDistance = 16384.0f;
        constexpr float kSourceGraceSeconds = 0.18f;
        constexpr float kSourceSwitchResetDegrees = 110.0f;
        constexpr std::uint32_t kBurstLogs = 36;
        constexpr std::uint64_t kPeriodicLogFrames = 120;

        struct State
        {
            FrameCommand command{};
            std::uint64_t frameSerial{ 0 };

            bool headingInitialized{ false };
            float filteredHeadingRadians{ 0.0f };
            float currentYawRateRadPerSecond{ 0.0f };
            bool turning{ false };
            int turnSign{ 0 };

            float secondsSinceFreshSource{ 1000.0f };
            Source lastFreshSource{ Source::kNone };
            RE::FormID lastTargetFormID{ 0 };
            RE::NiPoint3 lastGoalPosition{};
            float lastGoalDistanceXY{ 0.0f };
            float lastRawHeadingRadians{ 0.0f };

            std::uint32_t logCount{ 0 };
            Source lastLoggedSource{ Source::kNone };
            int lastLoggedSign{ 0 };
            bool lastLoggedValid{ false };

            std::uint64_t sourcePath{ 0 };
            std::uint64_t sourceAITarget{ 0 };
            std::uint64_t sourceCombat{ 0 };
            std::uint64_t sourceFollow{ 0 };
            std::uint64_t sourceHeld{ 0 };
            std::uint64_t sourceNone{ 0 };
            std::uint64_t signReversalsBlocked{ 0 };
        };

        struct GoalSample
        {
            bool valid{ false };
            Source source{ Source::kNone };
            RE::FormID targetFormID{ 0 };
            RE::NiPoint3 position{};
            float distanceXY{ 0.0f };
        };

        std::unordered_map<RE::FormID, State> g_states;
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

        [[nodiscard]] float SanitizeDeltaTime(float value) noexcept
        {
            if (!std::isfinite(value) || value <= 1.0e-5f) {
                return kFallbackDeltaTime;
            }
            return std::clamp(value, kMinDeltaTime, kMaxDeltaTime);
        }

        [[nodiscard]] bool FinitePoint(const RE::NiPoint3& point) noexcept
        {
            return std::isfinite(point.x) && std::isfinite(point.y) && std::isfinite(point.z);
        }

        [[nodiscard]] float DistanceXY(const RE::NiPoint3& a, const RE::NiPoint3& b) noexcept
        {
            return std::hypot(b.x - a.x, b.y - a.y);
        }

        [[nodiscard]] bool ReadSteeringYaw(RE::Character* actor, float& outYaw) noexcept
        {
            outYaw = 0.0f;
            if (!actor) {
                return false;
            }

            // Control against the REAL capsule orientation, not Actor::data.angle.z.
            // R30 proved that our PMD yaw reaches bhkCharacterController; using that same
            // physical transform here closes only ALR's own heading-error loop and avoids
            // depending on a logical angle that may lag animation-driven root motion.
            // The bhk matrix yaw has the opposite sign from the actor/world steering
            // convention used by atan2(worldDX, worldDY), hence the single negation.
            if (auto* controller = actor->GetCharController()) {
                RE::hkTransform transform{};
                controller->GetTransformImpl(transform);
                alignas(16) float c0[4]{};
                _mm_store_ps(c0, transform.rotation.col0.quad);
                const float physicalMatrixYaw = std::atan2(c0[1], c0[0]);
                if (std::isfinite(physicalMatrixYaw)) {
                    outYaw = NormalizeRadians(-physicalMatrixYaw);
                    return true;
                }
            }

            // Last-resort bootstrap only when a physical controller is unavailable.
            if (std::isfinite(actor->data.angle.z)) {
                outYaw = NormalizeRadians(actor->data.angle.z);
                return true;
            }
            return false;
        }

        [[nodiscard]] bool RuntimeEligible(RE::Actor* actor) noexcept
        {
            return actor && !actor->IsPlayerRef() && actor->GetParentCell() && actor->Is3DLoaded() &&
                   !actor->IsDisabled() && !actor->IsDead() &&
                   !actor->IsInRagdollState() && !actor->IsInKillMove() &&
                   !actor->IsInMidair() && !actor->IsInWater() &&
                   !actor->IsBeingRidden();
        }

        [[nodiscard]] bool SampleReferenceGoal(
            RE::Actor* actor,
            RE::TESObjectREFR* target,
            Source source,
            const RE::NiPoint3& actorPosition,
            GoalSample& out) noexcept
        {
            if (!actor || !target || target == actor) {
                return false;
            }
            const auto targetPosition = target->GetPosition();
            if (!FinitePoint(targetPosition)) {
                return false;
            }
            const float distance = DistanceXY(actorPosition, targetPosition);
            if (!std::isfinite(distance) || distance < kReferenceTargetMinDistance ||
                distance > kReferenceTargetMaxDistance) {
                return false;
            }

            out.valid = true;
            out.source = source;
            out.targetFormID = target->GetFormID();
            out.position = targetPosition;
            out.distanceXY = distance;
            return true;
        }

        [[nodiscard]] GoalSample ResolveGoal(RE::Character* actor) noexcept
        {
            GoalSample out{};
            if (!actor) {
                return out;
            }

            const RE::NiPoint3 actorPosition = actor->GetPosition();
            auto* process = actor->GetActorRuntimeData().currentProcess;
            auto* high = process ? process->high : nullptr;

            // The highest exposed stable layer below the opaque MovementPathManager.
            // A plausible pathingDesiredPosition wins over object targets because it
            // represents the route/path bend rather than encouraging obstacle cutting.
            if (high && FinitePoint(high->pathingDesiredPosition)) {
                const auto& candidate = high->pathingDesiredPosition;
                const float distance = DistanceXY(actorPosition, candidate);
                const float vertical = std::abs(candidate.z - actorPosition.z);
                if (std::isfinite(distance) && std::isfinite(vertical) &&
                    distance >= kPathPointMinDistance && distance <= kPathPointMaxDistance &&
                    vertical <= kPathPointMaxVerticalDelta) {
                    out.valid = true;
                    out.source = Source::kPathDesiredPosition;
                    out.position = candidate;
                    out.distanceXY = distance;
                    return out;
                }
            }

            // If path data is unavailable, use genuine movement/combat package targets.
            // Do NOT use pathLookAtTarget: that can be head/look-at intent and must not
            // steer the capsule.
            if (auto combatTarget = actor->GetActorRuntimeData().currentCombatTarget.get();
                SampleReferenceGoal(actor, combatTarget.get(), Source::kCombatTarget, actorPosition, out)) {
                return out;
            }

            if (process) {
                auto follow = RE::TESObjectREFR::LookupByHandle(process->followTarget);
                if (follow && SampleReferenceGoal(
                        actor, follow.get(), Source::kFollowTarget, actorPosition, out)) {
                    return out;
                }

                auto target = RE::TESObjectREFR::LookupByHandle(process->target);
                if (target && SampleReferenceGoal(
                        actor, target.get(), Source::kAIProcessTarget, actorPosition, out)) {
                    return out;
                }
            }

            return out;
        }

        [[nodiscard]] float Approach(float current, float target, float maxDelta) noexcept
        {
            if (current < target) {
                return std::min(current + maxDelta, target);
            }
            if (current > target) {
                return std::max(current - maxDelta, target);
            }
            return current;
        }

        void CountSource(State& state, Source source) noexcept
        {
            switch (source) {
            case Source::kPathDesiredPosition: ++state.sourcePath; break;
            case Source::kAIProcessTarget: ++state.sourceAITarget; break;
            case Source::kCombatTarget: ++state.sourceCombat; break;
            case Source::kFollowTarget: ++state.sourceFollow; break;
            case Source::kHeldLastHeading: ++state.sourceHeld; break;
            default: ++state.sourceNone; break;
            }
        }
    }

    const char* SourceName(Source source) noexcept
    {
        switch (source) {
        case Source::kPathDesiredPosition: return "PATH_DESIRED_POSITION";
        case Source::kAIProcessTarget: return "AI_PROCESS_TARGET";
        case Source::kCombatTarget: return "COMBAT_TARGET";
        case Source::kFollowTarget: return "FOLLOW_TARGET";
        case Source::kHeldLastHeading: return "HELD_LAST_HEADING";
        default: return "NONE";
        }
    }

    bool SupportsActor(RE::Actor* actor) noexcept
    {
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        return RuntimeEligible(actor) && profile && profile->rotation.enabled;
    }

    void BeginFrame(RE::Character* actor, float deltaTime)
    {
        if (!actor || !SupportsActor(actor)) {
            if (actor) {
                std::scoped_lock lock{ g_mutex };
                g_states.erase(actor->GetFormID());
            }
            return;
        }

        const float dt = SanitizeDeltaTime(deltaTime);
        float actorYaw = 0.0f;
        if (!ReadSteeringYaw(actor, actorYaw)) {
            std::scoped_lock lock{ g_mutex };
            g_states.erase(actor->GetFormID());
            return;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        const float configuredMaxRateDeg = profile && profile->capsuleSlerp.enabled &&
                std::isfinite(profile->capsuleSlerp.maxYawRateDegreesPerSecond)
            ? std::max(1.0f, profile->capsuleSlerp.maxYawRateDegreesPerSecond)
            : 60.0f;
        const float maxRate = configuredMaxRateDeg * kDegToRad;
        const float maxAccel = kMaxYawAccelerationDegPerSecond2 * kDegToRad;
        const float maxDecel = kMaxYawDecelerationDegPerSecond2 * kDegToRad;

        GoalSample goal = ResolveGoal(actor);
        const RE::NiPoint3 actorPosition = actor->GetPosition();

        FrameCommand command{};
        bool emitLog = false;
        bool blockedReversal = false;

        {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_states[actor->GetFormID()];
            ++state.frameSerial;

            Source effectiveSource = goal.source;
            float rawHeading = state.lastRawHeadingRadians;
            RE::NiPoint3 effectiveGoal = goal.position;
            RE::FormID targetFormID = goal.targetFormID;
            float goalDistance = goal.distanceXY;
            bool sourceValid = goal.valid;

            if (goal.valid) {
                const Source previousSource = state.lastFreshSource;
                const RE::FormID previousTarget = state.lastTargetFormID;
                const float candidateHeading = NormalizeRadians(std::atan2(
                    goal.position.x - actorPosition.x,
                    goal.position.y - actorPosition.y));
                const float sourceJumpDeg = state.headingInitialized
                    ? std::abs(NormalizeRadians(candidateHeading - state.filteredHeadingRadians)) * kRadToDeg
                    : 0.0f;
                const bool hardSourceSwitch = state.headingInitialized &&
                    (goal.source != previousSource || goal.targetFormID != previousTarget) &&
                    sourceJumpDeg >= kSourceSwitchResetDegrees;

                state.secondsSinceFreshSource = 0.0f;
                state.lastFreshSource = goal.source;
                state.lastTargetFormID = goal.targetFormID;
                state.lastGoalPosition = goal.position;
                state.lastGoalDistanceXY = goal.distanceXY;
                rawHeading = candidateHeading;
                state.lastRawHeadingRadians = rawHeading;

                if (hardSourceSwitch) {
                    // A completely different AI objective must not spend a long time
                    // filtering through the old objective. Rebase heading, while yaw-rate
                    // acceleration limiting still prevents a physical snap.
                    state.filteredHeadingRadians = rawHeading;
                }
            } else {
                state.secondsSinceFreshSource += dt;
                if (state.headingInitialized && state.secondsSinceFreshSource <= kSourceGraceSeconds) {
                    sourceValid = true;
                    effectiveSource = Source::kHeldLastHeading;
                    effectiveGoal = state.lastGoalPosition;
                    targetFormID = state.lastTargetFormID;
                    goalDistance = state.lastGoalDistanceXY;
                    rawHeading = state.lastRawHeadingRadians;
                } else if (state.secondsSinceFreshSource > kSourceGraceSeconds) {
                    // Once the source really disappeared, discard the old heading. A later
                    // path/package acquisition starts clean instead of steering toward stale AI.
                    state.headingInitialized = false;
                }
            }

            if (sourceValid) {
                if (!state.headingInitialized || !std::isfinite(state.filteredHeadingRadians)) {
                    state.filteredHeadingRadians = rawHeading;
                    state.headingInitialized = true;
                } else {
                    const float alpha = std::clamp(
                        1.0f - std::exp(-kHeadingFilterResponsePerSecond * dt),
                        0.0f,
                        1.0f);
                    state.filteredHeadingRadians = NormalizeRadians(
                        state.filteredHeadingRadians +
                        NormalizeRadians(rawHeading - state.filteredHeadingRadians) * alpha);
                }
            }

            float rawError = 0.0f;
            float filteredError = 0.0f;
            float targetRate = 0.0f;

            if (sourceValid && state.headingInitialized) {
                rawError = NormalizeRadians(rawHeading - actorYaw);
                filteredError = NormalizeRadians(state.filteredHeadingRadians - actorYaw);
                const float absErrorDeg = std::abs(filteredError) * kRadToDeg;

                if (state.turning) {
                    if (absErrorDeg <= kTurnExitDeadZoneDeg) {
                        state.turning = false;
                        state.turnSign = 0;
                    }
                } else if (absErrorDeg >= kTurnEnterDeadZoneDeg) {
                    state.turning = true;
                    state.turnSign = filteredError > 0.0f ? 1 : -1;
                }

                if (state.turning) {
                    const int requestedSign = filteredError > 0.0f ? 1 : (filteredError < 0.0f ? -1 : 0);
                    if (state.turnSign != 0 && requestedSign != 0 && requestedSign != state.turnSign) {
                        if (absErrorDeg < kReverseHysteresisDeg) {
                            blockedReversal = true;
                            ++state.signReversalsBlocked;
                        } else {
                            state.turnSign = requestedSign;
                        }
                    }

                    if (!blockedReversal) {
                        targetRate = std::clamp(
                            filteredError * kYawProportionalGainPerSecond,
                            -maxRate,
                            +maxRate);
                    }
                }
            } else {
                state.turning = false;
                state.turnSign = 0;
            }

            const bool braking = std::abs(targetRate) < std::abs(state.currentYawRateRadPerSecond) ||
                (targetRate * state.currentYawRateRadPerSecond < 0.0f);
            const float rateDelta = (braking ? maxDecel : maxAccel) * dt;
            state.currentYawRateRadPerSecond = Approach(
                state.currentYawRateRadPerSecond,
                targetRate,
                rateDelta);
            state.currentYawRateRadPerSecond = std::clamp(
                state.currentYawRateRadPerSecond,
                -maxRate,
                +maxRate);

            if (!sourceValid && std::abs(state.currentYawRateRadPerSecond) < 0.05f * kDegToRad) {
                state.currentYawRateRadPerSecond = 0.0f;
            }

            command.valid = sourceValid;
            command.frameSerial = state.frameSerial;
            command.source = effectiveSource;
            command.targetFormID = targetFormID;
            command.actorPosition = actorPosition;
            command.goalPosition = effectiveGoal;
            command.goalDistanceXY = goalDistance;
            command.actorYawRadians = actorYaw;
            command.rawHeadingRadians = rawHeading;
            command.filteredHeadingRadians = state.filteredHeadingRadians;
            command.rawErrorRadians = rawError;
            command.filteredErrorRadians = filteredError;
            command.targetYawRateRadPerSecond = targetRate;
            command.currentYawRateRadPerSecond = state.currentYawRateRadPerSecond;
            command.yawStepRadians = state.currentYawRateRadPerSecond * dt;
            command.deltaTime = dt;
            state.command = command;

            CountSource(state, effectiveSource);

            const int commandSign = command.currentYawRateRadPerSecond > 1.0e-5f ? 1 :
                (command.currentYawRateRadPerSecond < -1.0e-5f ? -1 : 0);
            const bool sourceChanged = state.lastLoggedSource != effectiveSource;
            const bool validityChanged = state.lastLoggedValid != command.valid;
            const bool signChanged = state.lastLoggedSign != commandSign;
            emitLog = state.logCount < kBurstLogs || sourceChanged || validityChanged || signChanged ||
                blockedReversal || (state.frameSerial % kPeriodicLogFrames) == 0;
            if (emitLog) {
                ++state.logCount;
                state.lastLoggedSource = effectiveSource;
                state.lastLoggedValid = command.valid;
                state.lastLoggedSign = commandSign;
            }
        }

        if (emitLog) {
            SKSE::log::info(
                "[AI_STEERING][FRAME] form={:08X} frame={} valid={} source={} target={:08X} "
                "actorPos=({:+.2f},{:+.2f},{:+.2f}) goal=({:+.2f},{:+.2f},{:+.2f}) distXY={:.2f} "
                "yaw[actor={:+.2f} rawHeading={:+.2f} filteredHeading={:+.2f} rawError={:+.2f} filteredError={:+.2f}] "
                "rate[target={:+.2f} current={:+.2f} max={:.2f}] step={:+.3f}deg reversalBlocked={} "
                "controller=ALR_AI_GOAL_FILTERED_PROPORTIONAL_ACCEL_LIMITED",
                actor->GetFormID(),
                command.frameSerial,
                command.valid,
                SourceName(command.source),
                command.targetFormID,
                command.actorPosition.x, command.actorPosition.y, command.actorPosition.z,
                command.goalPosition.x, command.goalPosition.y, command.goalPosition.z,
                command.goalDistanceXY,
                command.actorYawRadians * kRadToDeg,
                command.rawHeadingRadians * kRadToDeg,
                command.filteredHeadingRadians * kRadToDeg,
                command.rawErrorRadians * kRadToDeg,
                command.filteredErrorRadians * kRadToDeg,
                command.targetYawRateRadPerSecond * kRadToDeg,
                command.currentYawRateRadPerSecond * kRadToDeg,
                configuredMaxRateDeg,
                command.yawStepRadians * kRadToDeg,
                blockedReversal);
        }
    }

    bool GetFrameCommand(RE::Actor* actor, FrameCommand& out) noexcept
    {
        out = {};
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }
        std::scoped_lock lock{ g_mutex };
        const auto it = g_states.find(actor->GetFormID());
        if (it == g_states.end()) {
            return false;
        }
        out = it->second.command;
        return out.valid && std::isfinite(out.yawStepRadians) &&
            std::isfinite(out.filteredErrorRadians);
    }

    void Clear()
    {
        std::scoped_lock lock{ g_mutex };
        for (const auto& [formID, state] : g_states) {
            SKSE::log::info(
                "[AI_STEERING][SUMMARY] form={:08X} frames={} sources[path={} aiTarget={} combat={} follow={} held={} none={}] reversalsBlocked={} last[source={} valid={} error={:+.2f}deg rate={:+.2f}deg/s]",
                formID,
                state.frameSerial,
                state.sourcePath,
                state.sourceAITarget,
                state.sourceCombat,
                state.sourceFollow,
                state.sourceHeld,
                state.sourceNone,
                state.signReversalsBlocked,
                SourceName(state.command.source),
                state.command.valid,
                state.command.filteredErrorRadians * kRadToDeg,
                state.command.currentYawRateRadPerSecond * kRadToDeg);
        }
        g_states.clear();
    }
}

