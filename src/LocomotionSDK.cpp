// BUILD_ID: ALR-SDK-V1-READ-ONLY-R69-2026-09-01
#include "PCH.h"

#include "LocomotionSDK.h"
#include "AnimalLocomotionRevolutionAPI.h"
#include "AnimationMotionDatabase.h"
#include "AnimationPriority.h"
#include "MotionProfileDatabase.h"
#include "SpeedIntentTranslator.h"
#include "SteeringIntentController.h"
#include "SyntheticMotionInjector.h"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string_view>
#include <unordered_map>

// Existing read-only R35/R40/R50 witnesses. Their implementation remains in the exact
// R67 SyntheticMotionInjector; SDK V1 only snapshots them after Character::Update.
namespace SyntheticMotionInjector
{
    [[nodiscard]] bool HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasProgressingAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] std::uint8_t RuntimeYawLocomotionFamilyMaskForCurrentOrPreviousFrame(RE::Actor* actor);
}

namespace
{
    constexpr const char* kSdkBuildId = "ALR-R69-SDK-V1-READ-ONLY-SNAPSHOT-2026-09-01";

    struct CachedActorSnapshot
    {
        ALR_API::LocomotionSnapshotV1 snapshot{};
        std::uint64_t serial{ 0 };
    };

    std::unordered_map<std::uint32_t, CachedActorSnapshot> g_snapshots;
    std::mutex g_snapshotMutex;
    std::atomic_bool g_consumerActive{ false };

    template <std::size_t N>
    void CopyText(char (&out)[N], std::string_view value) noexcept
    {
        static_assert(N > 0);
        const auto count = (std::min)(value.size(), N - 1);
        if (count > 0) {
            std::memcpy(out, value.data(), count);
        }
        out[count] = '\0';
    }

    [[nodiscard]] ALR_API::Gait MapGait(SpeedIntentTranslator::Family family) noexcept
    {
        switch (family) {
        case SpeedIntentTranslator::Family::kWalkForward: return ALR_API::Gait::kWalk;
        case SpeedIntentTranslator::Family::kTrotForward: return ALR_API::Gait::kTrot;
        case SpeedIntentTranslator::Family::kRunForward: return ALR_API::Gait::kRun;
        default: return ALR_API::Gait::kNone;
        }
    }

    [[nodiscard]] bool ReadActionBypass(
        RE::Actor* actor,
        const MotionProfileDatabase::Profile& profile) noexcept
    {
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

    [[nodiscard]] bool BuildSnapshot(
        RE::Character* actor,
        ALR_API::LocomotionSnapshotV1& out) noexcept
    {
        out = {};
        out.structSize = sizeof(out);
        out.interfaceVersion = ALR_API::kInterfaceVersion1;

        if (!actor || actor->IsPlayerRef()) {
            return false;
        }

        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || !MotionProfileDatabase::SupportsActor(actor)) {
            return false;
        }

        const auto actorFormID = actor->GetFormID();
        out.actorFormID = actorFormID;
        out.flags |= ALR_API::kSupportedActor;
        CopyText(out.profile, profile->folder.empty() ? profile->name : profile->folder);

        const auto worldPosition = actor->GetPosition();
        out.worldX = worldPosition.x;
        out.worldY = worldPosition.y;
        out.worldZ = worldPosition.z;
        out.worldYawRadians = actor->data.angle.z;

        out.walkGoal = profile->speed.walkGoal;
        out.trotGoal = profile->speed.trotGoal;
        out.runGoal = profile->speed.runGoal;
        if (!profile->speed.graphSpeed.empty()) {
            (void)actor->GetGraphVariableFloat(profile->speed.graphSpeed.c_str(), out.graphSpeed);
        }

        bool locomotionActive = false;
        if (!profile->animationPriority.graphLocomotionActiveVariable.empty() &&
            actor->GetGraphVariableBool(
                profile->animationPriority.graphLocomotionActiveVariable.c_str(), locomotionActive) &&
            locomotionActive) {
            out.flags |= ALR_API::kGraphLocomotionActive;
        }

        bool graphDriven = false;
        if (!profile->animationPriority.graphAnimationDrivenVariable.empty() &&
            actor->GetGraphVariableBool(
                profile->animationPriority.graphAnimationDrivenVariable.c_str(), graphDriven) &&
            graphDriven) {
            out.flags |= ALR_API::kGraphAnimationDriven;
        }

        if (const auto* state = actor->AsActorState()) {
            const bool forward = state->actorState1.movingForward != 0;
            const bool backward = state->actorState1.movingBack != 0;
            const bool left = state->actorState1.movingLeft != 0;
            const bool right = state->actorState1.movingRight != 0;
            if (forward || backward || left || right) out.flags |= ALR_API::kDirectionalMovement;
            if (forward) out.flags |= ALR_API::kMovingForward;
            if (backward) out.flags |= ALR_API::kMovingBackward;
            if (left) out.flags |= ALR_API::kMovingLeft;
            if (right) out.flags |= ALR_API::kMovingRight;
        }

        out.desiredGait = MapGait(SpeedIntentTranslator::LatchedFamily(actorFormID));
        out.intentRevision = SpeedIntentTranslator::Revision(actorFormID);
        out.runtimeCarrierMask =
            SyntheticMotionInjector::RuntimeYawLocomotionFamilyMaskForCurrentOrPreviousFrame(actor);
        if (out.runtimeCarrierMask != 0) {
            out.flags |= ALR_API::kRuntimeYawCarrier;
        }

        SteeringIntentController::FrameCommand command{};
        if (SteeringIntentController::GetFrameCommand(actor, command) && command.valid) {
            out.flags |= ALR_API::kSteeringValid;
            out.steeringFrameSerial = command.frameSerial;
            out.targetFormID = command.targetFormID;
            out.steeringSourceRaw = static_cast<std::uint8_t>(command.source);
            out.worldYawRadians = command.actorYawRadians;  // real capsule-facing basis used by steering
            out.goalX = command.goalPosition.x;
            out.goalY = command.goalPosition.y;
            out.goalZ = command.goalPosition.z;
            out.goalDistanceXY = command.goalDistanceXY;
            out.rawHeadingRadians = command.rawHeadingRadians;
            out.filteredHeadingRadians = command.filteredHeadingRadians;
            out.rawErrorRadians = command.rawErrorRadians;
            out.filteredErrorRadians = command.filteredErrorRadians;
            out.targetYawRateRadPerSecond = command.targetYawRateRadPerSecond;
            out.currentYawRateRadPerSecond = command.currentYawRateRadPerSecond;
            out.yawStepRadians = command.yawStepRadians;
            out.steeringDeltaTime = command.deltaTime;
            CopyText(out.steeringSourceName, SteeringIntentController::SourceName(command.source));
        }

        if (AnimationPriority::IsBusy(actor)) out.flags |= ALR_API::kPivotBusy;
        if (ReadActionBypass(actor, *profile)) out.flags |= ALR_API::kActionBypassActive;
        if (SyntheticMotionInjector::HasDbMotionSourceAuthorityForCurrentFrame(actor)) {
            out.flags |= ALR_API::kDbRootAuthority;
        }
        if (SyntheticMotionInjector::HasExternalTranslationForCurrentFrame(actor)) {
            out.flags |= ALR_API::kExternalTranslationWritten;
        }
        if (SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor)) {
            out.flags |= ALR_API::kAuthoredRotationObserved;
        }
        if (SyntheticMotionInjector::HasProgressingAuthoredRotationClipForCurrentOrPreviousFrame(actor)) {
            out.flags |= ALR_API::kAuthoredRotationProgressing;
        }
        return true;
    }

    std::uint32_t QueryLocomotion(
        std::uint32_t actorFormID,
        ALR_API::LocomotionSnapshotV1* out) noexcept
    {
        if (!out || out->structSize < sizeof(ALR_API::LocomotionSnapshotV1) || actorFormID == 0) {
            return 0;
        }

        std::scoped_lock lock{ g_snapshotMutex };
        const auto it = g_snapshots.find(actorFormID);
        if (it == g_snapshots.end()) {
            return 0;
        }
        *out = it->second.snapshot;
        return 1;
    }

    std::uint32_t QueryClipMetrics(
        const char* profile,
        const char* animationName,
        ALR_API::ClipMetricsV1* out) noexcept
    {
        if (!out || out->structSize < sizeof(ALR_API::ClipMetricsV1) ||
            !profile || !animationName) {
            return 0;
        }

        *out = {};
        out->structSize = sizeof(ALR_API::ClipMetricsV1);
        out->interfaceVersion = ALR_API::kInterfaceVersion1;
        const auto metrics = AnimationMotionDatabase::GetClipMetrics(profile, animationName);
        if (!metrics.valid) {
            return 0;
        }

        out->valid = 1;
        out->yawValid = metrics.yawValid ? 1 : 0;
        out->duration = metrics.duration;
        out->cycleX = metrics.cycleX;
        out->cycleY = metrics.cycleY;
        out->cycleZ = metrics.cycleZ;
        out->cycleYaw = metrics.cycleYaw;
        out->distanceXY = metrics.distanceXY;
        CopyText(out->profile, metrics.profile);
        CopyText(out->key, metrics.key);
        return 1;
    }

    std::uint32_t SampleCumulative(
        const char* profile,
        const char* animationName,
        float motionTime,
        ALR_API::CumulativeSampleV1* out) noexcept
    {
        if (!out || out->structSize < sizeof(ALR_API::CumulativeSampleV1) ||
            !profile || !animationName || !std::isfinite(motionTime)) {
            return 0;
        }

        *out = {};
        out->structSize = sizeof(ALR_API::CumulativeSampleV1);
        out->interfaceVersion = ALR_API::kInterfaceVersion1;
        const auto sample =
            AnimationMotionDatabase::SampleCumulativeSeconds(profile, animationName, motionTime);
        if (!sample.valid) {
            return 0;
        }

        out->valid = 1;
        out->yawValid = sample.yawValid ? 1 : 0;
        out->clipDuration = sample.clipDuration;
        out->motionTime = sample.motionTime;
        out->x = sample.x;
        out->y = sample.y;
        out->z = sample.z;
        out->yaw = sample.yaw;
        CopyText(out->profile, sample.profile);
        CopyText(out->key, sample.key);
        return 1;
    }

    const ALR_API::InterfaceV1 g_interface{
        sizeof(ALR_API::InterfaceV1),
        ALR_API::kInterfaceVersion1,
        kSdkBuildId,
        QueryLocomotion,
        QueryClipMetrics,
        SampleCumulative
    };
}

namespace LocomotionSDK
{
    void PublishActorFrame(RE::Character* actor) noexcept
    {
        if (!g_consumerActive.load(std::memory_order_relaxed)) {
            return;
        }

        ALR_API::LocomotionSnapshotV1 snapshot{};
        if (!BuildSnapshot(actor, snapshot)) {
            return;
        }

        std::scoped_lock lock{ g_snapshotMutex };
        auto& cached = g_snapshots[snapshot.actorFormID];
        snapshot.snapshotSerial = ++cached.serial;
        cached.snapshot = snapshot;
    }

    void Clear() noexcept
    {
        std::scoped_lock lock{ g_snapshotMutex };
        g_snapshots.clear();
    }
}

extern "C" ALR_SDK_API const ALR_API::InterfaceV1* ALR_RequestInterface(
    std::uint32_t version) noexcept
{
    const bool supported = version == ALR_API::kInterfaceVersion1;
    if (supported) {
        const bool wasActive = g_consumerActive.exchange(true, std::memory_order_relaxed);
        if (!wasActive) {
            SKSE::log::info(
                "[ALR_SDK][CONSUMER_ACTIVE] version={} mode=READ_ONLY_SNAPSHOT "
                "publish=POST_CHARACTER_UPDATE noGraphWrites=true noRootWrites=true noCapsuleWrites=true",
                version);
        }
    }
    return supported ? std::addressof(g_interface) : nullptr;
}

