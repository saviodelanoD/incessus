// BUILD_ID: INCESSUS-R116-R85-CANNED-90-PARITY-2026-09-06
// BASELINE: ALR-UNIVERSAL-R15_8-ROOT-AUTHORITY-LIFECYCLE-LATCH-2026-08-22
// BASELINE Hooks.cpp SHA256: 7a4d8c2172ee523854df1c9833b01ad24f1755b505f9f7d920b15fcee86e99f0 (R93 user baseline)
#include "PCH.h"
#include "Hooks.h"

#include <chrono>

#include "AnimationMotionDatabase.h"
#include "AnimationPriority.h"
#include "CapsuleSlerp.h"
#include "MotionProfileDatabase.h"
#include "MotionTrajectoryController.h"
#include "LocomotionSDK.h"
#include "RotationIntentTranslator.h"
#include "SpeedIntentTranslator.h"
#include "SteeringIntentController.h"
#include "SyntheticMotionInjector.h"
#include "TranslationAuthority.h"

#include "RE/H/hkbClipGenerator.h"
#include "RE/H/hkbContext.h"
#include "RE/M/Misc.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <limits>
#include <memory>
#include <string>
#include <string_view>
#include <cstdint>
#include <cstring>
#include <mutex>
#include <unordered_map>
#include <vector>

// R38 adds one read-only classification query without changing the shared header/ABI.
// Keeping this declaration local means the candidate still ships only the two files that
// actually changed; existing CapsuleSlerp/header consumers remain untouched.
namespace SyntheticMotionInjector
{
    [[nodiscard]] bool HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasProgressingAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool ConsumeNativeRootExitToDefaultWitness(RE::Actor* actor);
    [[nodiscard]] bool ConsumeRecentLocomotionToStandingWitness(RE::Actor* actor, std::uint64_t& outAgeFrames);
    [[nodiscard]] std::uint8_t RuntimeYawLocomotionFamilyMaskForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] RE::FormID ResolveActorFormIDForHkbContext(const RE::hkbContext& context) noexcept;
    void ApplyConfiguredDirectionalPhaseSyncAfterUpdate(RE::hkbClipGenerator* clip, const RE::hkbContext& context, float inputDeltaTime);
}

namespace Hooks
{
    namespace detail
    {
        bool g_installed{ false };

        // R15.16: profile-driven gait graph router + moving turn blend on top of GOLDEN R15.8.
        // It owns only the semantic state-machine EVENT request. It never writes
        // SpeedSampled/Direction and never touches MotionDB/root authority.
        //
        // Each profile may declare Data/MotionDB/<profile>/GaitRouter.ini.
        // Routes are graph-native states (for the wolf: iSyncForwardState 0=Forward,
        // 1=Backward). Each semantic family maps to an existing behavior event.
        // Equal Walk/Trot events intentionally collapse both families into the same
        // graph state without restarting it when semantic WALK <-> TROT changes.
        struct GaitRouteConfig
        {
            bool configured{ false };
            std::int32_t value{ 0 };
            std::string name;
            std::string walkEvent;
            std::string trotEvent;
            std::string runEvent;
        };

        struct TurnBlendConfig
        {
            bool enabled{ false };
            std::int32_t routeValue{ 0 };
            std::string inputVariable;
            std::string dampedVariable;
            float deadZoneDegrees{ 2.0f };
            float walkFullErrorDegrees{ 15.0f };
            float trotFullErrorDegrees{ 22.0f };
            float runFullErrorDegrees{ 30.0f };
            float walkFullBlend{ 75.0f };
            float trotFullBlend{ 135.0f };
            float runFullBlend{ 270.0f };
            std::uint32_t telemetryEveryFrames{ 30 };
        };

        enum class SpeedCadencePolicy : std::uint8_t
        {
            kNativeContinuity = 0,
            kStableFamilyGoal
        };

        [[nodiscard]] const char* SpeedCadencePolicyName(SpeedCadencePolicy policy) noexcept
        {
            switch (policy) {
            case SpeedCadencePolicy::kStableFamilyGoal: return "STABLE_FAMILY_GOAL";
            default: return "NATIVE_CONTINUITY";
            }
        }

        enum class ParentIntentSource : std::uint8_t
        {
            kGraphFloat = 0,
            kSteering
        };

        [[nodiscard]] const char* ParentIntentSourceName(ParentIntentSource source) noexcept
        {
            switch (source) {
            case ParentIntentSource::kSteering: return "STEERING";
            default: return "GRAPH_FLOAT";
            }
        }

        struct DirectionIntentConfig
        {
            bool enabled{ false };
            std::string variable;
            float cycleMin{ 0.0f };
            float cycleMax{ 1.0f };
            float forwardValue{ 0.0f };
            bool positiveYawIncreases{ false };
            std::uint32_t telemetryEveryFrames{ 60 };
        };

        struct TemporalAuthorityConfig
        {
            bool enabled{ false };
            float playbackRate{ 1.0f };
            std::array<std::string, 16> assets{};
            std::array<float, 16> durations{};
            std::size_t assetCount{ 0 };
            std::uint32_t telemetryEveryFrames{ 60 };
        };

        struct HkxAuthoredCadenceConfig
        {
            bool enabled{ false };
            bool includeRuntimeYawClips{ true };
            bool includeRotationClips{ true };
            bool rotationRequiresStablePivot{ false };
            // R96: optional profile gate for native turn-in-place clips. When enabled,
            // RotationClips are clocked only while no RuntimeYaw locomotion carrier exists.
            bool rotationRequiresNoRuntimeCarrier{ false };
            float authoredRate{ 1.0f };
            float minActiveRate{ 0.02f };
            float minPoseWeight{ 0.001f };
            // R99: some Behavior graphs author multiple aliases of the same HKX at fixed
            // internal rates (for example Troll 90Slow=0.05 and 90=0.5). hkbClipGenerator::Update
            // restores/uses those authored rates internally, so writing playbackSpeed externally
            // is too late. Profile-declared node-rate pairs compensate the native Update timestep
            // instead, keeping pose and extracted motion on the same native timeline.
            std::array<std::string, 8> nativeRateOverrideNodes{};
            std::array<float, 8> nativeRateOverrideRates{};
            std::size_t nativeRateOverrideCount{ 0 };
            // R113: cadence-only aliases used exclusively while the profile's latched native
            // standing-turn window is active. These nodes are NOT RotationClips and gain no
            // root/yaw ownership; only hkbClipGenerator::Update receives the reciprocal native
            // rate timestep. This keeps Troll 90Slow/90 on the same authored 1x clock without
            // reviving R100 TurnDelta writes or R107 localTime phase repairs.
            std::array<std::string, 8> nativeStandingTurnRateOverrideNodes{};
            std::array<float, 8> nativeStandingTurnRateOverrideRates{};
            std::size_t nativeStandingTurnRateOverrideCount{ 0 };
            // R100: when true, RotationClips not explicitly named above are left fully native.
            // This prevents zero/transition-weight Slow aliases from receiving a generic clock.
            bool nativeRateOverridesExclusive{ false };
            std::uint32_t telemetryEveryFrames{ 120 };
        };

        // R93: profile-driven replacement for the vanilla continuous locomotion sampler.
        // The behavior remains the executor, but SpeedSampled/Direction cease to be physical
        // selection authorities. INCESSUS writes one exact authored speed sample and one exact
        // directional sector before native graph selection, then only reasserts the same decision
        // at the final boundary. Normal state is one
        // physical HKX; a direction crossfade may temporarily involve two.
        struct StableSampleSelectorConfig
        {
            bool enabled{ false };
            std::string speedVariable;
            std::string directionVariable;
            float walkSample{ 0.0f };
            float trotSample{ 0.0f };
            float runSample{ 0.0f };
            float cycleMin{ 0.0f };
            float cycleMax{ 1.0f };
            float forwardValue{ 0.0f };
            bool positiveYawIncreases{ false };
            std::uint32_t directionSectors{ 8 };
            float directionSwitchHysteresisDegrees{ 10.0f };
            std::uint32_t directionConfirmFrames{ 4 };
            float axisSwitchCooldownSeconds{ 0.22f };
            float minGoalDistance{ 0.0f };

            // R93: profile opt-in for discrete authored pivots when continuous locomotion
            // would otherwise solve a very large heading error by orbiting/backpedalling.
            // Event names/angles remain in Intent.ini::AnimationPriority; this flag only
            // changes when the stable selector is allowed to preempt locomotion.
            bool authoredPivotEnabled{ false };
            // R110: profiles may leave the native looping 90-degree behavior entirely to
            // Bethesda while retaining the discrete StablePivot preference for 180 degrees.
            // When false, the 90 band never enters StablePivot: no moveStop, no turn event,
            // no capsule hold/yaw override, and no selector candidate hold from this adapter.
            bool authoredPivot90Enabled{ true };
            std::uint32_t authoredPivotConfirmFrames{ 4 };
            // Default preserves R93. Profiles whose graph locomotion witness can remain
            // asserted after moveStop may opt into physical-carrier standing proof.
            bool authoredPivotRequireGraphInactive{ true };
            // Optional route threshold for the stable-pivot adapter. 0 preserves the
            // AnimationPriority profile threshold. Profiles may choose the geometric midpoint
            // between 90 and 180 (135 degrees) when the authored 180 should be preferred sooner.
            float authoredPivot180RouteMinAngle{ 0.0f };
            // R111: when false, 180 is a standing-only preference. Incessus never sends
            // moveStop to manufacture a 180 opportunity; it may emit the configured 180
            // event only after Bethesda has already left locomotion naturally.
            bool authoredPivot180PreemptLocomotion{ true };

            // R113: native standing-turn commitment. When StablePivot90 is disabled, a
            // profile may still ask INCESSUS to stop locomotion once for a confirmed large
            // heading, then leave the entire standing 90-loop lifecycle/yaw to Bethesda.
            // The window is latched across transient RuntimeYaw carrier flicker so the
            // StableSample final reassertion cannot restart locomotion mid-turn.
            bool nativeStandingTurnPreemptLocomotion{ false };
            float nativeStandingTurnMinGoalDistance{ 0.0f };
            float nativeStandingTurnCommitAngleDegrees{ 0.0f };
            float nativeStandingTurnReleaseAngleDegrees{ 45.0f };
            std::uint32_t nativeStandingTurnReleaseConfirmFrames{ 4 };
            float nativeStandingTurnTimeoutSeconds{ 5.0f };
            float nativeStandingTurnNoProgressTimeoutSeconds{ 1.0f };
            float nativeStandingTurnMinProgressPerFrameDegrees{ 0.02f };
            float nativeStandingTurnFailureCooldownSeconds{ 1.0f };
            // R114 start/stop quarantine: after the one-shot moveStop, keep the outgoing
            // locomotion sample frozen until the physical RuntimeYaw carrier has actually
            // drained. After the turn releases, hold parent re-entry briefly so Bethesda's
            // standing->locomotion transition cannot collide with the just-finished turn.
            std::uint32_t nativeStandingTurnStopDrainConfirmFrames{ 2 };
            float nativeStandingTurnStartQuarantineSeconds{ 0.20f };
            // Legacy R115 compatibility knob. R116 does not consume this value: patched
            // canned 90/180 clips own their authored yaw natively.
            float nativeStandingTurnPhysicalYawRateDegreesPerSecond{ 0.0f };

            // R100: optional graph parameter owned only while the stable selector has a
            // confirmed 90-degree authored pivot. This is not the old moving TurnDelta writer:
            // outside the pivot handshake INCESSUS never touches the variable.
            std::string authoredPivotTurnVariable;
            // Optional semantic node mapping used to scope the 90-degree cadence override to
            // the same left/right route latched by StablePivot.
            std::string authoredPivotLeft90Node;
            std::string authoredPivotRight90Node;
            std::string authoredPivotLeft180Node;
            std::string authoredPivotRight180Node;

            // R102: the native 90-degree Troll turn is a looping state, not a canned
            // single-play clip. StablePivot ends that loop from real capsule yaw progress.
            float authoredPivot90StopDegrees{ 85.0f };
            // R107: physical yaw rate used only by the V13 StablePivot ACTIVE servo.
            // This is independent of Bethesda TurnDelta/rotationSpeed.z and profile-driven.
            float authoredPivot90YawRateDegreesPerSecond{ 90.0f };
            // R108: if the certified same-side turn pose disappears before the physical
            // target, pause capsule yaw and allow only a bounded native event reacquire.
            std::uint32_t authoredPivot90MaxReacquireAttempts{ 1 };
            float authoredPivot90StallSeconds{ 0.25f };
            float authoredPivot90MinProgressDegrees{ 3.0f };

            std::uint32_t telemetryEveryFrames{ 120 };
        };

        enum class StablePivotPhase : std::uint8_t
        {
            kIdle = 0,
            kStopping,
            kPending,
            kActive
        };

        struct GaitRouterConfig
        {
            bool loaded{ false };
            bool enabled{ false };
            std::string profile;
            std::string routeVariable;
            std::string activeVariable;
            std::int32_t activeValue{ 1 };
            std::string parentStartEvent;
            // R81 generic continuous-parent entry mode. Profiles whose behavior has one
            // locomotion parent (rather than discrete gait-entry events) may use graph
            // intent to wake that parent without granting native capsule translation.
            bool parentOnly{ false };
            ParentIntentSource parentIntentSource{ ParentIntentSource::kGraphFloat };
            std::string parentIntentVariable;
            float parentIntentThreshold{ 5.0f };
            std::uint32_t parentIntentConfirmFrames{ 2 };
            // R84: profile opt-in for retrying parent-entry events for as long as upstream
            // movement intent remains physically unresolved.
            bool parentPersistentWhileIntent{ false };
            // R96: some behavior parents pulse inactive/active during ordinary blendspace
            // transitions. Re-sending the same gait event on that edge restarts the state
            // machine even though semantic intent did not change. Profiles may disable it.
            bool reassertSameSemanticOnActiveEdge{ true };
            float retrySeconds{ 0.10f };
            std::uint32_t maxAttempts{ 8 };
            SpeedCadencePolicy speedCadencePolicy{ SpeedCadencePolicy::kNativeContinuity };
            std::array<GaitRouteConfig, 8> routes{};
            TurnBlendConfig turnBlend{};
            DirectionIntentConfig directionIntent{};
            TemporalAuthorityConfig temporalAuthority{};
        };

        enum class StructuralRecoverySource : std::uint8_t
        {
            kNone = 0,
            kNativeRootActionExit,
            kLocomotionStandingDrop
        };

        [[nodiscard]] const char* StructuralRecoverySourceName(StructuralRecoverySource source) noexcept
        {
            switch (source) {
            case StructuralRecoverySource::kNativeRootActionExit: return "NATIVE_ROOT_ACTION_EXIT";
            case StructuralRecoverySource::kLocomotionStandingDrop: return "LOCOMOTION_TO_STANDING";
            default: return "NONE";
            }
        }

        struct GaitRouterActorState
        {
            bool seen{ false };
            bool routeReadFailureLogged{ false };
            bool activeReadFailureLogged{ false };
            bool lastActive{ false };
            std::int32_t lastRouteValue{ (std::numeric_limits<std::int32_t>::min)() };
            std::string lastDesiredEvent;
            std::string pendingEvent;
            std::string pendingRouteName;
            SpeedIntentTranslator::Family lastFamily{ SpeedIntentTranslator::Family::kNone };
            std::uint64_t lastRevision{ 0 };
            float retryRemaining{ 0.0f };
            std::uint32_t requestAttempts{ 0 };
            std::uint64_t requests{ 0 };
            std::uint64_t attempts{ 0 };
            std::uint64_t accepted{ 0 };
            std::uint64_t rejected{ 0 };
            std::uint64_t exhausted{ 0 };
            std::uint64_t routeChanges{ 0 };
            std::uint64_t sameSemanticActiveEdgeSuppressions{ 0 };

            // R81 continuous-parent-only bootstrap state. This is independent from
            // WALK/TROT/RUN semantic routing and is driven entirely by profile data.
            std::uint32_t parentOnlyIntentFrames{ 0 };
            std::uint32_t parentOnlyAttempts{ 0 };
            float parentOnlyRetryRemaining{ 0.0f };
            std::uint64_t parentOnlyRequests{ 0 };
            std::uint64_t parentOnlyAccepted{ 0 };
            std::uint64_t parentOnlyRejected{ 0 };
            std::uint64_t parentOnlyExhausted{ 0 };
            bool parentOnlyExhaustedEpisode{ false };

            // R91 stable-parent continuity. Once a real RuntimeYaw carrier has been
            // observed, brief parent/graph drops are treated as transition noise rather
            // than a new locomotion request. This prevents moveStart from repeatedly
            // restarting a locomotion tree that was already physically established.
            bool parentOnlyPhysicalReadySeen{ false };
            std::uint32_t parentOnlyPostCarrierMissingFrames{ 0 };
            std::uint64_t parentOnlyDropGraceSuppressions{ 0 };
            // R113 latched native standing-turn window. Unlike R111 this is not an
            // instantaneous mirror of RuntimeYaw carrier presence. Once committed it stays
            // active across transition-carrier flicker until heading completion, action
            // takeover, lost intent grace, or a bounded fail-open timeout.
            bool nativeStandingTurnPassthrough{ false };
            bool nativeStandingTurnStopAccepted{ false };
            bool nativeStandingTurnCannedOffered{ false };
            std::uint32_t nativeStandingTurnCannedRouteDegrees{ 0 };
            bool nativeStandingTurnAuthoredWitnessSeen{ false };
            std::uint32_t nativeStandingTurnAuthoredMissingFrames{ 0 };
            std::uint32_t nativeStandingTurnCandidateFrames{ 0 };
            std::uint32_t nativeStandingTurnReleaseFrames{ 0 };
            std::uint32_t nativeStandingTurnMissingIntentFrames{ 0 };
            float nativeStandingTurnElapsed{ 0.0f };
            float nativeStandingTurnCooldown{ 0.0f };
            float nativeStandingTurnEntryErrorDegrees{ 0.0f };
            bool nativeStandingTurnLeft{ false };
            bool nativeStandingTurnPhysicalYawInitialized{ false };
            float nativeStandingTurnLastPhysicalYawRadians{ 0.0f };
            float nativeStandingTurnNoProgressSeconds{ 0.0f };
            float nativeStandingTurnPhysicalProgressDegrees{ 0.0f };
            // R114: explicit bridge around Bethesda's parent transition.
            bool nativeStandingTurnStopDrain{ false };
            std::uint32_t nativeStandingTurnStopDrainClearFrames{ 0 };
            float nativeStandingTurnStartQuarantineRemaining{ 0.0f };
            std::uint64_t nativeStandingTurnWindows{ 0 };

            // R82 profile-driven direction-input ownership. This is graph intent only;
            // it never rotates the capsule or injects root by itself.
            std::uint64_t directionIntentWrites{ 0 };
            float lastDirectionIntentValue{ (std::numeric_limits<float>::quiet_NaN)() };
            std::uint64_t temporalAuthorityWrites{ 0 };
            std::uint64_t temporalCanonicalTimeWrites{ 0 };
            std::uint64_t hkxAuthoredCadenceCalls{ 0 };
            std::uint64_t hkxNativeRateResultSamples{ 0 };
            // R96: Character::Update's real frame delta cached in the same module that
            // owns hkbClipGenerator::Update. This removes cross-module clock ambiguity.
            float physicalFrameDeltaTime{ 0.0f };

            // R93 stable physical sample selection. These are selector decisions, not
            // Behavior state mirrors. A selected sample persists until semantic gait or
            // steering leaves its hysteresis envelope.
            bool stableSampleInitialized{ false };
            SpeedIntentTranslator::Family stableSampleFamily{ SpeedIntentTranslator::Family::kNone };
            std::int32_t stableDirectionSector{ 0 };
            std::int32_t stablePendingDirectionSector{ -1 };
            std::uint32_t stablePendingDirectionFrames{ 0 };
            float stableAxisSwitchCooldown{ 0.0f };
            std::uint64_t stableSampleFrames{ 0 };
            std::uint64_t stableSampleWrites{ 0 };
            std::uint64_t stableFamilySwitches{ 0 };
            std::uint64_t stableDirectionSwitches{ 0 };
            std::uint64_t stableAxisDeferrals{ 0 };
            std::uint64_t stableFinalReassertCorrections{ 0 };

            // R93 authored-pivot lifecycle. This is deliberately local to the stable
            // sample controller so profiles that do not opt in preserve R71 unchanged.
            StablePivotPhase stablePivotPhase{ StablePivotPhase::kIdle };
            float stablePivotElapsed{ 0.0f };
            float stablePivotCooldown{ 0.0f };
            float stablePivotRequestedErrorDegrees{ 0.0f };
            std::uint32_t stablePivotRouteDegrees{ 0 };
            bool stablePivotLeft{ false };
            std::uint32_t stablePivotCandidateFrames{ 0 };
            std::uint32_t stablePivotStandingFrames{ 0 };
            // R97: 180-degree canned events may reject for a few frames while the
            // standing state machine settles. Keep the exact 180 route latched and
            // retry it briefly instead of falling back into a later 90-degree request.
            std::uint32_t stablePivotDispatchAttempts{ 0 };
            std::uint32_t stablePivotExitFrames{ 0 };
            std::uint64_t stablePivotRequests{ 0 };
            std::uint64_t stablePivotAccepted{ 0 };
            std::uint64_t stablePivotCompleted{ 0 };
            std::uint64_t stablePivotFailures{ 0 };
            std::uint64_t stablePivotTurnParameterWrites{ 0 };

            // R95: native profile-declared RotationClips may exist in the behavior without
            // MotionDB files. Track the actual hkbClipGenerator update as the pivot witness;
            // this is observation only and grants no DB/root authority.
            double stablePivotNativeRotationLastSeenSeconds{ 0.0 };
            std::uint64_t stablePivotNativeRotationSeenFrame{ 0 };
            float stablePivotNativeRotationPoseWeight{ 0.0f };
            std::string stablePivotNativeRotationKey;

            // R102: physical lifecycle for the looping 90-degree turn. The capsule yaw is
            // sampled from bhkCharacterController, so a pose-only turn can never masquerade
            // as successful authored rotation.
            bool stablePivot90PhysicalYawInitialized{ false };
            float stablePivot90PhysicalYawStartRadians{ 0.0f };
            float stablePivot90PhysicalYawMaxDegrees{ 0.0f };
            bool stablePivot90TurnStopIssued{ false };
            bool stablePivot90StallBreak{ false };
            std::uint32_t stablePivot90TurnStopAttempts{ 0 };
            std::uint32_t stablePivot90ReacquireAttempts{ 0 };

            // R108: actor-frame serial is the visual-witness clock. A weighted HKB turn
            // clip is valid only through the immediately following pre-native boundary.
            std::uint64_t stablePivotFrameSerial{ 0 };

            bool nativeRootRecoveryActive{ false };
            bool nativeRootRecoveryAcceptedWait{ false };
            // R49: parent re-entry alone is not recovery. A nested gait event must be
            // accepted before a RuntimeYaw carrier may prove completion.
            bool nativeRootRecoveryNestedGaitAccepted{ false };
            bool nativeRootRecoveryRequiresParentSettle{ false };
            StructuralRecoverySource nativeRootRecoverySource{ StructuralRecoverySource::kNone };
            std::string nativeRootRecoveryEvent;
            std::string nativeRootRecoveryRouteName;
            std::uint32_t nativeRootRecoveryAcceptedWaitFrames{ 0 };
            std::uint32_t nativeRootRecoveryInactiveFrames{ 0 };
            std::uint32_t nativeRootRecoveryActiveStableFrames{ 0 };
            std::uint32_t nativeRootRecoveryCarrierStableFrames{ 0 };
            std::uint32_t nativeRootRecoveryParentEventCooldownFrames{ 0 };
            std::uint32_t nativeRootRecoveryEventRetryFrames{ 0 };
            std::uint32_t nativeRootRecoveryParentEventAttempts{ 0 };
            std::uint64_t nativeRootRecoveriesStarted{ 0 };
            std::uint64_t nativeRootRecoveriesCarrierConfirmed{ 0 };
            std::uint64_t nativeRootRecoveriesExhausted{ 0 };
            std::uint64_t nativeRootRecoveryParentEventsAccepted{ 0 };
            std::uint64_t nativeRootRecoveryParentEventsRejected{ 0 };
            std::uint64_t nativeRootActionRecoveriesStarted{ 0 };
            std::uint64_t nativeRootActionRecoveriesConfirmed{ 0 };
            std::uint64_t locomotionStandingRecoveriesStarted{ 0 };
            std::uint64_t locomotionStandingRecoveriesConfirmed{ 0 };

            // R61: one exhausted R46 Locomotion->Standing witness may be reused exactly
            // once if the semantic gait destination changes afterwards (for example,
            // walkStart -> runStart). This is not a watchdog: it remains provenance-bound
            // to the prior structural witness and cannot repeat after the semantic retry.
            bool nativeRootRecoverySemanticRearmEligible{ false };
            bool nativeRootRecoverySemanticRearmUsed{ false };
            std::string nativeRootRecoveryExhaustedEvent;
            std::uint64_t nativeRootRecoveryExhaustedRevision{ 0 };

            // R48: graph Speed is a gait-family blend input, but Bethesda's
            // bIsMoving/directional flags can pulse false while the parent locomotion
            // state is natively active. Hold the already-latched family goal only in
            // that proven parent-active route; never use Speed to start locomotion.
            bool speedGoalHoldActive{ false };
            std::uint64_t speedGoalHoldChecks{ 0 };
            std::uint64_t speedGoalHoldWrites{ 0 };
            // R50: a RUN speed goal must never time-scale a proven WALK/TROT carrier.
            // Distinct runStart remains responsible for entering the RUN state.
            bool runSpeedHoldMismatchSuppressed{ false };
            std::uint64_t runSpeedHoldMismatchSuppressions{ 0 };
            // R51: R50 blocked only the fallback/post hold. The normal SpeedIntent adapter
            // still ran first in ModifyAnimationUpdateData and could write RUN=833 before
            // Havok evaluated a proven WALK/TROT-only carrier. Track those pre-animation
            // blocks separately so the log can prove the write was prevented at the sink.
            bool runSpeedPreAnimationBlocked{ false };
            std::uint64_t runSpeedPreAnimationBlocks{ 0 };

            // R72: preserve Bethesda/native non-zero transitional Speed instead of
            // reasserting an exact family goal at multiple frame boundaries. The
            // normal pre-Havok SpeedIntent adapter remains the primary semantic writer;
            // R48/R54 are reduced to near-zero stall rescue only. Native root actions
            // bypass every locomotion Speed writer entirely.
            bool r72NativeSpeedPreserveActive{ false };
            std::uint64_t r72NativeSpeedPreserves{ 0 };
            std::uint64_t r72ActionSpeedPreserves{ 0 };

            // R53: R50/R51 correctly refuse to apply the RUN speed goal while the
            // runtime still proves only WALK/TROT carriers. That safety gate can
            // expose a second structural failure: the semantic family remains RUN
            // after an accepted runStart, but the native nested behavior later falls
            // back to lower-family carriers and ping-pongs between them. Reassert the
            // configured distinct RUN event on an actor-frame cadence until a real RUN
            // carrier is observed again. This lane never writes Speed and never starts
            // the parent locomotion state.
            bool runCarrierReassertActive{ false };
            bool runCarrierReassertExhausted{ false };
            std::uint32_t runCarrierMismatchFrames{ 0 };
            std::uint32_t runCarrierStableFrames{ 0 };
            std::uint32_t runCarrierReassertCooldownFrames{ 0 };
            std::uint32_t runCarrierReassertAttempts{ 0 };
            // R62: preserve the R53 mismatch episode across a very short parent-active
            // flicker. No event is sent while the parent is inactive; this only prevents
            // one-frame inactive pulses from erasing the already-proven lower-carrier mismatch.
            std::uint32_t runCarrierParentGapFrames{ 0 };
            std::uint64_t runCarrierActiveEdgeDeferrals{ 0 };
            std::uint64_t runCarrierReassertEpisodes{ 0 };
            std::uint64_t runCarrierReassertAccepted{ 0 };
            std::uint64_t runCarrierReassertRejected{ 0 };
            std::uint64_t runCarrierReassertRecoveries{ 0 };
        };

        struct TurnBlendActorState
        {
            bool seen{ false };
            bool inputReadFailureLogged{ false };
            bool dampedReadFailureLogged{ false };
            std::uint64_t frames{ 0 };
            std::uint64_t writes{ 0 };
            std::uint64_t writeFailures{ 0 };
            std::uint64_t rawUnavailable{ 0 };
            float lastRawDegrees{ 0.0f };
            float lastCommand{ 0.0f };
            float lastDamped{ 0.0f };
            SpeedIntentTranslator::Family lastFamily{ SpeedIntentTranslator::Family::kNone };
        };

        std::unordered_map<std::string, std::shared_ptr<const GaitRouterConfig>> g_gaitRouterConfigs;
        std::unordered_map<std::string, std::shared_ptr<const HkxAuthoredCadenceConfig>> g_hkxAuthoredCadenceConfigs;
        std::unordered_map<std::string, std::shared_ptr<const StableSampleSelectorConfig>> g_stableSampleSelectorConfigs;
        std::unordered_map<RE::FormID, GaitRouterActorState> g_gaitRouterActors;
        std::unordered_map<RE::FormID, TurnBlendActorState> g_turnBlendActors;
        std::mutex g_gaitRouterMutex;

        // R77: generic, profile-local behavior extension router. The DLL knows only
        // primitive trigger types (graph edges and numeric threshold crossings).
        // Creature names, behavior names, variables, event names, and optional guards
        // live entirely in Data/MotionDB/<profile>/Extensions/*.ini.
        enum class BehaviorExtensionTrigger : std::uint8_t
        {
            kInvalid = 0,
            kGraphBoolEdge,
            kGraphIntTransition,
            kGraphFloatCrossUp,
            kGraphFloatCrossDown
        };

        struct BehaviorExtensionRuleConfig
        {
            bool configured{ false };
            bool enabled{ true };
            std::string sourceFile;
            std::string section;
            std::string name;
            BehaviorExtensionTrigger trigger{ BehaviorExtensionTrigger::kInvalid };
            std::string variable;
            std::string event;
            bool fromProvided{ false };
            bool toProvided{ false };
            bool thresholdProvided{ false };
            bool fromBoolValid{ false };
            bool toBoolValid{ false };
            bool fromIntValid{ false };
            bool toIntValid{ false };
            bool fromBool{ false };
            bool toBool{ false };
            std::int32_t fromInt{ 0 };
            std::int32_t toInt{ 0 };
            float threshold{ 0.0f };
            std::string requireBoolVariable;
            bool requireBoolValue{ true };
            float retrySeconds{ 0.08f };
            std::uint32_t maxAttempts{ 4 };
        };

        struct BehaviorExtensionsConfig
        {
            bool loaded{ false };
            std::string profile;
            std::vector<BehaviorExtensionRuleConfig> rules;
        };

        struct BehaviorExtensionRuleState
        {
            bool initialized{ false };
            bool readFailureLogged{ false };
            bool guardReadFailureLogged{ false };
            bool lastBool{ false };
            std::int32_t lastInt{ 0 };
            float lastFloat{ 0.0f };
            std::string pendingEvent;
            float retryRemaining{ 0.0f };
            std::uint32_t attempt{ 0 };
            std::uint64_t epoch{ 0 };
            std::uint64_t triggers{ 0 };
            std::uint64_t attempts{ 0 };
            std::uint64_t accepted{ 0 };
            std::uint64_t rejected{ 0 };
            std::uint64_t exhausted{ 0 };
            std::uint64_t staleCompletions{ 0 };
        };

        struct BehaviorExtensionActorState
        {
            std::string profile;
            std::vector<BehaviorExtensionRuleState> rules;
        };

        std::unordered_map<std::string, std::shared_ptr<const BehaviorExtensionsConfig>> g_behaviorExtensionConfigs;
        std::unordered_map<RE::FormID, BehaviorExtensionActorState> g_behaviorExtensionActors;
        std::mutex g_behaviorExtensionMutex;

        // R41: R40 progress proof retained; unhealthy episodes now survive brief observation gaps.
        // A stuck clip can remain observed at nearly constant local time while a DB session
        // repeatedly enters pending state. Health now requires key-specific authored yaw
        // progress. Track that condition separately and issue at most one turnStop per
        // unhealthy episode, after a short continuous grace window.
        // R46 preserves the R45 parent-event handshake exactly. The only new entry path is a
        // second structural witness produced by the WFM transition observer: LocomotionBehavior
        // -> StandingBehavior. Unlike the R42 native-action exit witness, this witness is consumed
        // only after the transition has aged out of its short blend cluster AND current runtime
        // evidence still says parent inactive + valid steering + desired gait + no action bypass +
        // no RuntimeYaw locomotion carrier. Once armed, both sources use the same bounded moveStart
        // handshake, two-frame native-parent settle, nested gait retry, and two-frame carrier proof.
        // This is deliberately not a generic carrier-loss/inactivity watchdog.
        constexpr std::uint32_t kNativeRootRecoveryInactiveFramesBeforeParentEvent = 2;
        constexpr std::uint32_t kLocomotionStandingRecoveryInactiveFramesBeforeParentEvent = 8;
        constexpr std::uint32_t kNativeRootRecoveryParentSettleFrames = 2;
        constexpr std::uint32_t kNativeRootRecoveryParentEventRetryFrames = 8;
        constexpr std::uint32_t kNativeRootRecoveryEventRetryFrames = 6;
        constexpr std::uint32_t kNativeRootRecoveryAcceptedCarrierWaitFrames = 6;
        constexpr std::uint32_t kNativeRootRecoveryParentEventMaxAttempts = 6;
        constexpr std::uint32_t kNativeRootRecoveryCarrierConfirmFrames = 2;

        // R53 semantic carrier reassert. This is deliberately actor-frame based and
        // exists only for the configured distinct RUN event after a proven lower-family
        // RuntimeYaw carrier mismatch. It is not a generic gait watchdog.
        constexpr std::uint32_t kRunCarrierMismatchConfirmFrames = 3;
        constexpr std::uint32_t kRunCarrierReassertRetryFrames = 6;
        constexpr std::uint32_t kRunCarrierReassertMaxAttempts = 6;
        constexpr std::uint32_t kRunCarrierStableConfirmFrames = 2;
        constexpr std::uint32_t kRunCarrierParentGapHoldFrames = 2;

        constexpr float kTurnLoopUnhealthyBreakSeconds = 0.18f;
        // R41: the wolf behavior can pulse TurnLoop -> StandingIdle -> TurnLoop over
        // one or a few frames while still being stuck in the same logical turn episode.
        // Do not let those tiny observation holes erase the unhealthy timer. A real
        // locomotion carrier resumes immediately resets the episode.
        constexpr float kTurnLoopObservationGapGraceSeconds = 0.10f;
        constexpr float kTurnLoopMinSteeringErrorDegrees = 10.0f;
        constexpr float kRadToDeg = 57.29577951308232f;
        constexpr float kTwoPi = 6.28318530717958647692f;
        constexpr std::uint8_t kRuntimeFamilyWalkMask = 0x01;
        constexpr std::uint8_t kRuntimeFamilyTrotMask = 0x02;
        constexpr std::uint8_t kRuntimeFamilyRunMask = 0x04;

        [[nodiscard]] bool RunSpeedCarrierMismatch(
            RE::Actor* actor,
            std::uint8_t& outCarrierMask) noexcept
        {
            outCarrierMask = 0;
            if (!actor || actor->IsPlayerRef() || !SpeedIntentTranslator::SupportsActor(actor)) {
                return false;
            }

            if (SpeedIntentTranslator::LatchedFamily(actor->GetFormID()) !=
                SpeedIntentTranslator::Family::kRunForward) {
                return false;
            }

            outCarrierMask =
                SyntheticMotionInjector::RuntimeYawLocomotionFamilyMaskForCurrentOrPreviousFrame(actor);
            const bool lowerFamilyCarrier =
                (outCarrierMask & (kRuntimeFamilyWalkMask | kRuntimeFamilyTrotMask)) != 0;
            const bool runCarrier = (outCarrierMask & kRuntimeFamilyRunMask) != 0;
            return lowerFamilyCarrier && !runCarrier;
        }

        struct TurnLoopHealthActorState
        {
            bool lastObserved{ false };
            bool lastHealthy{ false };
            bool unhealthyEpisode{ false };
            bool breakIssued{ false };
            bool observationGapActive{ false };
            float unhealthySeconds{ 0.0f };
            float observationGapSeconds{ 0.0f };
            std::uint64_t episodes{ 0 };
            std::uint64_t breakAttempts{ 0 };
            std::uint64_t breakAccepted{ 0 };
            std::uint64_t recoveries{ 0 };
        };

        std::unordered_map<RE::FormID, TurnLoopHealthActorState> g_turnLoopHealthActors;
        std::mutex g_turnLoopHealthMutex;

        [[nodiscard]] std::string TrimCopy(std::string value)
        {
            const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
            value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
            value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
            return value;
        }

        [[nodiscard]] std::string LowerCopy(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            return value;
        }

        [[nodiscard]] bool ParseBoolValue(const std::string& raw, bool fallback)
        {
            const auto value = LowerCopy(TrimCopy(raw));
            if (value == "true" || value == "1" || value == "yes" || value == "on") return true;
            if (value == "false" || value == "0" || value == "no" || value == "off") return false;
            return fallback;
        }

        [[nodiscard]] bool ParseStrictBoolValue(const std::string& raw, bool& outValue)
        {
            const auto value = LowerCopy(TrimCopy(raw));
            if (value == "true" || value == "1" || value == "yes" || value == "on") {
                outValue = true;
                return true;
            }
            if (value == "false" || value == "0" || value == "no" || value == "off") {
                outValue = false;
                return true;
            }
            return false;
        }

        [[nodiscard]] bool ParseStrictInt32Value(const std::string& raw, std::int32_t& outValue)
        {
            try {
                std::size_t consumed = 0;
                const long parsed = std::stol(TrimCopy(raw), std::addressof(consumed), 10);
                const auto trimmed = TrimCopy(raw);
                if (consumed != trimmed.size()) return false;
                if (parsed < static_cast<long>((std::numeric_limits<std::int32_t>::min)()) ||
                    parsed > static_cast<long>((std::numeric_limits<std::int32_t>::max)())) {
                    return false;
                }
                outValue = static_cast<std::int32_t>(parsed);
                return true;
            } catch (...) {
                return false;
            }
        }

        [[nodiscard]] float ParsePositiveFloat(const std::string& raw, float fallback, float minValue, float maxValue)
        {
            try {
                const float parsed = std::stof(raw);
                if (std::isfinite(parsed)) {
                    return std::clamp(parsed, minValue, maxValue);
                }
            } catch (...) {}
            return fallback;
        }

        [[nodiscard]] const char* BehaviorExtensionTriggerName(BehaviorExtensionTrigger trigger) noexcept
        {
            switch (trigger) {
            case BehaviorExtensionTrigger::kGraphBoolEdge: return "GRAPH_BOOL_EDGE";
            case BehaviorExtensionTrigger::kGraphIntTransition: return "GRAPH_INT_TRANSITION";
            case BehaviorExtensionTrigger::kGraphFloatCrossUp: return "GRAPH_FLOAT_CROSS_UP";
            case BehaviorExtensionTrigger::kGraphFloatCrossDown: return "GRAPH_FLOAT_CROSS_DOWN";
            default: return "INVALID";
            }
        }

        [[nodiscard]] BehaviorExtensionTrigger ParseBehaviorExtensionTrigger(std::string raw)
        {
            raw = LowerCopy(TrimCopy(std::move(raw)));
            raw.erase(std::remove_if(raw.begin(), raw.end(), [](unsigned char c) {
                return std::isspace(c) != 0 || c == '_' || c == '-';
            }), raw.end());
            if (raw == "graphbooledge" || raw == "booledge") return BehaviorExtensionTrigger::kGraphBoolEdge;
            if (raw == "graphinttransition" || raw == "inttransition") return BehaviorExtensionTrigger::kGraphIntTransition;
            if (raw == "graphfloatcrossup" || raw == "floatcrossup") return BehaviorExtensionTrigger::kGraphFloatCrossUp;
            if (raw == "graphfloatcrossdown" || raw == "floatcrossdown") return BehaviorExtensionTrigger::kGraphFloatCrossDown;
            return BehaviorExtensionTrigger::kInvalid;
        }

        [[nodiscard]] bool IsBehaviorExtensionRuleSection(const std::string& section)
        {
            return section.rfind("rule", 0) == 0 || section.rfind("eventroute", 0) == 0;
        }

        [[nodiscard]] BehaviorExtensionsConfig LoadBehaviorExtensionsConfig(const std::string& profile)
        {
            BehaviorExtensionsConfig config{};
            config.loaded = true;
            config.profile = profile;
            if (profile.empty()) return config;

            const std::filesystem::path directory =
                std::filesystem::path("Data") / "MotionDB" / profile / "Extensions";
            std::error_code ec;
            if (!std::filesystem::exists(directory, ec) || !std::filesystem::is_directory(directory, ec)) {
                SKSE::log::info(
                    "[BehaviorExtensions][SCAN] profile=\"{}\" directory=\"{}\" exists=false files=0 rules=0 action=DORMANT",
                    profile, directory.string());
                return config;
            }

            std::vector<std::filesystem::path> files;
            for (std::filesystem::directory_iterator it(directory, ec), end; !ec && it != end; it.increment(ec)) {
                if (!it->is_regular_file()) continue;
                auto ext = LowerCopy(it->path().extension().string());
                if (ext == ".ini") files.push_back(it->path());
            }
            std::sort(files.begin(), files.end(), [](const auto& a, const auto& b) {
                return LowerCopy(a.filename().string()) < LowerCopy(b.filename().string());
            });

            std::size_t enabledFiles = 0;
            for (const auto& path : files) {
                std::ifstream file(path);
                if (!file.is_open()) {
                    SKSE::log::warn(
                        "[BehaviorExtensions][FILE_OPEN_FAIL] profile=\"{}\" path=\"{}\" action=SKIP_FILE",
                        profile, path.string());
                    continue;
                }

                bool extensionEnabled = true;
                std::string section;
                std::vector<BehaviorExtensionRuleConfig> localRules;
                std::unordered_map<std::string, std::size_t> ruleBySection;
                std::string line;
                while (std::getline(file, line)) {
                    line = TrimCopy(line);
                    if (line.empty() || line.front() == ';' || line.front() == '#') continue;
                    if (line.front() == '[' && line.back() == ']') {
                        section = LowerCopy(TrimCopy(line.substr(1, line.size() - 2)));
                        if (IsBehaviorExtensionRuleSection(section) && ruleBySection.find(section) == ruleBySection.end()) {
                            BehaviorExtensionRuleConfig rule{};
                            rule.sourceFile = path.filename().string();
                            rule.section = section;
                            rule.name = section;
                            ruleBySection.emplace(section, localRules.size());
                            localRules.push_back(std::move(rule));
                        }
                        continue;
                    }

                    const auto equals = line.find('=');
                    if (equals == std::string::npos) continue;
                    const auto key = LowerCopy(TrimCopy(line.substr(0, equals)));
                    const auto value = TrimCopy(line.substr(equals + 1));

                    if (section == "extension") {
                        if (key == "enabled") extensionEnabled = ParseBoolValue(value, extensionEnabled);
                        continue;
                    }
                    if (!IsBehaviorExtensionRuleSection(section)) continue;
                    const auto sectionIt = ruleBySection.find(section);
                    if (sectionIt == ruleBySection.end()) continue;
                    auto& rule = localRules[sectionIt->second];

                    if (key == "enabled") {
                        rule.enabled = ParseBoolValue(value, rule.enabled);
                    } else if (key == "name") {
                        rule.name = value;
                    } else if (key == "trigger") {
                        rule.trigger = ParseBehaviorExtensionTrigger(value);
                    } else if (key == "variable") {
                        rule.variable = value;
                    } else if (key == "event") {
                        rule.event = value;
                    } else if (key == "from") {
                        rule.fromProvided = true;
                        rule.fromBoolValid = ParseStrictBoolValue(value, rule.fromBool);
                        rule.fromIntValid = ParseStrictInt32Value(value, rule.fromInt);
                    } else if (key == "to") {
                        rule.toProvided = true;
                        rule.toBoolValid = ParseStrictBoolValue(value, rule.toBool);
                        rule.toIntValid = ParseStrictInt32Value(value, rule.toInt);
                    } else if (key == "threshold") {
                        try {
                            const float parsed = std::stof(value);
                            if (std::isfinite(parsed)) {
                                rule.threshold = parsed;
                                rule.thresholdProvided = true;
                            }
                        } catch (...) {}
                    } else if (key == "requireboolvariable") {
                        rule.requireBoolVariable = value;
                    } else if (key == "requireboolvalue") {
                        rule.requireBoolValue = ParseBoolValue(value, rule.requireBoolValue);
                    } else if (key == "retryseconds") {
                        rule.retrySeconds = ParsePositiveFloat(value, rule.retrySeconds, 0.01f, 2.0f);
                    } else if (key == "maxattempts") {
                        try {
                            const auto parsed = std::stoul(value);
                            rule.maxAttempts = static_cast<std::uint32_t>(std::clamp<unsigned long>(parsed, 1UL, 60UL));
                        } catch (...) {}
                    }
                }

                if (!extensionEnabled) {
                    SKSE::log::info(
                        "[BehaviorExtensions][FILE] profile=\"{}\" file=\"{}\" enabled=false rules=0 action=SKIP_FILE",
                        profile, path.filename().string());
                    continue;
                }
                ++enabledFiles;

                for (auto& rule : localRules) {
                    if (!rule.enabled) {
                        SKSE::log::info(
                            "[BehaviorExtensions][RULE] profile=\"{}\" file=\"{}\" name=\"{}\" enabled=false action=SKIP_RULE",
                            profile, rule.sourceFile, rule.name);
                        continue;
                    }
                    bool valid = rule.trigger != BehaviorExtensionTrigger::kInvalid && !rule.event.empty();
                    if (rule.trigger == BehaviorExtensionTrigger::kGraphBoolEdge) {
                        valid = valid && !rule.variable.empty() && rule.fromProvided && rule.toProvided &&
                            rule.fromBoolValid && rule.toBoolValid;
                    } else if (rule.trigger == BehaviorExtensionTrigger::kGraphIntTransition) {
                        valid = valid && !rule.variable.empty() && rule.fromProvided && rule.toProvided &&
                            rule.fromIntValid && rule.toIntValid;
                    } else if (rule.trigger == BehaviorExtensionTrigger::kGraphFloatCrossUp ||
                        rule.trigger == BehaviorExtensionTrigger::kGraphFloatCrossDown) {
                        valid = valid && !rule.variable.empty() && rule.thresholdProvided;
                    }
                    if (!valid) {
                        SKSE::log::warn(
                            "[BehaviorExtensions][RULE_REJECT] profile=\"{}\" file=\"{}\" section=\"{}\" name=\"{}\" trigger={} variable=\"{}\" event=\"{}\" reason=INVALID_OR_INCOMPLETE action=SKIP_RULE",
                            profile, rule.sourceFile, rule.section, rule.name,
                            BehaviorExtensionTriggerName(rule.trigger), rule.variable, rule.event);
                        continue;
                    }
                    rule.configured = true;
                    SKSE::log::info(
                        "[BehaviorExtensions][RULE] profile=\"{}\" file=\"{}\" name=\"{}\" trigger={} variable=\"{}\" event=\"{}\" guard=\"{}\" guardValue={} retry={:.3f}s maxAttempts={}",
                        profile, rule.sourceFile, rule.name, BehaviorExtensionTriggerName(rule.trigger),
                        rule.variable, rule.event, rule.requireBoolVariable, rule.requireBoolValue,
                        rule.retrySeconds, rule.maxAttempts);
                    config.rules.push_back(std::move(rule));
                }
            }

            SKSE::log::info(
                "[BehaviorExtensions][SCAN] profile=\"{}\" directory=\"{}\" exists=true files={} enabledFiles={} rules={} speciesHardcode=ZERO",
                profile, directory.string(), files.size(), enabledFiles, config.rules.size());
            return config;
        }

        [[nodiscard]] std::shared_ptr<const BehaviorExtensionsConfig> BehaviorExtensionsConfigForProfile(
            const std::string& profile)
        {
            std::scoped_lock lock{ g_behaviorExtensionMutex };
            auto it = g_behaviorExtensionConfigs.find(profile);
            if (it == g_behaviorExtensionConfigs.end()) {
                auto loaded = std::make_shared<const BehaviorExtensionsConfig>(LoadBehaviorExtensionsConfig(profile));
                it = g_behaviorExtensionConfigs.emplace(profile, std::move(loaded)).first;
            }
            return it->second;
        }

        void UpdateBehaviorExtensions(RE::Actor* actor, float deltaTime)
        {
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) return;

            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = BehaviorExtensionsConfigForProfile(profile);
            if (!config || config->rules.empty()) return;

            for (std::size_t i = 0; i < config->rules.size(); ++i) {
                const auto& rule = config->rules[i];
                if (!rule.configured) continue;

                bool sampleRead = true;
                bool boolSample = false;
                std::int32_t intSample = 0;
                float floatSample = 0.0f;
                switch (rule.trigger) {
                case BehaviorExtensionTrigger::kGraphBoolEdge:
                    sampleRead = actor->GetGraphVariableBool(rule.variable.c_str(), boolSample);
                    break;
                case BehaviorExtensionTrigger::kGraphIntTransition:
                    sampleRead = actor->GetGraphVariableInt(rule.variable.c_str(), intSample);
                    break;
                case BehaviorExtensionTrigger::kGraphFloatCrossUp:
                case BehaviorExtensionTrigger::kGraphFloatCrossDown:
                    sampleRead = actor->GetGraphVariableFloat(rule.variable.c_str(), floatSample) && std::isfinite(floatSample);
                    break;
                default:
                    sampleRead = false;
                    break;
                }

                bool guardPass = true;
                bool guardRead = true;
                if (!rule.requireBoolVariable.empty()) {
                    bool guardValue = false;
                    guardRead = actor->GetGraphVariableBool(rule.requireBoolVariable.c_str(), guardValue);
                    guardPass = guardRead && guardValue == rule.requireBoolValue;
                }

                bool triggerNow = false;
                bool logReadFailure = false;
                bool logGuardFailure = false;
                std::string eventToSend;
                std::string ruleName;
                std::uint32_t attemptNumber = 0;
                std::uint64_t epochToSend = 0;

                {
                    std::scoped_lock lock{ g_behaviorExtensionMutex };
                    auto& actorState = g_behaviorExtensionActors[formID];
                    if (actorState.profile != profile || actorState.rules.size() != config->rules.size()) {
                        actorState.profile = profile;
                        actorState.rules.clear();
                        actorState.rules.resize(config->rules.size());
                    }
                    auto& state = actorState.rules[i];

                    if (!sampleRead) {
                        if (!state.readFailureLogged) {
                            state.readFailureLogged = true;
                            logReadFailure = true;
                        }
                        state.initialized = false;
                        state.pendingEvent.clear();
                        state.retryRemaining = 0.0f;
                        state.attempt = 0;
                    } else {
                        state.readFailureLogged = false;
                    }

                    if (!guardRead) {
                        if (!state.guardReadFailureLogged) {
                            state.guardReadFailureLogged = true;
                            logGuardFailure = true;
                        }
                        guardPass = false;
                    } else {
                        state.guardReadFailureLogged = false;
                    }

                    if (sampleRead) {
                        if (!state.initialized) {
                            state.initialized = true;
                            state.lastBool = boolSample;
                            state.lastInt = intSample;
                            state.lastFloat = floatSample;
                        } else {
                            switch (rule.trigger) {
                            case BehaviorExtensionTrigger::kGraphBoolEdge:
                                triggerNow = state.lastBool == rule.fromBool && boolSample == rule.toBool && state.lastBool != boolSample;
                                break;
                            case BehaviorExtensionTrigger::kGraphIntTransition:
                                triggerNow = state.lastInt == rule.fromInt && intSample == rule.toInt && state.lastInt != intSample;
                                break;
                            case BehaviorExtensionTrigger::kGraphFloatCrossUp:
                                triggerNow = state.lastFloat < rule.threshold && floatSample >= rule.threshold;
                                break;
                            case BehaviorExtensionTrigger::kGraphFloatCrossDown:
                                triggerNow = state.lastFloat > rule.threshold && floatSample <= rule.threshold;
                                break;
                            default:
                                break;
                            }
                            triggerNow = triggerNow && guardPass;
                            state.lastBool = boolSample;
                            state.lastInt = intSample;
                            state.lastFloat = floatSample;
                        }
                    }

                    if (triggerNow) {
                        ++state.epoch;
                        ++state.triggers;
                        state.pendingEvent = rule.event;
                        state.retryRemaining = 0.0f;
                        state.attempt = 0;
                    }

                    if (state.retryRemaining > 0.0f) {
                        state.retryRemaining = std::max(0.0f, state.retryRemaining - std::max(deltaTime, 0.0f));
                    }
                    if (!state.pendingEvent.empty() && state.retryRemaining <= 0.0f && state.attempt < rule.maxAttempts) {
                        ++state.attempt;
                        ++state.attempts;
                        eventToSend = state.pendingEvent;
                        ruleName = rule.name;
                        attemptNumber = state.attempt;
                        epochToSend = state.epoch;
                    }
                }

                if (logReadFailure) {
                    SKSE::log::warn(
                        "[BehaviorExtensions][READ_FAIL] form={:08X} profile=\"{}\" rule=\"{}\" trigger={} variable=\"{}\" action=FAIL_CLOSED",
                        formID, profile, rule.name, BehaviorExtensionTriggerName(rule.trigger), rule.variable);
                }
                if (logGuardFailure) {
                    SKSE::log::warn(
                        "[BehaviorExtensions][GUARD_READ_FAIL] form={:08X} profile=\"{}\" rule=\"{}\" variable=\"{}\" action=FAIL_CLOSED",
                        formID, profile, rule.name, rule.requireBoolVariable);
                }
                if (eventToSend.empty()) continue;

                const bool accepted = actor->NotifyAnimationGraph(eventToSend.c_str());
                bool exhausted = false;
                bool stale = false;
                {
                    std::scoped_lock lock{ g_behaviorExtensionMutex };
                    auto actorIt = g_behaviorExtensionActors.find(formID);
                    if (actorIt == g_behaviorExtensionActors.end() || i >= actorIt->second.rules.size()) {
                        stale = true;
                    } else {
                        auto& state = actorIt->second.rules[i];
                        if (state.epoch != epochToSend || state.pendingEvent != eventToSend) {
                            ++state.staleCompletions;
                            stale = true;
                        } else if (accepted) {
                            ++state.accepted;
                            state.pendingEvent.clear();
                            state.retryRemaining = 0.0f;
                            state.attempt = 0;
                        } else {
                            ++state.rejected;
                            if (state.attempt >= rule.maxAttempts) {
                                ++state.exhausted;
                                state.pendingEvent.clear();
                                state.retryRemaining = 0.0f;
                                state.attempt = 0;
                                exhausted = true;
                            } else {
                                state.retryRemaining = rule.retrySeconds;
                            }
                        }
                    }
                }

                SKSE::log::info(
                    "[BehaviorExtensions][EVENT] form={:08X} profile=\"{}\" rule=\"{}\" epoch={} trigger={} event=\"{}\" attempt={}/{} accepted={} exhausted={} stale={} speciesHardcode=ZERO",
                    formID, profile, ruleName, epochToSend, BehaviorExtensionTriggerName(rule.trigger),
                    eventToSend, attemptNumber, rule.maxAttempts, accepted, exhausted, stale);
            }
        }

        void ClearBehaviorExtensions()
        {
            std::scoped_lock lock{ g_behaviorExtensionMutex };
            for (const auto& [formID, actorState] : g_behaviorExtensionActors) {
                std::uint64_t triggers = 0;
                std::uint64_t attempts = 0;
                std::uint64_t accepted = 0;
                std::uint64_t rejected = 0;
                std::uint64_t exhausted = 0;
                std::uint64_t stale = 0;
                for (const auto& state : actorState.rules) {
                    triggers += state.triggers;
                    attempts += state.attempts;
                    accepted += state.accepted;
                    rejected += state.rejected;
                    exhausted += state.exhausted;
                    stale += state.staleCompletions;
                }
                SKSE::log::info(
                    "[BehaviorExtensions][SUMMARY] form={:08X} profile=\"{}\" rules={} triggers={} attempts={} accepted={} rejected={} exhausted={} stale={}",
                    formID, actorState.profile, actorState.rules.size(), triggers, attempts, accepted, rejected, exhausted, stale);
            }
            g_behaviorExtensionActors.clear();
            g_behaviorExtensionConfigs.clear();
        }

        [[nodiscard]] GaitRouterConfig LoadGaitRouterConfig(const std::string& profile)
        {
            GaitRouterConfig config{};
            config.loaded = true;
            config.profile = profile;

            if (profile.empty()) {
                return config;
            }

            const std::filesystem::path path =
                std::filesystem::path("Data") / "MotionDB" / profile / "GaitRouter.ini";
            std::ifstream file(path);
            if (!file.is_open()) {
                SKSE::log::info(
                    "[GaitRouter][CONFIG_MISSING] profile=\"{}\" path=\"{}\" action=DISABLED_FOR_PROFILE",
                    profile, path.string());
                return config;
            }

            std::string section;
            std::string line;
            while (std::getline(file, line)) {
                line = TrimCopy(line);
                if (line.empty() || line.front() == ';' || line.front() == '#') {
                    continue;
                }

                if (line.front() == '[' && line.back() == ']') {
                    section = LowerCopy(TrimCopy(line.substr(1, line.size() - 2)));
                    continue;
                }

                const auto equals = line.find('=');
                if (equals == std::string::npos) {
                    continue;
                }

                const auto key = LowerCopy(TrimCopy(line.substr(0, equals)));
                const auto value = TrimCopy(line.substr(equals + 1));

                if (section == "gaitrouter") {
                    if (key == "enabled") {
                        config.enabled = ParseBoolValue(value, config.enabled);
                    } else if (key == "routevariable") {
                        config.routeVariable = value;
                    } else if (key == "activevariable") {
                        config.activeVariable = value;
                    } else if (key == "activevalue") {
                        try {
                            config.activeValue = static_cast<std::int32_t>(std::stol(value));
                        } catch (...) {}
                    } else if (key == "parentstartevent") {
                        config.parentStartEvent = value;
                    } else if (key == "parentonly") {
                        config.parentOnly = ParseBoolValue(value, config.parentOnly);
                    } else if (key == "parentintentsource") {
                        const auto source = LowerCopy(value);
                        if (source == "steering") {
                            config.parentIntentSource = ParentIntentSource::kSteering;
                        } else if (source == "graphfloat" || source == "graph_float") {
                            config.parentIntentSource = ParentIntentSource::kGraphFloat;
                        } else {
                            SKSE::log::warn(
                                "[GaitRouter][R83_PARENT_INTENT_SOURCE_REJECT] profile=\"{}\" value=\"{}\" action=FALLBACK_GRAPH_FLOAT",
                                profile, value);
                            config.parentIntentSource = ParentIntentSource::kGraphFloat;
                        }
                    } else if (key == "parentintentvariable") {
                        config.parentIntentVariable = value;
                    } else if (key == "parentintentthreshold") {
                        config.parentIntentThreshold = ParsePositiveFloat(
                            value, config.parentIntentThreshold, 0.0f, 100000.0f);
                    } else if (key == "parentintentconfirmframes") {
                        try {
                            const auto parsed = std::stoul(value);
                            config.parentIntentConfirmFrames = static_cast<std::uint32_t>(
                                std::clamp<unsigned long>(parsed, 1UL, 60UL));
                        } catch (...) {}
                    } else if (key == "parentpersistentwhileintent") {
                        config.parentPersistentWhileIntent = ParseBoolValue(
                            value, config.parentPersistentWhileIntent);
                    } else if (key == "reassertsamesemanticonactiveedge") {
                        config.reassertSameSemanticOnActiveEdge = ParseBoolValue(
                            value, config.reassertSameSemanticOnActiveEdge);
                    } else if (key == "retryseconds") {
                        try {
                            config.retrySeconds = std::clamp(std::stof(value), 0.01f, 1.0f);
                        } catch (...) {}
                    } else if (key == "maxattempts") {
                        try {
                            const auto parsed = std::stoul(value);
                            config.maxAttempts = static_cast<std::uint32_t>(std::clamp<unsigned long>(parsed, 1UL, 60UL));
                        } catch (...) {}
                    }
                    continue;
                }

                if (section == "directionintent") {
                    auto& direction = config.directionIntent;
                    if (key == "enabled") {
                        direction.enabled = ParseBoolValue(value, direction.enabled);
                    } else if (key == "variable") {
                        direction.variable = value;
                    } else if (key == "cyclemin") {
                        try { direction.cycleMin = std::stof(value); } catch (...) {}
                    } else if (key == "cyclemax") {
                        try { direction.cycleMax = std::stof(value); } catch (...) {}
                    } else if (key == "forwardvalue") {
                        try { direction.forwardValue = std::stof(value); } catch (...) {}
                    } else if (key == "positiveyawincreases") {
                        direction.positiveYawIncreases = ParseBoolValue(value, direction.positiveYawIncreases);
                    } else if (key == "telemetryeveryframes") {
                        try {
                            const auto parsed = std::stoul(value);
                            direction.telemetryEveryFrames = static_cast<std::uint32_t>(
                                std::clamp<unsigned long>(parsed, 1UL, 600UL));
                        } catch (...) {}
                    }
                    continue;
                }

                if (section == "temporalauthority") {
                    auto& temporal = config.temporalAuthority;
                    if (key == "enabled") {
                        temporal.enabled = ParseBoolValue(value, temporal.enabled);
                    } else if (key == "playbackrate") {
                        temporal.playbackRate = ParsePositiveFloat(value, temporal.playbackRate, 0.01f, 4.0f);
                    } else if (key.rfind("asset", 0) == 0 && !value.empty()) {
                        if (temporal.assetCount < temporal.assets.size()) {
                            temporal.assets[temporal.assetCount++] = LowerCopy(value);
                        }
                    } else if (key.rfind("duration", 0) == 0) {
                        try {
                            const auto indexText = key.substr(std::string{"duration"}.size());
                            const auto index = static_cast<std::size_t>(std::stoul(indexText));
                            if (index < temporal.durations.size()) {
                                temporal.durations[index] = ParsePositiveFloat(
                                    value, temporal.durations[index], 0.01f, 60.0f);
                            }
                        } catch (...) {}
                    } else if (key == "telemetryeveryframes") {
                        try {
                            const auto parsed = std::stoul(value);
                            temporal.telemetryEveryFrames = static_cast<std::uint32_t>(
                                std::clamp<unsigned long>(parsed, 1UL, 600UL));
                        } catch (...) {}
                    }
                    continue;
                }

                if (section == "turnblend") {
                    auto& turn = config.turnBlend;
                    if (key == "enabled") {
                        turn.enabled = ParseBoolValue(value, turn.enabled);
                    } else if (key == "routevalue") {
                        try { turn.routeValue = static_cast<std::int32_t>(std::stol(value)); } catch (...) {}
                    } else if (key == "inputvariable") {
                        turn.inputVariable = value;
                    } else if (key == "dampedvariable") {
                        turn.dampedVariable = value;
                    } else if (key == "deadzonedegrees") {
                        turn.deadZoneDegrees = ParsePositiveFloat(value, turn.deadZoneDegrees, 0.0f, 30.0f);
                    } else if (key == "walkfullerrordegrees") {
                        turn.walkFullErrorDegrees = ParsePositiveFloat(value, turn.walkFullErrorDegrees, 1.0f, 180.0f);
                    } else if (key == "trotfullerrordegrees") {
                        turn.trotFullErrorDegrees = ParsePositiveFloat(value, turn.trotFullErrorDegrees, 1.0f, 180.0f);
                    } else if (key == "runfullerrordegrees") {
                        turn.runFullErrorDegrees = ParsePositiveFloat(value, turn.runFullErrorDegrees, 1.0f, 180.0f);
                    } else if (key == "walkfullblend") {
                        turn.walkFullBlend = ParsePositiveFloat(value, turn.walkFullBlend, 1.0f, 720.0f);
                    } else if (key == "trotfullblend") {
                        turn.trotFullBlend = ParsePositiveFloat(value, turn.trotFullBlend, 1.0f, 720.0f);
                    } else if (key == "runfullblend") {
                        turn.runFullBlend = ParsePositiveFloat(value, turn.runFullBlend, 1.0f, 720.0f);
                    } else if (key == "telemetryeveryframes") {
                        try {
                            const auto parsed = std::stoul(value);
                            turn.telemetryEveryFrames = static_cast<std::uint32_t>(std::clamp<unsigned long>(parsed, 1UL, 600UL));
                        } catch (...) {}
                    }
                    continue;
                }

                if (section.rfind("route", 0) != 0 || section.size() <= 5) {
                    continue;
                }

                std::size_t routeIndex = 0;
                try {
                    routeIndex = static_cast<std::size_t>(std::stoul(section.substr(5)));
                } catch (...) {
                    continue;
                }
                if (routeIndex >= config.routes.size()) {
                    continue;
                }

                auto& route = config.routes[routeIndex];
                if (key == "value") {
                    try {
                        route.value = static_cast<std::int32_t>(std::stol(value));
                        route.configured = true;
                    } catch (...) {}
                } else if (key == "name") {
                    route.name = value;
                } else if (key == "walkevent") {
                    route.walkEvent = value;
                } else if (key == "trotevent") {
                    route.trotEvent = value;
                } else if (key == "runevent") {
                    route.runEvent = value;
                }
            }

            // R78: animation cadence is profile data, not species logic. The optional
            // LocomotionPolicy.ini keeps existing GaitRouter.ini topology untouched.
            // Missing or invalid policy falls back to the exact R72 NativeContinuity behavior.
            const std::filesystem::path locomotionPolicyPath =
                std::filesystem::path("Data") / "MotionDB" / profile / "LocomotionPolicy.ini";
            if (std::ifstream policyFile{ locomotionPolicyPath }; policyFile.is_open()) {
                std::string policySection;
                std::string policyLine;
                while (std::getline(policyFile, policyLine)) {
                    policyLine = TrimCopy(policyLine);
                    if (policyLine.empty() || policyLine.front() == ';' || policyLine.front() == '#') {
                        continue;
                    }
                    if (policyLine.front() == '[' && policyLine.back() == ']') {
                        policySection = LowerCopy(TrimCopy(policyLine.substr(1, policyLine.size() - 2)));
                        continue;
                    }
                    const auto policyEquals = policyLine.find('=');
                    if (policyEquals == std::string::npos || policySection != "speed") {
                        continue;
                    }
                    const auto policyKey = LowerCopy(TrimCopy(policyLine.substr(0, policyEquals)));
                    const auto policyValue = LowerCopy(TrimCopy(policyLine.substr(policyEquals + 1)));
                    if (policyKey != "cadencepolicy") {
                        continue;
                    }
                    if (policyValue == "stablefamilygoal" || policyValue == "stable_family_goal") {
                        config.speedCadencePolicy = SpeedCadencePolicy::kStableFamilyGoal;
                    } else if (policyValue == "nativecontinuity" || policyValue == "native_continuity") {
                        config.speedCadencePolicy = SpeedCadencePolicy::kNativeContinuity;
                    } else {
                        SKSE::log::warn(
                            "[LocomotionPolicy][REJECT] profile=\"{}\" path=\"{}\" key=CadencePolicy value=\"{}\" action=FALLBACK_NATIVE_CONTINUITY",
                            profile, locomotionPolicyPath.string(), policyValue);
                        config.speedCadencePolicy = SpeedCadencePolicy::kNativeContinuity;
                    }
                }
                SKSE::log::info(
                    "[LocomotionPolicy][SPEED] profile=\"{}\" path=\"{}\" cadence={} speciesHardcode=ZERO",
                    profile, locomotionPolicyPath.string(), SpeedCadencePolicyName(config.speedCadencePolicy));
            } else {
                SKSE::log::info(
                    "[LocomotionPolicy][SPEED] profile=\"{}\" path=\"{}\" cadence=NATIVE_CONTINUITY source=DEFAULT_MISSING_FILE",
                    profile, locomotionPolicyPath.string());
            }

            std::size_t configuredRoutes = 0;
            for (const auto& route : config.routes) {
                if (route.configured) ++configuredRoutes;
            }

            // R83 parent-only mode may source motion intent either from a legacy graph
            // float or directly from the upstream SteeringIntentController. Steering is
            // intentionally pre-behavior so a Standing graph cannot deadlock by waiting on
            // its own Speed variable before receiving moveStart.
            if (config.parentOnly) {
                const bool graphIntentIncomplete =
                    config.parentIntentSource == ParentIntentSource::kGraphFloat &&
                    config.parentIntentVariable.empty();
                if (config.parentStartEvent.empty() || graphIntentIncomplete) {
                    config.enabled = false;
                    SKSE::log::warn(
                        "[GaitRouter][R83_PARENT_ONLY_CONFIG_REJECT] profile=\"{}\" parentStartEvent=\"{}\" intentSource={} intentVariable=\"{}\" reason=INCOMPLETE action=DISABLE",
                        profile, config.parentStartEvent, ParentIntentSourceName(config.parentIntentSource),
                        config.parentIntentVariable);
                }
            } else if (configuredRoutes == 0) {
                config.enabled = false;
            }

            if (config.turnBlend.enabled && config.turnBlend.inputVariable.empty()) {
                config.turnBlend.enabled = false;
                SKSE::log::warn(
                    "[GaitRouter][TURN_CONFIG_REJECT] profile=\"{}\" reason=EMPTY_INPUT_VARIABLE action=TURN_BLEND_DISABLED",
                    profile);
            }

            if (config.directionIntent.enabled) {
                const float span = config.directionIntent.cycleMax - config.directionIntent.cycleMin;
                if (config.directionIntent.variable.empty() ||
                    !std::isfinite(config.directionIntent.cycleMin) ||
                    !std::isfinite(config.directionIntent.cycleMax) ||
                    !std::isfinite(config.directionIntent.forwardValue) ||
                    span <= 1.0e-5f) {
                    config.directionIntent.enabled = false;
                    SKSE::log::warn(
                        "[GaitRouter][R83_DIRECTION_CONFIG_REJECT] profile=\"{}\" variable=\"{}\" cycle=[{:.3f},{:.3f}) forward={:.3f} action=DIRECTION_INTENT_DISABLED",
                        profile, config.directionIntent.variable, config.directionIntent.cycleMin,
                        config.directionIntent.cycleMax, config.directionIntent.forwardValue);
                }
            }

            if (config.temporalAuthority.enabled && config.temporalAuthority.assetCount == 0) {
                config.temporalAuthority.enabled = false;
                SKSE::log::warn(
                    "[GaitRouter][R83_TEMPORAL_CONFIG_REJECT] profile=\"{}\" reason=NO_ASSETS action=TEMPORAL_AUTHORITY_DISABLED",
                    profile);
            }

            SKSE::log::info(
                "[GaitRouter][CONFIG] profile=\"{}\" path=\"{}\" enabled={} parentOnly={} activeVariable=\"{}\" activeValue={} parentStartEvent=\"{}\" parentIntentSource={} parentIntentVariable=\"{}\" parentIntentThreshold={:.3f} parentIntentConfirmFrames={} parentPersistentWhileIntent={} reassertSameSemanticOnActiveEdge={} routeVariable=\"{}\" routes={} retry={:.3f}s maxAttempts={}",
                profile, path.string(), config.enabled, config.parentOnly,
                config.activeVariable, config.activeValue, config.parentStartEvent,
                ParentIntentSourceName(config.parentIntentSource), config.parentIntentVariable,
                config.parentIntentThreshold, config.parentIntentConfirmFrames,
                config.parentPersistentWhileIntent, config.reassertSameSemanticOnActiveEdge,
                config.routeVariable, configuredRoutes,
                config.retrySeconds, config.maxAttempts);

            SKSE::log::info(
                "[GaitRouter][R83_DIRECTION_CONFIG] profile=\"{}\" enabled={} variable=\"{}\" cycle=[{:.3f},{:.3f}) forward={:.3f} positiveYawIncreases={} telemetryEvery={} source=STEERING_FILTERED_ERROR",
                profile, config.directionIntent.enabled, config.directionIntent.variable,
                config.directionIntent.cycleMin, config.directionIntent.cycleMax,
                config.directionIntent.forwardValue, config.directionIntent.positiveYawIncreases,
                config.directionIntent.telemetryEveryFrames);

            SKSE::log::info(
                "[GaitRouter][R83_TEMPORAL_CONFIG] profile=\"{}\" enabled={} playbackRate={:.3f} assets={} phase=CLIP_UPDATE_PRE_AND_POST_NATIVE scope=PROFILE_DECLARED_ASSETS_ONLY",
                profile, config.temporalAuthority.enabled, config.temporalAuthority.playbackRate,
                config.temporalAuthority.assetCount);

            SKSE::log::info(
                "[GaitRouter][TURN_CONFIG] profile=\"{}\" enabled={} routeValue={} input=\"{}\" damped=\"{}\" deadZone={:.2f} fullError[W={:.2f} T={:.2f} R={:.2f}] fullBlend[W={:.2f} T={:.2f} R={:.2f}] telemetryEvery={}",
                profile, config.turnBlend.enabled, config.turnBlend.routeValue,
                config.turnBlend.inputVariable, config.turnBlend.dampedVariable,
                config.turnBlend.deadZoneDegrees, config.turnBlend.walkFullErrorDegrees,
                config.turnBlend.trotFullErrorDegrees, config.turnBlend.runFullErrorDegrees,
                config.turnBlend.walkFullBlend, config.turnBlend.trotFullBlend, config.turnBlend.runFullBlend,
                config.turnBlend.telemetryEveryFrames);

            for (std::size_t i = 0; i < config.routes.size(); ++i) {
                const auto& route = config.routes[i];
                if (!route.configured) continue;
                SKSE::log::info(
                    "[GaitRouter][ROUTE] profile=\"{}\" index={} name=\"{}\" value={} events[WALK=\"{}\" TROT=\"{}\" RUN=\"{}\"]",
                    profile, i, route.name, route.value,
                    route.walkEvent, route.trotEvent, route.runEvent);
            }

            return config;
        }

        [[nodiscard]] std::shared_ptr<const GaitRouterConfig> GaitRouterConfigForProfile(const std::string& profile)
        {
            std::scoped_lock lock{ g_gaitRouterMutex };
            auto it = g_gaitRouterConfigs.find(profile);
            if (it == g_gaitRouterConfigs.end()) {
                auto loaded = std::make_shared<const GaitRouterConfig>(LoadGaitRouterConfig(profile));
                it = g_gaitRouterConfigs.emplace(profile, std::move(loaded)).first;
            }
            return it->second;
        }

        [[nodiscard]] HkxAuthoredCadenceConfig LoadHkxAuthoredCadenceConfig(
            const std::string& profile)
        {
            HkxAuthoredCadenceConfig config{};
            if (profile.empty()) {
                return config;
            }

            const std::filesystem::path path =
                std::filesystem::path("Data") / "MotionDB" / profile / "LocomotionPolicy.ini";
            std::ifstream file(path);
            if (!file.is_open()) {
                return config;
            }

            std::string section;
            std::string line;
            while (std::getline(file, line)) {
                line = TrimCopy(line);
                if (line.empty() || line.front() == ';' || line.front() == '#') {
                    continue;
                }
                if (line.front() == '[' && line.back() == ']') {
                    section = LowerCopy(TrimCopy(line.substr(1, line.size() - 2)));
                    continue;
                }
                if (section != "playback") {
                    continue;
                }
                const auto equals = line.find('=');
                if (equals == std::string::npos) {
                    continue;
                }
                const auto key = LowerCopy(TrimCopy(line.substr(0, equals)));
                const auto value = TrimCopy(line.substr(equals + 1));
                if (key == "enabled") {
                    config.enabled = ParseBoolValue(value, config.enabled);
                } else if (key == "includeruntimeyawclips") {
                    config.includeRuntimeYawClips = ParseBoolValue(value, config.includeRuntimeYawClips);
                } else if (key == "includerotationclips") {
                    config.includeRotationClips = ParseBoolValue(value, config.includeRotationClips);
                } else if (key == "rotationrequiresstablepivot") {
                    config.rotationRequiresStablePivot = ParseBoolValue(value, config.rotationRequiresStablePivot);
                } else if (key == "rotationrequiresnoruntimecarrier") {
                    config.rotationRequiresNoRuntimeCarrier = ParseBoolValue(
                        value, config.rotationRequiresNoRuntimeCarrier);
                } else if (key == "nativerateoverridesexclusive") {
                    config.nativeRateOverridesExclusive = ParseBoolValue(
                        value, config.nativeRateOverridesExclusive);
                } else if (key == "nativerateoverrides") {
                    config.nativeRateOverrideCount = 0;
                    std::size_t start = 0;
                    while (start <= value.size() &&
                           config.nativeRateOverrideCount < config.nativeRateOverrideNodes.size()) {
                        const auto end = value.find_first_of(",;", start);
                        const auto entry = TrimCopy(value.substr(
                            start, end == std::string::npos ? std::string::npos : end - start));
                        const auto colon = entry.rfind(':');
                        if (colon != std::string::npos) {
                            auto node = LowerCopy(TrimCopy(entry.substr(0, colon)));
                            const auto rateText = TrimCopy(entry.substr(colon + 1));
                            try {
                                const float rate = std::stof(rateText);
                                if (!node.empty() && std::isfinite(rate) && rate > 0.001f && rate <= 2.0f) {
                                    const auto index = config.nativeRateOverrideCount++;
                                    config.nativeRateOverrideNodes[index] = std::move(node);
                                    config.nativeRateOverrideRates[index] = rate;
                                }
                            } catch (...) {}
                        }
                        if (end == std::string::npos) {
                            break;
                        }
                        start = end + 1;
                    }
                } else if (key == "nativestandingturnrateoverrides") {
                    config.nativeStandingTurnRateOverrideCount = 0;
                    std::size_t start = 0;
                    while (start <= value.size() &&
                           config.nativeStandingTurnRateOverrideCount < config.nativeStandingTurnRateOverrideNodes.size()) {
                        const auto end = value.find_first_of(",;", start);
                        const auto entry = TrimCopy(value.substr(
                            start, end == std::string::npos ? std::string::npos : end - start));
                        const auto colon = entry.rfind(':');
                        if (colon != std::string::npos) {
                            auto node = LowerCopy(TrimCopy(entry.substr(0, colon)));
                            const auto rateText = TrimCopy(entry.substr(colon + 1));
                            try {
                                const float rate = std::stof(rateText);
                                if (!node.empty() && std::isfinite(rate) && rate > 0.001f && rate <= 2.0f) {
                                    const auto index = config.nativeStandingTurnRateOverrideCount++;
                                    config.nativeStandingTurnRateOverrideNodes[index] = std::move(node);
                                    config.nativeStandingTurnRateOverrideRates[index] = rate;
                                }
                            } catch (...) {}
                        }
                        if (end == std::string::npos) break;
                        start = end + 1;
                    }
                } else if (key == "authoredrate") {
                    config.authoredRate = ParsePositiveFloat(value, config.authoredRate, 0.05f, 2.0f);
                } else if (key == "minactiverate") {
                    config.minActiveRate = ParsePositiveFloat(value, config.minActiveRate, 0.001f, 0.25f);
                } else if (key == "minposeweight") {
                    config.minPoseWeight = ParsePositiveFloat(value, config.minPoseWeight, 0.00001f, 0.25f);
                } else if (key == "telemetryeveryframes") {
                    try {
                        const auto parsed = std::stoul(value);
                        config.telemetryEveryFrames = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(parsed, 1UL, 1200UL));
                    } catch (...) {}
                }
            }

            config.minActiveRate = std::clamp(config.minActiveRate, 0.001f, 0.25f);
            config.minPoseWeight = std::clamp(config.minPoseWeight, 0.00001f, 0.25f);
            if (config.enabled) {
                SKSE::log::info(
                    "[LocomotionPolicy][R114_NATIVE_RATE_CLOCK] profile=\"{}\" path=\"{}\" authoredRate={:.3f} scope[runtimeYaw={} rotation={} rotationRequiresStablePivot={} rotationRequiresNoRuntimeCarrier={}] nativeRateOverrides[count={} exclusive={}] nativeStandingTurnOverrides[count={}] minActiveRate={:.3f} minPoseWeight={:.5f} mechanism=PROFILE_NODE_RATE_RECIPROCAL_TIMESTEP childSyncTimestep=IGNORED lifecycle=NATIVE_HAVOK poseRoot=SAME_NATIVE_UPDATE speciesHardcode=ZERO behaviorHardcode=ZERO",
                    profile, path.string(), config.authoredRate,
                    config.includeRuntimeYawClips, config.includeRotationClips,
                    config.rotationRequiresStablePivot, config.rotationRequiresNoRuntimeCarrier,
                    config.nativeRateOverrideCount, config.nativeRateOverridesExclusive,
                    config.nativeStandingTurnRateOverrideCount,
                    config.minActiveRate, config.minPoseWeight);
            }
            return config;
        }

        [[nodiscard]] std::shared_ptr<const HkxAuthoredCadenceConfig>
        HkxAuthoredCadenceConfigForProfile(const std::string& profile)
        {
            std::scoped_lock lock{ g_gaitRouterMutex };
            auto it = g_hkxAuthoredCadenceConfigs.find(profile);
            if (it == g_hkxAuthoredCadenceConfigs.end()) {
                auto loaded = std::make_shared<const HkxAuthoredCadenceConfig>(
                    LoadHkxAuthoredCadenceConfig(profile));
                it = g_hkxAuthoredCadenceConfigs.emplace(profile, std::move(loaded)).first;
            }
            return it->second;
        }

        [[nodiscard]] StableSampleSelectorConfig LoadStableSampleSelectorConfig(
            const std::string& profile)
        {
            StableSampleSelectorConfig config{};
            if (profile.empty()) {
                return config;
            }

            const std::filesystem::path path =
                std::filesystem::path("Data") / "MotionDB" / profile / "LocomotionPolicy.ini";
            std::ifstream file(path);
            if (!file.is_open()) {
                return config;
            }

            std::string section;
            std::string line;
            while (std::getline(file, line)) {
                line = TrimCopy(line);
                if (line.empty() || line.front() == ';' || line.front() == '#') continue;
                if (line.front() == '[' && line.back() == ']') {
                    section = LowerCopy(TrimCopy(line.substr(1, line.size() - 2)));
                    continue;
                }
                if (section != "stablesampleselector") continue;
                const auto equals = line.find('=');
                if (equals == std::string::npos) continue;
                const auto key = LowerCopy(TrimCopy(line.substr(0, equals)));
                const auto value = TrimCopy(line.substr(equals + 1));

                if (key == "enabled") {
                    config.enabled = ParseBoolValue(value, config.enabled);
                } else if (key == "speedvariable") {
                    config.speedVariable = value;
                } else if (key == "directionvariable") {
                    config.directionVariable = value;
                } else if (key == "walksample") {
                    config.walkSample = ParsePositiveFloat(value, config.walkSample, -10000.0f, 10000.0f);
                } else if (key == "trotsample") {
                    config.trotSample = ParsePositiveFloat(value, config.trotSample, -10000.0f, 10000.0f);
                } else if (key == "runsample") {
                    config.runSample = ParsePositiveFloat(value, config.runSample, -10000.0f, 10000.0f);
                } else if (key == "cyclemin") {
                    config.cycleMin = ParsePositiveFloat(value, config.cycleMin, -100.0f, 100.0f);
                } else if (key == "cyclemax") {
                    config.cycleMax = ParsePositiveFloat(value, config.cycleMax, -100.0f, 100.0f);
                } else if (key == "forwardvalue") {
                    config.forwardValue = ParsePositiveFloat(value, config.forwardValue, -100.0f, 100.0f);
                } else if (key == "positiveyawincreases") {
                    config.positiveYawIncreases = ParseBoolValue(value, config.positiveYawIncreases);
                } else if (key == "directionsectors") {
                    try {
                        config.directionSectors = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(std::stoul(value), 1UL, 16UL));
                    } catch (...) {}
                } else if (key == "directionswitchhysteresisdegrees") {
                    config.directionSwitchHysteresisDegrees = ParsePositiveFloat(
                        value, config.directionSwitchHysteresisDegrees, 0.0f, 40.0f);
                } else if (key == "directionconfirmframes") {
                    try {
                        config.directionConfirmFrames = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(std::stoul(value), 1UL, 60UL));
                    } catch (...) {}
                } else if (key == "axisswitchcooldownseconds") {
                    config.axisSwitchCooldownSeconds = ParsePositiveFloat(
                        value, config.axisSwitchCooldownSeconds, 0.0f, 2.0f);
                } else if (key == "mingoaldistance") {
                    config.minGoalDistance = ParsePositiveFloat(value, config.minGoalDistance, 0.0f, 10000.0f);
                } else if (key == "authoredpivotenabled") {
                    config.authoredPivotEnabled = ParseBoolValue(value, config.authoredPivotEnabled);
                } else if (key == "authoredpivot90enabled") {
                    config.authoredPivot90Enabled = ParseBoolValue(value, config.authoredPivot90Enabled);
                } else if (key == "authoredpivotconfirmframes") {
                    try {
                        config.authoredPivotConfirmFrames = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(std::stoul(value), 1UL, 60UL));
                    } catch (...) {}
                } else if (key == "authoredpivotrequiregraphinactive") {
                    config.authoredPivotRequireGraphInactive = ParseBoolValue(
                        value, config.authoredPivotRequireGraphInactive);
                } else if (key == "authoredpivot180routeminangle") {
                    config.authoredPivot180RouteMinAngle = ParsePositiveFloat(
                        value, config.authoredPivot180RouteMinAngle, 0.0f, 180.0f);
                } else if (key == "authoredpivot180preemptlocomotion") {
                    config.authoredPivot180PreemptLocomotion = ParseBoolValue(
                        value, config.authoredPivot180PreemptLocomotion);
                } else if (key == "nativestandingturnpreemptlocomotion") {
                    config.nativeStandingTurnPreemptLocomotion = ParseBoolValue(
                        value, config.nativeStandingTurnPreemptLocomotion);
                } else if (key == "nativestandingturnmingoaldistance") {
                    config.nativeStandingTurnMinGoalDistance = ParsePositiveFloat(
                        value, config.nativeStandingTurnMinGoalDistance, 0.0f, 10000.0f);
                } else if (key == "nativestandingturncommitangledegrees") {
                    config.nativeStandingTurnCommitAngleDegrees = ParsePositiveFloat(
                        value, config.nativeStandingTurnCommitAngleDegrees, 0.0f, 135.0f);
                } else if (key == "nativestandingturnreleaseangledegrees") {
                    config.nativeStandingTurnReleaseAngleDegrees = ParsePositiveFloat(
                        value, config.nativeStandingTurnReleaseAngleDegrees, 5.0f, 89.0f);
                } else if (key == "nativestandingturnreleaseconfirmframes") {
                    try {
                        config.nativeStandingTurnReleaseConfirmFrames = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(std::stoul(value), 1UL, 60UL));
                    } catch (...) {}
                } else if (key == "nativestandingturntimeoutseconds") {
                    config.nativeStandingTurnTimeoutSeconds = ParsePositiveFloat(
                        value, config.nativeStandingTurnTimeoutSeconds, 0.5f, 15.0f);
                } else if (key == "nativestandingturnnoprogresstimeoutseconds") {
                    config.nativeStandingTurnNoProgressTimeoutSeconds = ParsePositiveFloat(
                        value, config.nativeStandingTurnNoProgressTimeoutSeconds, 0.25f, 5.0f);
                } else if (key == "nativestandingturnminprogressperframedegrees") {
                    config.nativeStandingTurnMinProgressPerFrameDegrees = ParsePositiveFloat(
                        value, config.nativeStandingTurnMinProgressPerFrameDegrees, 0.001f, 1.0f);
                } else if (key == "nativestandingturnfailurecooldownseconds") {
                    config.nativeStandingTurnFailureCooldownSeconds = ParsePositiveFloat(
                        value, config.nativeStandingTurnFailureCooldownSeconds, 0.25f, 5.0f);
                } else if (key == "nativestandingturnstopdrainconfirmframes") {
                    try {
                        config.nativeStandingTurnStopDrainConfirmFrames = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(std::stoul(value), 1UL, 30UL));
                    } catch (...) {}
                } else if (key == "nativestandingturnstartquarantineseconds") {
                    config.nativeStandingTurnStartQuarantineSeconds = ParsePositiveFloat(
                        value, config.nativeStandingTurnStartQuarantineSeconds, 0.0f, 2.0f);
                } else if (key == "nativestandingturnphysicalyawratedegreespersecond") {
                    config.nativeStandingTurnPhysicalYawRateDegreesPerSecond = ParsePositiveFloat(
                        value, config.nativeStandingTurnPhysicalYawRateDegreesPerSecond, 10.0f, 360.0f);
                } else if (key == "authoredpivotturnvariable") {
                    config.authoredPivotTurnVariable = value;
                } else if (key == "authoredpivotleft90node") {
                    config.authoredPivotLeft90Node = value;
                } else if (key == "authoredpivotright90node") {
                    config.authoredPivotRight90Node = value;
                } else if (key == "authoredpivotleft180node") {
                    config.authoredPivotLeft180Node = value;
                } else if (key == "authoredpivotright180node") {
                    config.authoredPivotRight180Node = value;
                } else if (key == "authoredpivot90stopdegrees") {
                    config.authoredPivot90StopDegrees = ParsePositiveFloat(
                        value, config.authoredPivot90StopDegrees, 45.0f, 90.0f);
                } else if (key == "authoredpivot90yawratedegreespersecond") {
                    config.authoredPivot90YawRateDegreesPerSecond = ParsePositiveFloat(
                        value, config.authoredPivot90YawRateDegreesPerSecond, 10.0f, 360.0f);
                } else if (key == "authoredpivot90maxreacquireattempts") {
                    try {
                        const auto parsed = std::stoul(value);
                        config.authoredPivot90MaxReacquireAttempts = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(parsed, 0UL, 3UL));
                    } catch (...) {}
                } else if (key == "authoredpivot90stallseconds") {
                    config.authoredPivot90StallSeconds = ParsePositiveFloat(
                        value, config.authoredPivot90StallSeconds, 0.05f, 1.0f);
                } else if (key == "authoredpivot90minprogressdegrees") {
                    config.authoredPivot90MinProgressDegrees = ParsePositiveFloat(
                        value, config.authoredPivot90MinProgressDegrees, 0.1f, 30.0f);
                } else if (key == "telemetryeveryframes") {
                    try {
                        config.telemetryEveryFrames = static_cast<std::uint32_t>(
                            std::clamp<unsigned long>(std::stoul(value), 1UL, 2400UL));
                    } catch (...) {}
                }
            }

            const float span = config.cycleMax - config.cycleMin;
            if (config.enabled && (
                config.speedVariable.empty() || config.directionVariable.empty() ||
                !std::isfinite(span) || span <= 1.0e-5f ||
                !std::isfinite(config.walkSample) || !std::isfinite(config.trotSample) ||
                !std::isfinite(config.runSample))) {
                config.enabled = false;
                SKSE::log::warn(
                    "[LocomotionPolicy][R93_SELECTOR_REJECT] profile=\"{}\" path=\"{}\" speedVar=\"{}\" directionVar=\"{}\" cycle=[{:.3f},{:.3f}) action=DISABLE",
                    profile, path.string(), config.speedVariable, config.directionVariable,
                    config.cycleMin, config.cycleMax);
            }

            if (config.enabled) {
                SKSE::log::info(
                    "[LocomotionPolicy][R116_STABLE_SELECTOR] profile=\"{}\" path=\"{}\" variables[speed=\"{}\" direction=\"{}\"] samples[walk={:.6f} trot={:.6f} run={:.6f}] direction[cycle={:.3f}..{:.3f} forward={:.3f} sectors={} positiveYawIncreases={} hysteresis={:.1f}deg confirmFrames={}] axisCooldown={:.3f}s minGoalDistance={:.1f} authoredPivot[enabled={} route90Enabled={} confirmFrames={} requireGraphInactive={} route180MinOverride={:.1f} route180PreemptLocomotion={}] nativeTurn[preemptLocomotion={} minGoalDistance={:.1f} commit={:.1f}deg release={:.1f}deg releaseConfirm={} timeout={:.2f}s noProgress={:.2f}s minFrameProgress={:.3f}deg failCooldown={:.2f}s stopDrainConfirm={} startQuarantine={:.3f}s physicalRate={:.1f}deg/s] turnVariable=\"{}\" nodes[left90=\"{}\" right90=\"{}\" left180=\"{}\" right180=\"{}\"] loop90[stop={:.1f}deg rate={:.1f}deg/s reacquireMax={} stall={:.3f}s minProgress={:.1f}deg]] authority=PRE_NATIVE_DECISION_PLUS_FINAL_REASSERT nativeStanding90=LATCHED_WINDOW_ZERO_SELECTOR_WRITES vanillaSampler=OVERRIDDEN_BEFORE_SELECTION speciesHardcode=ZERO behaviorHardcode=ZERO",
                    profile, path.string(), config.speedVariable, config.directionVariable,
                    config.walkSample, config.trotSample, config.runSample,
                    config.cycleMin, config.cycleMax, config.forwardValue,
                    config.directionSectors, config.positiveYawIncreases,
                    config.directionSwitchHysteresisDegrees, config.directionConfirmFrames,
                    config.axisSwitchCooldownSeconds, config.minGoalDistance,
                    config.authoredPivotEnabled, config.authoredPivot90Enabled,
                    config.authoredPivotConfirmFrames, config.authoredPivotRequireGraphInactive,
                    config.authoredPivot180RouteMinAngle, config.authoredPivot180PreemptLocomotion,
                    config.nativeStandingTurnPreemptLocomotion,
                    config.nativeStandingTurnMinGoalDistance,
                    config.nativeStandingTurnCommitAngleDegrees,
                    config.nativeStandingTurnReleaseAngleDegrees,
                    config.nativeStandingTurnReleaseConfirmFrames,
                    config.nativeStandingTurnTimeoutSeconds,
                    config.nativeStandingTurnNoProgressTimeoutSeconds,
                    config.nativeStandingTurnMinProgressPerFrameDegrees,
                    config.nativeStandingTurnFailureCooldownSeconds,
                    config.nativeStandingTurnStopDrainConfirmFrames,
                    config.nativeStandingTurnStartQuarantineSeconds,
                    config.nativeStandingTurnPhysicalYawRateDegreesPerSecond,
                    config.authoredPivotTurnVariable, config.authoredPivotLeft90Node,
                    config.authoredPivotRight90Node, config.authoredPivotLeft180Node,
                    config.authoredPivotRight180Node, config.authoredPivot90StopDegrees,
                    config.authoredPivot90YawRateDegreesPerSecond,
                    config.authoredPivot90MaxReacquireAttempts,
                    config.authoredPivot90StallSeconds, config.authoredPivot90MinProgressDegrees);
            }
            return config;
        }

        [[nodiscard]] std::shared_ptr<const StableSampleSelectorConfig>
        StableSampleSelectorConfigForProfile(const std::string& profile)
        {
            std::scoped_lock lock{ g_gaitRouterMutex };
            auto it = g_stableSampleSelectorConfigs.find(profile);
            if (it == g_stableSampleSelectorConfigs.end()) {
                auto loaded = std::make_shared<const StableSampleSelectorConfig>(
                    LoadStableSampleSelectorConfig(profile));
                it = g_stableSampleSelectorConfigs.emplace(profile, std::move(loaded)).first;
            }
            return it->second;
        }

        [[nodiscard]] bool StableSampleSelectorEnabledForActor(RE::Actor* actor)
        {
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) return false;
            const auto profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = StableSampleSelectorConfigForProfile(profile);
            return config && config->enabled;
        }

        [[nodiscard]] const GaitRouteConfig* FindGaitRoute(
            const GaitRouterConfig& config,
            std::int32_t routeValue)
        {
            for (const auto& route : config.routes) {
                if (route.configured && route.value == routeValue) {
                    return std::addressof(route);
                }
            }
            return nullptr;
        }

        [[nodiscard]] const std::string& EventForFamily(
            const GaitRouteConfig& route,
            SpeedIntentTranslator::Family family)
        {
            switch (family) {
            case SpeedIntentTranslator::Family::kWalkForward:
                return route.walkEvent;
            case SpeedIntentTranslator::Family::kTrotForward:
                return route.trotEvent;
            case SpeedIntentTranslator::Family::kRunForward:
                return route.runEvent;
            default:
                break;
            }
            static const std::string empty;
            return empty;
        }

        [[nodiscard]] const char* FamilyName(SpeedIntentTranslator::Family family) noexcept
        {
            switch (family) {
            case SpeedIntentTranslator::Family::kWalkForward: return "WALK";
            case SpeedIntentTranslator::Family::kTrotForward: return "TROT";
            case SpeedIntentTranslator::Family::kRunForward: return "RUN";
            default: return "NONE";
            }
        }

        [[nodiscard]] float SpeedGoalForFamily(
            SpeedIntentTranslator::Family family,
            const MotionProfileDatabase::SpeedIntentConfig& config) noexcept
        {
            switch (family) {
            case SpeedIntentTranslator::Family::kWalkForward: return config.walkGoal;
            case SpeedIntentTranslator::Family::kTrotForward: return config.trotGoal;
            case SpeedIntentTranslator::Family::kRunForward: return config.runGoal;
            default: return 0.0f;
            }
        }

        [[nodiscard]] bool IsRotationActionBypassActive(
            RE::Actor* actor,
            const MotionProfileDatabase::Profile& profile);

        void NoteR72NativeSpeedPreserve(
            RE::Actor* actor,
            SpeedIntentTranslator::Family family,
            const char* phase,
            const char* reason,
            bool priorRead,
            float prior,
            float goal)
        {
            if (!actor) {
                return;
            }

            const auto formID = actor->GetFormID();
            std::uint64_t preserves = 0;
            bool emit = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                preserves = ++state.r72NativeSpeedPreserves;
                if (reason && std::string_view(reason) == "ACTION_BYPASS") {
                    ++state.r72ActionSpeedPreserves;
                }
                emit = !state.r72NativeSpeedPreserveActive || preserves <= 8 || (preserves % 120) == 0;
                state.r72NativeSpeedPreserveActive = true;
                state.speedGoalHoldActive = false;
            }

            if (emit) {
                SKSE::log::info(
                    "[SpeedIntent][R72_NATIVE_SPEED_PRESERVE] form={:08X} family={} phase={} reason={} prior[read={} value={:.3f}] goal={:.3f} preserves={} action=KEEP_NATIVE_GRAPH_SPEED",
                    formID, FamilyName(family), phase ? phase : "UNKNOWN", reason ? reason : "UNKNOWN",
                    priorRead, prior, goal, preserves);
            }
        }

        void ClearR72NativeSpeedPreserveFlag(RE::FormID formID)
        {
            std::scoped_lock lock{ g_gaitRouterMutex };
            const auto it = g_gaitRouterActors.find(formID);
            if (it != g_gaitRouterActors.end()) {
                it->second.r72NativeSpeedPreserveActive = false;
            }
        }

        // R48: the original SpeedIntent adapter intentionally refuses to write Speed when
        // ActorState directional flags and graph bIsMoving are both false. The R46 log
        // proves those signals can be false while a native locomotion parent is already
        // active, allowing the behavior's family blend input to drift away from the
        // semantic WALK/TROT/RUN latch.
        //
        // This fallback does NOT invent movement. It is legal only when the existing
        // GaitRouter reports the parent locomotion state natively active, the active route
        // actually defines an event for the held family, and the normal adapter declined
        // to write. R45/R46 moveStart remains the only structural parent reentry path.
        bool ApplyParentActiveSpeedGoalHold(RE::Actor* actor, bool normalAdapterWrote, bool postCharacterPhase, bool requireCertifiedRunCarrier = false)
        {
            if (!actor || actor->IsPlayerRef() || !SpeedIntentTranslator::SupportsActor(actor)) {
                return false;
            }

            const auto formID = actor->GetFormID();
            if (normalAdapterWrote) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto it = g_gaitRouterActors.find(formID);
                if (it != g_gaitRouterActors.end()) {
                    it->second.speedGoalHoldActive = false;
                }
                return false;
            }

            const auto* profileConfig = MotionProfileDatabase::ResolveProfile(actor);
            if (!profileConfig || !profileConfig->speed.enabled || profileConfig->speed.graphSpeed.empty()) {
                return false;
            }

            // R72: native finite root actions (attack/stagger/etc. configured through the
            // existing rotation ActionBypassVariables) own their graph timing. R48/R54 may
            // never rewrite locomotion Speed while that lane is active. The R71 log proved
            // repeated WALK 533/695/708 -> 169.802 corrections during AttackPowerForwardShort.
            if (IsRotationActionBypassActive(actor, *profileConfig)) {
                const auto family = SpeedIntentTranslator::LatchedFamily(formID);
                const float goal = SpeedGoalForFamily(family, profileConfig->speed);
                float prior = 0.0f;
                const bool priorRead = actor->GetGraphVariableFloat(profileConfig->speed.graphSpeed.c_str(), prior);
                NoteR72NativeSpeedPreserve(
                    actor, family,
                    requireCertifiedRunCarrier ? "PRE_CHARACTER_UPDATE" :
                        (postCharacterPhase ? "POST_CHARACTER_UPDATE" : "MODIFY_ANIMATION_UPDATE_FALLBACK"),
                    "ACTION_BYPASS", priorRead, prior, goal);
                return false;
            }

            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto gaitConfig = GaitRouterConfigForProfile(profile);
            if (!gaitConfig || !gaitConfig->enabled || gaitConfig->activeVariable.empty()) {
                return false;
            }

            std::int32_t activeValue = 0;
            if (!actor->GetGraphVariableInt(gaitConfig->activeVariable.c_str(), activeValue) ||
                activeValue != gaitConfig->activeValue) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto it = g_gaitRouterActors.find(formID);
                if (it != g_gaitRouterActors.end()) {
                    it->second.speedGoalHoldActive = false;
                }
                return false;
            }

            std::int32_t routeValue = 0;
            if (!gaitConfig->routeVariable.empty() &&
                !actor->GetGraphVariableInt(gaitConfig->routeVariable.c_str(), routeValue)) {
                return false;
            }
            const auto* route = FindGaitRoute(*gaitConfig, routeValue);
            if (!route) {
                return false;
            }

            const auto family = SpeedIntentTranslator::LatchedFamily(formID);
            if (family == SpeedIntentTranslator::Family::kNone || EventForFamily(*route, family).empty()) {
                return false;
            }

            // R54: the earliest Character::Update hold is deliberately narrower than the
            // existing R47/R48 maintenance lanes. It may run only when the semantic family
            // is RUN and the injector proves a real RUN RuntimeYaw carrier existed in the
            // current/previous generated frame. This prevents Bethesda's early update from
            // seeing a transient low native Speed and selecting ForwardRunBlend -> Walk,
            // while retaining R50/R51's absolute protection against writing the RUN goal
            // onto WALK/TROT-only or carrierless states.
            if (requireCertifiedRunCarrier) {
                if (family != SpeedIntentTranslator::Family::kRunForward) {
                    return false;
                }
                const auto carrierMask =
                    SyntheticMotionInjector::RuntimeYawLocomotionFamilyMaskForCurrentOrPreviousFrame(actor);
                if ((carrierMask & kRuntimeFamilyRunMask) == 0) {
                    return false;
                }
            }

            // R50: RUN has its own semantic graph event (runStart). The R49 test proved that
            // while a tight curve can temporarily leave only WALK/TROT RuntimeYaw carriers
            // alive, forcing the RUN Speed goal (833 for the wolf profile) onto those lower
            // family clips can time-scale the center walk pose by ~20x while the directional
            // child remains nearly frozen. That is pose/cadence corruption, not root speed.
            // Fail closed only for this proven distinct-event mismatch: if generated DB root
            // says WALK/TROT but no RUN carrier exists, do not hold RUN Speed. Once a RUN
            // carrier appears, the existing R47/R48 hold resumes unchanged. WALK/TROT are
            // intentionally not gated here because both share walkStart in the profile.
            if (family == SpeedIntentTranslator::Family::kRunForward) {
                std::uint8_t carrierMask = 0;
                if (RunSpeedCarrierMismatch(actor, carrierMask)) {
                    std::uint64_t suppressions = 0;
                    bool logSuppression = false;
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        suppressions = ++state.runSpeedHoldMismatchSuppressions;
                        logSuppression = !state.runSpeedHoldMismatchSuppressed || (suppressions % 120) == 0;
                        state.runSpeedHoldMismatchSuppressed = true;
                        state.speedGoalHoldActive = false;
                    }
                    if (logSuppression) {
                        SKSE::log::info(
                            "[SpeedIntent][R50_RUN_HOLD_SUPPRESSED] form={:08X} profile=\"{}\" route=\"{}\" family=RUN phase={} carrierMask=0x{:02X} proof=GENERATED_RUNTIME_ROOT_HAS_WALK_OR_TROT_AND_NO_RUN action=PRESERVE_NATIVE_GRAPH_SPEED_UNTIL_RUN_CARRIER",
                            formID, profile, route->name,
                            requireCertifiedRunCarrier ? "PRE_CHARACTER_UPDATE" :
                                (postCharacterPhase ? "POST_CHARACTER_UPDATE" : "MODIFY_ANIMATION_UPDATE_FALLBACK"),
                            carrierMask);
                    }
                    return false;
                }

                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                state.runSpeedHoldMismatchSuppressed = false;
            }

            const float goal = SpeedGoalForFamily(family, profileConfig->speed);
            if (!std::isfinite(goal) || goal <= 0.0f) {
                return false;
            }

            float prior = 0.0f;
            const bool priorRead = actor->GetGraphVariableFloat(profileConfig->speed.graphSpeed.c_str(), prior);
            const bool alreadyAtGoal = priorRead && std::isfinite(prior) && std::fabs(prior - goal) <= 0.05f;

            // R72 turns R48/R54 from an exact-goal hammer into a stall rescue. Bethesda
            // legitimately writes intermediate Speed values while gait blends settle; the
            // R71 log repeatedly showed those values being overwritten at the final/early
            // Character boundaries. Preserve any finite positive native value above a small
            // profile-relative near-zero floor. The normal pre-Havok SpeedIntent writer still
            // supplies the semantic family goal; this fallback only rescues a collapsed value.
            const float nearZeroRescueCeiling =
                std::max(1.0f, std::max(0.0f, profileConfig->speed.walkGoal) * 0.10f);
            const bool stableFamilyGoal =
                gaitConfig->speedCadencePolicy == SpeedCadencePolicy::kStableFamilyGoal;
            if (!stableFamilyGoal && !alreadyAtGoal && priorRead && std::isfinite(prior) && prior > nearZeroRescueCeiling) {
                NoteR72NativeSpeedPreserve(
                    actor, family,
                    requireCertifiedRunCarrier ? "PRE_CHARACTER_UPDATE" :
                        (postCharacterPhase ? "POST_CHARACTER_UPDATE" : "MODIFY_ANIMATION_UPDATE_FALLBACK"),
                    "NATIVE_NONZERO_TRANSITION", priorRead, prior, goal);
                return false;
            }

            ClearR72NativeSpeedPreserveFlag(formID);

            bool logEntry = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                ++state.speedGoalHoldChecks;
                logEntry = !state.speedGoalHoldActive;
                state.speedGoalHoldActive = true;
            }

            if (alreadyAtGoal) {
                return false;
            }

            const bool wrote = actor->SetGraphVariableFloat(profileConfig->speed.graphSpeed.c_str(), goal);
            std::uint64_t holdWrites = 0;
            bool logPeriodic = false;
            if (wrote) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                holdWrites = ++state.speedGoalHoldWrites;
                logPeriodic = (holdWrites % 120) == 0;
            }

            const bool boundaryPhase = postCharacterPhase || requireCertifiedRunCarrier;
            const bool logBoundary =
                boundaryPhase && (holdWrites <= 8 || (holdWrites % 120) == 0);
            if (wrote && ((!boundaryPhase && (logEntry || logPeriodic)) || logBoundary)) {
                const char* phase =
                    requireCertifiedRunCarrier ? "PRE_CHARACTER_UPDATE" :
                        (postCharacterPhase ? "POST_CHARACTER_UPDATE" : "MODIFY_ANIMATION_UPDATE_FALLBACK");
                SKSE::log::info(
                    "[SpeedIntent][{}] form={:08X} profile=\"{}\" route=\"{}\" family={} phase={} prior[read={} value={:.3f}] goal={:.3f} write={} holdWrites={} cadence={} proof=NATIVE_PARENT_ACTIVE_AND_CONFIGURED_GAIT_ROUTE action=MAINTAIN_FAMILY_BLEND_ONLY_NO_MOVE_START",
                    requireCertifiedRunCarrier ? "R54_PRE_CHARACTER_RUN_HOLD" : "R48_PARENT_ACTIVE_HOLD",
                    formID, profile, route->name, FamilyName(family), phase,
                    priorRead, prior, goal, wrote, holdWrites, SpeedCadencePolicyName(gaitConfig->speedCadencePolicy));
            }
            return wrote;
        }

        [[nodiscard]] float WrapSignedDegrees(float degrees) noexcept
        {
            if (!std::isfinite(degrees)) return 0.0f;
            degrees = std::fmod(degrees + 180.0f, 360.0f);
            if (degrees < 0.0f) degrees += 360.0f;
            return degrees - 180.0f;
        }

        [[nodiscard]] float StableSampleForFamily(
            SpeedIntentTranslator::Family family,
            const StableSampleSelectorConfig& config) noexcept
        {
            switch (family) {
            case SpeedIntentTranslator::Family::kWalkForward: return config.walkSample;
            case SpeedIntentTranslator::Family::kTrotForward: return config.trotSample;
            case SpeedIntentTranslator::Family::kRunForward: return config.runSample;
            default: return config.walkSample;
            }
        }

        [[nodiscard]] std::int32_t StableDirectionSectorForError(
            float errorDegrees,
            const StableSampleSelectorConfig& config) noexcept
        {
            const float span = config.cycleMax - config.cycleMin;
            const float step = span / static_cast<float>(std::max<std::uint32_t>(1, config.directionSectors));
            const float sign = config.positiveYawIncreases ? 1.0f : -1.0f;
            float raw = config.forwardValue + sign * (WrapSignedDegrees(errorDegrees) / 360.0f) * span;
            while (raw < config.cycleMin) raw += span;
            while (raw >= config.cycleMax) raw -= span;

            float relative = raw - config.forwardValue;
            while (relative < 0.0f) relative += span;
            while (relative >= span) relative -= span;
            const auto sectors = static_cast<std::int32_t>(std::max<std::uint32_t>(1, config.directionSectors));
            auto sector = static_cast<std::int32_t>(std::lround(relative / step)) % sectors;
            if (sector < 0) sector += sectors;
            return sector;
        }

        [[nodiscard]] float StableDirectionValueForSector(
            std::int32_t sector,
            const StableSampleSelectorConfig& config) noexcept
        {
            const float span = config.cycleMax - config.cycleMin;
            const auto sectors = static_cast<std::int32_t>(std::max<std::uint32_t>(1, config.directionSectors));
            sector %= sectors;
            if (sector < 0) sector += sectors;
            float value = config.forwardValue +
                (span / static_cast<float>(sectors)) * static_cast<float>(sector);
            while (value < config.cycleMin) value += span;
            while (value >= config.cycleMax) value -= span;
            return value;
        }

        [[nodiscard]] float StableSteeringCenterDegreesForSector(
            std::int32_t sector,
            const StableSampleSelectorConfig& config) noexcept
        {
            const auto sectors = static_cast<std::int32_t>(std::max<std::uint32_t>(1, config.directionSectors));
            sector %= sectors;
            if (sector < 0) sector += sectors;
            const float cycleDegrees = (360.0f / static_cast<float>(sectors)) * static_cast<float>(sector);
            const float sign = config.positiveYawIncreases ? 1.0f : -1.0f;
            return WrapSignedDegrees(cycleDegrees / sign);
        }

        [[nodiscard]] const char* StablePivotPhaseName(StablePivotPhase phase) noexcept
        {
            switch (phase) {
            case StablePivotPhase::kStopping: return "STOPPING";
            case StablePivotPhase::kPending: return "PENDING";
            case StablePivotPhase::kActive: return "ACTIVE";
            default: return "IDLE";
            }
        }

        [[nodiscard]] StablePivotPhase StablePivotPhaseForActor(RE::FormID formID)
        {
            std::scoped_lock lock{ g_gaitRouterMutex };
            const auto it = g_gaitRouterActors.find(formID);
            return it != g_gaitRouterActors.end()
                ? it->second.stablePivotPhase
                : StablePivotPhase::kIdle;
        }

        [[nodiscard]] double StablePivotMonotonicSeconds() noexcept
        {
            return std::chrono::duration<double>(
                std::chrono::steady_clock::now().time_since_epoch()).count();
        }

        [[nodiscard]] bool StableSamplePivotBusy(RE::Actor* actor)
        {
            return actor && StablePivotPhaseForActor(actor->GetFormID()) != StablePivotPhase::kIdle;
        }

        [[nodiscard]] bool HasRecentStablePivotNativeRotationClip(RE::Actor* actor)
        {
            if (!actor) return false;
            std::scoped_lock lock{ g_gaitRouterMutex };
            const auto it = g_gaitRouterActors.find(actor->GetFormID());
            if (it == g_gaitRouterActors.end()) return false;
            const auto& state = it->second;
            if (state.stablePivotNativeRotationSeenFrame == 0 ||
                state.stablePivotNativeRotationKey.empty()) {
                return false;
            }
            // R108: ClipGenerator::Update runs inside native Character::Update. At the next
            // pre-native boundary the strongest possible evidence is therefore a matching
            // weighted clip seen in the immediately preceding actor frame. The old 120 ms
            // wall-clock grace kept V13 rotating for ~8-10 frames after the visual turn had
            // already disappeared, producing the observed in-place spin.
            return state.stablePivotFrameSerial <= state.stablePivotNativeRotationSeenFrame + 1;
        }

        [[nodiscard]] bool ReadStablePivotLocomotionActive(
            RE::Actor* actor,
            const std::string& profile,
            const MotionProfileDatabase::AnimationPriorityConfig& pivot,
            bool& outActive)
        {
            outActive = false;
            if (!actor) return false;

            // Primary witness is explicitly profile data. Troll uses bAnimationDriven here
            // because IsMoving/iSyncIdleLocomotion may remain asserted while physical
            // locomotion has already ended.
            if (!pivot.graphLocomotionActiveVariable.empty()) {
                bool value = false;
                if (actor->GetGraphVariableBool(
                    pivot.graphLocomotionActiveVariable.c_str(), value)) {
                    outActive = value;
                    return true;
                }
            }

            // Fallback only: some profiles expose locomotion parent activity as an int.
            const auto gait = GaitRouterConfigForProfile(profile);
            if (!gait || !gait->enabled || gait->activeVariable.empty()) {
                return false;
            }
            std::int32_t value = 0;
            if (!actor->GetGraphVariableInt(gait->activeVariable.c_str(), value)) {
                return false;
            }
            outActive = value == gait->activeValue;
            return true;
        }

        struct NativeStandingTurnGateSnapshot
        {
            bool active{ false };
            bool stopDrain{ false };
            std::uint32_t candidateFrames{ 0 };
            float startQuarantineRemaining{ 0.0f };
        };

        [[nodiscard]] NativeStandingTurnGateSnapshot NativeStandingTurnGateForActor(RE::Actor* actor)
        {
            NativeStandingTurnGateSnapshot result{};
            if (!actor) return result;
            std::scoped_lock lock{ g_gaitRouterMutex };
            const auto it = g_gaitRouterActors.find(actor->GetFormID());
            if (it == g_gaitRouterActors.end()) return result;
            result.active = it->second.nativeStandingTurnPassthrough;
            result.stopDrain = it->second.nativeStandingTurnStopDrain;
            result.candidateFrames = it->second.nativeStandingTurnCandidateFrames;
            result.startQuarantineRemaining =
                it->second.nativeStandingTurnStartQuarantineRemaining;
            return result;
        }

        [[nodiscard]] bool ShouldDeferToNativeStandingTurn(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config)
        {
            if (!actor || config.authoredPivot90Enabled ||
                !config.nativeStandingTurnPreemptLocomotion) {
                return false;
            }
            return NativeStandingTurnGateForActor(actor).active;
        }

        [[nodiscard]] bool ShouldInterlockNativeStandingTurnStart(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config)
        {
            if (!actor || config.authoredPivot90Enabled ||
                !config.nativeStandingTurnPreemptLocomotion) {
                return false;
            }
            const auto gate = NativeStandingTurnGateForActor(actor);
            return gate.active || gate.candidateFrames > 0 ||
                gate.startQuarantineRemaining > 0.0f;
        }

        [[nodiscard]] bool NativeStandingTurnLatchedSide(RE::Actor* actor, bool& outLeft)
        {
            outLeft = false;
            if (!actor) return false;
            std::scoped_lock lock{ g_gaitRouterMutex };
            const auto it = g_gaitRouterActors.find(actor->GetFormID());
            // Cadence starts only after STOP_DRAIN; mixing a turn clock with the exiting
            // locomotion tree recreates the exact transition overlap R114 is preventing.
            if (it == g_gaitRouterActors.end() ||
                !it->second.nativeStandingTurnPassthrough ||
                it->second.nativeStandingTurnStopDrain) {
                return false;
            }
            outLeft = it->second.nativeStandingTurnLeft;
            return true;
        }

        [[nodiscard]] bool NativeStandingTurnCadenceNodeMatches(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config,
            const char* rawNodeName)
        {
            if (!actor || !rawNodeName || !rawNodeName[0]) return false;
            bool left = false;
            if (!NativeStandingTurnLatchedSide(actor, left)) return false;
            const auto expected = LowerCopy(left ? config.authoredPivotLeft90Node : config.authoredPivotRight90Node);
            if (expected.empty()) return false;
            const auto node = LowerCopy(std::string(rawNodeName));
            return node == expected || (node.size() > expected.size() && node.rfind(expected, 0) == 0);
        }

        // R116: one serialized canned-turn window for the patched Troll behavior.
        // INCESSUS may ask locomotion to stop once; after STOP_DRAIN it dispatches exactly one
        // profile event: turnLeft/turnRight for canned 90, or turnLeft180/turnRight180 for canned 180.
        // All four clips are authored single-play states in MT_Canned_Turn_Behavior. INCESSUS never
        // writes TurnDelta/rotationSpeed/localTime and never synthesizes standing yaw. RuntimeYaw
        // locomotion leaves remain suppressed while the canned clip owns authored yaw.
        // R114 start/stop serialization is preserved.
        [[nodiscard]] bool UpdateNativeStandingTurnWindowPreNative(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config,
            float deltaTime)
        {
            if (!actor || config.authoredPivot90Enabled ||
                !config.nativeStandingTurnPreemptLocomotion) {
                return false;
            }

            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!motionProfile) return false;
            const auto& pivot = motionProfile->animationPriority;
            if (!std::isfinite(pivot.turn90MinAngle) || pivot.turn90MinAngle <= 0.0f) {
                return false;
            }

            const float dt = std::isfinite(deltaTime) && deltaTime > 0.0f && deltaTime <= 0.100f
                ? deltaTime : (1.0f / 60.0f);
            const bool actionBypass = IsRotationActionBypassActive(actor, *motionProfile);

            SteeringIntentController::FrameCommand steering{};
            const bool steeringUsable =
                SteeringIntentController::GetFrameCommand(actor, steering) &&
                steering.valid &&
                std::isfinite(steering.filteredErrorRadians) &&
                std::isfinite(steering.goalDistanceXY) &&
                steering.goalDistanceXY >= config.nativeStandingTurnMinGoalDistance;
            const float errorDegrees = steeringUsable
                ? WrapSignedDegrees(steering.filteredErrorRadians * kRadToDeg) : 0.0f;
            const float absErrorDegrees = std::abs(errorDegrees);

            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            bool parentActive = false;
            const bool parentRead = ReadStablePivotLocomotionActive(
                actor, profile, pivot, parentActive);
            const bool runtimeCarrier =
                SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(actor);
            const bool authoredRotationObserved =
                SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor);
            float physicalYawRadians = 0.0f;
            const bool physicalYawRead =
                CapsuleSlerp::TryGetPhysicalYawRadians(actor, physicalYawRadians) && std::isfinite(physicalYawRadians);
            const float commitAngleDegrees =
                std::isfinite(config.nativeStandingTurnCommitAngleDegrees) && config.nativeStandingTurnCommitAngleDegrees > 0.0f
                    ? config.nativeStandingTurnCommitAngleDegrees : pivot.turn90MinAngle;

            float canned180ThresholdDegrees = pivot.turn180MinAngle;
            if (std::isfinite(config.authoredPivot180RouteMinAngle) &&
                config.authoredPivot180RouteMinAngle > pivot.turn90MinAngle &&
                config.authoredPivot180RouteMinAngle <= 180.0f) {
                canned180ThresholdDegrees = config.authoredPivot180RouteMinAngle;
            }

            auto dispatchCannedTurnOnce = [&](float currentAbsErrorDegrees) -> bool {
                bool turnLeft = false;
                std::uint32_t routeDegrees = 0;
                float basisErrorDegrees = currentAbsErrorDegrees;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[actor->GetFormID()];
                    if (!state.nativeStandingTurnPassthrough ||
                        state.nativeStandingTurnStopDrain ||
                        state.nativeStandingTurnCannedOffered) {
                        return true;
                    }
                    if (!std::isfinite(basisErrorDegrees) || basisErrorDegrees <= 0.0f) {
                        basisErrorDegrees = std::abs(state.nativeStandingTurnEntryErrorDegrees);
                    }
                    turnLeft = state.nativeStandingTurnLeft;
                    routeDegrees = basisErrorDegrees >= canned180ThresholdDegrees ? 180u : 90u;
                    state.nativeStandingTurnCannedOffered = true;
                    state.nativeStandingTurnCannedRouteDegrees = routeDegrees;
                    state.nativeStandingTurnAuthoredWitnessSeen = false;
                    state.nativeStandingTurnAuthoredMissingFrames = 0;
                    state.nativeStandingTurnNoProgressSeconds = 0.0f;
                }

                const auto& events = routeDegrees >= 180
                    ? (turnLeft ? pivot.turnLeft180Events : pivot.turnRight180Events)
                    : (turnLeft ? pivot.turnLeft90Events : pivot.turnRight90Events);
                bool accepted = false;
                std::string acceptedEvent;
                for (const auto& event : events) {
                    if (event.empty()) continue;
                    if (actor->NotifyAnimationGraph(event.c_str())) {
                        accepted = true;
                        acceptedEvent = event;
                        break;
                    }
                }

                SKSE::log::info(
                    "[NativeTurn][R116_CANNED_DISPATCH] form={:08X} profile=\"{}\" route={} side={} basisError={:.2f}deg threshold180={:.1f} event=\"{}\" accepted={} action=ONE_SHOT_CANNED_STATE_NO_PLAYBACK_OR_V13_RATE_OVERRIDE",
                    actor->GetFormID(), profile, routeDegrees, turnLeft ? "LEFT" : "RIGHT",
                    basisErrorDegrees, canned180ThresholdDegrees, acceptedEvent, accepted);

                if (accepted) return true;

                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[actor->GetFormID()];
                    state.nativeStandingTurnPassthrough = false;
                    state.nativeStandingTurnStopAccepted = false;
                    state.nativeStandingTurnCannedOffered = false;
                    state.nativeStandingTurnCannedRouteDegrees = 0;
                    state.nativeStandingTurnAuthoredWitnessSeen = false;
                    state.nativeStandingTurnAuthoredMissingFrames = 0;
                    state.nativeStandingTurnCandidateFrames = 0;
                    state.nativeStandingTurnStopDrain = false;
                    state.nativeStandingTurnStopDrainClearFrames = 0;
                    state.nativeStandingTurnStartQuarantineRemaining =
                        config.nativeStandingTurnStartQuarantineSeconds;
                    state.nativeStandingTurnCooldown = std::max(
                        state.nativeStandingTurnCooldown,
                        std::max(pivot.retryCooldownSeconds, config.nativeStandingTurnFailureCooldownSeconds));
                }
                SKSE::log::warn(
                    "[NativeTurn][R116_CANNED_REJECT] form={:08X} profile=\"{}\" route={} side={} action=FAIL_OPEN_RESTORE_AFTER_QUARANTINE",
                    actor->GetFormID(), profile, routeDegrees, turnLeft ? "LEFT" : "RIGHT");
                return false;
            };

            bool active = false;
            bool release = false;
            bool standingReadyEdge = false;
            bool quarantineExpiredEdge = false;
            const char* releaseReason = nullptr;
            float elapsed = 0.0f;
            float noProgressSeconds = 0.0f;
            float physicalProgressDegrees = 0.0f;
            std::uint32_t releaseFrames = 0;
            std::uint32_t missingIntentFrames = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[actor->GetFormID()];
                state.nativeStandingTurnCooldown =
                    std::max(0.0f, state.nativeStandingTurnCooldown - dt);

                if (!state.nativeStandingTurnPassthrough &&
                    state.nativeStandingTurnStartQuarantineRemaining > 0.0f) {
                    const float before = state.nativeStandingTurnStartQuarantineRemaining;
                    state.nativeStandingTurnStartQuarantineRemaining =
                        std::max(0.0f, before - dt);
                    quarantineExpiredEdge =
                        before > 0.0f && state.nativeStandingTurnStartQuarantineRemaining <= 0.0f;
                }

                if (actionBypass) {
                    state.nativeStandingTurnStartQuarantineRemaining = 0.0f;
                }

                if (state.nativeStandingTurnPassthrough) {
                    state.nativeStandingTurnElapsed += dt;
                    elapsed = state.nativeStandingTurnElapsed;

                    if (physicalYawRead) {
                        if (state.nativeStandingTurnPhysicalYawInitialized) {
                            const float deltaDegrees = std::abs(WrapSignedDegrees(
                                (physicalYawRadians - state.nativeStandingTurnLastPhysicalYawRadians) * kRadToDeg));
                            state.nativeStandingTurnPhysicalProgressDegrees += deltaDegrees;
                            if (state.nativeStandingTurnStopDrain) {
                                state.nativeStandingTurnNoProgressSeconds = 0.0f;
                            } else if (deltaDegrees >= config.nativeStandingTurnMinProgressPerFrameDegrees) {
                                state.nativeStandingTurnNoProgressSeconds = 0.0f;
                            } else {
                                state.nativeStandingTurnNoProgressSeconds += dt;
                            }
                        } else {
                            state.nativeStandingTurnPhysicalYawInitialized = true;
                            state.nativeStandingTurnNoProgressSeconds = 0.0f;
                        }
                        state.nativeStandingTurnLastPhysicalYawRadians = physicalYawRadians;
                    }

                    if (state.nativeStandingTurnStopDrain) {
                        if (!runtimeCarrier) {
                            if (state.nativeStandingTurnStopDrainClearFrames < 255) {
                                ++state.nativeStandingTurnStopDrainClearFrames;
                            }
                        } else {
                            state.nativeStandingTurnStopDrainClearFrames = 0;
                        }
                        if (state.nativeStandingTurnStopDrainClearFrames >=
                            std::max<std::uint32_t>(1u, config.nativeStandingTurnStopDrainConfirmFrames)) {
                            state.nativeStandingTurnStopDrain = false;
                            state.nativeStandingTurnStopDrainClearFrames = 0;
                            state.nativeStandingTurnNoProgressSeconds = 0.0f;
                            if (physicalYawRead) {
                                state.nativeStandingTurnPhysicalYawInitialized = true;
                                state.nativeStandingTurnLastPhysicalYawRadians = physicalYawRadians;
                            }
                            standingReadyEdge = true;
                        }
                    }

                    noProgressSeconds = state.nativeStandingTurnNoProgressSeconds;
                    physicalProgressDegrees = state.nativeStandingTurnPhysicalProgressDegrees;

                    if (actionBypass) {
                        release = true;
                        releaseReason = "ACTION_BYPASS";
                    } else if (!steeringUsable) {
                        if (state.nativeStandingTurnMissingIntentFrames < 255) {
                            ++state.nativeStandingTurnMissingIntentFrames;
                        }
                        missingIntentFrames = state.nativeStandingTurnMissingIntentFrames;
                        if (state.nativeStandingTurnMissingIntentFrames >= 8) {
                            release = true;
                            releaseReason = "STEERING_LOST_8_FRAMES";
                        }
                    } else {
                        state.nativeStandingTurnMissingIntentFrames = 0;
                        // R116: once the canned event is offered, the authored clip lifecycle owns
                        // completion. Do not release merely because heading crosses <=30 degrees
                        // mid-clip; that would cut a single-play 90/180 state before TurnInPlaceEnd.
                        state.nativeStandingTurnReleaseFrames = 0;
                        releaseFrames = 0;
                    }

                    if (!state.nativeStandingTurnStopDrain &&
                        state.nativeStandingTurnCannedOffered) {
                        if (authoredRotationObserved) {
                            state.nativeStandingTurnAuthoredWitnessSeen = true;
                            state.nativeStandingTurnAuthoredMissingFrames = 0;
                        } else if (state.nativeStandingTurnAuthoredWitnessSeen &&
                                   state.nativeStandingTurnAuthoredMissingFrames < 255) {
                            ++state.nativeStandingTurnAuthoredMissingFrames;
                        }
                        if (state.nativeStandingTurnAuthoredWitnessSeen &&
                            state.nativeStandingTurnAuthoredMissingFrames >= 2) {
                            release = true;
                            releaseReason = "CANNED_CLIP_COMPLETE";
                        }
                    }

                    if (!release && !state.nativeStandingTurnStopDrain && physicalYawRead &&
                        state.nativeStandingTurnNoProgressSeconds >= config.nativeStandingTurnNoProgressTimeoutSeconds) {
                        release = true;
                        releaseReason = "NO_PHYSICAL_YAW_PROGRESS";
                    }
                    if (!release &&
                        state.nativeStandingTurnElapsed >= config.nativeStandingTurnTimeoutSeconds) {
                        release = true;
                        releaseReason = "TIMEOUT_FAIL_OPEN";
                    }

                    if (release) {
                        const bool actionRelease = releaseReason &&
                            std::string_view{ releaseReason } == "ACTION_BYPASS";
                        state.nativeStandingTurnPassthrough = false;
                        state.nativeStandingTurnStopAccepted = false;
                        state.nativeStandingTurnCannedOffered = false;
                        state.nativeStandingTurnCannedRouteDegrees = 0;
                        state.nativeStandingTurnAuthoredWitnessSeen = false;
                        state.nativeStandingTurnAuthoredMissingFrames = 0;
                        state.nativeStandingTurnCandidateFrames = 0;
                        state.nativeStandingTurnReleaseFrames = 0;
                        state.nativeStandingTurnMissingIntentFrames = 0;
                        state.nativeStandingTurnElapsed = 0.0f;
                        state.nativeStandingTurnEntryErrorDegrees = 0.0f;
                        state.nativeStandingTurnLeft = false;
                        state.nativeStandingTurnPhysicalYawInitialized = false;
                        state.nativeStandingTurnLastPhysicalYawRadians = 0.0f;
                        state.nativeStandingTurnNoProgressSeconds = 0.0f;
                        state.nativeStandingTurnPhysicalProgressDegrees = 0.0f;
                        state.nativeStandingTurnStopDrain = false;
                        state.nativeStandingTurnStopDrainClearFrames = 0;
                        state.nativeStandingTurnStartQuarantineRemaining =
                            actionRelease ? 0.0f : config.nativeStandingTurnStartQuarantineSeconds;
                        const bool failedTurnWindow = releaseReason &&
                            (std::string_view{ releaseReason } == "NO_PHYSICAL_YAW_PROGRESS" ||
                             std::string_view{ releaseReason } == "TIMEOUT_FAIL_OPEN");
                        const float releaseCooldown = failedTurnWindow
                            ? std::max(pivot.retryCooldownSeconds, config.nativeStandingTurnFailureCooldownSeconds)
                            : pivot.retryCooldownSeconds;
                        state.nativeStandingTurnCooldown =
                            std::max(state.nativeStandingTurnCooldown, releaseCooldown);
                    } else {
                        active = true;
                    }
                } else {
                    if (actionBypass ||
                        state.nativeStandingTurnStartQuarantineRemaining > 0.0f ||
                        !steeringUsable ||
                        absErrorDegrees < commitAngleDegrees ||
                        state.nativeStandingTurnCooldown > 0.0f) {
                        state.nativeStandingTurnCandidateFrames = 0;
                    } else if (state.nativeStandingTurnCandidateFrames < 255) {
                        ++state.nativeStandingTurnCandidateFrames;
                    }
                }
            }

            if (quarantineExpiredEdge) {
                SKSE::log::info(
                    "[NativeTurn][R116_START_QUARANTINE_RELEASE] form={:08X} profile=\"{}\" action=PARENT_BOOTSTRAP_MAY_RESUME",
                    actor->GetFormID(), profile);
            }
            if (standingReadyEdge) {
                SKSE::log::info(
                    "[NativeTurn][R116_STANDING_TURN_READY] form={:08X} profile=\"{}\" error={:+.2f}deg carrier=false confirmFrames={} action=STOP_DRAIN_COMPLETE_DISPATCH_ONE_CANNED_90_OR_180",
                    actor->GetFormID(), profile, errorDegrees,
                    config.nativeStandingTurnStopDrainConfirmFrames);
            }

            if (!release && active) {
                if (!dispatchCannedTurnOnce(absErrorDegrees)) {
                    return false;
                }
            }

            if (release) {
                SKSE::log::info(
                    "[NativeTurn][R116_WINDOW_RELEASE] form={:08X} profile=\"{}\" reason={} error={:+.2f}deg elapsed={:.3f}s releaseFrames={} missingIntentFrames={} physical[read={} progress={:.2f}deg noProgress={:.3f}s] postStartQuarantine={:.3f}s action=RESTORE_AFTER_QUARANTINE",
                    actor->GetFormID(), profile, releaseReason ? releaseReason : "UNKNOWN",
                    errorDegrees, elapsed, releaseFrames, missingIntentFrames,
                    physicalYawRead, physicalProgressDegrees, noProgressSeconds,
                    (releaseReason && std::string_view{ releaseReason } == "ACTION_BYPASS")
                        ? 0.0f : config.nativeStandingTurnStartQuarantineSeconds);
                return false;
            }
            if (active) return true;

            std::uint32_t candidateFrames = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                candidateFrames = g_gaitRouterActors[actor->GetFormID()].nativeStandingTurnCandidateFrames;
            }
            const std::uint32_t requiredFrames =
                std::max<std::uint32_t>(1u, config.authoredPivotConfirmFrames);
            if (candidateFrames < requiredFrames) return false;

            const bool needsStop = runtimeCarrier || (parentRead && parentActive);
            bool stopAccepted = true;
            if (needsStop) {
                stopAccepted = !pivot.moveStopEvent.empty() &&
                    actor->NotifyAnimationGraph(pivot.moveStopEvent.c_str());
            }
            if (!stopAccepted) {
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[actor->GetFormID()];
                    state.nativeStandingTurnCandidateFrames = 0;
                    state.nativeStandingTurnCooldown =
                        std::max(state.nativeStandingTurnCooldown, pivot.retryCooldownSeconds);
                }
                SKSE::log::info(
                    "[NativeTurn][R114_STOP_REJECT] form={:08X} profile=\"{}\" error={:+.2f}deg parent[read={} active={}] runtimeCarrier={} event=\"{}\" action=FAIL_OPEN_KEEP_LOCOMOTION",
                    actor->GetFormID(), profile, errorDegrees, parentRead, parentActive,
                    runtimeCarrier, pivot.moveStopEvent);
                return false;
            }

            const bool positiveMeansLeft = LowerCopy(pivot.positiveTurnDirection) != "right";
            const bool latchedTurnLeft = errorDegrees >= 0.0f ? positiveMeansLeft : !positiveMeansLeft;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[actor->GetFormID()];
                state.nativeStandingTurnPassthrough = true;
                state.nativeStandingTurnStopAccepted = true;
                state.nativeStandingTurnCannedOffered = false;
                state.nativeStandingTurnCannedRouteDegrees = 0;
                state.nativeStandingTurnAuthoredWitnessSeen = false;
                state.nativeStandingTurnAuthoredMissingFrames = 0;
                state.nativeStandingTurnCandidateFrames = 0;
                state.nativeStandingTurnReleaseFrames = 0;
                state.nativeStandingTurnMissingIntentFrames = 0;
                state.nativeStandingTurnElapsed = 0.0f;
                state.nativeStandingTurnEntryErrorDegrees = errorDegrees;
                state.nativeStandingTurnLeft = latchedTurnLeft;
                state.nativeStandingTurnPhysicalYawInitialized = physicalYawRead;
                state.nativeStandingTurnLastPhysicalYawRadians = physicalYawRead ? physicalYawRadians : 0.0f;
                state.nativeStandingTurnNoProgressSeconds = 0.0f;
                state.nativeStandingTurnPhysicalProgressDegrees = 0.0f;
                state.nativeStandingTurnStopDrain = needsStop;
                state.nativeStandingTurnStopDrainClearFrames = 0;
                state.nativeStandingTurnStartQuarantineRemaining = 0.0f;
                ++state.nativeStandingTurnWindows;
                state.parentOnlyPhysicalReadySeen = false;
                state.parentOnlyPostCarrierMissingFrames = 0;
            }

            SKSE::log::info(
                "[NativeTurn][R116_WINDOW_COMMIT] form={:08X} profile=\"{}\" error={:+.2f}deg goalDist={:.1f} side={} parent[read={} active={}] runtimeCarrier={} moveStop[needed={} accepted={}] phase={} policy[commit>={:.1f} threshold180={:.1f}deg confirmFrames={} stopDrainConfirm={} timeout={:.2f}s noProgress={:.2f}s] physicalYawRead={} action=LATCH_SERIALIZED_CANNED_TURN",
                actor->GetFormID(), profile, errorDegrees,
                steeringUsable ? steering.goalDistanceXY : 0.0f, latchedTurnLeft ? "LEFT" : "RIGHT",
                parentRead, parentActive, runtimeCarrier, needsStop, stopAccepted,
                needsStop ? "STOP_DRAIN" : "TURN_READY",
                commitAngleDegrees, canned180ThresholdDegrees,
                config.authoredPivotConfirmFrames,
                config.nativeStandingTurnStopDrainConfirmFrames,
                config.nativeStandingTurnTimeoutSeconds, config.nativeStandingTurnNoProgressTimeoutSeconds,
                physicalYawRead);
            if (!needsStop && !dispatchCannedTurnOnce(absErrorDegrees)) {
                return false;
            }
            return true;
        }

        [[nodiscard]] bool NotifyFirstAcceptedStablePivotEvent(
            RE::Actor* actor,
            const std::vector<std::string>& events,
            std::string& outAccepted)
        {
            outAccepted.clear();
            if (!actor) return false;
            for (const auto& event : events) {
                if (event.empty()) continue;
                if (actor->NotifyAnimationGraph(event.c_str())) {
                    outAccepted = event;
                    return true;
                }
            }
            return false;
        }

        void ResetStablePivotLocked(
            GaitRouterActorState& state,
            float cooldownSeconds,
            bool completed,
            bool failed)
        {
            state.stablePivotPhase = StablePivotPhase::kIdle;
            state.stablePivotElapsed = 0.0f;
            state.stablePivotCooldown = std::max(state.stablePivotCooldown, cooldownSeconds);
            state.stablePivotRequestedErrorDegrees = 0.0f;
            state.stablePivotRouteDegrees = 0;
            state.stablePivotLeft = false;
            state.stablePivotCandidateFrames = 0;
            // R107: directional hysteresis accumulated before/during a pivot must not survive
            // the pivot reset and immediately select a different locomotion child.
            state.stablePendingDirectionSector = -1;
            state.stablePendingDirectionFrames = 0;
            state.stablePivotStandingFrames = 0;
            state.stablePivotDispatchAttempts = 0;
            state.stablePivotExitFrames = 0;
            state.stablePivotNativeRotationLastSeenSeconds = 0.0;
            state.stablePivotNativeRotationSeenFrame = 0;
            state.stablePivotNativeRotationPoseWeight = 0.0f;
            state.stablePivotNativeRotationKey.clear();
            state.stablePivot90PhysicalYawInitialized = false;
            state.stablePivot90PhysicalYawStartRadians = 0.0f;
            state.stablePivot90PhysicalYawMaxDegrees = 0.0f;
            state.stablePivot90TurnStopIssued = false;
            state.stablePivot90StallBreak = false;
            state.stablePivot90TurnStopAttempts = 0;
            state.stablePivot90ReacquireAttempts = 0;
            if (completed) {
                ++state.stablePivotCompleted;
                // The old locomotion carrier intentionally ended for this authored pivot.
                // Do not interpret that deliberate exit as an R91 transient parent drop when
                // we need to resume locomotion after the turn.
                state.parentOnlyPhysicalReadySeen = false;
                state.parentOnlyPostCarrierMissingFrames = 0;
                state.stableAxisSwitchCooldown = 0.0f;
            }
            if (failed) ++state.stablePivotFailures;
        }

        [[nodiscard]] bool ApplyStablePivotTurnParameter(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config,
            const char* boundary,
            bool allowStopping)
        {
            if (!actor || config.authoredPivotTurnVariable.empty()) return false;

            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!motionProfile) return false;

            const auto formID = actor->GetFormID();
            StablePivotPhase phase = StablePivotPhase::kIdle;
            std::uint32_t routeDegrees = 0;
            bool turnLeft = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto it = g_gaitRouterActors.find(formID);
                if (it == g_gaitRouterActors.end()) return false;
                phase = it->second.stablePivotPhase;
                routeDegrees = it->second.stablePivotRouteDegrees;
                turnLeft = it->second.stablePivotLeft;
            }

            const bool phaseEligible =
                phase == StablePivotPhase::kPending ||
                phase == StablePivotPhase::kActive ||
                (allowStopping && phase == StablePivotPhase::kStopping);
            if (!phaseEligible || routeDegrees != 90) return false;

            // The profile's PositiveTurnDirection describes the graph's signed turn axis.
            // For Troll that is Left => +90 and Right => -90, matching the serialized
            // MT_TurnLeft/Right_Blend child coordinates exactly.
            const bool positiveMeansLeft =
                LowerCopy(motionProfile->animationPriority.positiveTurnDirection) != "right";
            const float command = (turnLeft == positiveMeansLeft) ? 90.0f : -90.0f;

            float prior = 0.0f;
            const bool priorRead = actor->GetGraphVariableFloat(
                config.authoredPivotTurnVariable.c_str(), prior) && std::isfinite(prior);
            const bool wrote = actor->SetGraphVariableFloat(
                config.authoredPivotTurnVariable.c_str(), command);

            std::uint64_t writes = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                if (wrote) writes = ++state.stablePivotTurnParameterWrites;
            }

            const bool changed = !priorRead || std::abs(prior - command) > 0.10f;
            if (wrote && (changed || writes <= 12 || (writes % 120) == 0)) {
                SKSE::log::info(
                    "[StablePivot][R100_TURN_PARAMETER] form={:08X} profile=\"{}\" boundary={} phase={} route=90 direction={} variable=\"{}\" prior[read={} value={:+.2f}] command={:+.2f} write=true authority=STABLE_PIVOT_ONLY release=WHEN_PIVOT_ENDS",
                    formID, MotionProfileDatabase::DatabaseFolderForActor(actor),
                    boundary ? boundary : "UNKNOWN", StablePivotPhaseName(phase),
                    turnLeft ? "LEFT" : "RIGHT", config.authoredPivotTurnVariable,
                    priorRead, prior, command);
            }
            return wrote;
        }

        // R109: a 90-degree pivot witness is the FINAL composed behavior extracted-motion
        // yaw, not a child hkbClipGenerator. The final Havok output uses the physical world-yaw
        // convention already converted by CapsuleSlerp. This helper is read-only and is used
        // only to certify PENDING->ACTIVE and native state exit for the latched side.
        [[nodiscard]] bool HasFinalBehaviorPivotYawWitness(
            RE::Actor* actor,
            bool turnLeft,
            float& outPhysicalYawDeltaDegrees,
            std::int32_t& outGeneratorOutputIndex) noexcept
        {
            outPhysicalYawDeltaDegrees = 0.0f;
            outGeneratorOutputIndex = -1;
            if (!CapsuleSlerp::TryGetFinalBehaviorPhysicalYawDeltaDegrees(
                    actor, outPhysicalYawDeltaDegrees, &outGeneratorOutputIndex)) {
                return false;
            }
            if (!std::isfinite(outPhysicalYawDeltaDegrees)) return false;
            const float magnitude = std::abs(outPhysicalYawDeltaDegrees);
            if (magnitude <= 0.005f || magnitude > 45.0f) return false;
            // R104/R107 physical convention: left decreases world yaw, right increases it.
            return turnLeft ? outPhysicalYawDeltaDegrees < 0.0f : outPhysicalYawDeltaDegrees > 0.0f;
        }

        // R93: large heading errors are no longer converted into backward/sideways locomotion
        // indefinitely. Profiles may opt into an authored-pivot handshake. The stable selector
        // requests moveStop, waits for the native locomotion parent to be genuinely inactive,
        // dispatches the profile's authored 90/180 event, then waits for the MotionDB-classified
        // RotationClip to finish before locomotion may bootstrap again.
        [[nodiscard]] bool UpdateStableAuthoredPivotPreNative(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config,
            float deltaTime)
        {
            if (!actor || actor->IsPlayerRef() || !config.authoredPivotEnabled) return false;

            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!motionProfile) return false;
            const auto& pivot = motionProfile->animationPriority;

            // Profile data must fully describe the native route even when the legacy
            // AnimationPriority router itself remains disabled.
            const bool routesReady =
                !pivot.moveStopEvent.empty() &&
                (!config.authoredPivot90Enabled ||
                 (!pivot.turnLeft90Events.empty() && !pivot.turnRight90Events.empty())) &&
                !pivot.turnLeft180Events.empty() && !pivot.turnRight180Events.empty() &&
                std::isfinite(pivot.turn90MinAngle) && pivot.turn90MinAngle > 0.0f &&
                std::isfinite(pivot.turn180MinAngle) &&
                pivot.turn180MinAngle > pivot.turn90MinAngle &&
                pivot.turn180MinAngle <= 180.0f;
            if (!routesReady) return false;

            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const float dt = std::isfinite(deltaTime) && deltaTime > 0.0f && deltaTime <= 0.100f
                ? deltaTime : (1.0f / 60.0f);

            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                ++g_gaitRouterActors[formID].stablePivotFrameSerial;
            }

            const bool actionBypass = IsRotationActionBypassActive(actor, *motionProfile);
            const bool motionDbAuthoredRotation =
                SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor);
            const bool nativeProfileAuthoredRotation =
                HasRecentStablePivotNativeRotationClip(actor);
            const bool authoredRotation =
                motionDbAuthoredRotation || nativeProfileAuthoredRotation;
            const bool runtimeCarrier =
                SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(actor);

            SteeringIntentController::FrameCommand steering{};
            const bool steeringReadable = SteeringIntentController::GetFrameCommand(actor, steering);
            const bool steeringUsable = steeringReadable && steering.valid &&
                std::isfinite(steering.filteredErrorRadians) &&
                std::isfinite(steering.goalDistanceXY) &&
                steering.goalDistanceXY >= config.minGoalDistance;
            const float errorDegrees = steeringUsable
                ? WrapSignedDegrees(steering.filteredErrorRadians * kRadToDeg)
                : 0.0f;
            const float absError = std::abs(errorDegrees);

            bool parentActive = false;
            const bool parentRead = ReadStablePivotLocomotionActive(actor, profile, pivot, parentActive);

            StablePivotPhase phase = StablePivotPhase::kIdle;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                state.stablePivotCooldown = std::max(0.0f, state.stablePivotCooldown - dt);
                phase = state.stablePivotPhase;
            }

            if (actionBypass) {
                bool cancelled = false;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    if (state.stablePivotPhase != StablePivotPhase::kIdle) {
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                        cancelled = true;
                    } else {
                        state.stablePivotCandidateFrames = 0;
                    }
                }
                if (cancelled) {
                    SKSE::log::info(
                        "[StablePivot][R93_CANCEL] form={:08X} profile=\"{}\" reason=ACTION_BYPASS action=NATIVE_ACTION_WINS",
                        formID, profile);
                }
                return false;
            }

            if (phase == StablePivotPhase::kIdle) {
                bool request = false;
                std::uint32_t routeDegrees = 0;
                bool turnLeft = false;
                std::uint32_t candidateFrames = 0;
                float cooldown = 0.0f;

                float route180Threshold = pivot.turn180MinAngle;
                if (std::isfinite(config.authoredPivot180RouteMinAngle) &&
                    config.authoredPivot180RouteMinAngle > pivot.turn90MinAngle &&
                    config.authoredPivot180RouteMinAngle <= 180.0f) {
                    route180Threshold = config.authoredPivot180RouteMinAngle;
                }

                if (steeringUsable && absError >= route180Threshold) {
                    routeDegrees = 180u;
                } else if (config.authoredPivot90Enabled &&
                           steeringUsable && absError >= pivot.turn90MinAngle) {
                    routeDegrees = 90u;
                }

                if (routeDegrees != 0) {
                    const bool passive180 = routeDegrees >= 180 &&
                        !config.authoredPivot180PreemptLocomotion;
                    bool nativeTurnWindowActive = false;
                    bool nativeTurnStopDrain = false;
                    bool nativeTurn180Offered = false;
                    if (passive180) {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        const auto& state = g_gaitRouterActors[formID];
                        nativeTurnWindowActive = state.nativeStandingTurnPassthrough;
                        nativeTurnStopDrain = state.nativeStandingTurnStopDrain;
                        nativeTurn180Offered = state.nativeStandingTurnCannedOffered;
                    }

                    // R114: the 180 preference is subordinate to STOP_DRAIN as well as the
                    // native turn window. One transient carrier-free frame is not enough to
                    // dispatch 180 while the locomotion parent is still crossing state.
                    if (passive180 &&
                        (!nativeTurnWindowActive || nativeTurnStopDrain ||
                         runtimeCarrier || nativeTurn180Offered)) {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        state.stablePivotCandidateFrames = 0;
                        state.stablePivotRouteDegrees = 0;
                        return false;
                    }

                    const bool positiveMeansLeft = LowerCopy(pivot.positiveTurnDirection) != "right";
                    turnLeft = errorDegrees >= 0.0f ? positiveMeansLeft : !positiveMeansLeft;
                    if (passive180 && nativeTurnWindowActive) {
                        bool latchedLeft = false;
                        if (NativeStandingTurnLatchedSide(actor, latchedLeft)) turnLeft = latchedLeft;
                    }

                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    cooldown = state.stablePivotCooldown;
                    if (cooldown <= 0.0f) {
                        const bool sameCandidate =
                            state.stablePivotRouteDegrees == routeDegrees &&
                            state.stablePivotLeft == turnLeft;
                        if (sameCandidate) {
                            if (state.stablePivotCandidateFrames < 255) ++state.stablePivotCandidateFrames;
                        } else {
                            state.stablePivotRouteDegrees = routeDegrees;
                            state.stablePivotLeft = turnLeft;
                            state.stablePivotCandidateFrames = 1;
                        }
                        state.stablePivotRequestedErrorDegrees = errorDegrees;
                        candidateFrames = state.stablePivotCandidateFrames;
                        // Passive 180 is a preference offered at the first already-standing
                        // boundary. Waiting four frames lets Bethesda enter its 90 loop first,
                        // after which sending 180 would again become a competing intervention.
                        const std::uint32_t requiredFrames = passive180
                            ? 1u : config.authoredPivotConfirmFrames;
                        request = candidateFrames >= requiredFrames;
                    }
                } else {
                    // R110: this includes the native 90-degree band when route90 is disabled.
                    // Clear StablePivot candidacy completely so no candidate hold/moveStop/event
                    // can perturb Bethesda's own looping turn selection.
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    state.stablePivotCandidateFrames = 0;
                    state.stablePivotRouteDegrees = 0;
                }

                if (request && routeDegrees >= 180 &&
                    !config.authoredPivot180PreemptLocomotion) {
                    const auto& events = turnLeft ? pivot.turnLeft180Events : pivot.turnRight180Events;
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        // One 180 offer per R113 window. Mark before Notify so a rejection
                        // cannot be retried by another boundary in the same native turn.
                        state.nativeStandingTurnCannedOffered = true;
                        state.stablePivotCandidateFrames = 0;
                    }

                    std::string acceptedEvent;
                    const bool accepted = NotifyFirstAcceptedStablePivotEvent(
                        actor, events, acceptedEvent);
                    if (accepted) {
                        {
                            std::scoped_lock lock{ g_gaitRouterMutex };
                            auto& state = g_gaitRouterActors[formID];
                            state.stablePivotPhase = StablePivotPhase::kPending;
                            state.stablePivotElapsed = 0.0f;
                            state.stablePivotStandingFrames = 0;
                            state.stablePivotDispatchAttempts = 0;
                            state.stablePivotExitFrames = 0;
                            ++state.stablePivotRequests;
                            ++state.stablePivotAccepted;
                        }
                        SKSE::log::info(
                            "[StablePivot][R114_180_PASSIVE_DISPATCH] form={:08X} profile=\"{}\" error={:+.2f}deg direction={} event=\"{}\" parent[read={} active={}] runtimeCarrier={} action=ONE_OFFER_INSIDE_LATCHED_NATIVE_TURN_WINDOW",
                            formID, profile, errorDegrees, turnLeft ? "LEFT" : "RIGHT",
                            acceptedEvent, parentRead, parentActive, runtimeCarrier);
                        return true;
                    }

                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        ++state.stablePivotRequests;
                        ++state.stablePivotFailures;
                        state.stablePivotRouteDegrees = 0;
                        state.stablePivotCandidateFrames = 0;
                    }
                    SKSE::log::info(
                        "[StablePivot][R114_180_PASSIVE_REJECT] form={:08X} profile=\"{}\" error={:+.2f}deg direction={} action=NO_RETRY_THIS_WINDOW_LEAVE_BETHESDA_90_LOOP_UNTOUCHED",
                        formID, profile, errorDegrees, turnLeft ? "LEFT" : "RIGHT");
                    return false;
                }

                if (request) {
                    bool stopAccepted = true;
                    if (!parentRead || parentActive || runtimeCarrier) {
                        stopAccepted = actor->NotifyAnimationGraph(pivot.moveStopEvent.c_str());
                    }

                    if (stopAccepted) {
                        {
                            std::scoped_lock lock{ g_gaitRouterMutex };
                            auto& state = g_gaitRouterActors[formID];
                            state.stablePivotPhase = StablePivotPhase::kStopping;
                            state.stablePivotElapsed = 0.0f;
                            state.stablePivotStandingFrames = 0;
                            state.stablePivotDispatchAttempts = 0;
                            state.stablePivotExitFrames = 0;
                            ++state.stablePivotRequests;
                        }
                        SKSE::log::info(
                            "[StablePivot][R93_REQUEST] form={:08X} profile=\"{}\" error={:+.2f}deg route={} direction={} goalDist={:.1f} parent[read={} active={}] runtimeCarrier={} moveStop=\"{}\" accepted=true action=STOP_LOCOMOTION_FOR_AUTHORED_PIVOT",
                            formID, profile, errorDegrees, routeDegrees, turnLeft ? "LEFT" : "RIGHT",
                            steering.goalDistanceXY, parentRead, parentActive, runtimeCarrier,
                            pivot.moveStopEvent);
                        return true;
                    }

                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                    }
                    SKSE::log::warn(
                        "[StablePivot][R93_REQUEST_REJECTED] form={:08X} profile=\"{}\" error={:+.2f}deg route={} direction={} moveStop=\"{}\" action=COOLDOWN_THEN_REEVALUATE",
                        formID, profile, errorDegrees, routeDegrees, turnLeft ? "LEFT" : "RIGHT",
                        pivot.moveStopEvent);
                    return false;
                }

                return false;
            }

            if (phase == StablePivotPhase::kStopping) {
                bool standingReady = false;
                bool timeout = false;
                std::uint32_t standingFrames = 0;
                std::uint32_t routeDegrees = 0;
                bool turnLeft = false;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    state.stablePivotElapsed += dt;
                    const bool graphStanding = parentRead && !parentActive;
                    const bool standingWitness = !runtimeCarrier &&
                        (!config.authoredPivotRequireGraphInactive || graphStanding);
                    if (standingWitness) {
                        if (state.stablePivotStandingFrames < 255) ++state.stablePivotStandingFrames;
                    } else {
                        state.stablePivotStandingFrames = 0;
                    }
                    standingFrames = state.stablePivotStandingFrames;
                    routeDegrees = state.stablePivotRouteDegrees;
                    turnLeft = state.stablePivotLeft;
                    standingReady = standingFrames >= std::max<std::uint8_t>(1, pivot.standingConfirmFrames);
                    timeout = state.stablePivotElapsed >= std::max(0.05f, pivot.stopTimeoutSeconds);
                }

                if (standingReady) {
                    const std::vector<std::string>* events = nullptr;
                    if (routeDegrees >= 180) {
                        events = turnLeft ? &pivot.turnLeft180Events : &pivot.turnRight180Events;
                    } else {
                        events = turnLeft ? &pivot.turnLeft90Events : &pivot.turnRight90Events;
                    }

                    if (routeDegrees == 90) {
                        // R100: select the exact authored 90 point before the turn event enters
                        // the MT_TurnLeft/Right_Blend. This prevents the parent from choosing
                        // the 9-degree Slow alias from a stale/native TurnDelta value.
                        (void)ApplyStablePivotTurnParameter(actor, config, "PRE_DISPATCH", true);
                    }

                    std::string acceptedEvent;
                    const bool accepted = events &&
                        NotifyFirstAcceptedStablePivotEvent(actor, *events, acceptedEvent);
                    if (accepted) {
                        float physicalYaw = 0.0f;
                        const bool physicalYawRead = routeDegrees == 90 &&
                            CapsuleSlerp::TryGetPhysicalYawRadians(actor, physicalYaw);
                        {
                            std::scoped_lock lock{ g_gaitRouterMutex };
                            auto& state = g_gaitRouterActors[formID];
                            state.stablePivotPhase = StablePivotPhase::kPending;
                            state.stablePivotElapsed = 0.0f;
                            state.stablePivotDispatchAttempts = 0;
                            state.stablePivotExitFrames = 0;
                            state.stablePivot90PhysicalYawInitialized = physicalYawRead;
                            state.stablePivot90PhysicalYawStartRadians = physicalYawRead ? physicalYaw : 0.0f;
                            state.stablePivot90PhysicalYawMaxDegrees = 0.0f;
                            state.stablePivot90TurnStopIssued = false;
                            state.stablePivot90StallBreak = false;
                            state.stablePivot90TurnStopAttempts = 0;
                            state.stablePivot90ReacquireAttempts = 0;
                            ++state.stablePivotAccepted;
                        }
                        SKSE::log::info(
                            "[StablePivot][R103_DISPATCH] form={:08X} profile=\"{}\" event=\"{}\" route={} direction={} standingFrames={} physicalYaw[read={} start={:+.2f}deg] action=WAIT_FOR_NATIVE_ROTATION_WITNESS",
                            formID, profile, acceptedEvent, routeDegrees,
                            turnLeft ? "LEFT" : "RIGHT", standingFrames,
                            physicalYawRead, physicalYaw * kRadToDeg);
                        return true;
                    }

                    // R100: 180 is a dedicated canned transition and must not be abandoned
                    // after an arbitrary 8 render frames. Retry the exact latched 180 event for
                    // the entire configured stop/settle budget. Only the episode timeout may
                    // release it; there is no 180 -> 90 downgrade inside this stop episode.
                    if (routeDegrees >= 180 && !timeout) {
                        std::uint32_t attempt = 0;
                        float elapsed = 0.0f;
                        {
                            std::scoped_lock lock{ g_gaitRouterMutex };
                            auto& state = g_gaitRouterActors[formID];
                            attempt = ++state.stablePivotDispatchAttempts;
                            elapsed = state.stablePivotElapsed;
                        }
                        if (attempt <= 8 || (attempt % 8) == 0) {
                            SKSE::log::info(
                                "[StablePivot][R100_180_DISPATCH_RETRY] form={:08X} profile=\"{}\" route=180 direction={} attempt={} elapsed={:.3f}/{:.3f}s standingFrames={} action=KEEP_EXACT_180_UNTIL_STOP_TIMEOUT",
                                formID, profile, turnLeft ? "LEFT" : "RIGHT", attempt, elapsed,
                                std::max(0.05f, pivot.stopTimeoutSeconds), standingFrames);
                        }
                        return true;
                    }

                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                    }
                    if (routeDegrees >= 180) {
                        SKSE::log::warn(
                            "[StablePivot][R100_180_DISPATCH_TIMEOUT] form={:08X} profile=\"{}\" direction={} action=FAIL_OPEN_AFTER_FULL_STOP_BUDGET_NO_90_DOWNGRADE_INSIDE_EPISODE",
                            formID, profile, turnLeft ? "LEFT" : "RIGHT");
                    } else {
                        SKSE::log::warn(
                            "[StablePivot][R100_DISPATCH_REJECTED] form={:08X} profile=\"{}\" route={} direction={} action=COOLDOWN_THEN_REEVALUATE",
                            formID, profile, routeDegrees, turnLeft ? "LEFT" : "RIGHT");
                    }
                    return false;
                }

                if (timeout) {
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                    }
                    SKSE::log::warn(
                        "[StablePivot][R94_STOP_TIMEOUT] form={:08X} profile=\"{}\" parent[read={} active={}] runtimeCarrier={} requireGraphInactive={} action=FAIL_OPEN_REEVALUATE",
                        formID, profile, parentRead, parentActive, runtimeCarrier,
                        config.authoredPivotRequireGraphInactive);
                    return false;
                }
                return true;
            }

            if (phase == StablePivotPhase::kPending) {
                // R109: no TurnDelta/clip-rate/alias intervention. Wait for the FINAL composed
                // behavior output to contain a sane yaw delta in the latched direction.
                bool timeout = false;
                std::uint32_t pendingRouteDegrees = 0;
                bool pendingLeft = false;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    const auto& state = g_gaitRouterActors[formID];
                    pendingRouteDegrees = state.stablePivotRouteDegrees;
                    pendingLeft = state.stablePivotLeft;
                }
                float finalBehaviorPhysicalYaw = 0.0f;
                std::int32_t finalBehaviorOutputIndex = -1;
                const bool finalBehavior90Witness = pendingRouteDegrees == 90 &&
                    HasFinalBehaviorPivotYawWitness(
                        actor, pendingLeft, finalBehaviorPhysicalYaw, finalBehaviorOutputIndex);
                const bool pendingAuthoredRotation = pendingRouteDegrees == 90
                    ? finalBehavior90Witness
                    : authoredRotation;
                if (pendingAuthoredRotation) {
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        state.stablePivotPhase = StablePivotPhase::kActive;
                        state.stablePivotElapsed = 0.0f;
                        state.stablePivotExitFrames = 0;
                    }
                    if (pendingRouteDegrees == 90) {
                        SKSE::log::info(
                            "[StablePivot][R109_ACTIVE_FINAL_BEHAVIOR] form={:08X} profile=\"{}\" direction={} output={} physicalDelta={:+.4f}deg action=FINAL_COMPOSED_BEHAVIOR_ROOT_OWNS_90_YAW",
                            formID, profile, pendingLeft ? "LEFT" : "RIGHT",
                            finalBehaviorOutputIndex, finalBehaviorPhysicalYaw);
                    } else {
                        SKSE::log::info(
                            "[StablePivot][R95_ACTIVE] form={:08X} profile=\"{}\" witness={} action=PROFILE_DECLARED_NATIVE_ROTATIONCLIP_OWNS_YAW",
                            formID, profile,
                            motionDbAuthoredRotation ? "MOTIONDB" : "NATIVE_HKB_CLIP");
                    }
                    return true;
                }
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    state.stablePivotElapsed += dt;
                    timeout = state.stablePivotElapsed >=
                        std::max(0.05f, pivot.eventConfirmTimeoutSeconds);
                }
                if (timeout) {
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                    }
                    SKSE::log::warn(
                        "[StablePivot][R93_EVENT_TIMEOUT] form={:08X} profile=\"{}\" action=FAIL_OPEN_REEVALUATE",
                        formID, profile);
                    return false;
                }
                return true;
            }

            // ACTIVE. R103 preserves the R102 route split and enforces strict 90 completion:
            // - 180 is a native single-play canned turn and retains the R97/R100 lifecycle.
            // - 90 is a native LOOPING turn state. It must be stopped by turnStop when the
            //   real bhkCharacterController has physically rotated the requested amount.
            std::uint32_t activeRouteDegrees = 0;
            bool activeTurnLeft = false;
            float activeElapsed = 0.0f;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                state.stablePivotElapsed += dt;
                activeRouteDegrees = state.stablePivotRouteDegrees;
                activeTurnLeft = state.stablePivotLeft;
                activeElapsed = state.stablePivotElapsed;
            }

            float activeFinalBehaviorPhysicalYaw = 0.0f;
            std::int32_t activeFinalBehaviorOutputIndex = -1;
            const bool activeFinalBehavior90Witness = activeRouteDegrees == 90 &&
                HasFinalBehaviorPivotYawWitness(
                    actor, activeTurnLeft, activeFinalBehaviorPhysicalYaw, activeFinalBehaviorOutputIndex);
            const bool activeAuthoredRotation = activeRouteDegrees == 90
                ? activeFinalBehavior90Witness
                : authoredRotation;

            if (activeRouteDegrees == 90) {
                float currentPhysicalYaw = 0.0f;
                const bool physicalYawRead =
                    CapsuleSlerp::TryGetPhysicalYawRadians(actor, currentPhysicalYaw);
                float progressDegrees = 0.0f;
                float maxProgressDegrees = 0.0f;
                bool requestTurnStop = false;
                bool stallBreak = false;
                std::uint32_t stopAttempt = 0;

                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    if (physicalYawRead) {
                        if (!state.stablePivot90PhysicalYawInitialized) {
                            state.stablePivot90PhysicalYawInitialized = true;
                            state.stablePivot90PhysicalYawStartRadians = currentPhysicalYaw;
                        }
                        progressDegrees = std::abs(WrapSignedDegrees(
                            (currentPhysicalYaw - state.stablePivot90PhysicalYawStartRadians) * kRadToDeg));
                        state.stablePivot90PhysicalYawMaxDegrees = std::max(
                            state.stablePivot90PhysicalYawMaxDegrees, progressDegrees);
                    }
                    maxProgressDegrees = state.stablePivot90PhysicalYawMaxDegrees;

                    if (!state.stablePivot90TurnStopIssued) {
                        const bool reachedTarget =
                            maxProgressDegrees >= config.authoredPivot90StopDegrees;
                        const bool noPhysicalProgress =
                            activeElapsed >= config.authoredPivot90StallSeconds &&
                            maxProgressDegrees < config.authoredPivot90MinProgressDegrees;
                        if (reachedTarget || noPhysicalProgress) {
                            requestTurnStop = true;
                            stallBreak = noPhysicalProgress;
                            stopAttempt = ++state.stablePivot90TurnStopAttempts;
                        }
                    }
                }

                if (requestTurnStop) {
                    const bool accepted = !pivot.turnLoopStopEvent.empty() &&
                        actor->NotifyAnimationGraph(pivot.turnLoopStopEvent.c_str());
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        if (accepted) {
                            state.stablePivot90TurnStopIssued = true;
                            state.stablePivot90StallBreak = stallBreak;
                        }
                    }

                    if (stallBreak) {
                        SKSE::log::warn(
                            "[StablePivot][R103_90_STALL_BREAK] form={:08X} profile=\"{}\" elapsed={:.3f}s physicalProgress={:.2f}deg minRequired={:.2f}deg event=\"{}\" attempt={} accepted={} action=STOP_VISUAL_ONLY_LOOP_AND_RELEASE_TO_REPLAN",
                            formID, profile, activeElapsed, maxProgressDegrees,
                            config.authoredPivot90MinProgressDegrees, pivot.turnLoopStopEvent,
                            stopAttempt, accepted);
                    } else {
                        SKSE::log::info(
                            "[StablePivot][R103_90_TARGET_STOP] form={:08X} profile=\"{}\" physicalProgress={:.2f}deg target={:.2f}deg event=\"{}\" attempt={} accepted={} action=END_NATIVE_LOOP_AT_PHYSICAL_TARGET",
                            formID, profile, maxProgressDegrees,
                            config.authoredPivot90StopDegrees, pivot.turnLoopStopEvent,
                            stopAttempt, accepted);
                    }
                }
            }

            // R109: no child-clip reacquire/event resend. If the FINAL composed behavior
            // yaw disappears before turnStop, V13 immediately holds physical yaw and the
            // existing cleanup-confirm path below fails the episode cleanly. This prevents
            // animation-event churn and guarantees there is no ghost physical continuation.

            bool completed = false;
            bool timeout = false;
            bool failedPhysical90 = false;
            bool stallRecovery = false;
            bool clipLostBeforeStop = false;
            bool turnStopIssued = false;
            std::uint32_t exitFrames = 0;
            float finalPhysical90Progress = 0.0f;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                if (activeAuthoredRotation) {
                    state.stablePivotExitFrames = 0;
                } else if (state.stablePivotExitFrames < 255) {
                    ++state.stablePivotExitFrames;
                }
                exitFrames = state.stablePivotExitFrames;
                const bool nativeStateExited =
                    exitFrames >= std::max<std::uint8_t>(1, pivot.cleanupConfirmFrames);
                timeout = state.stablePivotElapsed >= std::max(0.25f, pivot.activeTimeoutSeconds);
                finalPhysical90Progress = state.stablePivot90PhysicalYawMaxDegrees;
                stallRecovery = state.stablePivot90StallBreak;
                turnStopIssued = state.stablePivot90TurnStopIssued;

                if (state.stablePivotRouteDegrees == 90) {
                    // R103: a looping 90 has no natural completion contract. The child
                    // disappearing before our accepted turnStop is NOT success; R102 falsely
                    // counted 5-22 degree partial turns as complete. Only an accepted turnStop
                    // followed by the native state exit may complete the pivot.
                    clipLostBeforeStop = nativeStateExited && !turnStopIssued;
                    completed = nativeStateExited && turnStopIssued;
                    failedPhysical90 = completed &&
                        (stallRecovery ||
                         finalPhysical90Progress < config.authoredPivot90MinProgressDegrees);
                } else {
                    completed = nativeStateExited;
                }

                if (clipLostBeforeStop) {
                    ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                } else if (completed) {
                    if (failedPhysical90) {
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                    } else {
                        ResetStablePivotLocked(state, pivot.retryCooldownSeconds, true, false);
                    }
                } else if (timeout) {
                    ResetStablePivotLocked(state, pivot.retryCooldownSeconds, false, true);
                }
            }
            if (clipLostBeforeStop) {
                SKSE::log::warn(
                    "[StablePivot][R103_90_CLIP_LOST_BEFORE_STOP] form={:08X} profile=\"{}\" exitFrames={} physicalProgress={:.2f}deg turnStopIssued=false action=FAIL_REPLAN_DO_NOT_COUNT_PARTIAL_LOOP_AS_SUCCESS",
                    formID, profile, exitFrames, finalPhysical90Progress);
                return false;
            }
            if (completed) {
                if (failedPhysical90) {
                    SKSE::log::warn(
                        "[StablePivot][R103_90_ABORT_AFTER_STOP] form={:08X} profile=\"{}\" exitFrames={} physicalProgress={:.2f}deg stallRecovery={} action=REPLAN_WITHOUT_SUCCESS",
                        formID, profile, exitFrames, finalPhysical90Progress, stallRecovery);
                } else {
                    SKSE::log::info(
                        "[StablePivot][R103_COMPLETE] form={:08X} profile=\"{}\" route={} exitFrames={} physical90={:.2f}deg turnStopIssued={} action=REPLAN_LOCOMOTION_FROM_CURRENT_TRAJECTORY",
                        formID, profile, activeRouteDegrees, exitFrames,
                        finalPhysical90Progress, turnStopIssued);
                }
                return false;
            }
            if (timeout) {
                SKSE::log::warn(
                    "[StablePivot][R103_ACTIVE_TIMEOUT] form={:08X} profile=\"{}\" route={} physical90={:.2f}deg turnStopIssued={} action=FAIL_OPEN_REEVALUATE",
                    formID, profile, activeRouteDegrees, finalPhysical90Progress, turnStopIssued);
                return false;
            }
            return true;
        }

        [[nodiscard]] bool WriteStableSampleSelection(
            RE::Actor* actor,
            const StableSampleSelectorConfig& config,
            SpeedIntentTranslator::Family selectedFamily,
            std::int32_t selectedSector,
            const char* marker,
            bool emitCorrectionTelemetry)
        {
            if (!actor || selectedFamily == SpeedIntentTranslator::Family::kNone) return false;
            const float selectedSpeed = StableSampleForFamily(selectedFamily, config);
            const float selectedDirection = StableDirectionValueForSector(selectedSector, config);
            float priorSpeed = 0.0f;
            float priorDirection = 0.0f;
            const bool priorSpeedRead = actor->GetGraphVariableFloat(config.speedVariable.c_str(), priorSpeed) &&
                std::isfinite(priorSpeed);
            const bool priorDirectionRead = actor->GetGraphVariableFloat(config.directionVariable.c_str(), priorDirection) &&
                std::isfinite(priorDirection);
            const bool speedWrote = actor->SetGraphVariableFloat(config.speedVariable.c_str(), selectedSpeed);
            const bool directionWrote = actor->SetGraphVariableFloat(config.directionVariable.c_str(), selectedDirection);

            if (emitCorrectionTelemetry) {
                const bool corrected =
                    (priorSpeedRead && std::abs(priorSpeed - selectedSpeed) > 0.001f) ||
                    (priorDirectionRead && std::abs(priorDirection - selectedDirection) > 0.0005f);
                if (corrected) {
                    bool emit = false;
                    std::uint64_t corrections = 0;
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[actor->GetFormID()];
                        corrections = ++state.stableFinalReassertCorrections;
                        emit = corrections <= 24 || (corrections % 120) == 0;
                    }
                    if (emit) {
                        SKSE::log::info(
                            "[StableSample][{}] form={:08X} corrections={} speed[prior={:.3f} exact={:.3f}] direction[prior={:.4f} exact={:.4f}] action=REASSERT_PRE_NATIVE_DECISION_NO_STATE_CHANGE",
                            marker, actor->GetFormID(), corrections,
                            priorSpeedRead ? priorSpeed : 0.0f, selectedSpeed,
                            priorDirectionRead ? priorDirection : 0.0f, selectedDirection);
                    }
                }
            }
            return speedWrote || directionWrote;
        }

        // R93 decision phase: execute before native Character::Update so the Behavior sees the
        // authored sample we selected when it chooses/advances child generators. This is the
        // critical difference from R92, whose post-native-only write could not prevent Slow/Fast
        // or stale directional children from already entering the graph.
        [[nodiscard]] bool UpdateStableSampleSelectorPreNative(RE::Actor* actor, float deltaTime)
        {
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) return false;
            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = StableSampleSelectorConfigForProfile(profile);
            if (!config || !config->enabled || !SpeedIntentTranslator::SupportsActor(actor)) return false;

            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!motionProfile || IsRotationActionBypassActive(actor, *motionProfile)) return false;

            // R107: snapshot the lifecycle before advancing it. A timeout/completion can reset
            // STOPPING/PENDING/ACTIVE -> IDLE inside UpdateStableAuthoredPivotPreNative; without
            // this edge witness the selector immediately chose a new direction in that SAME
            // Character::Update, which is visible as an animation jerk. Candidate confirmation
            // is also a discrete-pivot decision and must not pre-switch locomotion sectors.
            StablePivotPhase pivotPhaseBefore = StablePivotPhase::kIdle;
            bool pivotCandidateBefore = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto& state = g_gaitRouterActors[formID];
                pivotPhaseBefore = state.stablePivotPhase;
                pivotCandidateBefore = state.stablePivotPhase == StablePivotPhase::kIdle &&
                    state.stablePivotRouteDegrees > 0 && state.stablePivotCandidateFrames > 0;
            }

            (void)UpdateStableAuthoredPivotPreNative(actor, *config, deltaTime);

            StablePivotPhase pivotPhase = StablePivotPhase::kIdle;
            bool pivotCandidateAfter = false;
            bool heldInitialized = false;
            SpeedIntentTranslator::Family heldFamily = SpeedIntentTranslator::Family::kNone;
            std::int32_t heldSector = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                pivotPhase = state.stablePivotPhase;
                pivotCandidateAfter = state.stablePivotPhase == StablePivotPhase::kIdle &&
                    state.stablePivotRouteDegrees > 0 && state.stablePivotCandidateFrames > 0;
                heldInitialized = state.stableSampleInitialized;
                heldFamily = state.stableSampleFamily;
                heldSector = state.stableDirectionSector;
            }

            if (pivotPhase == StablePivotPhase::kPending || pivotPhase == StablePivotPhase::kActive) {
                return false;
            }

            // R114: candidate/STOP_DRAIN/start-quarantine freeze the exact outgoing
            // sample. The actual TURN phase is fully write-free for Bethesda.
            if (pivotPhase == StablePivotPhase::kIdle) {
                const auto nativeGate = NativeStandingTurnGateForActor(actor);
                if (nativeGate.active && !nativeGate.stopDrain) {
                    return false;
                }
                const bool holdForNativeTransition =
                    nativeGate.stopDrain ||
                    nativeGate.candidateFrames > 0 ||
                    nativeGate.startQuarantineRemaining > 0.0f;
                if (holdForNativeTransition) {
                    if (!heldInitialized) return false;
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        state.stablePendingDirectionSector = -1;
                        state.stablePendingDirectionFrames = 0;
                    }
                    const char* marker = nativeGate.stopDrain
                        ? "R114_STOP_DRAIN_HOLD"
                        : (nativeGate.candidateFrames > 0
                            ? "R114_TURN_COMMIT_INTERLOCK"
                            : "R114_START_QUARANTINE_HOLD");
                    return WriteStableSampleSelection(
                        actor, *config, heldFamily, heldSector, marker, false);
                }
            }

            // During STOPPING freeze the last physical choice. Do not let a changing target
            // select another locomotion child while the parent is exiting.
            if (pivotPhase == StablePivotPhase::kStopping) {
                if (!heldInitialized) return false;
                return WriteStableSampleSelection(
                    actor, *config, heldFamily, heldSector, "R93_PRE_NATIVE_STOPPING_HOLD", false);
            }

            const bool pivotExitedThisFrame =
                pivotPhaseBefore != StablePivotPhase::kIdle && pivotPhase == StablePivotPhase::kIdle;
            const bool pivotCandidateHold =
                pivotPhase == StablePivotPhase::kIdle && (pivotCandidateBefore || pivotCandidateAfter);
            if (heldInitialized && (pivotExitedThisFrame || pivotCandidateHold)) {
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    state.stablePendingDirectionSector = -1;
                    state.stablePendingDirectionFrames = 0;
                }
                const char* marker = pivotExitedThisFrame
                    ? "R107_POST_PIVOT_SELECTION_HOLD"
                    : "R107_PIVOT_CANDIDATE_HOLD";
                SKSE::log::info(
                    "[StableSample][{}] form={:08X} profile=\"{}\" priorPhase={} currentPhase={} candidate[before={} after={}] selected={} sector={} action=KEEP_OUTGOING_SAMPLE_NO_DIRECTION_SWITCH_THIS_FRAME",
                    marker, formID, profile, StablePivotPhaseName(pivotPhaseBefore),
                    StablePivotPhaseName(pivotPhase), pivotCandidateBefore, pivotCandidateAfter,
                    FamilyName(heldFamily), heldSector);
                return WriteStableSampleSelection(actor, *config, heldFamily, heldSector, marker, false);
            }

            if (SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor)) {
                return false;
            }

            SteeringIntentController::FrameCommand steering{};
            const bool steeringReadable = SteeringIntentController::GetFrameCommand(actor, steering);
            const bool steeringUsable = steeringReadable && steering.valid &&
                std::isfinite(steering.filteredErrorRadians) &&
                std::isfinite(steering.goalDistanceXY) &&
                steering.goalDistanceXY >= config->minGoalDistance;

            const auto semanticFamily = SpeedIntentTranslator::LatchedFamily(formID);
            const float steeringErrorDeg = steeringUsable
                ? WrapSignedDegrees(steering.filteredErrorRadians * kRadToDeg)
                : 0.0f;
            const auto desiredSector = steeringUsable
                ? StableDirectionSectorForError(steeringErrorDeg, *config)
                : 0;
            const float dt = std::isfinite(deltaTime) && deltaTime > 0.0f && deltaTime <= 0.100f
                ? deltaTime : (1.0f / 60.0f);

            SpeedIntentTranslator::Family selectedFamily = SpeedIntentTranslator::Family::kNone;
            std::int32_t selectedSector = 0;
            bool familyChanged = false;
            bool directionChanged = false;
            bool axisDeferred = false;
            bool continuityHold = false;
            std::uint64_t frame = 0;
            std::uint64_t writes = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                ++state.stableSampleFrames;
                frame = state.stableSampleFrames;
                state.stableAxisSwitchCooldown = std::max(0.0f, state.stableAxisSwitchCooldown - dt);

                if (!state.stableSampleInitialized) {
                    if (!steeringUsable || semanticFamily == SpeedIntentTranslator::Family::kNone) {
                        return false;
                    }
                    state.stableSampleInitialized = true;
                    state.stableSampleFamily = semanticFamily;
                    state.stableDirectionSector = desiredSector;
                    state.stablePendingDirectionSector = -1;
                    state.stablePendingDirectionFrames = 0;
                } else if (steeringUsable) {
                    const float sectorWidthDeg = 360.0f /
                        static_cast<float>(std::max<std::uint32_t>(1, config->directionSectors));
                    const float currentCenterDeg = StableSteeringCenterDegreesForSector(
                        state.stableDirectionSector, *config);
                    const float errorFromCurrentCenter = std::abs(WrapSignedDegrees(
                        steeringErrorDeg - currentCenterDeg));
                    const bool leftCurrentEnvelope = desiredSector != state.stableDirectionSector &&
                        errorFromCurrentCenter > (sectorWidthDeg * 0.5f + config->directionSwitchHysteresisDegrees);

                    if (leftCurrentEnvelope) {
                        if (state.stablePendingDirectionSector == desiredSector) {
                            if (state.stablePendingDirectionFrames < 255) ++state.stablePendingDirectionFrames;
                        } else {
                            state.stablePendingDirectionSector = desiredSector;
                            state.stablePendingDirectionFrames = 1;
                        }
                    } else {
                        state.stablePendingDirectionSector = -1;
                        state.stablePendingDirectionFrames = 0;
                    }

                    const bool directionReady = state.stablePendingDirectionSector >= 0 &&
                        state.stablePendingDirectionFrames >= config->directionConfirmFrames;
                    if (directionReady && state.stableAxisSwitchCooldown <= 0.0f) {
                        state.stableDirectionSector = state.stablePendingDirectionSector;
                        state.stablePendingDirectionSector = -1;
                        state.stablePendingDirectionFrames = 0;
                        state.stableAxisSwitchCooldown = config->axisSwitchCooldownSeconds;
                        ++state.stableDirectionSwitches;
                        directionChanged = true;
                    }

                    if (semanticFamily != SpeedIntentTranslator::Family::kNone &&
                        semanticFamily != state.stableSampleFamily) {
                        if (!directionChanged && state.stableAxisSwitchCooldown <= 0.0f) {
                            state.stableSampleFamily = semanticFamily;
                            state.stableAxisSwitchCooldown = config->axisSwitchCooldownSeconds;
                            ++state.stableFamilySwitches;
                            familyChanged = true;
                        } else {
                            ++state.stableAxisDeferrals;
                            axisDeferred = true;
                        }
                    }
                } else {
                    state.stablePendingDirectionSector = -1;
                    state.stablePendingDirectionFrames = 0;
                    continuityHold = true;
                }

                selectedFamily = state.stableSampleFamily;
                selectedSector = state.stableDirectionSector;
                writes = state.stableSampleWrites;
            }

            if (selectedFamily == SpeedIntentTranslator::Family::kNone) return false;
            const bool wrote = WriteStableSampleSelection(
                actor, *config, selectedFamily, selectedSector, "R93_PRE_NATIVE", false);

            bool emit = false;
            if (wrote) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                writes = ++state.stableSampleWrites;
                emit = writes <= 12 || familyChanged || directionChanged || axisDeferred ||
                    (continuityHold && writes <= 30) ||
                    (config->telemetryEveryFrames > 0 && writes % config->telemetryEveryFrames == 0);
            }
            if (emit) {
                SKSE::log::info(
                    "[StableSample][R93_PRE_NATIVE] form={:08X} profile=\"{}\" frame={} semantic={} selected={} sector={}/{} steering[usable={} error={:+.2f}deg goalDist={:.1f}] changed[family={} direction={}] axisDeferred={} continuityHold={} authority=BEFORE_NATIVE_GRAPH_SELECTION expectedPhysicalContributors=ONE_OR_TRANSITION_TWO",
                    formID, profile, frame, FamilyName(semanticFamily), FamilyName(selectedFamily),
                    selectedSector, config->directionSectors, steeringUsable, steeringErrorDeg,
                    steeringReadable && std::isfinite(steering.goalDistanceXY) ? steering.goalDistanceXY : 0.0f,
                    familyChanged, directionChanged, axisDeferred, continuityHold);
            }
            return wrote;
        }

        // Final boundary is now a pure reassertion. It may correct a native writer that ran
        // after our pre-native decision, but it is forbidden to advance hysteresis/cooldowns,
        // change family/sector, or make a second decision in the same frame.
        [[nodiscard]] bool ReassertStableSampleSelectorFinal(RE::Actor* actor)
        {
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) return false;
            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = StableSampleSelectorConfigForProfile(profile);
            if (!config || !config->enabled || !SpeedIntentTranslator::SupportsActor(actor)) return false;

            const auto phase = StablePivotPhaseForActor(formID);
            if (phase == StablePivotPhase::kPending || phase == StablePivotPhase::kActive) return false;
            if (phase == StablePivotPhase::kIdle) {
                const auto nativeGate = NativeStandingTurnGateForActor(actor);
                if (nativeGate.active && !nativeGate.stopDrain) return false;
            }

            SpeedIntentTranslator::Family selectedFamily = SpeedIntentTranslator::Family::kNone;
            std::int32_t selectedSector = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto it = g_gaitRouterActors.find(formID);
                if (it == g_gaitRouterActors.end() || !it->second.stableSampleInitialized) return false;
                selectedFamily = it->second.stableSampleFamily;
                selectedSector = it->second.stableDirectionSector;
            }
            return WriteStableSampleSelection(
                actor, *config, selectedFamily, selectedSector, "R93_FINAL_REASSERT", true);
        }

        void ApplyProfileDirectionIntentFinal(RE::Actor* actor)
        {
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) {
                return;
            }
            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = GaitRouterConfigForProfile(profile);
            if (!config || !config->enabled || !config->directionIntent.enabled) {
                return;
            }
            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!motionProfile || IsRotationActionBypassActive(actor, *motionProfile) ||
                SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor)) {
                return;
            }
            SteeringIntentController::FrameCommand command{};
            if (!SteeringIntentController::GetFrameCommand(actor, command) ||
                !command.valid || !std::isfinite(command.filteredErrorRadians)) {
                return;
            }

            const float span = config->directionIntent.cycleMax - config->directionIntent.cycleMin;
            const float sign = config->directionIntent.positiveYawIncreases ? 1.0f : -1.0f;
            float desired = config->directionIntent.forwardValue +
                sign * (command.filteredErrorRadians / kTwoPi) * span;
            while (desired < config->directionIntent.cycleMin) desired += span;
            while (desired >= config->directionIntent.cycleMax) desired -= span;

            float prior = 0.0f;
            const bool priorRead = actor->GetGraphVariableFloat(
                config->directionIntent.variable.c_str(), prior) && std::isfinite(prior);
            const bool wrote = actor->SetGraphVariableFloat(
                config->directionIntent.variable.c_str(), desired);

            std::uint64_t writes = 0;
            bool logNow = false;
            if (wrote) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                writes = ++state.directionIntentWrites;
                const bool first = writes <= 8;
                const bool periodic =
                    (writes % std::max<std::uint32_t>(1, config->directionIntent.telemetryEveryFrames)) == 0;
                float delta = 0.0f;
                if (priorRead) {
                    delta = std::fmod(std::abs(desired - prior), span);
                    if (delta < 0.0f) delta += span;
                    delta = std::min(delta, span - delta);
                }
                logNow = first || periodic || (priorRead && delta >= span * 0.20f);
                state.lastDirectionIntentValue = desired;
            }
            if (wrote && logNow) {
                SKSE::log::info(
                    "[GaitRouter][R83_DIRECTION_FINAL] form={:08X} profile=\"{}\" phase=POST_NATIVE_PRE_HAVOK steeringError={:+.2f}deg graph[variable=\"{}\" priorRead={} nativePrior={:.4f} desired={:.4f} cycle=[{:.3f},{:.3f})] writes={} action=FINAL_PROFILE_DIRECTION_INPUT_ROOT_YAW_UNCHANGED",
                    formID, profile, command.filteredErrorRadians * kRadToDeg,
                    config->directionIntent.variable, priorRead, prior, desired,
                    config->directionIntent.cycleMin, config->directionIntent.cycleMax, writes);
            }
        }

        void ApplyGaitGraphRouter(RE::Actor* actor, float deltaTime)
        {
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) {
                return;
            }

            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = GaitRouterConfigForProfile(profile);
            if (!config || !config->enabled) {
                return;
            }

            // R83: DirectionIntent is deliberately NOT written here. Character::Update
            // contains native Direction writers/damping after this boundary; R82 proved that
            // a pre-native write simply fought those writers and produced blendspace tremor.
            // The final profile direction command is applied in ModifyAnimationUpdateData
            // after native graph updates and immediately before Havok evaluation.

            // R81: some behaviors expose a single continuous locomotion parent and NO
            // discrete walk/trot/run entry events. They still need a graph event to enter
            // that parent. The old absolute planar lock intentionally forbids capsule
            // translation before animation ownership, so relying on native pre-animation
            // movement can deadlock such a graph forever.
            //
            // ParentOnly solves only that bootstrap: a profile-declared graph float says
            // "movement is requested", and the profile-declared ParentStartEvent wakes the
            // behavior parent. It writes NO Speed/Direction/localTime/root/yaw and grants NO
            // translation authority. MotionDB must still certify animation-driven root after
            // the parent actually becomes active.
            if (config->parentOnly) {
                bool active = false;
                bool activeRead = true;
                if (!config->activeVariable.empty()) {
                    std::int32_t activeValue = 0;
                    activeRead = actor->GetGraphVariableInt(
                        config->activeVariable.c_str(), activeValue);
                    active = activeRead && activeValue == config->activeValue;
                }

                float intentValue = 0.0f;
                bool intentRead = false;
                bool intentActive = false;
                if (config->parentIntentSource == ParentIntentSource::kSteering) {
                    SteeringIntentController::FrameCommand command{};
                    intentRead = SteeringIntentController::GetFrameCommand(actor, command);
                    // R84: in Steering mode the threshold is world-space goal distance.
                    // A valid facing target alone is not necessarily a movement request.
                    intentValue = intentRead && std::isfinite(command.goalDistanceXY)
                        ? command.goalDistanceXY
                        : 0.0f;
                    intentActive = intentRead && command.valid &&
                        intentValue >= config->parentIntentThreshold;
                } else {
                    intentRead = actor->GetGraphVariableFloat(
                        config->parentIntentVariable.c_str(), intentValue) &&
                        std::isfinite(intentValue);
                    intentActive = intentRead && std::abs(intentValue) >= config->parentIntentThreshold;
                }

                const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
                bool actionBypass = false;
                if (motionProfile) {
                    for (const auto& variable : motionProfile->rotation.actionBypassVariables) {
                        if (variable.empty()) continue;
                        bool value = false;
                        if (actor->GetGraphVariableBool(variable.c_str(), value) && value) {
                            actionBypass = true;
                            break;
                        }
                    }
                }

                const bool runtimeCarrier =
                    SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(actor);
                const bool authoredRotation =
                    SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor);

                // R91: once locomotion produced a real physical RuntimeYaw carrier, the
                // graph is not allowed to reinterpret a short parent-active/false pulse as
                // a brand-new locomotion request. Those pulses are normal during authored
                // transitions and were causing repeated moveStart restarts in the R90 log.
                constexpr std::uint32_t kPostCarrierParentDropConfirmFrames = 8;
                bool postCarrierContinuityHold = false;
                bool logDropGrace = false;
                std::uint32_t missingCarrierFrames = 0;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    if (runtimeCarrier) {
                        state.parentOnlyPhysicalReadySeen = true;
                        state.parentOnlyPostCarrierMissingFrames = 0;
                    } else if (state.parentOnlyPhysicalReadySeen) {
                        // Treat carrier loss itself as the continuity signal. The graph's
                        // active flag remains advisory, exactly as in R84: after the grace
                        // expires, a false-active parent cannot deadlock re-entry forever.
                        if (state.parentOnlyPostCarrierMissingFrames < 255) {
                            ++state.parentOnlyPostCarrierMissingFrames;
                        }
                        missingCarrierFrames = state.parentOnlyPostCarrierMissingFrames;
                        postCarrierContinuityHold =
                            missingCarrierFrames < kPostCarrierParentDropConfirmFrames;
                        if (postCarrierContinuityHold && missingCarrierFrames == 1) {
                            ++state.parentOnlyDropGraceSuppressions;
                            logDropGrace = true;
                        }
                    }
                }
                if (logDropGrace) {
                    SKSE::log::info(
                        "[GaitRouter][R91_PARENT_DROP_GRACE] form={:08X} profile=\"{}\" graphActive={} runtimeCarrier=false intent={:.3f} graceFrames={}/{} action=KEEP_EXISTING_LOCOMOTION_CONTINUITY_NO_MOVESTART",
                        formID, profile, active, intentValue, missingCarrierFrames, kPostCarrierParentDropConfirmFrames);
                }

                // R114: interlock parent start from the FIRST large-heading candidate
                // frame, then keep it blocked through STOP_DRAIN/TURN and a short post-turn
                // quarantine. R113 allowed moveStart and moveStop to collide ~63 ms apart.
                NativeStandingTurnGateSnapshot nativeTurnGate{};
                bool nativeTurnStartInterlock = false;
                if (const auto stableSelector = StableSampleSelectorConfigForProfile(profile);
                    stableSelector && stableSelector->enabled) {
                    nativeTurnGate = NativeStandingTurnGateForActor(actor);
                    nativeTurnStartInterlock = ShouldInterlockNativeStandingTurnStart(
                        actor, *stableSelector);
                }

                if (!nativeTurnGate.active &&
                    (nativeTurnGate.candidateFrames == 1 ||
                     nativeTurnGate.startQuarantineRemaining > 0.15f)) {
                    SKSE::log::info(
                        "[GaitRouter][R114_TURN_COMMIT_INTERLOCK] form={:08X} profile=\"{}\" candidateFrames={} startQuarantine={:.3f}s graphActive={} runtimeCarrier={} intent={:.1f} action=BLOCK_MOVESTART_BEFORE_STATE_COLLISION",
                        formID, profile, nativeTurnGate.candidateFrames,
                        nativeTurnGate.startQuarantineRemaining, active, runtimeCarrier, intentValue);
                }

                // Before the first physical carrier, retain R84's false-active bootstrap
                // behavior. After physical ownership has existed, transient graph drops
                // must survive the grace window before a new parent entry can be requested.
                const bool safeToRequest =
                    intentActive && !actionBypass && !runtimeCarrier && !authoredRotation &&
                    !postCarrierContinuityHold && !nativeTurnStartInterlock;

                std::string eventToSend;
                std::uint32_t attemptNumber = 0;
                bool exhaustedBeforeSend = false;
                bool logReadFailure = false;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];

                    if (!activeRead) {
                        if (!state.activeReadFailureLogged) {
                            state.activeReadFailureLogged = true;
                            logReadFailure = true;
                        }
                    } else {
                        state.activeReadFailureLogged = false;
                    }

                    if (!safeToRequest || !intentRead) {
                        state.parentOnlyIntentFrames = 0;
                        state.parentOnlyAttempts = 0;
                        state.parentOnlyRetryRemaining = 0.0f;
                        state.parentOnlyExhaustedEpisode = false;
                    } else {
                        if (state.parentOnlyIntentFrames < 255) {
                            ++state.parentOnlyIntentFrames;
                        }
                        if (state.parentOnlyRetryRemaining > 0.0f) {
                            state.parentOnlyRetryRemaining = std::max(
                                0.0f,
                                state.parentOnlyRetryRemaining - std::max(deltaTime, 0.0f));
                        }

                        if (state.parentOnlyIntentFrames >= config->parentIntentConfirmFrames &&
                            state.parentOnlyRetryRemaining <= 0.0f &&
                            !state.parentOnlyExhaustedEpisode) {
                            if (state.parentOnlyAttempts < config->maxAttempts) {
                                attemptNumber = ++state.parentOnlyAttempts;
                                ++state.parentOnlyRequests;
                                eventToSend = config->parentStartEvent;
                            } else if (config->parentPersistentWhileIntent) {
                                state.parentOnlyAttempts = 0;
                                state.parentOnlyRetryRemaining = std::max(config->retrySeconds, 0.25f);
                                ++state.parentOnlyExhausted;
                                exhaustedBeforeSend = true;
                            } else {
                                state.parentOnlyExhaustedEpisode = true;
                                ++state.parentOnlyExhausted;
                                exhaustedBeforeSend = true;
                            }
                        }
                    }
                }

                if (logReadFailure) {
                    SKSE::log::warn(
                        "[GaitRouter][R84_PARENT_ACTIVE_READ_FAIL] form={:08X} profile=\"{}\" activeVariable=\"{}\" intentSource={} intentVariable=\"{}\" action=ADVISORY_ONLY_PHYSICAL_CARRIER_REMAINS_AUTHORITY",
                        formID, profile, config->activeVariable, ParentIntentSourceName(config->parentIntentSource),
                        config->parentIntentVariable);
                }
                if (exhaustedBeforeSend) {
                    SKSE::log::warn(
                        "[GaitRouter][R84_PARENT_RETRY_CYCLE] form={:08X} profile=\"{}\" intent={:.3f} threshold={:.3f} event=\"{}\" attempts={} action=PROFILE_POLICY_PERSIST_OR_WAIT",
                        formID, profile, intentValue, config->parentIntentThreshold,
                        config->parentStartEvent, config->maxAttempts);
                }

                if (!eventToSend.empty()) {
                    const bool accepted = actor->NotifyAnimationGraph(eventToSend.c_str());
                    bool exhaustedNow = false;
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        if (accepted) {
                            ++state.parentOnlyAccepted;
                        } else {
                            ++state.parentOnlyRejected;
                        }
                        state.parentOnlyRetryRemaining = config->retrySeconds;
                        if (state.parentOnlyAttempts >= config->maxAttempts) {
                            if (!config->parentPersistentWhileIntent) {
                                state.parentOnlyExhaustedEpisode = true;
                            }
                            ++state.parentOnlyExhausted;
                            exhaustedNow = true;
                        }
                    }

                    SKSE::log::info(
                        "[GaitRouter][R84_PARENT_ONLY_ENTRY] form={:08X} profile=\"{}\" graphActive={} intent[source={} variable=\"{}\" value={:.3f} threshold={:.3f} confirmFrames={}] actionBypass={} runtimeCarrier={} authoredRotation={} event=\"{}\" attempt={}/{} accepted={} exhausted={} action=WAKE_BEHAVIOR_PARENT_UNTIL_PHYSICAL_RUNTIMEYAW_READY_NO_ROOT_NO_SPEED_NO_YAW",
                        formID, profile, active, ParentIntentSourceName(config->parentIntentSource),
                        config->parentIntentVariable, intentValue,
                        config->parentIntentThreshold, config->parentIntentConfirmFrames,
                        actionBypass, runtimeCarrier, authoredRotation,
                        eventToSend, attemptNumber, config->maxAttempts,
                        accepted, exhaustedNow);
                }
                return;
            }

            // Discrete family routing retains the exact pre-R81 contract.
            if (!SpeedIntentTranslator::SupportsActor(actor)) {
                return;
            }

            bool active = true;
            if (!config->activeVariable.empty()) {
                std::int32_t activeValue = 0;
                const bool activeRead = actor->GetGraphVariableInt(config->activeVariable.c_str(), activeValue);
                if (!activeRead) {
                    bool emit = false;
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[formID];
                        if (!state.activeReadFailureLogged) {
                            state.activeReadFailureLogged = true;
                            emit = true;
                        }
                    }
                    if (emit) {
                        SKSE::log::warn(
                            "[GaitRouter][ACTIVE_READ_FAIL] form={:08X} profile=\"{}\" variable=\"{}\" action=FAIL_CLOSED",
                            formID, profile, config->activeVariable);
                    }
                    return;
                }
                active = activeValue == config->activeValue;
            }

            std::int32_t routeValue = 0;
            bool routeRead = true;
            if (!config->routeVariable.empty()) {
                routeRead = actor->GetGraphVariableInt(config->routeVariable.c_str(), routeValue);
            } else {
                const auto first = std::find_if(config->routes.begin(), config->routes.end(), [](const auto& route) {
                    return route.configured;
                });
                if (first == config->routes.end()) return;
                routeValue = first->value;
            }

            if (!routeRead) {
                bool emit = false;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    if (!state.routeReadFailureLogged) {
                        state.routeReadFailureLogged = true;
                        emit = true;
                    }
                }
                if (emit) {
                    SKSE::log::warn(
                        "[GaitRouter][ROUTE_READ_FAIL] form={:08X} profile=\"{}\" variable=\"{}\" action=FAIL_CLOSED",
                        formID, profile, config->routeVariable);
                }
                return;
            }

            const auto* route = FindGaitRoute(*config, routeValue);
            if (!route) {
                return;
            }

            const auto family = SpeedIntentTranslator::LatchedFamily(formID);
            const auto revision = SpeedIntentTranslator::Revision(formID);
            const auto& desiredEvent = EventForFamily(*route, family);

            const bool runtimeLocomotionCarrier =
                SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(actor);
            const bool authoredRotationObserved =
                SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(actor);
            SteeringIntentController::FrameCommand steeringCommand{};
            const bool steeringValid =
                SteeringIntentController::GetFrameCommand(actor, steeringCommand) &&
                steeringCommand.valid;

            bool actionBypass = false;
            if (const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor)) {
                for (const auto& variable : motionProfile->rotation.actionBypassVariables) {
                    if (variable.empty()) {
                        continue;
                    }
                    bool value = false;
                    if (actor->GetGraphVariableBool(variable.c_str(), value) && value) {
                        actionBypass = true;
                        break;
                    }
                }
            }

            // R53: R50/R51 deliberately preserve native Speed while a RUN semantic
            // request is still carried by WALK/TROT runtime root. That is safe for pose
            // cadence, but it also means the nested behavior can fall back after an
            // earlier successful runStart and remain there because the semantic revision
            // did not change. Detect only the profile-configured distinct RUN event lane.
            // No species name, clip name, Speed value, or parent wake is authored here.
            std::uint8_t runtimeFamilyMask = 0;
            const bool runCarrierMismatch = RunSpeedCarrierMismatch(actor, runtimeFamilyMask);
            const bool runCarrierPresent =
                family == SpeedIntentTranslator::Family::kRunForward &&
                (runtimeFamilyMask & kRuntimeFamilyRunMask) != 0;
            const bool runEventDistinct =
                family == SpeedIntentTranslator::Family::kRunForward &&
                !desiredEvent.empty() &&
                desiredEvent != route->walkEvent &&
                desiredEvent != route->trotEvent;

            // R45 action-exit witness remains unchanged: a same/previous-frame carrier may be
            // stale overlap from the outgoing native action, so it is not a blocker here.
            const bool nativeRootExitWitness =
                steeringValid && !desiredEvent.empty() &&
                (active || !config->parentStartEvent.empty()) &&
                SyntheticMotionInjector::ConsumeNativeRootExitToDefaultWitness(actor);

            // R46 second structural source. The SyntheticMotionInjector consumer itself refuses
            // a fresh/ongoing LocomotionBehavior -> StandingBehavior blend and expires stale
            // witnesses. Here we additionally demand that the failure is still semantically real:
            // parent inactive, steering valid, gait desired, action lane clear, carrier gone, and
            // no structural recovery already in flight. This prevents a Standing blend produced
            // during the R45 handshake from resetting the existing bounded recovery.
            bool structuralRecoveryAlreadyActive = false;
            if (!nativeRootExitWitness && !active && !actionBypass && steeringValid &&
                !desiredEvent.empty() && !runtimeLocomotionCarrier && !authoredRotationObserved &&
                !config->parentStartEvent.empty()) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto it = g_gaitRouterActors.find(formID);
                structuralRecoveryAlreadyActive =
                    it != g_gaitRouterActors.end() && it->second.nativeRootRecoveryActive;
            }

            std::uint64_t locomotionStandingWitnessAgeFrames = 0;
            const bool locomotionStandingDropWitness =
                !nativeRootExitWitness && !structuralRecoveryAlreadyActive &&
                !active && !actionBypass && steeringValid &&
                !desiredEvent.empty() && !runtimeLocomotionCarrier && !authoredRotationObserved &&
                !config->parentStartEvent.empty() &&
                SyntheticMotionInjector::ConsumeRecentLocomotionToStandingWitness(
                    actor, locomotionStandingWitnessAgeFrames);

            const StructuralRecoverySource structuralRecoverySource =
                nativeRootExitWitness ? StructuralRecoverySource::kNativeRootActionExit :
                (locomotionStandingDropWitness ? StructuralRecoverySource::kLocomotionStandingDrop :
                 StructuralRecoverySource::kNone);

            std::string eventToSend;
            std::string routeName;
            std::uint32_t attemptNumber = 0;
            bool newRequest = false;
            bool recoveryRequest = false;
            bool recoveryStarted = false;
            bool recoverySemanticRearmStarted = false;
            bool recoverySemanticRearmPreserved = false;
            std::string recoverySemanticPriorEvent;
            std::uint64_t recoverySemanticPriorRevision = 0;
            bool recoveryCarrierConfirmed = false;
            bool recoveryRequeuedAfterAccept = false;
            bool recoveryParentEventExhausted = false;
            bool recoveryStandingCancelled = false;
            bool requestParentEntryEvent = false;
            std::string parentEntryEventName;
            std::uint32_t parentEntryEventAttemptNumber = 0;
            StructuralRecoverySource parentEntryRecoverySource = StructuralRecoverySource::kNone;
            StructuralRecoverySource recoveryCompletedSource = StructuralRecoverySource::kNone;

            // R53 semantic RUN-carrier reassert is a separate bounded lane. It never
            // replaces the normal R35 request or the R46/R49 structural recovery lane.
            std::string runCarrierReassertEventToSend;
            std::string runCarrierReassertRouteName;
            std::uint32_t runCarrierReassertAttemptNumber = 0;
            bool runCarrierReassertRecovered = false;
            bool runCarrierReassertExhaustedNow = false;
            bool runCarrierActiveEdgeDeferred = false;
            bool sameSemanticActiveEdgeSuppressed = false;
            std::uint64_t sameSemanticActiveEdgeSuppressionCount = 0;
            bool runCarrierParentGapHeld = false;
            std::uint32_t runCarrierParentGapCount = 0;
            StructuralRecoverySource recoveryExhaustedSource = StructuralRecoverySource::kNone;
            StructuralRecoverySource recoveryCancelledSource = StructuralRecoverySource::kNone;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                const bool routeChanged = state.seen && state.lastRouteValue != routeValue;
                const bool becameActive = active && (!state.seen || !state.lastActive);
                if (routeChanged) {
                    ++state.routeChanges;
                }

                // Compare the desired EVENT, not just the semantic family. Profiles may
                // intentionally map multiple semantic families to one event, so only a real
                // route/event change or an authoritative active re-entry starts a normal request.
                //
                // R62: if RUN semantics did not change and runtime still proves a lower-family
                // carrier, a brief inactive->active parent pulse belongs to R53. Recreating a
                // normal pending run event on every pulse blocks the proven bounded R53 lane.
                const bool sameSemanticRequest =
                    state.seen &&
                    !routeChanged &&
                    state.lastDesiredEvent == desiredEvent &&
                    state.lastRevision == revision;
                const bool deferRunActiveEdgeToR53 =
                    becameActive &&
                    sameSemanticRequest &&
                    runEventDistinct &&
                    runCarrierMismatch &&
                    steeringValid &&
                    !actionBypass &&
                    !authoredRotationObserved &&
                    !state.nativeRootRecoveryActive;

                const bool suppressSameSemanticActiveEdge =
                    becameActive && sameSemanticRequest &&
                    !config->reassertSameSemanticOnActiveEdge;
                if (suppressSameSemanticActiveEdge) {
                    sameSemanticActiveEdgeSuppressed = true;
                    sameSemanticActiveEdgeSuppressionCount =
                        ++state.sameSemanticActiveEdgeSuppressions;
                }

                const bool requestChanged =
                    active &&
                    (routeChanged ||
                     state.lastDesiredEvent != desiredEvent ||
                     (becameActive && !sameSemanticRequest && !deferRunActiveEdgeToR53) ||
                     (becameActive && sameSemanticRequest &&
                      config->reassertSameSemanticOnActiveEdge && !deferRunActiveEdgeToR53));

                if (deferRunActiveEdgeToR53) {
                    ++state.runCarrierActiveEdgeDeferrals;
                    runCarrierActiveEdgeDeferred = true;
                }

                // R46 Standing-drop recovery only exists while semantic locomotion intent
                // remains continuously present. If steering or the desired gait disappears,
                // cancel this source instead of keeping a stale structural witness alive.
                if (state.nativeRootRecoveryActive &&
                    state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop &&
                    (!steeringValid || desiredEvent.empty())) {
                    recoveryCancelledSource = state.nativeRootRecoverySource;
                    state.nativeRootRecoveryActive = false;
                    state.nativeRootRecoveryAcceptedWait = false;
                    state.nativeRootRecoveryNestedGaitAccepted = false;
                    state.nativeRootRecoveryRequiresParentSettle = false;
                    state.nativeRootRecoverySource = StructuralRecoverySource::kNone;
                    state.nativeRootRecoveryEvent.clear();
                    state.nativeRootRecoveryRouteName.clear();
                    state.nativeRootRecoveryAcceptedWaitFrames = 0;
                    state.nativeRootRecoveryInactiveFrames = 0;
                    state.nativeRootRecoveryActiveStableFrames = 0;
                    state.nativeRootRecoveryCarrierStableFrames = 0;
                    state.nativeRootRecoveryParentEventCooldownFrames = 0;
                    state.nativeRootRecoveryEventRetryFrames = 0;
                    state.nativeRootRecoveryParentEventAttempts = 0;
                    state.nativeRootRecoverySemanticRearmEligible = false;
                    state.nativeRootRecoverySemanticRearmUsed = false;
                    state.nativeRootRecoveryExhaustedEvent.clear();
                    state.nativeRootRecoveryExhaustedRevision = 0;
                    state.pendingEvent.clear();
                    state.pendingRouteName.clear();
                    state.requestAttempts = 0;
                    state.retryRemaining = 0.0f;
                    recoveryStandingCancelled = true;
                }

                // R49 recovery success requires the nested gait handshake first, then a stable post-action carrier.
                if (state.nativeRootRecoveryActive) {
                    const bool standingRecoveryBlockedByAuthoredRotation =
                        state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop &&
                        authoredRotationObserved;
                    if (state.nativeRootRecoveryNestedGaitAccepted &&
                        active && !actionBypass && !standingRecoveryBlockedByAuthoredRotation &&
                        runtimeLocomotionCarrier) {
                        if (state.nativeRootRecoveryCarrierStableFrames < kNativeRootRecoveryCarrierConfirmFrames) {
                            ++state.nativeRootRecoveryCarrierStableFrames;
                        }
                    } else {
                        state.nativeRootRecoveryCarrierStableFrames = 0;
                    }

                    if (state.nativeRootRecoveryCarrierStableFrames >= kNativeRootRecoveryCarrierConfirmFrames) {
                        recoveryCompletedSource = state.nativeRootRecoverySource;
                        if (state.nativeRootRecoverySource == StructuralRecoverySource::kNativeRootActionExit) {
                            ++state.nativeRootActionRecoveriesConfirmed;
                        } else if (state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop) {
                            ++state.locomotionStandingRecoveriesConfirmed;
                        }
                        state.nativeRootRecoveryActive = false;
                        state.nativeRootRecoveryAcceptedWait = false;
                        state.nativeRootRecoveryNestedGaitAccepted = false;
                        state.nativeRootRecoveryRequiresParentSettle = false;
                        state.nativeRootRecoverySource = StructuralRecoverySource::kNone;
                        state.nativeRootRecoveryEvent.clear();
                        state.nativeRootRecoveryRouteName.clear();
                        state.nativeRootRecoveryAcceptedWaitFrames = 0;
                        state.nativeRootRecoveryInactiveFrames = 0;
                        state.nativeRootRecoveryActiveStableFrames = 0;
                        state.nativeRootRecoveryCarrierStableFrames = 0;
                        state.nativeRootRecoveryParentEventCooldownFrames = 0;
                        state.nativeRootRecoveryEventRetryFrames = 0;
                        state.nativeRootRecoveryParentEventAttempts = 0;
                        state.nativeRootRecoverySemanticRearmEligible = false;
                        state.nativeRootRecoverySemanticRearmUsed = false;
                        state.nativeRootRecoveryExhaustedEvent.clear();
                        state.nativeRootRecoveryExhaustedRevision = 0;
                        state.pendingEvent.clear();
                        state.pendingRouteName.clear();
                        state.requestAttempts = 0;
                        state.retryRemaining = 0.0f;
                        ++state.nativeRootRecoveriesCarrierConfirmed;
                        recoveryCarrierConfirmed = true;
                    }
                }

                if (structuralRecoverySource != StructuralRecoverySource::kNone &&
                    steeringValid && !desiredEvent.empty()) {
                    state.nativeRootRecoveryActive = true;
                    state.nativeRootRecoveryAcceptedWait = false;
                    state.nativeRootRecoveryNestedGaitAccepted = false;
                    // Every witness must cross a clean post-action handshake. Even when active is
                    // still true on the exit frame, actionBypass or stale graph state may overlap.
                    state.nativeRootRecoveryRequiresParentSettle = true;
                    state.nativeRootRecoverySource = structuralRecoverySource;
                    state.nativeRootRecoveryEvent = desiredEvent;
                    state.nativeRootRecoveryRouteName = route->name;
                    state.nativeRootRecoveryAcceptedWaitFrames = 0;
                    state.nativeRootRecoveryInactiveFrames = 0;
                    state.nativeRootRecoveryActiveStableFrames = 0;
                    state.nativeRootRecoveryCarrierStableFrames = 0;
                    state.nativeRootRecoveryParentEventCooldownFrames = 0;
                    state.nativeRootRecoveryEventRetryFrames = 0;
                    state.nativeRootRecoveryParentEventAttempts = 0;
                    state.pendingEvent.clear();
                    state.pendingRouteName.clear();
                    state.requestAttempts = 0;
                    state.retryRemaining = 0.0f;
                    ++state.requests;
                    ++state.nativeRootRecoveriesStarted;
                    state.nativeRootRecoverySemanticRearmEligible = false;
                    state.nativeRootRecoverySemanticRearmUsed = false;
                    state.nativeRootRecoveryExhaustedEvent.clear();
                    state.nativeRootRecoveryExhaustedRevision = 0;
                    if (structuralRecoverySource == StructuralRecoverySource::kNativeRootActionExit) {
                        ++state.nativeRootActionRecoveriesStarted;
                    } else if (structuralRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop) {
                        ++state.locomotionStandingRecoveriesStarted;
                    }
                    recoveryStarted = true;
                }

                // R61: after a bounded R46 Standing-drop recovery exhausts, the graph can
                // remain in Standing forever and therefore can never produce another
                // Locomotion->Standing witness. Preserve that exact structural provenance
                // for one semantic retry only. A genuinely different configured gait event
                // and revision (e.g. walkStart -> runStart) may re-arm the same bounded
                // parent handshake once. WALK<->TROT profiles that share one event do not
                // qualify. Natural parent/carrier recovery or loss of steering invalidates
                // the retained provenance.
                if (!state.nativeRootRecoveryActive &&
                    state.nativeRootRecoverySemanticRearmEligible) {
                    if (active || runtimeLocomotionCarrier || !steeringValid || desiredEvent.empty()) {
                        state.nativeRootRecoverySemanticRearmEligible = false;
                        state.nativeRootRecoverySemanticRearmUsed = false;
                        state.nativeRootRecoveryExhaustedEvent.clear();
                        state.nativeRootRecoveryExhaustedRevision = 0;
                    } else if (!state.nativeRootRecoverySemanticRearmUsed &&
                               !actionBypass && !authoredRotationObserved &&
                               !config->parentStartEvent.empty() &&
                               revision != state.nativeRootRecoveryExhaustedRevision &&
                               desiredEvent != state.nativeRootRecoveryExhaustedEvent) {
                        recoverySemanticPriorEvent = state.nativeRootRecoveryExhaustedEvent;
                        recoverySemanticPriorRevision = state.nativeRootRecoveryExhaustedRevision;
                        state.nativeRootRecoveryActive = true;
                        state.nativeRootRecoveryAcceptedWait = false;
                        state.nativeRootRecoveryNestedGaitAccepted = false;
                        state.nativeRootRecoveryRequiresParentSettle = true;
                        state.nativeRootRecoverySource = StructuralRecoverySource::kLocomotionStandingDrop;
                        state.nativeRootRecoveryEvent = desiredEvent;
                        state.nativeRootRecoveryRouteName = route->name;
                        state.nativeRootRecoveryAcceptedWaitFrames = 0;
                        state.nativeRootRecoveryInactiveFrames = 0;
                        state.nativeRootRecoveryActiveStableFrames = 0;
                        state.nativeRootRecoveryCarrierStableFrames = 0;
                        state.nativeRootRecoveryParentEventCooldownFrames = 0;
                        state.nativeRootRecoveryEventRetryFrames = 0;
                        state.nativeRootRecoveryParentEventAttempts = 0;
                        state.pendingEvent.clear();
                        state.pendingRouteName.clear();
                        state.requestAttempts = 0;
                        state.retryRemaining = 0.0f;
                        state.nativeRootRecoverySemanticRearmEligible = false;
                        state.nativeRootRecoverySemanticRearmUsed = true;
                        ++state.requests;
                        ++state.nativeRootRecoveriesStarted;
                        ++state.locomotionStandingRecoveriesStarted;
                        recoveryStarted = true;
                        recoverySemanticRearmStarted = true;
                    }
                }

                state.lastActive = active;
                state.lastRouteValue = routeValue;
                state.lastDesiredEvent = desiredEvent;
                state.lastFamily = family;
                state.lastRevision = revision;
                state.seen = true;

                // During a structural recovery keep the latest semantic destination. This is
                // deliberately not a generic watchdog: the lane can exist only after either the
                // R42 native-action exit witness or the R46 Locomotion -> Standing witness.
                if (state.nativeRootRecoveryActive && !desiredEvent.empty() &&
                    (state.nativeRootRecoveryEvent != desiredEvent ||
                     state.nativeRootRecoveryRouteName != route->name)) {
                    state.nativeRootRecoveryEvent = desiredEvent;
                    state.nativeRootRecoveryRouteName = route->name;
                    state.nativeRootRecoveryAcceptedWait = false;
                    state.nativeRootRecoveryNestedGaitAccepted = false;
                    state.nativeRootRecoveryAcceptedWaitFrames = 0;
                    state.nativeRootRecoveryEventRetryFrames = 0;
                    state.pendingEvent.clear();
                    state.pendingRouteName.clear();
                    state.requestAttempts = 0;
                    state.retryRemaining = 0.0f;
                }

                if (state.retryRemaining > 0.0f) {
                    // Exact R35 normal-router cadence is intentionally untouched. R44 only
                    // removes deltaTime from the structural native-root recovery lane.
                    state.retryRemaining = std::max(0.0f, state.retryRemaining - std::max(0.0f, deltaTime));
                }

                if (state.nativeRootRecoveryActive) {
                    if (state.nativeRootRecoveryParentEventCooldownFrames > 0) {
                        --state.nativeRootRecoveryParentEventCooldownFrames;
                    }
                    if (state.nativeRootRecoveryEventRetryFrames > 0) {
                        --state.nativeRootRecoveryEventRetryFrames;
                    }
                    if (state.nativeRootRecoveryAcceptedWaitFrames > 0) {
                        --state.nativeRootRecoveryAcceptedWaitFrames;
                    }

                    // Never spend parent/nested recovery events while the native action lane is
                    // still active. For the R46 Standing-drop source, an authored RotationClip is
                    // also exclusive: wait for TurnLoop/turn ownership to disappear before moveStart.
                    const bool standingRecoveryBlockedByAuthoredRotation =
                        state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop &&
                        authoredRotationObserved;
                    // R49: once the parent is natively active, an initial WALK carrier is
                    // expected during re-entry and must NOT prevent the nested gait handshake.
                    // Carrier overlap only blocks while the parent is still inactive.
                    const bool standingRecoveryBlockedByCarrierOverlap =
                        state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop &&
                        !active && runtimeLocomotionCarrier;
                    if (actionBypass || standingRecoveryBlockedByAuthoredRotation ||
                        standingRecoveryBlockedByCarrierOverlap) {
                        state.pendingEvent.clear();
                        state.pendingRouteName.clear();
                        state.nativeRootRecoveryInactiveFrames = 0;
                        state.nativeRootRecoveryActiveStableFrames = 0;
                    } else if (!active) {
                        state.pendingEvent.clear();
                        state.pendingRouteName.clear();
                        state.retryRemaining = 0.0f;
                        // A nested gait acceptance belongs to the parent instance that accepted it.
                        // If the parent drops inactive again, the handshake must be earned again.
                        state.nativeRootRecoveryNestedGaitAccepted = false;
                        state.nativeRootRecoveryAcceptedWait = false;
                        state.nativeRootRecoveryAcceptedWaitFrames = 0;
                        state.nativeRootRecoveryRequiresParentSettle = true;
                        state.nativeRootRecoveryActiveStableFrames = 0;
                        if (state.nativeRootRecoveryInactiveFrames < std::numeric_limits<std::uint32_t>::max()) {
                            ++state.nativeRootRecoveryInactiveFrames;
                        }

                        const auto inactiveFramesBeforeParentEvent =
                            state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop ?
                            kLocomotionStandingRecoveryInactiveFramesBeforeParentEvent :
                            kNativeRootRecoveryInactiveFramesBeforeParentEvent;

                        if (steeringValid && !desiredEvent.empty() &&
                            state.nativeRootRecoveryInactiveFrames >= inactiveFramesBeforeParentEvent &&
                            state.nativeRootRecoveryParentEventCooldownFrames == 0 &&
                            state.nativeRootRecoveryParentEventAttempts < kNativeRootRecoveryParentEventMaxAttempts &&
                            !config->parentStartEvent.empty()) {
                            ++state.nativeRootRecoveryParentEventAttempts;
                            parentEntryEventAttemptNumber = state.nativeRootRecoveryParentEventAttempts;
                            parentEntryEventName = config->parentStartEvent;
                            parentEntryRecoverySource = state.nativeRootRecoverySource;
                            state.nativeRootRecoveryParentEventCooldownFrames = kNativeRootRecoveryParentEventRetryFrames;
                            state.nativeRootRecoveryInactiveFrames = 0;
                            requestParentEntryEvent = true;
                        }
                        else if (state.nativeRootRecoveryParentEventAttempts >= kNativeRootRecoveryParentEventMaxAttempts &&
                                 state.nativeRootRecoveryParentEventCooldownFrames == 0) {
                            recoveryExhaustedSource = state.nativeRootRecoverySource;
                            const bool retainStandingSemanticRearm =
                                state.nativeRootRecoverySource == StructuralRecoverySource::kLocomotionStandingDrop &&
                                !state.nativeRootRecoverySemanticRearmUsed;
                            if (retainStandingSemanticRearm) {
                                state.nativeRootRecoverySemanticRearmEligible = true;
                                state.nativeRootRecoveryExhaustedEvent = state.nativeRootRecoveryEvent;
                                state.nativeRootRecoveryExhaustedRevision = revision;
                                recoverySemanticRearmPreserved = true;
                            } else {
                                state.nativeRootRecoverySemanticRearmEligible = false;
                            }
                            state.nativeRootRecoveryActive = false;
                            state.nativeRootRecoveryAcceptedWait = false;
                            state.nativeRootRecoveryNestedGaitAccepted = false;
                            state.nativeRootRecoveryRequiresParentSettle = false;
                            state.nativeRootRecoverySource = StructuralRecoverySource::kNone;
                            state.nativeRootRecoveryEvent.clear();
                            state.nativeRootRecoveryRouteName.clear();
                            state.pendingEvent.clear();
                            state.pendingRouteName.clear();
                            state.requestAttempts = 0;
                            ++state.exhausted;
                            ++state.nativeRootRecoveriesExhausted;
                            recoveryParentEventExhausted = true;
                        }
                    } else {
                        state.nativeRootRecoveryInactiveFrames = 0;

                        if (state.nativeRootRecoveryRequiresParentSettle) {
                            if (state.nativeRootRecoveryActiveStableFrames < kNativeRootRecoveryParentSettleFrames) {
                                ++state.nativeRootRecoveryActiveStableFrames;
                            }
                            if (state.nativeRootRecoveryActiveStableFrames >= kNativeRootRecoveryParentSettleFrames) {
                                state.nativeRootRecoveryRequiresParentSettle = false;
                            }
                        }

                        if (state.nativeRootRecoveryAcceptedWait &&
                            state.nativeRootRecoveryAcceptedWaitFrames == 0 &&
                            state.nativeRootRecoveryCarrierStableFrames < kNativeRootRecoveryCarrierConfirmFrames) {
                            state.nativeRootRecoveryAcceptedWait = false;
                            state.nativeRootRecoveryNestedGaitAccepted = false;
                            state.nativeRootRecoveryEventRetryFrames = 0;
                            recoveryRequeuedAfterAccept = true;
                        }

                        if (!state.nativeRootRecoveryRequiresParentSettle &&
                            !state.nativeRootRecoveryAcceptedWait &&
                            state.nativeRootRecoveryEventRetryFrames == 0 &&
                            !desiredEvent.empty() &&
                            state.pendingEvent.empty() &&
                            !state.nativeRootRecoveryEvent.empty() &&
                            state.requestAttempts < config->maxAttempts) {
                            state.pendingEvent = state.nativeRootRecoveryEvent;
                            state.pendingRouteName = state.nativeRootRecoveryRouteName;
                        }
                    }
                } else if (!active) {
                    // Exact R35 non-recovery inactive behavior.
                    state.pendingEvent.clear();
                    state.pendingRouteName.clear();
                    state.requestAttempts = 0;
                    state.retryRemaining = 0.0f;
                } else if (requestChanged) {
                    state.pendingEvent = desiredEvent;
                    state.pendingRouteName = route->name;
                    state.requestAttempts = 0;
                    state.retryRemaining = 0.0f;
                    if (!desiredEvent.empty()) {
                        ++state.requests;
                        newRequest = true;
                    }
                }

                if (!state.pendingEvent.empty() && state.retryRemaining <= 0.0f) {
                    eventToSend = state.pendingEvent;
                    routeName = state.pendingRouteName;
                    ++state.requestAttempts;
                    ++state.attempts;
                    attemptNumber = state.requestAttempts;
                    recoveryRequest =
                        state.nativeRootRecoveryActive &&
                        state.nativeRootRecoveryEvent == eventToSend;
                }

                // R53: a distinct RUN event may have been accepted earlier and later
                // lost its semantic carrier while the family remains RUN. R50/R51 are
                // intentionally preventing the RUN Speed goal from being written onto
                // WALK/TROT, so the only safe way out is to reassert the configured
                // semantic event and wait for a real RUN carrier. This is armed only by
                // lower-family runtime-root proof; a carrierless parent/Standing state is
                // left entirely to the R46 structural recovery lane.
                if (state.runCarrierReassertCooldownFrames > 0) {
                    --state.runCarrierReassertCooldownFrames;
                }

                const bool runCarrierReassertBaseEligible =
                    runEventDistinct && steeringValid && !actionBypass &&
                    !authoredRotationObserved && !state.nativeRootRecoveryActive;

                // R62: an inactive parent frame is never authority to send runStart, but a
                // very short gap must not erase an already-developing R53 mismatch episode
                // when the semantic RUN request and lower-family carrier are unchanged.
                const bool holdR53AcrossParentGap =
                    runCarrierReassertBaseEligible &&
                    !active &&
                    sameSemanticRequest &&
                    runCarrierMismatch &&
                    (state.runCarrierMismatchFrames > 0 || state.runCarrierReassertActive) &&
                    state.runCarrierParentGapFrames < kRunCarrierParentGapHoldFrames;

                if (holdR53AcrossParentGap) {
                    ++state.runCarrierParentGapFrames;
                    runCarrierParentGapHeld = true;
                    runCarrierParentGapCount = state.runCarrierParentGapFrames;
                    state.runCarrierStableFrames = 0;
                    // Preserve mismatch/reassert counters. No event is sent while active=false.
                } else if (!runCarrierReassertBaseEligible || !active) {
                    state.runCarrierReassertActive = false;
                    state.runCarrierReassertExhausted = false;
                    state.runCarrierMismatchFrames = 0;
                    state.runCarrierStableFrames = 0;
                    state.runCarrierReassertCooldownFrames = 0;
                    state.runCarrierReassertAttempts = 0;
                    state.runCarrierParentGapFrames = 0;
                } else if (runCarrierPresent) {
                    state.runCarrierParentGapFrames = 0;
                    state.runCarrierMismatchFrames = 0;
                    state.runCarrierReassertExhausted = false;
                    if (state.runCarrierReassertActive) {
                        if (state.runCarrierStableFrames < kRunCarrierStableConfirmFrames) {
                            ++state.runCarrierStableFrames;
                        }
                        if (state.runCarrierStableFrames >= kRunCarrierStableConfirmFrames) {
                            state.runCarrierReassertActive = false;
                            state.runCarrierStableFrames = 0;
                            state.runCarrierReassertCooldownFrames = 0;
                            state.runCarrierReassertAttempts = 0;
                            ++state.runCarrierReassertRecoveries;
                            runCarrierReassertRecovered = true;
                        }
                    } else {
                        state.runCarrierStableFrames = 0;
                    }
                } else if (runCarrierMismatch) {
                    state.runCarrierParentGapFrames = 0;
                    state.runCarrierStableFrames = 0;
                    if (!state.runCarrierReassertExhausted) {
                        if (state.runCarrierMismatchFrames < kRunCarrierMismatchConfirmFrames) {
                            ++state.runCarrierMismatchFrames;
                        }

                        if (state.runCarrierMismatchFrames >= kRunCarrierMismatchConfirmFrames &&
                            state.runCarrierReassertCooldownFrames == 0 &&
                            state.runCarrierReassertAttempts < kRunCarrierReassertMaxAttempts &&
                            eventToSend.empty() && state.pendingEvent.empty()) {
                            if (!state.runCarrierReassertActive) {
                                state.runCarrierReassertActive = true;
                                ++state.runCarrierReassertEpisodes;
                            }
                            ++state.runCarrierReassertAttempts;
                            runCarrierReassertAttemptNumber = state.runCarrierReassertAttempts;
                            runCarrierReassertEventToSend = desiredEvent;
                            runCarrierReassertRouteName = route->name;
                            state.runCarrierReassertCooldownFrames = kRunCarrierReassertRetryFrames;
                        } else if (state.runCarrierReassertActive &&
                                   state.runCarrierReassertAttempts >= kRunCarrierReassertMaxAttempts &&
                                   state.runCarrierReassertCooldownFrames == 0) {
                            state.runCarrierReassertActive = false;
                            state.runCarrierReassertExhausted = true;
                            runCarrierReassertExhaustedNow = true;
                        }
                    }
                } else {
                    // No lower-family carrier means this is not the R53 failure mode.
                    // Do not turn carrier loss into a generic watchdog.
                    state.runCarrierReassertActive = false;
                    state.runCarrierReassertExhausted = false;
                    state.runCarrierMismatchFrames = 0;
                    state.runCarrierStableFrames = 0;
                    state.runCarrierReassertCooldownFrames = 0;
                    state.runCarrierReassertAttempts = 0;
                    state.runCarrierParentGapFrames = 0;
                }
            }

            if (requestParentEntryEvent) {
                const bool parentEventAccepted = actor->NotifyAnimationGraph(parentEntryEventName.c_str());
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    if (parentEventAccepted) {
                        ++state.nativeRootRecoveryParentEventsAccepted;
                    } else {
                        ++state.nativeRootRecoveryParentEventsRejected;
                    }
                }
                SKSE::log::info(
                    "[GaitRouter][R46_PARENT_EVENT] form={:08X} profile=\"{}\" event=\"{}\" attempt={}/{} accepted={} source={} proof=STRUCTURAL_WITNESS_PLUS_CURRENT_VALID_STEERING_ACTION_CLEAR_PARENT_INACTIVE action=REQUEST_NATIVE_PARENT_REENTRY",
                    formID, profile, parentEntryEventName, parentEntryEventAttemptNumber,
                    kNativeRootRecoveryParentEventMaxAttempts, parentEventAccepted,
                    StructuralRecoverySourceName(parentEntryRecoverySource));
            }

            if (recoverySemanticRearmStarted) {
                SKSE::log::info(
                    "[GaitRouter][R61_STRUCTURAL_SEMANTIC_REARM] form={:08X} profile=\"{}\" route=\"{}\" oldEvent=\"{}\" oldRevision={} newEvent=\"{}\" newRevision={} active={} steeringValid={} actionBypass={} carrier={} authoredRotationObserved={} proof=PRIOR_R46_LOCOMOTION_STANDING_WITNESS_EXHAUSTED_PLUS_NEW_CONFIGURED_GAIT_EVENT action=ONE_BOUNDED_PARENT_REENTRY_RETRY",
                    formID, profile, route->name, recoverySemanticPriorEvent, recoverySemanticPriorRevision,
                    desiredEvent, revision, active, steeringValid, actionBypass,
                    runtimeLocomotionCarrier, authoredRotationObserved);
            } else if (recoveryStarted) {
                SKSE::log::info(
                    "[GaitRouter][R46_STRUCTURAL_REENTRY_ARM] form={:08X} profile=\"{}\" route=\"{}\" family={} revision={} event=\"{}\" active={} steeringValid={} actionBypass={} carrier={} authoredRotationObserved={} source={} standingWitnessAgeFrames={} inactiveParentEventFrames={}",
                    formID, profile, route->name, FamilyName(family), revision, desiredEvent, active, steeringValid,
                    actionBypass, runtimeLocomotionCarrier, authoredRotationObserved,
                    StructuralRecoverySourceName(structuralRecoverySource),
                    locomotionStandingDropWitness ? locomotionStandingWitnessAgeFrames : 0u,
                    active ? 0u : (locomotionStandingDropWitness ?
                        kLocomotionStandingRecoveryInactiveFramesBeforeParentEvent :
                        kNativeRootRecoveryInactiveFramesBeforeParentEvent));
            }
            if (recoveryCarrierConfirmed) {
                SKSE::log::info(
                    "[GaitRouter][R49_STRUCTURAL_REENTRY_RECOVERED] form={:08X} profile=\"{}\" source={} proof=NESTED_GAIT_EVENT_ACCEPTED_PLUS_2_CONSECUTIVE_ACTIVE_ACTION_CLEAR_RUNTIME_LOCOMOTION_CARRIER action=DISARM",
                    formID, profile, StructuralRecoverySourceName(recoveryCompletedSource));
            }
            if (recoveryRequeuedAfterAccept) {
                SKSE::log::info(
                    "[GaitRouter][R46_STRUCTURAL_REENTRY_RETRY] form={:08X} profile=\"{}\" event=\"{}\" reason=ACCEPTED_BUT_NO_RUNTIME_CARRIER",
                    formID, profile, desiredEvent);
            }
            if (recoveryParentEventExhausted) {
                SKSE::log::warn(
                    "[GaitRouter][R46_PARENT_EVENT_EXHAUSTED] form={:08X} profile=\"{}\" event=\"{}\" attempts={} source={} result=PARENT_REMAINED_INACTIVE action=DISARM_RECOVERY",
                    formID, profile, config->parentStartEvent, kNativeRootRecoveryParentEventMaxAttempts,
                    StructuralRecoverySourceName(recoveryExhaustedSource));
                if (recoverySemanticRearmPreserved) {
                    SKSE::log::info(
                        "[GaitRouter][R61_STRUCTURAL_SEMANTIC_REARM_ARMED] form={:08X} profile=\"{}\" exhaustedEvent=\"{}\" exhaustedRevision={} policy=WAIT_FOR_DIFFERENT_CONFIGURED_GAIT_EVENT_AND_NEW_REVISION_MAX_ONE_RETRY",
                        formID, profile, desiredEvent, revision);
                }
            }
            if (recoveryStandingCancelled) {
                SKSE::log::info(
                    "[GaitRouter][R46_STRUCTURAL_REENTRY_CANCEL] form={:08X} profile=\"{}\" source={} reason=STEERING_OR_DESIRED_GAIT_LOST action=DISARM_WITHOUT_EVENT",
                    formID, profile, StructuralRecoverySourceName(recoveryCancelledSource));
            }

            if (sameSemanticActiveEdgeSuppressed &&
                (sameSemanticActiveEdgeSuppressionCount <= 8 ||
                 (sameSemanticActiveEdgeSuppressionCount % 120) == 0)) {
                SKSE::log::info(
                    "[GaitRouter][R96_SAME_SEMANTIC_EDGE_SUPPRESSED] form={:08X} profile=\"{}\" route=\"{}\" family={} revision={} event=\"{}\" count={} action=KEEP_CURRENT_GRAPH_STATE_STRUCTURAL_RECOVERY_REMAINS_SEPARATE",
                    formID, profile, route->name, FamilyName(family), revision, desiredEvent,
                    sameSemanticActiveEdgeSuppressionCount);
            }

            if (runCarrierActiveEdgeDeferred) {
                SKSE::log::info(
                    "[GaitRouter][R62_RUN_ACTIVE_EDGE_DEFERRED_TO_R53] form={:08X} profile=\"{}\" route=\"{}\" family=RUN revision={} event=\"{}\" carrierMask=0x{:02X} proof=SAME_SEMANTIC_RUN_PLUS_LOWER_CARRIER_PLUS_BRIEF_PARENT_REENTRY action=DO_NOT_CREATE_NORMAL_PENDING_EVENT",
                    formID, profile, route->name, revision, desiredEvent, runtimeFamilyMask);
            }
            if (runCarrierParentGapHeld) {
                SKSE::log::info(
                    "[GaitRouter][R62_R53_PARENT_GAP_HOLD] form={:08X} profile=\"{}\" family=RUN gapFrames={}/{} carrierMask=0x{:02X} proof=SAME_SEMANTIC_RUN_PLUS_LOWER_CARRIER action=PRESERVE_R53_MISMATCH_STATE_NO_EVENT_WHILE_PARENT_INACTIVE",
                    formID, profile, runCarrierParentGapCount, kRunCarrierParentGapHoldFrames, runtimeFamilyMask);
            }

            if (!runCarrierReassertEventToSend.empty()) {
                const bool accepted = actor->NotifyAnimationGraph(runCarrierReassertEventToSend.c_str());
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    auto& state = g_gaitRouterActors[formID];
                    if (accepted) {
                        ++state.runCarrierReassertAccepted;
                    } else {
                        ++state.runCarrierReassertRejected;
                    }
                }
                SKSE::log::info(
                    "[GaitRouter][R53_RUN_CARRIER_REASSERT] form={:08X} profile=\"{}\" route=\"{}\" family=RUN event=\"{}\" attempt={}/{} accepted={} carrierMask=0x{:02X} proof=DESIRED_DISTINCT_RUN_EVENT_PLUS_ACTIVE_PARENT_PLUS_VALID_STEERING_PLUS_LOWER_RUNTIME_CARRIER_WITHOUT_RUN action=REASSERT_SEMANTIC_GAIT_EVENT_NO_SPEED_WRITE",
                    formID, profile, runCarrierReassertRouteName, runCarrierReassertEventToSend,
                    runCarrierReassertAttemptNumber, kRunCarrierReassertMaxAttempts, accepted, runtimeFamilyMask);
            }
            if (runCarrierReassertRecovered) {
                SKSE::log::info(
                    "[GaitRouter][R53_RUN_CARRIER_CERTIFIED] form={:08X} profile=\"{}\" carrierMask=0x{:02X} proof=2_CONSECUTIVE_RUN_RUNTIME_ROOT_FRAMES action=DISARM_SEMANTIC_REASSERT_AND_ALLOW_EXISTING_R50_R51_SPEED_HOLD_TO_RESUME",
                    formID, profile, runtimeFamilyMask);
            }
            if (runCarrierReassertExhaustedNow) {
                SKSE::log::warn(
                    "[GaitRouter][R53_RUN_CARRIER_REASSERT_EXHAUSTED] form={:08X} profile=\"{}\" attempts={} carrierMask=0x{:02X} proof=LOWER_RUNTIME_CARRIER_PERSISTED_WITHOUT_RUN action=FAIL_CLOSED_NO_SPEED_WRITE_NO_GENERIC_WATCHDOG",
                    formID, profile, kRunCarrierReassertMaxAttempts, runtimeFamilyMask);
            }

            if (eventToSend.empty()) {
                return;
            }

            const bool accepted = actor->NotifyAnimationGraph(eventToSend.c_str());
            bool exhausted = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[formID];
                // Ignore completion from a stale request if route/family changed while
                // NotifyAnimationGraph was executing.
                if (state.pendingEvent == eventToSend) {
                    if (recoveryRequest && state.nativeRootRecoveryActive) {
                        if (accepted) {
                            ++state.accepted;
                            state.pendingEvent.clear();
                            state.pendingRouteName.clear();
                            state.retryRemaining = 0.0f;
                            state.nativeRootRecoveryNestedGaitAccepted = true;
                            state.nativeRootRecoveryCarrierStableFrames = 0;
                            state.nativeRootRecoveryAcceptedWait = true;
                            state.nativeRootRecoveryAcceptedWaitFrames = kNativeRootRecoveryAcceptedCarrierWaitFrames;
                        } else {
                            ++state.rejected;
                            state.nativeRootRecoveryNestedGaitAccepted = false;
                            if (state.requestAttempts >= config->maxAttempts) {
                                ++state.exhausted;
                                ++state.nativeRootRecoveriesExhausted;
                                state.pendingEvent.clear();
                                state.pendingRouteName.clear();
                                state.retryRemaining = 0.0f;
                                state.nativeRootRecoveryActive = false;
                                state.nativeRootRecoveryAcceptedWait = false;
                                state.nativeRootRecoveryNestedGaitAccepted = false;
                                state.nativeRootRecoveryRequiresParentSettle = false;
                                state.nativeRootRecoverySource = StructuralRecoverySource::kNone;
                                state.nativeRootRecoveryEvent.clear();
                                state.nativeRootRecoveryRouteName.clear();
                                state.nativeRootRecoveryAcceptedWaitFrames = 0;
                                state.nativeRootRecoveryInactiveFrames = 0;
                                state.nativeRootRecoveryActiveStableFrames = 0;
                                state.nativeRootRecoveryCarrierStableFrames = 0;
                                state.nativeRootRecoveryParentEventCooldownFrames = 0;
                                state.nativeRootRecoveryEventRetryFrames = 0;
                                state.nativeRootRecoveryParentEventAttempts = 0;
                                exhausted = true;
                            } else {
                                // R45 nested-gait recovery retries are actor-frame based.
                                // Clear the request and requeue only after a few clean updates.
                                state.pendingEvent.clear();
                                state.pendingRouteName.clear();
                                state.retryRemaining = 0.0f;
                                state.nativeRootRecoveryEventRetryFrames = kNativeRootRecoveryEventRetryFrames;
                            }
                        }
                    } else if (accepted) {
                        ++state.accepted;
                        state.pendingEvent.clear();
                        state.retryRemaining = 0.0f;
                    } else {
                        ++state.rejected;
                        if (state.requestAttempts >= config->maxAttempts) {
                            ++state.exhausted;
                            state.pendingEvent.clear();
                            state.retryRemaining = 0.0f;
                            exhausted = true;
                        } else {
                            state.retryRemaining = config->retrySeconds;
                        }
                    }
                }
            }

            SKSE::log::info(
                "[GaitRouter][EVENT] form={:08X} profile=\"{}\" route=\"{}\" routeValue={} active={} family={} revision={} event=\"{}\" attempt={}/{} accepted={} newRequest={} exhausted={} recovery={} writes[SpeedSampled=NONE Direction=NONE] root=GOLDEN_R15.8_UNCHANGED",
                formID, profile, routeName, routeValue, active, FamilyName(family), revision,
                eventToSend, attemptNumber, config->maxAttempts, accepted, newRequest, exhausted, recoveryRequest);
        }

        [[nodiscard]] bool TurnBlendFamilyParams(
            const TurnBlendConfig& config,
            SpeedIntentTranslator::Family family,
            float& outFullErrorDegrees,
            float& outFullBlend)
        {
            switch (family) {
            case SpeedIntentTranslator::Family::kWalkForward:
                outFullErrorDegrees = config.walkFullErrorDegrees;
                outFullBlend = config.walkFullBlend;
                return true;
            case SpeedIntentTranslator::Family::kTrotForward:
                outFullErrorDegrees = config.trotFullErrorDegrees;
                outFullBlend = config.trotFullBlend;
                return true;
            case SpeedIntentTranslator::Family::kRunForward:
                outFullErrorDegrees = config.runFullErrorDegrees;
                outFullBlend = config.runFullBlend;
                return true;
            default:
                return false;
            }
        }

        void ApplyMovingTurnBlend(RE::Actor* actor)
        {
            if (!actor || actor->IsPlayerRef() || !SpeedIntentTranslator::SupportsActor(actor)) {
                return;
            }

            const auto formID = actor->GetFormID();
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = GaitRouterConfigForProfile(profile);
            if (!config || !config->enabled || !config->turnBlend.enabled) {
                return;
            }

            // Direction authority is only for the configured moving route. For the
            // wolf this is Forward (iSyncForwardState=0). Backward remains vanilla.
            if (!config->activeVariable.empty()) {
                std::int32_t activeValue = 0;
                if (!actor->GetGraphVariableInt(config->activeVariable.c_str(), activeValue) ||
                    activeValue != config->activeValue) {
                    return;
                }
            }

            if (!config->routeVariable.empty()) {
                std::int32_t routeValue = 0;
                if (!actor->GetGraphVariableInt(config->routeVariable.c_str(), routeValue) ||
                    routeValue != config->turnBlend.routeValue) {
                    return;
                }
            }

            const auto family = SpeedIntentTranslator::LatchedFamily(formID);
            float fullErrorDegrees = 0.0f;
            float fullBlend = 0.0f;
            if (!TurnBlendFamilyParams(config->turnBlend, family, fullErrorDegrees, fullBlend)) {
                return;
            }

            // ------------------------------------------------------------------
            // R18: TurnDelta writing is DISABLED. This function is now observe-only.
            //
            // The R17 capture killed the idea on two independent counts.
            //
            // 1. Our heading error is garbage. Every [RotationIntent][LIVE] sample
            //    reported pathDesired=+0.00deg. HighProcessData::pathingDesiredOrientation.z
            //    is never populated for this actor, so the "error" we computed was just
            //    minus the actor's own yaw. It froze at +119.04deg for hundreds of
            //    consecutive frames and saturated the command at the full blend limit.
            //
            // 2. The write does not survive anyway. We wrote +270.00 every frame, yet
            //    the next frame read prior=-0.00 and TurnDeltaDamped stayed at +0.00.
            //    The engine rewrites TurnDelta after us, so the damping modifier and the
            //    L/C/R blender never saw our value.
            //
            // The engine's own TurnDelta is left alone. Rotation now comes from the
            // authored MotionDB yaw of the curved clips, applied as a per-frame delta.
            constexpr bool kWriteTurnDelta = false;

            // R17: consume the LIVE heading error, not the confirmed latch.
            // The latch is gated by candidateConfirmFrames and by a 5-degree candidate
            // epsilon; an actor that is actually turning changes heading error by more
            // than that every frame, so the candidate resets forever and the latch never
            // confirms. GetLatchedRawTurnError therefore returned false on 100% of frames
            // in the R16.13 capture, which collapsed command to 0.0.
            float rawDegrees = 0.0f;
            const bool rawAvailable = RotationIntentTranslator::GetLiveTurnError(actor, rawDegrees) &&
                std::isfinite(rawDegrees);

            // R17 CRITICAL: without a heading error of our own we must NOT touch
            // TurnDelta. ApplyMovingTurnBlend runs AFTER Func() inside
            // ModifyAnimationUpdateData, so an unconditional write of 0.0 erased the
            // TurnDelta the engine had just computed, every single frame. That alone
            // prevented the L/C/R blender from ever selecting a curved clip.
            if (!rawAvailable) {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_turnBlendActors[formID];
                state.seen = true;
                ++state.frames;
                ++state.rawUnavailable;
                state.lastFamily = family;
                return;
            }

            float command = 0.0f;
            {
                const float magnitude = std::abs(rawDegrees);
                const float deadZone = std::min(config->turnBlend.deadZoneDegrees, fullErrorDegrees - 0.001f);
                if (magnitude > deadZone) {
                    const float denominator = std::max(0.001f, fullErrorDegrees - deadZone);
                    const float normalized = std::clamp((magnitude - deadZone) / denominator, 0.0f, 1.0f);
                    command = std::copysign(fullBlend * normalized, rawDegrees);
                }
            }

            float priorInput = 0.0f;
            const bool priorInputRead = actor->GetGraphVariableFloat(config->turnBlend.inputVariable.c_str(), priorInput);

            bool wrote = false;
            if constexpr (kWriteTurnDelta) {
                wrote = actor->SetGraphVariableFloat(config->turnBlend.inputVariable.c_str(), command);
            }

            float damped = 0.0f;
            bool dampedRead = false;
            if (!config->turnBlend.dampedVariable.empty()) {
                dampedRead = actor->GetGraphVariableFloat(config->turnBlend.dampedVariable.c_str(), damped);
            }

            bool emit = false;
            std::uint64_t frame = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_turnBlendActors[formID];
                const bool familyChanged = state.seen && state.lastFamily != family;
                const bool directionChanged = state.seen &&
                    ((state.lastCommand > 1.0f && command < -1.0f) ||
                     (state.lastCommand < -1.0f && command > 1.0f));
                const bool commandChanged = state.seen && std::abs(command - state.lastCommand) >= 30.0f;
                state.seen = true;
                ++state.frames;
                frame = state.frames;
                if (wrote) ++state.writes;
                else ++state.writeFailures;
                if (!rawAvailable) ++state.rawUnavailable;
                emit = state.frames <= 8 || familyChanged || directionChanged || commandChanged ||
                    (config->turnBlend.telemetryEveryFrames > 0 &&
                     state.frames % config->turnBlend.telemetryEveryFrames == 0);
                state.lastRawDegrees = rawAvailable ? rawDegrees : 0.0f;
                state.lastCommand = command;
                state.lastDamped = dampedRead ? damped : 0.0f;
                state.lastFamily = family;
            }

            if (emit) {
                SKSE::log::info(
                    "[GaitRouter][TURN] form={:08X} profile=\"{}\" frame={} family={} raw[available={} deg={:+.2f}] mapping[deadZone={:.2f} fullError={:.2f} fullBlend={:.2f}] TurnDelta[input=\"{}\" priorRead={} prior={:+.2f} command={:+.2f} wrote={}] damped[var=\"{}\" read={} value={:+.2f}] ownership=OBSERVE_ONLY_R18 vanillaDamping=UNCHANGED root=GOLDEN_R15.8_UNCHANGED",
                    formID, profile, frame, FamilyName(family), rawAvailable, rawAvailable ? rawDegrees : 0.0f,
                    config->turnBlend.deadZoneDegrees, fullErrorDegrees, fullBlend,
                    config->turnBlend.inputVariable, priorInputRead, priorInput, command, wrote,
                    config->turnBlend.dampedVariable, dampedRead, damped);
            }
        }

        void ClearGaitGraphRouter()
        {
            std::scoped_lock lock{ g_gaitRouterMutex };
            for (const auto& [formID, state] : g_gaitRouterActors) {
                SKSE::log::info(
                    "[GaitRouter][SUMMARY] form={:08X} requests={} attempts={} accepted={} rejected={} exhausted={} routeChanges={} active={} lastRoute={} lastFamily={} lastRevision={} lastEvent=\"{}\" structuralRecovery[started={} carrierConfirmed={} exhausted={} parentEventAccepted={} parentEventRejected={} sources[actionStart={} actionRecovered={} standingStart={} standingRecovered={}] ] speedHold[active={} checks={} writes={}]",
                    formID, state.requests, state.attempts, state.accepted, state.rejected,
                    state.exhausted, state.routeChanges, state.lastActive, state.lastRouteValue,
                    FamilyName(state.lastFamily), state.lastRevision, state.lastDesiredEvent,
                    state.nativeRootRecoveriesStarted, state.nativeRootRecoveriesCarrierConfirmed,
                    state.nativeRootRecoveriesExhausted, state.nativeRootRecoveryParentEventsAccepted,
                    state.nativeRootRecoveryParentEventsRejected,
                    state.nativeRootActionRecoveriesStarted, state.nativeRootActionRecoveriesConfirmed,
                    state.locomotionStandingRecoveriesStarted, state.locomotionStandingRecoveriesConfirmed,
                    state.speedGoalHoldActive, state.speedGoalHoldChecks, state.speedGoalHoldWrites);
            }
            for (const auto& [formID, state] : g_turnBlendActors) {
                SKSE::log::info(
                    "[GaitRouter][TURN_SUMMARY] form={:08X} frames={} writes={} writeFailures={} rawUnavailable={} lastFamily={} lastRaw={:+.2f} lastCommand={:+.2f} lastDamped={:+.2f}",
                    formID, state.frames, state.writes, state.writeFailures, state.rawUnavailable,
                    FamilyName(state.lastFamily), state.lastRawDegrees, state.lastCommand, state.lastDamped);
            }
            g_turnBlendActors.clear();
            g_gaitRouterActors.clear();
            g_gaitRouterConfigs.clear();
            g_hkxAuthoredCadenceConfigs.clear();
            g_stableSampleSelectorConfigs.clear();
        }

        struct TranslationGateHookStats
        {
            std::uint64_t calls{ 0 };
            std::uint64_t closed{ 0 };
            std::uint64_t openDb{ 0 };
            std::uint64_t openNative{ 0 };
            std::uint64_t openNativeHold{ 0 };
            std::uint64_t bootstrapOwnedCalls{ 0 };
            std::uint64_t bootstrapNoTxClosed{ 0 };
            std::uint64_t vanillaChangedXY{ 0 };
            std::uint64_t vanillaChangedSecondary{ 0 };
            std::uint64_t logsEmitted{ 0 };
            float maxClosedAuthoredXY{ 0.0f };
            bool nativeGateLatched{ false };
            bool nativeDrivenSeen{ false };
            bool lastNativeDriven{ false };
            std::uint8_t lastAction{ 0xFF };
        };

        std::unordered_map<RE::FormID, TranslationGateHookStats> g_translationGateStats;
        std::mutex g_translationGateStatsMutex;

        [[nodiscard]] float PlanarMagnitude(const RE::NiPoint3& value) noexcept
        {
            const float result = std::hypot(value.x, value.y);
            return std::isfinite(result) ? result : 0.0f;
        }

        [[nodiscard]] float VectorMagnitude(const RE::NiPoint3& value) noexcept
        {
            const float result = std::sqrt(
                value.x * value.x + value.y * value.y + value.z * value.z);
            return std::isfinite(result) ? result : 0.0f;
        }

        struct NativeGateDecision
        {
            bool latchedBefore{ false };
            bool risingEdge{ false };
            bool admitted{ false };
            bool holdIgnoreTo{ false };
        };

        [[nodiscard]] NativeGateDecision UpdateNativeGateLatch(
            RE::Actor* actor,
            bool nativeDriven,
            bool ignoreFromAdmission,
            bool ignoreToContinuity)
        {
            NativeGateDecision result{};
            if (!actor) {
                return result;
            }

            std::scoped_lock lock{ g_translationGateStatsMutex };
            auto& stats = g_translationGateStats[actor->GetFormID()];
            result.latchedBefore = stats.nativeGateLatched;
            result.risingEdge =
                stats.nativeDrivenSeen &&
                !stats.lastNativeDriven &&
                nativeDriven;

            // Native admission needs an actual animation-driven state PLUS either:
            // 1) an observed false->true edge,
            // 2) the graph's raw IGNORE_FROM entry context, or
            // 3) an already-admitted native latch.
            //
            // This intentionally rejects a stale nativeDriven=true value seen on
            // the actor's very first sample (the exact spawn condition observed
            // on the hill) unless the graph subsequently confirms the route.
            result.admitted =
                nativeDriven &&
                (result.latchedBefore ||
                 result.risingEdge ||
                 ignoreFromAdmission);

            // IGNORE_TO is continuity only. It may keep a previously admitted
            // native gate open after nativeDriven falls, never open from CLOSED.
            result.holdIgnoreTo =
                !nativeDriven &&
                result.latchedBefore &&
                ignoreToContinuity;

            stats.nativeGateLatched =
                result.admitted || result.holdIgnoreTo;
            stats.lastNativeDriven = nativeDriven;
            stats.nativeDrivenSeen = true;
            return result;
        }

        void ObserveTranslationGate(
            RE::Actor* actor,
            float deltaTime,
            bool dbTranslationWrite,
            bool dbSyntheticWfm,
            bool dbOwnershipLatch,
            bool dbPmdSource,
            bool dbBootstrapDriven,
            bool nativeDriven,
            bool nativeAdmissionDriven,
            bool ignoreFromAdmission,
            bool ignoreToContinuity,
            const NativeGateDecision& nativeDecision,
            const RE::NiPoint3& authoredTranslation,
            const RE::NiPoint3& vanillaTranslation,
            const RE::NiPoint3& finalTranslation,
            const RE::NiPoint3& authoredSecondary,
            const RE::NiPoint3& vanillaSecondary,
            const RE::NiPoint3& finalSecondary,
            const char* action)
        {
            if (!actor) {
                return;
            }

            const RE::NiPoint3 vanillaAddedTranslation{
                vanillaTranslation.x - authoredTranslation.x,
                vanillaTranslation.y - authoredTranslation.y,
                vanillaTranslation.z - authoredTranslation.z
            };
            const RE::NiPoint3 vanillaAddedSecondary{
                vanillaSecondary.x - authoredSecondary.x,
                vanillaSecondary.y - authoredSecondary.y,
                vanillaSecondary.z - authoredSecondary.z
            };

            const float vanillaAddedXY = PlanarMagnitude(vanillaAddedTranslation);
            const float vanillaAddedSecondaryMagnitude = VectorMagnitude(vanillaAddedSecondary);
            const bool vanillaMutatedXY = vanillaAddedXY > 1.0e-4f;
            const bool vanillaMutatedSecondary = vanillaAddedSecondaryMagnitude > 1.0e-4f;
            const bool finalSecondaryIsVanilla =
                std::fabs(finalSecondary.x - vanillaSecondary.x) <= 1.0e-6f &&
                std::fabs(finalSecondary.y - vanillaSecondary.y) <= 1.0e-6f &&
                std::fabs(finalSecondary.z - vanillaSecondary.z) <= 1.0e-6f;

            bool emit = false;
            std::uint64_t sample = 0;
            TranslationGateHookStats snapshot{};
            {
                std::scoped_lock lock{ g_translationGateStatsMutex };
                auto& stats = g_translationGateStats[actor->GetFormID()];
                ++stats.calls;
                sample = stats.calls;

                if (dbBootstrapDriven) {
                    ++stats.bootstrapOwnedCalls;
                    if (!dbTranslationWrite && !dbSyntheticWfm && !dbOwnershipLatch && !dbPmdSource && std::strcmp(action, "CLOSED") == 0) {
                        ++stats.bootstrapNoTxClosed;
                    }
                }

                if (vanillaMutatedXY) {
                    ++stats.vanillaChangedXY;
                }
                if (vanillaMutatedSecondary) {
                    ++stats.vanillaChangedSecondary;
                }

                if (std::strcmp(action, "OPEN_DB_LATCH") == 0 ||
                    std::strcmp(action, "OPEN_DB_WFM") == 0 ||
                    std::strcmp(action, "OPEN_DB_TX") == 0) {
                    ++stats.openDb;
                } else if (std::strcmp(action, "OPEN_NATIVE") == 0 ||
                           std::strcmp(action, "OPEN_NATIVE_MW1") == 0 ||
                           std::strcmp(action, "OPEN_NATIVE_ROOT_LATCH") == 0) {
                    ++stats.openNative;
                } else if (std::strcmp(action, "OPEN_NATIVE_HOLD_IGNORE_TO") == 0 ||
                           std::strcmp(action, "OPEN_NATIVE_HOLD_IGNORE_TO_MW1") == 0) {
                    ++stats.openNativeHold;
                } else {
                    ++stats.closed;
                    stats.maxClosedAuthoredXY =
                        std::max(stats.maxClosedAuthoredXY, PlanarMagnitude(authoredTranslation));
                }

                const std::uint8_t actionCode =
                    (std::strcmp(action, "OPEN_DB_LATCH") == 0 ||
                     std::strcmp(action, "OPEN_DB_WFM") == 0 ||
                     std::strcmp(action, "OPEN_DB_TX") == 0) ? 1 :
                    (std::strcmp(action, "OPEN_NATIVE") == 0 ||
                     std::strcmp(action, "OPEN_NATIVE_MW1") == 0 ||
                     std::strcmp(action, "OPEN_NATIVE_ROOT_LATCH") == 0) ? 2 :
                    (std::strcmp(action, "OPEN_NATIVE_HOLD_IGNORE_TO") == 0 ||
                     std::strcmp(action, "OPEN_NATIVE_HOLD_IGNORE_TO_MW1") == 0) ? 3 : 0;
                const bool actionChanged = stats.lastAction != actionCode;
                stats.lastAction = actionCode;

                // High-signal without frame spam: first 32 samples, every 240th,
                // every gate state transition, and periodic meaningful CLOSED XY.
                const bool meaningfulClosedSample =
                    actionCode == 0 &&
                    PlanarMagnitude(authoredTranslation) > 0.10f &&
                    (stats.closed % 120 == 0);
                emit =
                    stats.logsEmitted < 32 ||
                    (stats.calls % 240 == 0) ||
                    actionChanged ||
                    meaningfulClosedSample;
                if (emit) {
                    ++stats.logsEmitted;
                }
                snapshot = stats;
            }

            if (!emit) {
                return;
            }

            SKSE::log::debug(
                R"([TRANSLATION_GATE][{}] form={:08X} sample={} dt={:.6f} dbTX={} dbWFM={} dbLatch={} dbPMD={} dbBootstrapOwned={} nativeDrivenRaw={} nativeAdmissionDriven={} ignoreFromAdmission={} ignoreToContinuity={} nativeLatchedBefore={} nativeRisingEdge={} nativeAdmitted={} authoredT=({:+.5f},{:+.5f},{:+.5f}) vanillaT=({:+.5f},{:+.5f},{:+.5f}) finalT=({:+.5f},{:+.5f},{:+.5f}) vanillaAddedXY={:.5f} secondary[authored=({:+.5f},{:+.5f},{:+.5f}) vanilla=({:+.5f},{:+.5f},{:+.5f}) final=({:+.5f},{:+.5f},{:+.5f}) untouchedByGate={}] vanillaAddedSecondary={:.5f} totals[closed={} db={} native={} nativeHold={}])",
                action,
                actor->GetFormID(),
                sample,
                deltaTime,
                dbTranslationWrite,
                dbSyntheticWfm,
                dbOwnershipLatch,
                dbPmdSource,
                dbBootstrapDriven,
                nativeDriven,
                nativeAdmissionDriven,
                ignoreFromAdmission,
                ignoreToContinuity,
                nativeDecision.latchedBefore,
                nativeDecision.risingEdge,
                nativeDecision.admitted,
                authoredTranslation.x, authoredTranslation.y, authoredTranslation.z,
                vanillaTranslation.x, vanillaTranslation.y, vanillaTranslation.z,
                finalTranslation.x, finalTranslation.y, finalTranslation.z,
                vanillaAddedXY,
                authoredSecondary.x, authoredSecondary.y, authoredSecondary.z,
                vanillaSecondary.x, vanillaSecondary.y, vanillaSecondary.z,
                finalSecondary.x, finalSecondary.y, finalSecondary.z,
                finalSecondaryIsVanilla,
                vanillaAddedSecondaryMagnitude,
                snapshot.closed,
                snapshot.openDb,
                snapshot.openNative,
                snapshot.openNativeHold);
        }

        [[nodiscard]] bool IsRotationActionBypassActive(
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

        void UpdateTurnLoopHealthRecovery(RE::Character* character, float deltaTime)
        {
            if (!character || character->IsPlayerRef()) {
                return;
            }

            const auto* profile = MotionProfileDatabase::ResolveProfile(character);
            if (!profile || profile->rotationClips.empty()) {
                return;
            }

            const auto& stopEvent = profile->animationPriority.turnLoopStopEvent;
            if (stopEvent.empty()) {
                return;
            }

            const bool observed =
                SyntheticMotionInjector::HasObservedAuthoredRotationClipForCurrentOrPreviousFrame(character);
            const bool healthy =
                SyntheticMotionInjector::HasProgressingAuthoredRotationClipForCurrentOrPreviousFrame(character);
            const bool actionBypass = IsRotationActionBypassActive(character, *profile);
            const bool runtimeLocomotionCarrier =
                SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(character);

            SteeringIntentController::FrameCommand command{};
            const bool steeringValid =
                SteeringIntentController::GetFrameCommand(character, command) &&
                command.valid && std::isfinite(command.filteredErrorRadians);
            const float steeringErrorDegrees = steeringValid
                ? command.filteredErrorRadians * kRadToDeg
                : 0.0f;
            const bool meaningfulSteering =
                steeringValid && std::fabs(steeringErrorDegrees) >= kTurnLoopMinSteeringErrorDegrees;
            const float dt = std::clamp(
                std::isfinite(deltaTime) ? deltaTime : 0.0f,
                0.0f,
                0.050f);

            const auto formID = character->GetFormID();
            bool issueBreak = false;
            float elapsedAtBreak = 0.0f;
            bool logUnhealthyBegin = false;
            bool logHealthy = false;
            bool logRecovered = false;
            bool logGapHold = false;
            bool logGapBridge = false;
            float gapAtLog = 0.0f;
            const char* recoveryReason = "";

            {
                std::scoped_lock lock{ g_turnLoopHealthMutex };
                auto& state = g_turnLoopHealthActors[formID];

                if (!observed) {
                    // R41: a brief observation hole is not recovery if no actual
                    // locomotion carrier came back. Preserve the unhealthy wall-clock
                    // episode across the common TurnLoop -> StandingIdle -> TurnLoop pulse.
                    if (state.unhealthyEpisode &&
                        !runtimeLocomotionCarrier && !actionBypass && meaningfulSteering) {
                        if (!state.observationGapActive) {
                            state.observationGapActive = true;
                            state.observationGapSeconds = 0.0f;
                            logGapHold = true;
                        }
                        state.observationGapSeconds += dt;
                        if (!state.breakIssued) {
                            state.unhealthySeconds += dt;
                        }
                        gapAtLog = state.observationGapSeconds;

                        if (state.observationGapSeconds <= kTurnLoopObservationGapGraceSeconds) {
                            state.lastObserved = false;
                            state.lastHealthy = false;
                        } else {
                            ++state.recoveries;
                            logRecovered = true;
                            recoveryReason = "ROTATIONCLIP_GAP_EXPIRED";
                            state.lastObserved = false;
                            state.lastHealthy = false;
                            state.unhealthyEpisode = false;
                            state.breakIssued = false;
                            state.observationGapActive = false;
                            state.unhealthySeconds = 0.0f;
                            state.observationGapSeconds = 0.0f;
                        }
                    } else {
                        if (state.unhealthyEpisode || state.breakIssued) {
                            ++state.recoveries;
                            logRecovered = true;
                            recoveryReason = runtimeLocomotionCarrier ?
                                "LOCOMOTION_CARRIER_RESUMED" :
                                "ROTATIONCLIP_DISAPPEARED";
                        }
                        state.lastObserved = false;
                        state.lastHealthy = false;
                        state.unhealthyEpisode = false;
                        state.breakIssued = false;
                        state.observationGapActive = false;
                        state.unhealthySeconds = 0.0f;
                        state.observationGapSeconds = 0.0f;
                    }
                } else if (healthy) {
                    if (!state.lastHealthy) {
                        logHealthy = true;
                    }
                    if (state.unhealthyEpisode || state.breakIssued) {
                        ++state.recoveries;
                        logRecovered = true;
                        recoveryReason = "VALID_AUTHORED_ROOT_RESUMED";
                    }
                    state.lastObserved = true;
                    state.lastHealthy = true;
                    state.unhealthyEpisode = false;
                    state.breakIssued = false;
                    state.observationGapActive = false;
                    state.unhealthySeconds = 0.0f;
                    state.observationGapSeconds = 0.0f;
                } else {
                    if (!state.unhealthyEpisode) {
                        state.unhealthyEpisode = true;
                        ++state.episodes;
                        logUnhealthyBegin = true;
                    } else if (state.observationGapActive) {
                        logGapBridge = true;
                        gapAtLog = state.observationGapSeconds;
                    }
                    state.lastObserved = true;
                    state.lastHealthy = false;
                    state.observationGapActive = false;
                    state.observationGapSeconds = 0.0f;

                    // Attacks/staggers and weak/missing steering are not a TurnLoop stall
                    // recovery condition. Restart the timer, but keep this same observed
                    // unhealthy episode so no event spam can occur.
                    if (actionBypass || !meaningfulSteering) {
                        state.unhealthySeconds = 0.0f;
                    } else if (!state.breakIssued) {
                        state.unhealthySeconds += dt;
                        if (state.unhealthySeconds >= kTurnLoopUnhealthyBreakSeconds) {
                            state.breakIssued = true;  // set before NotifyAnimationGraph: re-entry safe
                            ++state.breakAttempts;
                            elapsedAtBreak = state.unhealthySeconds;
                            issueBreak = true;
                        }
                    }
                }
            }

            if (logHealthy) {
                SKSE::log::info(
                    "[TurnLoopHealth][STATE] form={:08X} observed=true healthy=true authority=AUTHORED_ROTATIONCLIP_YAW_PROGRESS action=KEEP_EXCLUSIVE",
                    formID);
            }
            if (logUnhealthyBegin) {
                SKSE::log::warn(
                    "[TurnLoopHealth][UNHEALTHY_BEGIN] form={:08X} observed=true healthy=false steeringValid={} source={} filteredError={:+.2f}deg actionBypass={} grace={:.3f}s action=WAIT_CONTINUOUS_STALL",
                    formID,
                    steeringValid,
                    SteeringIntentController::SourceName(command.source),
                    steeringErrorDegrees,
                    actionBypass,
                    kTurnLoopUnhealthyBreakSeconds);
            }
            if (logGapHold) {
                SKSE::log::info(
                    "[TurnLoopHealth][R41_GAP_HOLD] form={:08X} observed=false healthy=false carrier={} gap={:.3f}/{:.3f}s action=PRESERVE_UNHEALTHY_EPISODE",
                    formID, runtimeLocomotionCarrier, gapAtLog, kTurnLoopObservationGapGraceSeconds);
            }
            if (logGapBridge) {
                SKSE::log::info(
                    "[TurnLoopHealth][R41_GAP_BRIDGED] form={:08X} observed=true healthy=false gap={:.3f}s unhealthyElapsed=CONTINUED action=DO_NOT_RESET_BREAK_TIMER",
                    formID, gapAtLog);
            }
            if (logRecovered) {
                SKSE::log::info(
                    "[TurnLoopHealth][RECOVERED] form={:08X} reason={} observed={} healthy={} action=RESET_ONE_SHOT_BREAKER",
                    formID, recoveryReason, observed, healthy);
            }

            if (!issueBreak) {
                return;
            }

            const bool accepted = character->NotifyAnimationGraph(stopEvent.c_str());
            {
                std::scoped_lock lock{ g_turnLoopHealthMutex };
                auto it = g_turnLoopHealthActors.find(formID);
                if (it != g_turnLoopHealthActors.end() && accepted) {
                    ++it->second.breakAccepted;
                }
            }

            SKSE::log::warn(
                "[TurnLoopHealth][STALL_BREAK] form={:08X} elapsed={:.3f}s observed=true healthy=false steering[source={} filteredError={:+.2f}deg] event=\"{}\" accepted={} action=ONE_SHOT_TURNSTOP_RETURN_TO_R35_GAIT noReassert=true",
                formID,
                elapsedAtBreak,
                SteeringIntentController::SourceName(command.source),
                steeringErrorDegrees,
                stopEvent,
                accepted);
        }

        void ClearTurnLoopHealthState()
        {
            std::scoped_lock lock{ g_turnLoopHealthMutex };
            for (const auto& [formID, state] : g_turnLoopHealthActors) {
                if (state.episodes == 0 && state.breakAttempts == 0) {
                    continue;
                }
                SKSE::log::info(
                    "[TurnLoopHealth][SUMMARY] form={:08X} episodes={} breakAttempts={} breakAccepted={} recoveries={}",
                    formID,
                    state.episodes,
                    state.breakAttempts,
                    state.breakAccepted,
                    state.recoveries);
            }
            g_turnLoopHealthActors.clear();
        }

        void ClearTranslationGateStats()
        {
            std::scoped_lock lock{ g_translationGateStatsMutex };
            for (const auto& [formID, stats] : g_translationGateStats) {
                SKSE::log::info(
                    "[TRANSLATION_GATE][SUMMARY] form={:08X} calls={} closed={} openDb={} openNative={} openNativeHold={} bootstrapOwnedCalls={} bootstrapNoTxClosed={} vanillaChangedXY={} vanillaChangedSecondary={} maxClosedAuthoredXY={:.5f}",
                    formID,
                    stats.calls,
                    stats.closed,
                    stats.openDb,
                    stats.openNative,
                    stats.openNativeHold,
                    stats.bootstrapOwnedCalls,
                    stats.bootstrapNoTxClosed,
                    stats.vanillaChangedXY,
                    stats.vanillaChangedSecondary,
                    stats.maxClosedAuthoredXY);
            }
            g_translationGateStats.clear();
        }

        struct CharacterUpdateHook
        {
            static void Thunk(RE::Character* character, float deltaTime)
            {
                const bool creatureEligible =
                    SyntheticMotionInjector::IsUniversalCreatureEligible(character);
                const bool motionActor = creatureEligible &&
                    MotionProfileDatabase::SupportsActor(character);
                const bool speedActor = motionActor && SpeedIntentTranslator::SupportsActor(character);
                const bool stableSampleSelector = motionActor && StableSampleSelectorEnabledForActor(character);
                const bool rotationActor = motionActor && RotationIntentTranslator::SupportsActor(character);
                bool nativeStandingTurnWindowActive = false;

                if (motionActor) {
                    // R96: cache the physical Character::Update timestep before any native
                    // graph/Havok work. The clip hook later consumes this exact actor-frame
                    // clock for 1x RotationClips instead of a child-local sync timestep.
                    {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        auto& state = g_gaitRouterActors[character->GetFormID()];
                        state.physicalFrameDeltaTime =
                            (std::isfinite(deltaTime) && deltaTime > 0.0f && deltaTime <= 0.100f)
                            ? deltaTime : 0.0f;
                    }

                    // R35: sample the AI/path goal before Bethesda's angular controller
                    // turns it into rotationSpeed/V13 state. ProcessMotionData consumes
                    // this frozen ALR steering command later in the same actor frame.
                    SteeringIntentController::BeginFrame(character, deltaTime);
                    RotationIntentTranslator::BeforeCharacterUpdate(character, deltaTime);

                    if (stableSampleSelector) {
                        const auto nativeTurnConfig = StableSampleSelectorConfigForProfile(
                            MotionProfileDatabase::DatabaseFolderForActor(character));
                        if (nativeTurnConfig && nativeTurnConfig->enabled) {
                            nativeStandingTurnWindowActive =
                                UpdateNativeStandingTurnWindowPreNative(
                                    character, *nativeTurnConfig, deltaTime);
                        }
                    }

                    TranslationAuthority::BeginFrame(character);
                    if (speedActor) {
                        SpeedIntentTranslator::BeforeUpdate(character, deltaTime);

                        // R54: R48 proves Bethesda can overwrite graph Speed to an
                        // intermediate/native value during the update even while a real RUN
                        // carrier is already active. R51 writes later, before Havok, but the
                        // state-machine transition can already have selected RUN -> WALK by
                        // then. Reassert the profile RUN goal at the earliest Character
                        // boundary, but ONLY with a certified RUN carrier from the generated
                        // current/previous frame. Lower-family and carrierless states remain
                        // protected by R50/R51 and receive no early RUN-speed write.
                        if (!stableSampleSelector && !AnimationPriority::IsBusy(character)) {
                            SteeringIntentController::FrameCommand preSteering{};
                            const bool preSteeringValid =
                                SteeringIntentController::GetFrameCommand(character, preSteering) &&
                                preSteering.valid;
                            if (preSteeringValid) {
                                (void)ApplyParentActiveSpeedGoalHold(character, false, false, true);
                            }
                        }

                    }

                    // R93: stable physical selection must exist BEFORE native Character::Update.
                    // R92 wrote only after native evaluation, which could not stop Bethesda from
                    // already selecting Slow/Fast or a stale directional child for this frame.
                    if (stableSampleSelector && !AnimationPriority::IsBusy(character)) {
                        (void)UpdateStableSampleSelectorPreNative(character, deltaTime);
                    }
                    const auto stablePivotPhase = stableSampleSelector
                        ? StablePivotPhaseForActor(character->GetFormID())
                        : StablePivotPhase::kIdle;
                    const bool stableAuthoredPivotBusy = stablePivotPhase != StablePivotPhase::kIdle;

                    // R103 behavior is preserved exactly: PENDING/ACTIVE publishes the
                    // external authored-turn witness and PENDING still opens the standing-yaw
                    // pass. R104 adds diagnostic context only; it does NOT change this policy.
                    bool stablePivotTurnStopIssued = false;
                    std::uint32_t stablePivotRouteDegrees = 0;
                    bool stablePivotNativeHkbWitness = false;
                    float stablePivotNativeHkbWitnessAgeSeconds = -1.0f;
                    float stablePivotNativeHkbPoseWeight = 0.0f;
                    std::string stablePivotNativeHkbKey;
                    bool stablePivotLeft = false;
                    float stablePivotPhysicalYawRateDegreesPerSecond = 0.0f;

                    if (stablePivotPhase != StablePivotPhase::kIdle) {
                        const double now = StablePivotMonotonicSeconds();
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        if (const auto it = g_gaitRouterActors.find(character->GetFormID());
                            it != g_gaitRouterActors.end()) {
                            stablePivotTurnStopIssued = it->second.stablePivot90TurnStopIssued;
                            stablePivotRouteDegrees = it->second.stablePivotRouteDegrees;
                            stablePivotLeft = it->second.stablePivotLeft;
                            stablePivotNativeHkbKey = it->second.stablePivotNativeRotationKey;
                            stablePivotNativeHkbPoseWeight = it->second.stablePivotNativeRotationPoseWeight;
                            stablePivotNativeHkbWitness =
                                it->second.stablePivotNativeRotationSeenFrame > 0 &&
                                it->second.stablePivotFrameSerial <=
                                    it->second.stablePivotNativeRotationSeenFrame + 1;

                            if (it->second.stablePivotNativeRotationLastSeenSeconds > 0.0) {
                                const double age = now - it->second.stablePivotNativeRotationLastSeenSeconds;
                                if (std::isfinite(age) && age >= 0.0) {
                                    stablePivotNativeHkbWitnessAgeSeconds = static_cast<float>(age);
                                }
                            }
                        }
                    }

                    if (stablePivotRouteDegrees == 90 && stablePivotPhase != StablePivotPhase::kIdle) {
                        // R109 diagnostics and lifecycle use the final composed behavior root,
                        // not child MTTurn clip activity. Overwrite the historical HKB fields
                        // with the final-output witness so YAW_CHAIN telemetry describes the
                        // same authority that V13 actually transports.
                        float finalBehaviorPhysicalYaw = 0.0f;
                        std::int32_t finalBehaviorOutputIndex = -1;
                        stablePivotNativeHkbWitness = HasFinalBehaviorPivotYawWitness(
                            character, stablePivotLeft, finalBehaviorPhysicalYaw, finalBehaviorOutputIndex);
                        stablePivotNativeHkbWitnessAgeSeconds = stablePivotNativeHkbWitness ? 0.0f : -1.0f;
                        stablePivotNativeHkbPoseWeight = stablePivotNativeHkbWitness
                            ? std::abs(finalBehaviorPhysicalYaw) : 0.0f;
                        stablePivotNativeHkbKey = stablePivotNativeHkbWitness
                            ? (std::string("FINAL_BEHAVIOR_OUTPUT_") + std::to_string(finalBehaviorOutputIndex))
                            : std::string{};
                    }

                    if (stablePivotRouteDegrees == 90 && stableSampleSelector) {
                        const auto pivotConfig = StableSampleSelectorConfigForProfile(
                            MotionProfileDatabase::DatabaseFolderForActor(character));
                        if (pivotConfig &&
                            std::isfinite(pivotConfig->authoredPivot90YawRateDegreesPerSecond) &&
                            pivotConfig->authoredPivot90YawRateDegreesPerSecond > 0.0f) {
                            const float magnitude = pivotConfig->authoredPivot90YawRateDegreesPerSecond;
                            // Physical convention recovered from R104/R106: right turns increase
                            // world yaw and left turns decrease it. Pass the signed command to V13.
                            stablePivotPhysicalYawRateDegreesPerSecond = stablePivotLeft ? -magnitude : magnitude;
                        }
                    }

                    const bool externalPivotWitness =
                        stablePivotPhase == StablePivotPhase::kPending ||
                        stablePivotPhase == StablePivotPhase::kActive ||
                        nativeStandingTurnWindowActive;
                    // R116: while the serialized canned-turn window is latched, external witness
                    // suppresses competing RuntimeYaw R1/R2 upstream. V13 remains open so the
                    // selected canned 90/180 clip can transport its authored native yaw.
                    const bool allowNativeStandingYaw = nativeStandingTurnWindowActive
                        ? true
                        : (stablePivotRouteDegrees == 90
                            ? (stablePivotPhase == StablePivotPhase::kActive &&
                               !stablePivotTurnStopIssued)
                            : externalPivotWitness);
                    CapsuleSlerp::SetExternalAuthoredTurnWitness(
                        character, externalPivotWitness, allowNativeStandingYaw);

                    // R116: patched behavior uses authored single-play canned 90/180 states.
                    // The old R115 physical-rate servo is explicitly disabled; V13 only passes
                    // the native/authored yaw produced by the selected canned clip.
                    CapsuleSlerp::SetNativeStandingTurnPhysicalYawRate(character, false);

                    // R104 diagnostic-only graph sample. Troll uses TurnDelta as the native
                    // MT_TurnLeft/Right_Blend parameter. This is READ ONLY; R100 proved that
                    // writing the variable makes the native graph oscillate. Keep the literal
                    // here intentionally scoped to the diagnostic build and only while a
                    // StablePivot exists.
                    float stablePivotTurnDelta = 0.0f;
                    const bool stablePivotTurnDeltaRead =
                        stablePivotPhase != StablePivotPhase::kIdle &&
                        character->GetGraphVariableFloat("TurnDelta", stablePivotTurnDelta) &&
                        std::isfinite(stablePivotTurnDelta);

                    CapsuleSlerp::SetStablePivotDiagnosticContext(
                        character,
                        static_cast<std::uint8_t>(stablePivotPhase),
                        stablePivotRouteDegrees,
                        stablePivotNativeHkbWitness,
                        stablePivotNativeHkbWitnessAgeSeconds,
                        stablePivotNativeHkbKey.c_str(),
                        stablePivotTurnDeltaRead,
                        stablePivotTurnDelta,
                        stablePivotPhysicalYawRateDegreesPerSecond);
                    if (stablePivotPhase != StablePivotPhase::kIdle && stablePivotNativeHkbWitness) {
                        SKSE::log::debug(
                            "[StablePivot][R109_FINAL_BEHAVIOR_WITNESS_STATE] form={:08X} phase={} route={} key=\"{}\" poseWeight={:.5f} ageMs={:.2f} proof=FINAL_COMPOSED_BEHAVIOR_EXTRACTED_MOTION",
                            character->GetFormID(), StablePivotPhaseName(stablePivotPhase),
                            stablePivotRouteDegrees, stablePivotNativeHkbKey,
                            stablePivotNativeHkbPoseWeight,
                            stablePivotNativeHkbWitnessAgeSeconds >= 0.0f
                                ? stablePivotNativeHkbWitnessAgeSeconds * 1000.0f : -1.0f);
                    }

                    // R82: GaitRouter is a profile graph adapter, not a SpeedIntent feature.
                    // ParentOnly and DirectionIntent must run for any MotionDB-supported actor
                    // even when SpeedIntent is intentionally disabled. The discrete family
                    // branch still self-gates on SpeedIntentTranslator::SupportsActor().
                    if (!AnimationPriority::IsBusy(character) && !stableAuthoredPivotBusy) {
                        ApplyGaitGraphRouter(character, deltaTime);
                    }

                    // R77: data-only behavior extensions run for every supported profile,
                    // independently of whether SpeedIntent/GaitRouter is enabled. Rules are
                    // edge-triggered and may only send configured graph events; they never
                    // write root, Speed, Direction, TurnDelta, capsule yaw, or clip localTime.
                    UpdateBehaviorExtensions(character, deltaTime);

                    // R71 continuous-first pivot arbitration: RotationIntent still supplies the
                    // validated combat-facing error, but AnimationPriority may only consume a
                    // Standing state that native AI/graph already established. It never sends
                    // moveStop/turnStop and never writes Speed. IsBusy() begins only when the
                    // canned animation actually owns bAnimationDriven (or safety cleanup).
                    AnimationPriority::UpdateActor(character, rotationActor, deltaTime);

                    SyntheticMotionInjector::BeginActorFrame(character, deltaTime);

                    // R71: pending canned requests are non-authoritative, so TurnLoop health and
                    // locomotion recovery continue normally until the canned animation actually
                    // becomes active. Only true animation ownership/safety cleanup suppresses it.
                    if (!AnimationPriority::IsBusy(character) && !stableAuthoredPivotBusy) {
                        detail::UpdateTurnLoopHealthRecovery(character, deltaTime);
                    }

                    SyntheticMotionInjector::BeginAnimationEvaluation(character);

                    // R28: snapshot CapsuleSlerp policy only AFTER the root lifecycle has
                    // established this frame's bAnimationDriven state. V13 remains purely
                    // physical and performs no graph/profile/injector queries itself.
                    CapsuleSlerp::BeginRuntimeYawFrame(character, true, deltaTime);
                } else if (creatureEligible) {
                    // C1 universal profileless semantic lane. AutoGraph remains responsible
                    // only for graph/gait selection. Physical root is independent: PMD supplies
                    // native leaf deltas to TRACK_EXTRACTED_MOTION and the final HKB graph output
                    // is consumed later at ModifyMovementData.
                    SteeringIntentController::BeginFrame(character, deltaTime);
                    SpeedIntentTranslator::BeforeUniversalUpdate(character, deltaTime);
                    SpeedIntentTranslator::ApplyAutoGraphAdapterPreNative(character, deltaTime);

                    CapsuleSlerp::SetExternalAuthoredTurnWitness(character, false);
                    CapsuleSlerp::SetNativeStandingTurnPhysicalYawRate(character, false);
                    CapsuleSlerp::SetStablePivotDiagnosticContext(
                        character, 0, 0, false, -1.0f, "", false, 0.0f, 0.0f);
                    AnimationPriority::UpdateActor(character, false, deltaTime);

                    // Frame bookkeeping only. C1 does not force bAnimationDriven and has no
                    // profileless root-session/PREARM/handoff state machine.
                    SyntheticMotionInjector::BeginActorFrame(character, deltaTime);

                    // Procedural yaw uses PMD R1/R2 independently of root acquisition. V13 is
                    // registered for the actor lifetime only as a native-capsule-yaw firewall.
                    CapsuleSlerp::BeginRuntimeYawFrame(character, true, deltaTime);
                }

                Func(character, deltaTime);

                if (motionActor) {
                    SyntheticMotionInjector::EndAnimationEvaluation(character);
                    TranslationAuthority::EndFrame(character);

                    // R48: ModifyAnimationUpdateData is not the final writer of graph Speed.
                    // The R47 runtime log proved Bethesda can overwrite our semantic gait
                    // goal later in Character::Update, leaving family=RUN with Speed=5 and
                    // WalkSlowForward clips barely advancing. Re-assert only at the final
                    // Character::Update boundary, and only while current AI steering still
                    // proves locomotion intent. Parent-active/route/family gates inside the
                    // helper still prevent this write from starting locomotion.
                    if (speedActor && !stableSampleSelector && !AnimationPriority::IsBusy(character)) {
                        SteeringIntentController::FrameCommand postSteering{};
                        const bool postSteeringValid =
                            SteeringIntentController::GetFrameCommand(character, postSteering) &&
                            postSteering.valid;
                        if (postSteeringValid) {
                            (void)ApplyParentActiveSpeedGoalHold(character, false, true);
                        }
                    }

                    // R70 SDK producer: publish one immutable read-only snapshot only after
                    // the animal frame has completed ALR/native update and final graph holds.
                    LocomotionSDK::PublishActorFrame(character);
                }
            }
            static inline REL::Relocation<decltype(Thunk)> Func;
        };

        struct ModifyAnimationUpdateDataHook
        {
            static void Thunk(RE::Actor* actor, RE::BSAnimationUpdateData& data)
            {
                Func(actor, data);

                // B8 final structural-selector boundary. The BSSpeedSampler may rewrite
                // SpeedSampled during native graph update, so reassert ONLY the selector
                // already decided pre-native. No events, no gait/direction re-decision and
                // no physical authority change occur here. This is required even before
                // root handoff so the graph can reach the intended locomotion carrier.
                if (actor && SyntheticMotionInjector::IsUniversalCreatureEligible(actor) &&
                    !MotionProfileDatabase::SupportsActor(actor)) {
                    SpeedIntentTranslator::ApplyAutoGraphAdapterFinal(actor);
                }

                // R55 priority rule: explicit profile-driven canned pivot authority wins
                // over every locomotion graph writer. STOPPING/PENDING/ACTIVE/CLEANUP may
                // therefore reassert Speed/TurnDelta/suppression through AnimationPriority,
                // while R47-R54 speed maintenance and R38 curve input remain dormant.
                const bool animationPriorityBusy = actor && !actor->IsPlayerRef() &&
                    AnimationPriority::IsBusy(actor);
                const bool stableAuthoredPivotBusy = actor && !actor->IsPlayerRef() &&
                    StableSamplePivotBusy(actor);
                const bool pivotBusy = animationPriorityBusy || stableAuthoredPivotBusy;

                const auto* motionProfile = actor && !actor->IsPlayerRef()
                    ? MotionProfileDatabase::ResolveProfile(actor)
                    : nullptr;
                const bool actionBypass = motionProfile &&
                    IsRotationActionBypassActive(actor, *motionProfile);
                const bool stableSampleSelector = actor && !actor->IsPlayerRef() &&
                    StableSampleSelectorEnabledForActor(actor);

                // R83: final profile Direction writer. R82 wrote before Character::Update,
                // then Bethesda rewrote/damped Direction and produced frame-to-frame fights.
                // At this boundary native graph updates are complete and Havok has not yet
                // evaluated the blendspace, so this becomes the single final intent input.
                if (!pivotBusy && !actionBypass && !stableSampleSelector) {
                    ApplyProfileDirectionIntentFinal(actor);
                }

                if (!animationPriorityBusy && !actionBypass && stableSampleSelector) {
                    if (stableAuthoredPivotBusy) {
                        const auto profileFolder = MotionProfileDatabase::DatabaseFolderForActor(actor);
                        const auto stableConfig = StableSampleSelectorConfigForProfile(profileFolder);
                        if (stableConfig) {
                            (void)ApplyStablePivotTurnParameter(
                                actor, *stableConfig, "FINAL_PRE_HAVOK", false);
                        }
                    }

                    // R93 final boundary is a pure reassert of the PRE-NATIVE decision.
                    // It cannot advance selector state. During authored PENDING/ACTIVE pivot
                    // phases the helper intentionally writes nothing; STOPPING may hold the
                    // outgoing sample stable while the locomotion parent exits.
                    (void)ReassertStableSampleSelectorFinal(actor);
                } else if (!pivotBusy && !actionBypass) {
                    // R51: prevent RUN speed injection while runtime proves only WALK/TROT.
                    std::uint8_t runMismatchMask = 0;
                    const bool blockNormalRunSpeed =
                        RunSpeedCarrierMismatch(actor, runMismatchMask);

                    bool normalSpeedGoalWrote = false;
                    if (!blockNormalRunSpeed) {
                        normalSpeedGoalWrote = SpeedIntentTranslator::ApplyGraphGoal(actor);
                    } else if (actor && !actor->IsPlayerRef()) {
                        const auto formID = actor->GetFormID();
                        std::uint64_t blocks = 0;
                        bool emit = false;
                        {
                            std::scoped_lock lock{ g_gaitRouterMutex };
                            auto& state = g_gaitRouterActors[formID];
                            blocks = ++state.runSpeedPreAnimationBlocks;
                            emit = !state.runSpeedPreAnimationBlocked || (blocks % 120) == 0;
                            state.runSpeedPreAnimationBlocked = true;
                        }
                        if (emit) {
                            SKSE::log::info(
                                "[SpeedIntent][R51_RUN_WRITE_BLOCKED_PRE_ANIMATION] form={:08X} carrierMask=0x{:02X} family=RUN proof=GENERATED_RUNTIME_ROOT_HAS_WALK_OR_TROT_AND_NO_RUN action=SKIP_NORMAL_SPEEDINTENT_RUN_GOAL_PRESERVE_NATIVE_GRAPH_SPEED",
                                formID, runMismatchMask);
                        }
                    }

                    if (!blockNormalRunSpeed && actor && !actor->IsPlayerRef()) {
                        std::scoped_lock lock{ g_gaitRouterMutex };
                        const auto it = g_gaitRouterActors.find(actor->GetFormID());
                        if (it != g_gaitRouterActors.end()) {
                            it->second.runSpeedPreAnimationBlocked = false;
                        }
                    }

                    (void)ApplyParentActiveSpeedGoalHold(actor, normalSpeedGoalWrote, false);
                    ApplyMovingTurnBlend(actor);
                } else if (!pivotBusy && actionBypass && actor && motionProfile) {
                    const auto family = SpeedIntentTranslator::LatchedFamily(actor->GetFormID());
                    const float goal = SpeedGoalForFamily(family, motionProfile->speed);
                    float prior = 0.0f;
                    const bool priorRead = !motionProfile->speed.graphSpeed.empty() &&
                        actor->GetGraphVariableFloat(motionProfile->speed.graphSpeed.c_str(), prior);
                    NoteR72NativeSpeedPreserve(
                        actor, family, "MODIFY_ANIMATION_UPDATE_PRE_HAVOK",
                        "ACTION_BYPASS", priorRead, prior, goal);
                }

                // Must be last: during pivot staging/ownership this reasserts the profile's
                // explicit HOLD/ANIMATION graph state after every other writer. In ordinary
                // locomotion it is a no-op because AnimationPriority is idle.
                (void)AnimationPriority::ApplyGraphAuthority(actor);
            }
            static inline REL::Relocation<decltype(Thunk)> Func;
        };

        struct ModifyMovementDataHook
        {
            static void Thunk(
                RE::Actor* actor,
                float deltaTime,
                RE::NiPoint3& translation,
                RE::NiPoint3& secondaryMovement)
            {
                const bool creatureEligible =
                    SyntheticMotionInjector::IsUniversalCreatureEligible(actor);
                const bool profileMotionActor =
                    creatureEligible && MotionProfileDatabase::SupportsActor(actor);

                // C1 profileless physical lane: the capsule gate never consumes raw PMD T1/T2.
                // Native MotionData was already written into locomotion leaf TRACK_EXTRACTED_MOTION;
                // Havok composed it through the live behavior graph. Only that FINAL composed root
                // is admitted here. Before first proof we fail open to vanilla; after acquisition a
                // bounded four-frame velocity bridge preserves continuity, then stale root fails closed.
                if (creatureEligible && !profileMotionActor) {
                    Func(actor, deltaTime, translation, secondaryMovement);
                    RE::NiPoint3 composedRoot{};
                    const auto rootResult = SyntheticMotionInjector::ResolveUniversalExtractedTranslation(
                        actor, deltaTime, composedRoot);
                    switch (rootResult) {
                    case SyntheticMotionInjector::UniversalRootResult::kExact:
                    case SyntheticMotionInjector::UniversalRootResult::kBridge:
                        translation.x = composedRoot.x;
                        translation.y = composedRoot.y;
                        break;
                    case SyntheticMotionInjector::UniversalRootResult::kHoldZero:
                        translation.x = 0.0f;
                        translation.y = 0.0f;
                        break;
                    case SyntheticMotionInjector::UniversalRootResult::kNone:
                    default:
                        break;
                    }
                    return;
                }

                const bool motionActor = profileMotionActor;
                if (!motionActor) {
                    Func(actor, deltaTime, translation, secondaryMovement);
                    return;
                }

                // The gate owns PLANAR TRANSLATION ONLY.
                // Rotation/secondary is intentionally outside this subsystem.
                const RE::NiPoint3 authoredTranslation = translation;
                const RE::NiPoint3 authoredSecondary = secondaryMovement;

                Func(actor, deltaTime, translation, secondaryMovement);

                // Authority is evaluated after vanilla. Creature MotionDB authority
                // is the synthetic clip WFM generated in the current/previous graph
                // evaluation; TX_WRITE remains only a legacy/player fallback.
                const bool dbTranslationWrite =
                    SyntheticMotionInjector::HasExternalTranslationForCurrentFrame(actor);
                const bool dbSyntheticWfm =
                    SyntheticMotionInjector::HasSyntheticWfmTranslationForCurrentOrPreviousFrame(actor);
                const bool nativeMotionWitness =
                    SyntheticMotionInjector::HasNativeMotionWitnessForCurrentOrPreviousFrame(actor);
                const bool nativeDriven = actor->IsAnimationDriven();

                // B2 keeps the existing planar gate, but native locomotion has its own bounded session. The acquisition proof is
                // a real vanilla MotionDataContainer T1->T2 or R1->R2 delta observed at the
                // already-hooked ProcessMotionData call sites. MotionDB membership is no
                // longer required to certify the root session.
                if (nativeMotionWitness && nativeDriven) {
                    SyntheticMotionInjector::ConfirmNativeLocomotionRootAuthorityAtGate(actor);
                }
                const bool nativeRootOwnership =
                    SyntheticMotionInjector::HasNativeLocomotionRootAuthorityForCurrentFrame(actor);
                const bool dbOwnershipLatch =
                    SyntheticMotionInjector::HasDbMotionSourceAuthorityForCurrentFrame(actor);
                // R15.8: PMD never directly opens the gate. Matching PMD only
                // refreshes an already-acquired lifecycle session inside the injector.
                const bool dbPmdSource = false;
                const bool dbBootstrapDriven =
                    SyntheticMotionInjector::IsDbBootstrapDrivenForCurrentFrame(actor);
                const bool nativeBootstrapDriven =
                    SyntheticMotionInjector::IsNativeLocomotionBootstrapDrivenForCurrentFrame(actor);

                // A frame-scoped bAnimationDriven=true written by the DB/bootstrap lane
                // exists ONLY to make Havok enter ProcessMotionData. It is explicitly
                // forbidden from masquerading as vanilla/native root authority.
                // It cannot masquerade as native authority; DB WFM has its own gate key.
                const bool nativeAdmissionDriven =
                    nativeDriven && !dbBootstrapDriven && !nativeBootstrapDriven;

                // Raw WFM tags never create authority by themselves.
                // IGNORE_FROM is admission context only while nativeAdmissionDriven=true.
                // IGNORE_TO is continuity context only for an already-open native gate.
                const bool ignoreFromAdmission =
                    SyntheticMotionInjector::HasIgnoreFromTransitionForCurrentOrPreviousFrame(actor);
                const bool ignoreToContinuity =
                    SyntheticMotionInjector::HasIgnoreToTransitionForCurrentOrPreviousFrame(actor);
                const NativeGateDecision nativeDecision = UpdateNativeGateLatch(
                    actor,
                    nativeAdmissionDriven,
                    ignoreFromAdmission,
                    ignoreToContinuity);

                const RE::NiPoint3 vanillaTranslation = translation;
                const RE::NiPoint3 vanillaSecondary = secondaryMovement;
                const char* action = "CLOSED";

                const bool dbRootAuthority =
                    dbOwnershipLatch || dbSyntheticWfm || dbTranslationWrite;
                const bool dbRootCarrierAdmitted =
                    dbRootAuthority && nativeDriven;

                // R67 MW1 sits strictly INSIDE the already-proven native action gate.
                // It cannot open authority and it never touches MotionDB root. The input
                // is the same pre-vanilla authored translation R15.8 already admits; MW1
                // may only rescale XY for a configured finite native combat action.
                RE::NiPoint3 nativeAuthoredTranslation = authoredTranslation;
                bool motionWarped = false;
                if (!dbRootCarrierAdmitted && (nativeDecision.admitted || nativeDecision.holdIgnoreTo)) {
                    motionWarped = MotionTrajectoryController::WarpNativeActionTranslation(
                        actor, deltaTime, authoredTranslation, nativeAuthoredTranslation);
                }

                if (nativeRootOwnership && nativeDriven) {
                    // B2 universal native locomotion carrier. Authority is acquired only
                    // after a real vanilla T1->T2 translation delta inside the bounded
                    // native locomotion session. MotionDB/WFM is not part of this lane.
                    translation.x = authoredTranslation.x;
                    translation.y = authoredTranslation.y;
                    action = "OPEN_NATIVE_ROOT_LATCH";
                } else if (dbRootCarrierAdmitted) {
                    // R28 physical carrier contract: MotionDB proof is necessary but not
                    // sufficient. The graph must currently report bAnimationDriven=true.
                    // This forbids synthetic planar movement without an animation carrier.
                    // R15.8 lifecycle:
                    // - OPEN_DB_WFM is acquisition proof only.
                    // - OPEN_DB_LATCH is a previously certified ownership that is
                    //   still alive; missing Generate callbacks do not close it.
                    // - PMD never directly enters this branch.
                    translation.x = authoredTranslation.x;
                    translation.y = authoredTranslation.y;
                    action = dbOwnershipLatch ? "OPEN_DB_LATCH" :
                        (dbSyntheticWfm ? "OPEN_DB_WFM" : "OPEN_DB_TX");

                    if (dbSyntheticWfm) {
                        TranslationAuthority::ObserveMotionDbTranslation(actor);
                        const float authoredPlanar = std::hypot(
                            authoredTranslation.x, authoredTranslation.y);
                        if (std::isfinite(authoredPlanar) && authoredPlanar > 1.0e-4f) {
                            SyntheticMotionInjector::ConfirmDbRootOwnershipAtGate(actor);
                        }
                    }
                    TranslationAuthority::ObservePlanarOverride(
                        actor, authoredTranslation, vanillaTranslation, translation);
                } else if (nativeDecision.admitted) {
                    // Native animation-driven state has been admitted by a real
                    // false->true edge, IGNORE_FROM entry context, or an existing
                    // admitted latch. Only pre-vanilla authored XY crosses.
                    translation.x = nativeAuthoredTranslation.x;
                    translation.y = nativeAuthoredTranslation.y;
                    action = motionWarped ? "OPEN_NATIVE_MW1" : "OPEN_NATIVE";
                } else if (nativeDecision.holdIgnoreTo) {
                    // Continuity only: IGNORE_TO says the blend is still preserving
                    // the FROM worldFromModel. It can keep an already-open native
                    // gate alive, but never open a closed gate by itself.
                    translation.x = nativeAuthoredTranslation.x;
                    translation.y = nativeAuthoredTranslation.y;
                    action = motionWarped ? "OPEN_NATIVE_HOLD_IGNORE_TO_MW1" : "OPEN_NATIVE_HOLD_IGNORE_TO";
                } else {
                    // Absolute planar translation lock. Vertical movement remains
                    // post-vanilla for gravity/terrain/collision. If MotionDB evidence
                    // exists but bAnimationDriven is false, this is an intentional hard
                    // reject: the capsule may never translate without its animation carrier.
                    translation.x = 0.0f;
                    translation.y = 0.0f;
                    action = dbRootAuthority && !nativeDriven ?
                        "CLOSED_DB_NO_ANIMDRIVEN" : "CLOSED";
                }

                // CRITICAL R15.6 CONTRACT:
                // NEVER touch secondaryMovement here. Capsule rotation/angle belongs
                // to a separate subsystem and must remain exactly as vanilla left it.

                ObserveTranslationGate(
                    actor,
                    deltaTime,
                    dbTranslationWrite,
                    dbSyntheticWfm,
                    dbOwnershipLatch,
                    dbPmdSource,
                    dbBootstrapDriven,
                    nativeDriven,
                    nativeAdmissionDriven,
                    ignoreFromAdmission,
                    ignoreToContinuity,
                    nativeDecision,
                    authoredTranslation,
                    vanillaTranslation,
                    translation,
                    authoredSecondary,
                    vanillaSecondary,
                    secondaryMovement,
                    action);
            }
            static inline REL::Relocation<decltype(Thunk)> Func;
        };

        [[nodiscard]] bool EndsWithLower(std::string_view value, std::string_view suffix) noexcept
        {
            return value.size() >= suffix.size() &&
                value.substr(value.size() - suffix.size()) == suffix;
        }

        // R94: match AnimationMotionDatabase::NormalizeKey semantics. HKX assets may carry
        // their profile folder as a suffix (e.g. TurnLoopingR_Wolf.hkx), while Intent.ini
        // and MotionDB use the canonical key turnloopingr. R91 compared the raw stem and
        // therefore silently missed the entire Wolf physical-clock path.
        [[nodiscard]] std::string CanonicalHkxKeyFromAnimationName(
            const char* rawAnim,
            std::string_view profileFolder)
        {
            if (!rawAnim || !rawAnim[0]) {
                return {};
            }
            std::string normalized = LowerCopy(rawAnim);
            std::replace(normalized.begin(), normalized.end(), '\\', '/');
            const auto slash = normalized.find_last_of('/');
            if (slash != std::string::npos) {
                normalized.erase(0, slash + 1);
            }
            if (const auto dot = normalized.rfind('.'); dot != std::string::npos) {
                normalized.erase(dot);
            }

            const std::string rawProfile = LowerCopy(std::string{ profileFolder });
            const std::string profileSuffix = rawProfile.empty() ? std::string{} : "_" + rawProfile;
            if (!profileSuffix.empty() && EndsWithLower(normalized, profileSuffix)) {
                normalized.resize(normalized.size() - profileSuffix.size());
            }
            normalized.erase(std::remove_if(normalized.begin(), normalized.end(), [](unsigned char c) {
                return c == '_' || c == ' ' || c == '-';
            }), normalized.end());
            return normalized;
        }

        [[nodiscard]] bool IsConfiguredAuthoredCadenceKey(
            const auto& profile,
            const HkxAuthoredCadenceConfig& config,
            const std::string& key,
            bool stablePivotBusy)
        {
            if (key.empty()) {
                return false;
            }
            const bool runtimeYaw = config.includeRuntimeYawClips &&
                std::find(profile.runtimeYawClips.begin(), profile.runtimeYawClips.end(), key) !=
                    profile.runtimeYawClips.end();
            bool rotation = config.includeRotationClips &&
                std::find(profile.rotationClips.begin(), profile.rotationClips.end(), key) !=
                    profile.rotationClips.end();
            if (rotation && config.rotationRequiresStablePivot && !stablePivotBusy) {
                rotation = false;
            }
            return runtimeYaw || rotation;
        }

        struct AuthoredCadenceUpdateDecision
        {
            bool apply{ false };
            bool temporaryPlaybackRateOverride{ false };
            bool nativeRateCompensation{ false };
            float physicalTimestep{ 0.0f };
            float nativeUpdateTimestep{ 0.0f };
            float inputTimestep{ 0.0f };
            float configuredNativeRate{ 0.0f };
            float preClipLocalTime{ 0.0f };
            float priorClipRate{ 1.0f };
            float priorControlRate{ 1.0f };
            float poseWeight{ 0.0f };
            RE::hkaDefaultAnimationControl* control{ nullptr };
            RE::FormID actorFormID{ 0 };
            std::string profileFolder;
            std::string key;
            std::string nodeName;
            std::string animationName;
            float authoredRate{ 1.0f };
        };

        [[nodiscard]] bool StablePivotNodeMatchesLatchedRoute(
            RE::Actor* actor,
            const char* rawNodeName,
            const std::string& animationKey)
        {
            if (!actor || !rawNodeName || !rawNodeName[0]) return false;
            const auto profileFolder = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto stableConfig = StableSampleSelectorConfigForProfile(profileFolder);
            if (!stableConfig) return true;

            bool turnLeft = false;
            std::uint32_t routeDegrees = 0;
            bool turnStopIssued = false;
            StablePivotPhase phase = StablePivotPhase::kIdle;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                const auto it = g_gaitRouterActors.find(actor->GetFormID());
                if (it == g_gaitRouterActors.end()) return true;
                phase = it->second.stablePivotPhase;
                routeDegrees = it->second.stablePivotRouteDegrees;
                turnLeft = it->second.stablePivotLeft;
                turnStopIssued = it->second.stablePivot90TurnStopIssued;
            }

            // No RotationClip may become a pivot witness before the event was accepted.
            if (phase != StablePivotPhase::kPending && phase != StablePivotPhase::kActive) {
                return false;
            }
            // Once turnStop has been accepted, blend-out belongs entirely to the native
            // state machine. Do not keep forcing the 90 aliases to 1x after the stop edge.
            if (routeDegrees == 90 && turnStopIssued) {
                return false;
            }

            const std::string* expected = nullptr;
            if (routeDegrees >= 180) {
                expected = turnLeft
                    ? &stableConfig->authoredPivotLeft180Node
                    : &stableConfig->authoredPivotRight180Node;
            } else if (routeDegrees == 90) {
                expected = turnLeft
                    ? &stableConfig->authoredPivotLeft90Node
                    : &stableConfig->authoredPivotRight90Node;
            }
            if (!expected || expected->empty()) return true;

            const auto normalizedNode = LowerCopy(std::string(rawNodeName));
            const auto normalizedExpected = LowerCopy(*expected);
            if (normalizedNode == normalizedExpected) {
                return true;
            }

            // R101: the 90-degree Troll blender has two aliases of the same authored HKX:
            // MTTurn*90Slow (serialized 0.05) and MTTurn*90 (serialized 0.5).  R100 proved
            // that fighting TurnDelta every actor frame makes the native graph oscillate.
            // During a latched 90 pivot we therefore accept any alias whose *animation key*
            // is the exact configured side's 90 key. This keeps Slow+normal on one native
            // timeline while still rejecting the opposite direction.  180 remains exact-node
            // only because the canned .HKX00 state must not be confused with the idle blender.
            if (routeDegrees == 90 && !animationKey.empty() &&
                animationKey == normalizedExpected) {
                return true;
            }
            return false;
        }

        // R106: native HKB turn witness is lifecycle evidence, independent of playback policy.
        void ObserveStablePivotNativeRotationWitness(
            RE::hkbClipGenerator* clip,
            const RE::hkbContext& context)
        {
            if (!clip) return;
            const auto actorFormID = SyntheticMotionInjector::ResolveActorFormIDForHkbContext(context);
            if (actorFormID == 0) return;
            auto* actor = RE::TESForm::LookupByID<RE::Actor>(actorFormID);
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) return;

            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!motionProfile || !StableSamplePivotBusy(actor)) return;

            const auto profileFolder = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const char* rawAnim = clip->animationName.c_str();
            const char* rawNode = clip->name.c_str();
            const auto key = CanonicalHkxKeyFromAnimationName(rawAnim, profileFolder);
            if (key.empty()) return;

            const bool rotationKey =
                std::find(motionProfile->rotationClips.begin(), motionProfile->rotationClips.end(), key) !=
                motionProfile->rotationClips.end();
            if (!rotationKey || !StablePivotNodeMatchesLatchedRoute(actor, rawNode, key)) return;

            // R108: a generated hkbClipGenerator is not proof that its pose owns the actor.
            // The Troll turn blender updates multiple children (including the opposite side)
            // even when they are not visually authoritative. Certify only a weighted clip.
            float poseWeight = 1.0f;
            if (auto* control = clip->animationControl.get()) {
                const float controlWeight = std::isfinite(control->weight)
                    ? std::max(0.0f, control->weight) : 0.0f;
                const float masterWeight = std::isfinite(control->masterWeight)
                    ? std::max(0.0f, control->masterWeight) : 0.0f;
                poseWeight = controlWeight * masterWeight;
            }
            const auto cadenceConfig = HkxAuthoredCadenceConfigForProfile(profileFolder);
            const float minPoseWeight = cadenceConfig
                ? cadenceConfig->minPoseWeight : 0.001f;
            if (!std::isfinite(poseWeight) || poseWeight < minPoseWeight) return;

            std::uint64_t seenFrame = 0;
            bool visualEdge = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[actorFormID];
                seenFrame = state.stablePivotFrameSerial;
                visualEdge = state.stablePivotNativeRotationSeenFrame == 0 ||
                    seenFrame > state.stablePivotNativeRotationSeenFrame + 1 ||
                    state.stablePivotNativeRotationKey != key;
                state.stablePivotNativeRotationLastSeenSeconds = StablePivotMonotonicSeconds();
                state.stablePivotNativeRotationSeenFrame = seenFrame;
                state.stablePivotNativeRotationPoseWeight = poseWeight;
                state.stablePivotNativeRotationKey = key;
            }
            if (visualEdge) {
                SKSE::log::info(
                    "[StablePivot][R108_WEIGHTED_HKB_ACQUIRED] form={:08X} profile=\"{}\" frame={} key=\"{}\" node=\"{}\" poseWeight={:.5f} min={:.5f} action=CERTIFY_WEIGHTED_SAME_SIDE_VISUAL_TURN",
                    actorFormID, profileFolder, seenFrame, key, rawNode, poseWeight, minPoseWeight);
            } else {
                SKSE::log::debug(
                    "[StablePivot][R108_WEIGHTED_HKB_WITNESS] form={:08X} profile=\"{}\" frame={} key=\"{}\" node=\"{}\" poseWeight={:.5f} min={:.5f}",
                    actorFormID, profileFolder, seenFrame, key, rawNode, poseWeight, minPoseWeight);
            }
        }

        [[nodiscard]] bool FindConfiguredNativeRateOverride(
            const HkxAuthoredCadenceConfig& config,
            const char* rawNodeName,
            float& outRate)
        {
            outRate = 0.0f;
            if (!rawNodeName || !rawNodeName[0] || config.nativeRateOverrideCount == 0) {
                return false;
            }
            const auto node = LowerCopy(std::string(rawNodeName));
            for (std::size_t i = 0; i < config.nativeRateOverrideCount; ++i) {
                if (config.nativeRateOverrideNodes[i] == node) {
                    const float rate = config.nativeRateOverrideRates[i];
                    if (std::isfinite(rate) && rate > 0.001f && rate <= 2.0f) {
                        outRate = rate;
                        return true;
                    }
                }
            }
            return false;
        }

        [[nodiscard]] bool FindConfiguredNativeStandingTurnRateOverride(
            const HkxAuthoredCadenceConfig& config,
            const char* rawNodeName,
            float& outRate)
        {
            outRate = 0.0f;
            if (!rawNodeName || !rawNodeName[0] || config.nativeStandingTurnRateOverrideCount == 0) return false;
            const auto node = LowerCopy(std::string(rawNodeName));
            for (std::size_t i = 0; i < config.nativeStandingTurnRateOverrideCount; ++i) {
                if (config.nativeStandingTurnRateOverrideNodes[i] == node) {
                    const float rate = config.nativeStandingTurnRateOverrideRates[i];
                    if (std::isfinite(rate) && rate > 0.001f && rate <= 2.0f) {
                        outRate = rate;
                        return true;
                    }
                }
            }
            return false;
        }

        [[nodiscard]] bool PrepareProfileAuthoredCadenceUpdate(
            RE::hkbClipGenerator* clip,
            const RE::hkbContext& context,
            float inputTimestep,
            AuthoredCadenceUpdateDecision& out)
        {
            out = {};
            if (!clip || !std::isfinite(inputTimestep) || inputTimestep <= 0.0f) {
                return false;
            }
            const auto actorFormID = SyntheticMotionInjector::ResolveActorFormIDForHkbContext(context);
            if (actorFormID == 0) {
                return false;
            }
            auto* actor = RE::TESForm::LookupByID<RE::Actor>(actorFormID);
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) {
                return false;
            }

            const std::string profileFolder = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto cadenceConfig = HkxAuthoredCadenceConfigForProfile(profileFolder);
            const auto* motionProfile = MotionProfileDatabase::ResolveProfile(actor);
            if (!cadenceConfig || !cadenceConfig->enabled || !motionProfile) {
                return false;
            }

            const char* rawAnim = clip->animationName.c_str();
            const char* rawNode = clip->name.c_str();
            const auto key = CanonicalHkxKeyFromAnimationName(rawAnim, profileFolder);
            const bool stablePivotBusy = StableSamplePivotBusy(actor);
            const bool rotationKey =
                std::find(motionProfile->rotationClips.begin(), motionProfile->rotationClips.end(), key) !=
                motionProfile->rotationClips.end();
            const auto stableSelector = StableSampleSelectorConfigForProfile(profileFolder);
            float nativeStandingTurnConfiguredRate = 0.0f;
            const bool nativeStandingTurnCadence = stableSelector &&
                ShouldDeferToNativeStandingTurn(actor, *stableSelector) &&
                NativeStandingTurnCadenceNodeMatches(actor, *stableSelector, rawNode) &&
                FindConfiguredNativeStandingTurnRateOverride(*cadenceConfig, rawNode, nativeStandingTurnConfiguredRate);
            if (!nativeStandingTurnCadence && !IsConfiguredAuthoredCadenceKey(
                    *motionProfile, *cadenceConfig, key, stablePivotBusy)) {
                return false;
            }

            const bool runtimeCarrier =
                rotationKey && cadenceConfig->rotationRequiresNoRuntimeCarrier &&
                SyntheticMotionInjector::HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(actor);
            if (runtimeCarrier) {
                return false;
            }

            const bool rotationCadenceActive =
                rotationKey && cadenceConfig->includeRotationClips &&
                (!cadenceConfig->rotationRequiresStablePivot || stablePivotBusy) &&
                (!cadenceConfig->rotationRequiresNoRuntimeCarrier || !runtimeCarrier);
            const bool stablePivotRotation =
                rotationCadenceActive && stablePivotBusy && cadenceConfig->rotationRequiresStablePivot;
            if (stablePivotRotation && !StablePivotNodeMatchesLatchedRoute(actor, rawNode, key)) {
                return false;
            }
            float nativeRate = clip->playbackSpeed;
            float poseWeight = 1.0f;
            auto* control = clip->animationControl.get();
            if (control) {
                if (!stablePivotRotation && std::isfinite(control->playbackSpeed)) {
                    nativeRate = control->playbackSpeed;
                }
                const float controlWeight = std::isfinite(control->weight) ?
                    std::max(0.0f, control->weight) : 0.0f;
                const float masterWeight = std::isfinite(control->masterWeight) ?
                    std::max(0.0f, control->masterWeight) : 0.0f;
                poseWeight = controlWeight * masterWeight;
            }

            // R108: do not time-warp generated but visually inactive turn children. R107
            // compensated every same-side generator regardless of weight, which amplified
            // alias reactivation and phase churn. Rotation clips must pass the same pose
            // authority gate as ordinary cadence clips.
            if (!std::isfinite(poseWeight) || poseWeight < cadenceConfig->minPoseWeight) {
                return false;
            }
            if (!rotationCadenceActive && !nativeStandingTurnCadence &&
                (!std::isfinite(nativeRate) || nativeRate < cadenceConfig->minActiveRate)) {
                return false;
            }

            const float authoredRate = cadenceConfig->authoredRate;
            if (!std::isfinite(authoredRate) || authoredRate <= 0.0f) {
                return false;
            }

            // R99: the child-local timestep is not a physical clock for the Troll turn blender.
            // Use one engine frame as the canonical physical duration. For profile-declared
            // nodes with a known internally-authored rate, compensate the timestep *before*
            // native hkbClipGenerator::Update. This is deliberately different from R98:
            // no localTime is rewritten afterward, so pose, animationControl and extracted
            // motion are all produced by the same native update.
            float physicalDt = RE::GetSecondsSinceLastFrame();
            const bool engineFrameClock =
                std::isfinite(physicalDt) && physicalDt > 0.0f && physicalDt <= 0.100f;

            bool actorFrameClock = false;
            if (!engineFrameClock) {
                physicalDt = 0.0f;
                {
                    std::scoped_lock lock{ g_gaitRouterMutex };
                    const auto it = g_gaitRouterActors.find(actorFormID);
                    if (it != g_gaitRouterActors.end()) {
                        physicalDt = it->second.physicalFrameDeltaTime;
                    }
                }
                actorFrameClock =
                    std::isfinite(physicalDt) && physicalDt > 0.0f && physicalDt <= 0.100f;
            }

            if (!engineFrameClock && !actorFrameClock) {
                if (rotationCadenceActive || nativeStandingTurnCadence) {
                    SKSE::log::warn(
                        "[HKXPlayback][R114_CLOCK_SKIP] form={:08X} profile=\"{}\" key=\"{}\" node=\"{}\" reason=NO_UNWARPED_ENGINE_FRAME_DT inputChild={:.6f} action=NO_TEMPORAL_WRITE",
                        actorFormID, profileFolder, key, rawNode ? rawNode : "", inputTimestep);
                    return false;
                }
                if (!std::isfinite(inputTimestep) || inputTimestep <= 0.0f || inputTimestep > 0.100f) {
                    return false;
                }
                physicalDt = inputTimestep;
            }

            float configuredNativeRate = nativeStandingTurnCadence ? nativeStandingTurnConfiguredRate : 0.0f;
            const bool nativeRateCompensation = nativeStandingTurnCadence ||
                (rotationCadenceActive && FindConfiguredNativeRateOverride(*cadenceConfig, rawNode, configuredNativeRate));

            if (rotationCadenceActive && cadenceConfig->nativeRateOverridesExclusive &&
                cadenceConfig->nativeRateOverrideCount > 0 && !nativeRateCompensation) {
                return false;
            }
            float nativeUpdateDt = physicalDt;
            if (nativeRateCompensation) {
                nativeUpdateDt = physicalDt * (authoredRate / configuredNativeRate);
                // This is a child-generator timestep, not an actor frame. The 0.05 Troll
                // alias intentionally requires ~20x frame dt so native internal 0.05 scaling
                // still yields exactly one authored frame. Reject pathological configuration.
                if (!std::isfinite(nativeUpdateDt) || nativeUpdateDt <= 0.0f || nativeUpdateDt > 2.0f) {
                    SKSE::log::warn(
                        "[HKXPlayback][R99_RATE_COMPENSATION_SKIP] form={:08X} profile=\"{}\" node=\"{}\" configuredRate={:.4f} frameDt={:.6f} computedNativeDt={:.6f} action=NO_TEMPORAL_WRITE",
                        actorFormID, profileFolder, rawNode ? rawNode : "", configuredNativeRate,
                        physicalDt, nativeUpdateDt);
                    return false;
                }
            }

            out.apply = true;
            out.nativeRateCompensation = nativeRateCompensation;
            out.physicalTimestep = physicalDt;
            out.nativeUpdateTimestep = nativeUpdateDt;
            out.inputTimestep = inputTimestep;
            out.configuredNativeRate = configuredNativeRate;

            // R108: no clip->localTime authority for 90 aliases. R107's pre-native
            // canonical phase repair correlated with the animation-thrash episodes and
            // repeatedly teleported reactivated children along the HKX. Cadence may alter
            // only the native Update timestep of an already weighted same-side child.
            out.preClipLocalTime = clip->localTime;
            out.priorClipRate = clip->playbackSpeed;
            out.priorControlRate = control && std::isfinite(control->playbackSpeed)
                ? control->playbackSpeed : out.priorClipRate;
            out.poseWeight = poseWeight;
            out.control = control;
            out.actorFormID = actorFormID;
            out.profileFolder = profileFolder;
            out.key = key;
            out.nodeName = rawNode ? rawNode : "";
            out.animationName = rawAnim ? rawAnim : "";
            out.authoredRate = authoredRate;

            // Legacy R97 path remains for RotationClips that do not need a profile-native-rate
            // compensation. The Troll 90 aliases explicitly bypass this field write because
            // R98 proved native Update reinstates their internal 0.05/0.5 rates anyway.
            if (!nativeRateCompensation) {
                out.temporaryPlaybackRateOverride = true;
                clip->playbackSpeed = authoredRate;
                if (control) {
                    control->playbackSpeed = authoredRate;
                }
            }

            std::uint64_t calls = 0;
            bool emit = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[actorFormID];
                calls = ++state.hkxAuthoredCadenceCalls;
                const float warpRatio = physicalDt > 1.0e-6f ? inputTimestep / physicalDt : 0.0f;
                if (nativeRateCompensation) {
                    emit = calls <= 24 ||
                        (calls % std::max<std::uint32_t>(1, cadenceConfig->telemetryEveryFrames)) == 0;
                } else {
                    emit = calls <= 12 || warpRatio > 2.0f || warpRatio < 0.5f ||
                        (calls % std::max<std::uint32_t>(1, cadenceConfig->telemetryEveryFrames)) == 0;
                }
            }

            if (emit) {
                const float warpRatio = physicalDt > 1.0e-6f ? inputTimestep / physicalDt : 0.0f;
                const char* clockSource = engineFrameClock ? "ENGINE_SECONDS_SINCE_LAST_FRAME" :
                    (actorFrameClock ? "HOOKS_CHARACTER_UPDATE_CACHE" : "SAFE_CHILD_FALLBACK_NON_ROTATION");
                if (nativeRateCompensation) {
                    if (nativeStandingTurnCadence) {
                        SKSE::log::info(
                            "[HKXPlayback][R114_NATIVE_TURN_CADENCE] form={:08X} profile=\"{}\" key=\"{}\" node=\"{}\" anim=\"{}\" rate[behavior={:.4f} authored={:.4f}] timestep[inputChild={:.6f} physical={:.6f} nativeCall={:.6f} source={}] childWarpRatio={:.3f} expectedLocalAdvance={:.6f} action=CADENCE_ONLY_NO_ROTATIONCLIP_OWNERSHIP_NO_LOCALTIME_WRITE rootXY=UNCHANGED",
                            actorFormID, profileFolder, key, out.nodeName, out.animationName,
                            configuredNativeRate, authoredRate, inputTimestep, physicalDt, nativeUpdateDt,
                            clockSource, warpRatio, physicalDt * authoredRate);
                    } else {
                    SKSE::log::info(
                        "[HKXPlayback][R99_NATIVE_RATE_COMPENSATION] form={:08X} profile=\"{}\" key=\"{}\" node=\"{}\" anim=\"{}\" rate[behavior={:.4f} authored={:.4f}] timestep[inputChild={:.6f} physical={:.6f} nativeCall={:.6f} source={}] childWarpRatio={:.3f} expectedLocalAdvance={:.6f} action=CALL_NATIVE_WITH_RECIPROCAL_RATE_TIMESTEP poseRoot=SAME_NATIVE_UPDATE localTimePostWrite=ZERO rootXY=UNCHANGED",
                        actorFormID, profileFolder, key, out.nodeName, out.animationName,
                        configuredNativeRate, authoredRate, inputTimestep, physicalDt, nativeUpdateDt,
                        clockSource, warpRatio, physicalDt * authoredRate);
                    }
                } else {
                    SKSE::log::info(
                        "[HKXPlayback][R99_ROTATION_1X_CLOCK] form={:08X} profile=\"{}\" key=\"{}\" node=\"{}\" anim=\"{}\" rotationCadence={} pivotRotation={} poseWeight={:.5f} rate[bethesda={:.4f} temporaryAuthored={:.4f}] timestep[inputChild={:.6f} physical={:.6f} nativeCall={:.6f} source={}] childWarpRatio={:.3f} action=NATIVE_UPDATE_AT_AUTHORED_RATE_THEN_RESTORE_ALIAS_RATE rootXY=UNCHANGED",
                        actorFormID, profileFolder, key, out.nodeName, out.animationName,
                        rotationCadenceActive, stablePivotRotation, poseWeight, nativeRate,
                        authoredRate, inputTimestep, physicalDt, nativeUpdateDt, clockSource, warpRatio);
                }
            }
            return true;
        }

        void LogProfileAuthoredCadenceResultAfterUpdate(
            RE::hkbClipGenerator* clip,
            const AuthoredCadenceUpdateDecision& decision)
        {
            if (!clip || !decision.apply || !decision.nativeRateCompensation) {
                return;
            }
            const float postLocal = clip->localTime;
            if (!std::isfinite(postLocal) || !std::isfinite(decision.preClipLocalTime)) {
                return;
            }
            const float rawAdvance = postLocal - decision.preClipLocalTime;
            std::uint64_t samples = 0;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[decision.actorFormID];
                samples = ++state.hkxNativeRateResultSamples;
            }
            if (samples <= 24 || (samples % 120) == 0) {
                SKSE::log::info(
                    "[HKXPlayback][R99_NATIVE_RATE_RESULT] form={:08X} profile=\"{}\" key=\"{}\" node=\"{}\" localTime[pre={:.6f} post={:.6f} rawAdvance={:.6f} expected={:.6f}] rate[behavior={:.4f} authored={:.4f}] samples={} action=OBSERVE_NATIVE_RESULT_NO_POST_LOCALTIME_WRITE",
                    decision.actorFormID, decision.profileFolder, decision.key, decision.nodeName,
                    decision.preClipLocalTime, postLocal, rawAdvance,
                    decision.physicalTimestep * decision.authoredRate,
                    decision.configuredNativeRate, decision.authoredRate, samples);
            }
        }

        void RestoreProfileAuthoredCadenceUpdate(
            RE::hkbClipGenerator* clip,
            const AuthoredCadenceUpdateDecision& decision)
        {
            if (!clip || !decision.apply || !decision.temporaryPlaybackRateOverride) {
                return;
            }
            clip->playbackSpeed = decision.priorClipRate;
            if (decision.control) {
                decision.control->playbackSpeed = decision.priorControlRate;
            }
        }

        [[nodiscard]] bool ApplyProfileTemporalAuthority(
            RE::hkbClipGenerator* clip,
            const RE::hkbContext& context,
            const char* phase)
        {
            if (!clip) {
                return false;
            }
            const auto actorFormID = SyntheticMotionInjector::ResolveActorFormIDForHkbContext(context);
            if (actorFormID == 0) {
                return false;
            }
            auto* actor = RE::TESForm::LookupByID<RE::Actor>(actorFormID);
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) {
                return false;
            }
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = GaitRouterConfigForProfile(profile);
            if (!config || !config->temporalAuthority.enabled || config->temporalAuthority.assetCount == 0) {
                return false;
            }
            const char* rawAnim = clip->animationName.c_str();
            if (!rawAnim || !rawAnim[0]) {
                return false;
            }
            const auto animationName = LowerCopy(rawAnim);
            bool declared = false;
            for (std::size_t i = 0; i < config->temporalAuthority.assetCount; ++i) {
                if (animationName == config->temporalAuthority.assets[i]) {
                    declared = true;
                    break;
                }
            }
            if (!declared) {
                return false;
            }

            const float requestedRate = config->temporalAuthority.playbackRate;
            const float priorClip = clip->playbackSpeed;
            float priorControl = priorClip;
            auto* control = clip->animationControl.get();
            if (control && std::isfinite(control->playbackSpeed)) {
                priorControl = control->playbackSpeed;
            }
            clip->playbackSpeed = requestedRate;
            if (control) {
                control->playbackSpeed = requestedRate;
            }

            std::uint64_t writes = 0;
            bool emit = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[actorFormID];
                writes = ++state.temporalAuthorityWrites;
                emit = writes <= 8 ||
                    (writes % std::max<std::uint32_t>(1, config->temporalAuthority.telemetryEveryFrames)) == 0 ||
                    std::abs(priorClip - requestedRate) > 0.05f ||
                    std::abs(priorControl - requestedRate) > 0.05f;
            }
            if (emit) {
                SKSE::log::info(
                    "[TemporalAuthority][R84_RATE_REASSERT] form={:08X} profile=\"{}\" phase={} node=\"{}\" anim=\"{}\" rate[clipPrior={:.4f} controlPrior={:.4f} authored={:.4f}] writes={} action=DIAGNOSTIC_RATE_REASSERT_CANONICAL_LOCALTIME_IS_PHYSICAL_AUTHORITY",
                    actorFormID, profile, phase ? phase : "UNKNOWN",
                    clip->name.c_str(), rawAnim, priorClip, priorControl, requestedRate, writes);
            }
            return true;
        }

        bool ApplyProfileTemporalCanonicalTimeAfterUpdate(
            RE::hkbClipGenerator* clip,
            const RE::hkbContext& context,
            float preNativeLocalTime,
            float deltaTime)
        {
            if (!clip || !std::isfinite(preNativeLocalTime) || !std::isfinite(deltaTime)) {
                return false;
            }
            const auto actorFormID = SyntheticMotionInjector::ResolveActorFormIDForHkbContext(context);
            if (actorFormID == 0) {
                return false;
            }
            auto* actor = RE::TESForm::LookupByID<RE::Actor>(actorFormID);
            if (!actor || actor->IsPlayerRef() || !MotionProfileDatabase::SupportsActor(actor)) {
                return false;
            }
            const std::string profile = MotionProfileDatabase::DatabaseFolderForActor(actor);
            const auto config = GaitRouterConfigForProfile(profile);
            if (!config || !config->temporalAuthority.enabled || config->temporalAuthority.assetCount == 0) {
                return false;
            }
            const char* rawAnim = clip->animationName.c_str();
            if (!rawAnim || !rawAnim[0]) {
                return false;
            }
            const auto animationName = LowerCopy(rawAnim);
            bool declared = false;
            std::size_t declaredIndex = 0;
            for (std::size_t i = 0; i < config->temporalAuthority.assetCount; ++i) {
                if (animationName == config->temporalAuthority.assets[i]) {
                    declared = true;
                    declaredIndex = i;
                    break;
                }
            }
            if (!declared) {
                return false;
            }

            const auto metrics = AnimationMotionDatabase::GetClipMetrics(profile, rawAnim);
            float duration = metrics.valid ? metrics.duration : 0.0f;
            if (!(duration > 1.0e-6f) || !std::isfinite(duration)) {
                duration = config->temporalAuthority.durations[declaredIndex];
            }
            if (!(duration > 1.0e-6f) || !std::isfinite(duration)) {
                return false;
            }

            const float authoredRate = config->temporalAuthority.playbackRate;
            float canonicalLocal = preNativeLocalTime + std::max(deltaTime, 0.0f) * authoredRate;
            canonicalLocal = std::fmod(canonicalLocal, duration);
            if (canonicalLocal < 0.0f) {
                canonicalLocal += duration;
            }

            const float nativeLocal = clip->localTime;
            clip->localTime = canonicalLocal;

            std::uint64_t writes = 0;
            bool emit = false;
            {
                std::scoped_lock lock{ g_gaitRouterMutex };
                auto& state = g_gaitRouterActors[actorFormID];
                writes = ++state.temporalCanonicalTimeWrites;
                const float correction = std::isfinite(nativeLocal)
                    ? std::abs(canonicalLocal - nativeLocal)
                    : duration;
                emit = writes <= 12 || correction > 0.0025f ||
                    (writes % std::max<std::uint32_t>(1, config->temporalAuthority.telemetryEveryFrames)) == 0;
            }
            if (emit) {
                SKSE::log::info(
                    "[TemporalAuthority][R84_CANONICAL_TIME] form={:08X} profile=\"{}\" node=\"{}\" anim=\"{}\" time[pre={:.6f} nativePost={:.6f} canonicalPost={:.6f} duration={:.6f}] dt={:.6f} authoredRate={:.3f} writes={} action=POST_NATIVE_PRE_GENERATE_LOCALTIME_AUTHORITY",
                    actorFormID, profile, clip->name.c_str(), rawAnim,
                    preNativeLocalTime, nativeLocal, canonicalLocal, duration,
                    deltaTime, authoredRate, writes);
            }
            return true;
        }

        struct ClipGeneratorUpdateHook
        {
            static void Thunk(
                RE::hkbClipGenerator* clip,
                const RE::hkbContext& context,
                float deltaTime)
            {
                // R52 keeps the protected native update contract: observe pre-native,
                // pass deltaTime unchanged, call vanilla exactly once, then (only for
                // explicitly configured MotionDB phase groups) repair a proven stalled
                // directional localTime before Generate consumes the pose/root.
                const float preNativeLocalTime = clip ? clip->localTime : 0.0f;
                SyntheticMotionInjector::ObserveClipUpdate(clip, context, deltaTime, 0);
                // R84 keeps the R83 playback-rate reassertion for diagnostics, but R83
                // proved that Bethesda rewrites animationControl speed inside native Update.
                // The actual temporal authority therefore lives at localTime after native
                // Update and before Generate consumes pose/root.
                (void)ApplyProfileTemporalAuthority(clip, context, "PRE_NATIVE_CLIP_UPDATE");
                AuthoredCadenceUpdateDecision cadenceDecision{};
                const bool authoredCadence = PrepareProfileAuthoredCadenceUpdate(
                    clip, context, deltaTime, cadenceDecision);
                Func(clip, context, authoredCadence ? cadenceDecision.nativeUpdateTimestep : deltaTime);
                // R108: certify visual ownership after native Update, when animationControl
                // weights represent the child state just produced for this actor frame.
                ObserveStablePivotNativeRotationWitness(clip, context);
                if (authoredCadence) {
                    LogProfileAuthoredCadenceResultAfterUpdate(clip, cadenceDecision);
                    RestoreProfileAuthoredCadenceUpdate(clip, cadenceDecision);
                }
                (void)ApplyProfileTemporalAuthority(clip, context, "POST_NATIVE_CLIP_UPDATE");
                (void)ApplyProfileTemporalCanonicalTimeAfterUpdate(
                    clip, context, preNativeLocalTime, deltaTime);
                SyntheticMotionInjector::ApplyConfiguredDirectionalPhaseSyncAfterUpdate(
                    clip, context, deltaTime);

                // MotionTrajectoryController remains a separate controller at the
                // already-proven R52 phase: POST_NATIVE_CLIP_UPDATE_PRE_GENERATE. R67
                // performs no TurnLoop localTime writes here; this observer only arms
                // finite native MW1 sessions and keeps the Distance Matching query core.
                const auto actorFormID = SyntheticMotionInjector::ResolveActorFormIDForHkbContext(context);
                MotionTrajectoryController::ApplyAfterNativeClipUpdate(
                    actorFormID, clip, deltaTime);
            }
            static inline REL::Relocation<decltype(Thunk)> Func;
        };
    }

    void Install()
    {
        if (detail::g_installed) return;

        MotionProfileDatabase::EnsureLoaded();
        AnimationMotionDatabase::EnsureLoaded();

        REL::Relocation<std::uintptr_t> characterVTable{ RE::VTABLE_Character[0] };
        REL::Relocation<std::uintptr_t> clipGeneratorVTable{ RE::VTABLE_hkbClipGenerator[0] };

        detail::CharacterUpdateHook::Func = characterVTable.write_vfunc(0xAD, detail::CharacterUpdateHook::Thunk);
        detail::ModifyAnimationUpdateDataHook::Func = characterVTable.write_vfunc(0x79, detail::ModifyAnimationUpdateDataHook::Thunk);
        detail::ModifyMovementDataHook::Func = characterVTable.write_vfunc(0x11A, detail::ModifyMovementDataHook::Thunk);
        detail::ClipGeneratorUpdateHook::Func = clipGeneratorVTable.write_vfunc(0x05, detail::ClipGeneratorUpdateHook::Thunk);

        const bool externalMotionInstalled = SyntheticMotionInjector::Install();
        // R35 keeps the R30-proven ProcessMotionData yaw transport. The steering
        // controller samples AI/path goals above Bethesda's angular controller and produces
        // a stable ALR yaw step. CapsuleSlerp does not rewrite yaw in this build: V13 is the
        // downstream physical sink of the proven R30 transport and passes native unchanged.
        const bool capsuleRuntimeYawInstalled = CapsuleSlerp::InstallRuntimeYawBridge();

        detail::g_installed = true;
        SKSE::log::info("================ INCESSUS-C1-NATIVE-EXTRACTED-ROOT-PMD-YAW ================");
        SKSE::log::info("[Hooks][C1_PROFILELESS] Data/MotionDB is OPTIONAL legacy/profile compatibility only; profileless creature identity comes from runtime eligibility + active HKB graph discovery.");
        SKSE::log::info("[Hooks] ROOT_TRANSPORT profile=WFM_LEAF_INJECTION_LEGACY profileless=C1_NATIVE_PMD_DELTA_TO_TRACK_EXTRACTED_MOTION_TO_FINAL_HKB; ROOT_MOTION_YAW=PROCESS_MOTION_DATA_AI_STEERING_PAIR; injectorInstalled={}", externalMotionInstalled);
        SKSE::log::info(
            "[Hooks][R35_ROTATION] sourcePriority=HighProcessData::pathingDesiredPosition>Actor::currentCombatTarget>AIProcess::followTarget>AIProcess::target; "
            "controller=FILTERED_HEADING+DEADZONE+HYSTERESIS+ACCEL_LIMIT+PROFILE_MAX_RATE; sink=RuntimeYawClip_ProcessMotionData_R1_R2; "
            "pair=R1_IDENTITY_R2_ALR_YAW_STEP; rotationSpeed.z=NOT_SOURCE; V13[profile=PASS_NATIVE profileless=C1_HOLD_NATIVE_CAPSULE_YAW]; "
            "authoredTurn/action=NATIVE_PASS; WFMrotation=UNTOUCHED; profileless bAnimationDriven=NOT_ROOT_ADMISSION; installed={}.", capsuleRuntimeYawInstalled);
        SKSE::log::info(
            "[Hooks][R35_ROOT_CONTRACT] scope=LEGACY_PROFILE_ONLY baseline=R15.8+R15.12_EXIT_GRACE+27_08_EXACT_TURNLOOP; "
            "profileless=C1_FINAL_HKB_EXTRACTED_ROOT independentOfBAnimationDriven=true.");
        SKSE::log::info("[Hooks][R35_R34_LIFECYCLE_BASE] restoredGuard=NONPLAYER_SUPPORTED_MOTIONDB_ACTOR removedDeadGuard=preFrameYawEligible expectedChain=DB_SESSION_START>OPEN_DB_WFM>DB_OWNERSHIP_ACQUIRED>PMD_R1_R2_PLUS_XY");
        SKSE::log::info("[Hooks][R35_V13_POLICY] profile=PASS_NATIVE_UNMODIFIED; profileless=C1_ZERO_NATIVE_CAPSULE_YAW_EXCEPT_AUTHORED_TURN_OR_ACTION; AI yaw remains exclusively authored at ProcessMotionData R1/R2.");
        SKSE::log::info("[Hooks] TRANSLATION_GATE profile=R35_LEGACY_UNCHANGED; profileless=C1_FINAL_TRACK_EXTRACTED_MOTION exact+4frameBridge; raw PMD T1/T2 never reaches capsule; Z/secondary stay untouched.");
        SKSE::log::info("[Hooks] GAIT: exact R35 gait event router restored; NO R36/R37 debounce/latch/watchdog. TurnDelta/TurnDeltaDamped/L-C-R remain unchanged from R38.");
        SKSE::log::info("[C1_CONTRACT][INSTALL] scope=PROFILELESS_NONPLAYABLE_CREATURES graphDiscovery=B8_ACTIVE_HKB_STRING_TABLE graphControl=B8_SAMPLED_SELECTOR_PLUS_OPTIONAL_EVENTS physicalRoot=NATIVE_T1_T2_TO_LEAF_TRACK_EXTRACTED_MOTION_THEN_FINAL_HKB_COMPOSITION noCapsuleT1T2=true noPrearm=true bAnimationDrivenForced=false proceduralYaw=PMD_R1_R2_INDEPENDENT_OF_XY V13=ZERO_NATIVE_YAW_EXCEPT_AUTHORED_TURN_OR_ACTION existingProfiles=UNCHANGED");
        SKSE::log::info("[Hooks][R46_STRUCTURAL_REENTRY] sources=R42_ACTION_EXIT_TO_DEFAULT + R46_LOCOMOTION_TO_STANDING_RECENT_WITNESS; standingGate=PARENT_INACTIVE+STEERING_VALID+DESIRED_GAIT+ACTION_CLEAR+NO_CARRIER+NO_AUTHORED_ROTATION; standingDelay=RECENT_WITNESS_5FRAMES_PLUS_8_INACTIVE_FRAMES; parentWake=NO_GRAPH_VARIABLE_WRITES; parentHandshake=GaitRouter.ParentStartEvent_BOUNDED_6X; parentReady=2_CONSECUTIVE_NATIVE_ACTIVE_FRAMES; nestedRetry=6frames; completionPolicy=R49_NESTED_GAIT_HANDSHAKE; NO_GENERIC_WATCHDOG; normalR35Router/bAnimationDriven/XY/R30/R41=UNCHANGED.");
        SKSE::log::info("[Hooks][R48_SPEED_HOLD] source=R47_EXACT_BASE; normalSpeedIntentAdapter=UNCHANGED; preAnimationFallback=R47_PRESERVED; finalAuthority=POST_CHARACTER_UPDATE; postGate=CURRENT_VALID_STEERING+NATIVE_GAIT_PARENT_ACTIVE+CONFIGURED_ROUTE+LATCHED_FAMILY; writes=PROFILE_WALK_TROT_RUN_GRAPH_SPEED_GOAL_ONLY; neverStartsParent=true; rootXY/R30Yaw/R41TurnLoop=UNCHANGED.");
        SKSE::log::info("[Hooks][R49_STRUCTURAL_GAIT_HANDSHAKE] source=R46_EXACT_RECOVERY_BASE; parent moveStart is REENTRY_ONLY_NOT_SUCCESS; recovery success requires recovery walkStart/runStart accepted=true before carrier proof; carrierStable=2frames after accepted nested event; initial locomotion carrier after parent reentry does not block nested gait event; structural retry cadence remains actor-frame based; normalR35Router/rootXY/R30Yaw/R41TurnLoop=UNCHANGED.");
        SKSE::log::info("[Hooks][R50_RUN_SPEED_HOLD_CERT] source=R49_EXACT_BASE; R47/R48 speed hold preserved only when safe; family=RUN + generated RuntimeYaw root mask WALK/TROT without RUN => suppress fixed RUN Speed goal and preserve native graph Speed; runStart remains sole RUN-state entry authority; hold resumes automatically once RUN carrier is generated; WALK/TROT behavior unchanged; rootXY/R30Yaw/R41/R46/R49=UNCHANGED.");
        SKSE::log::info("[Hooks][R51_FULL_PHASE_RUN_SPEED_CERT] source=R50_EXACT_BASE; closes remaining pre-animation write window by applying the same WALK/TROT-without-RUN carrier mismatch gate before SpeedIntentTranslator::ApplyGraphGoal; blocked phase=MODIFY_ANIMATION_UPDATE_PRE_HAVOK; R50 fallback/post hold gate retained; no localTime/clip playback/root/yaw/TurnDelta writes added.");
        SKSE::log::info("[Hooks][R52_DIRECTIONAL_PHASE_REACQUIRE] source=R51_EXACT_BASE; phaseGroups=GaitRouter.ini::PhaseSync profile-driven; first key=center anchor; trigger=RECENT_GENERATED_DB_ROOT+N_CONSECUTIVE_STALLED_DIRECTIONAL_LOCALTIME+PHASE_ERROR_THRESHOLD; correction=POST_NATIVE_CLIP_UPDATE_PRE_GENERATE localTime only; native action takeover wins; root/yaw/TurnDelta/Speed/R49/R51 otherwise unchanged; no species/clip-name hardcode in C++.");
        SKSE::log::info("[Hooks][R53_RUN_CARRIER_SEMANTIC_REASSERT] source=R52_EXACT_BASE; trigger=PROFILE_DISTINCT_RUN_EVENT+ACTIVE_PARENT+VALID_STEERING+LOWER_RUNTIME_ROOT_CARRIER_WITHOUT_RUN_3FRAMES; action=REASSERT_CONFIGURED_RUN_EVENT_FRAME_CADENCE_6_MAX_6; success=2_CONSECUTIVE_RUN_CARRIER_FRAMES; R50/R51 keep RUN Speed suppressed until carrier certification; no Speed/root/yaw/TurnDelta/parent writes; not a generic watchdog.");
        SKSE::log::info("[Hooks][R54_PRE_CHARACTER_RUN_HOLD] source=R53_EXACT_BASE; earliestWrite=CHARACTER_UPDATE_BEFORE_NATIVE_FUNC; eligibility=VALID_STEERING+ACTIVE_PARENT+LATCHED_RUN+REAL_RUN_RUNTIME_CARRIER; writes=PROFILE_RUN_GOAL_ONLY; WALK_TROT_OR_CARRIERLESS=NO_WRITE; R50/R51/R53/root/yaw/TurnDelta/recovery=UNCHANGED.");
        SKSE::log::info("[Hooks][R55_PIVOT_AUTHORITY_RESTORED] source=R54_EXACT_BASE; profileDriven=true; AnimationPriority.UpdateActor=ON; ApplyGraphAuthority=FINAL_WRITER; busySuppresses=GaitRouter_R53+R54_PRE_SPEED+R48_POST_SPEED+R51_SPEED+R38_TURNDELTA+TURNLOOP_HEALTH; thresholds/events=MotionProfileDatabase.AnimationPriority; belowConfiguredPivotBand=LOCOMOTION_UNCHANGED; rootXY/R30Transport/R46/R49/R52=UNCHANGED.");
        SKSE::log::info("[Hooks][R60_PIVOT_INTENT] source=SteeringIntentController::filteredError; invalid steering=no pivot candidate; post-canned rearm requires valid confirmed steering below profile Turn90MinAngle; pathingDesiredOrientation is not pivot authority.");
        SKSE::log::info("[Hooks][R59_COMPACT_TELEMETRY] info=STATE_CHANGES_FAILURES_RECOVERY_PIVOT_INTENT; debug=PER_FRAME_WFM_PMD_ROOT_MIXER_TRANSLATION_GATE_CAPSULE_DETAIL; expectedNormalLogReduction=HIGH.");
        SKSE::log::info("[Hooks][R61_STRUCTURAL_SEMANTIC_REARM] source=R46/R49 exact structural recovery; one exhausted Locomotion->Standing witness may receive exactly one new bounded parent handshake only after a different configured gait event plus new semantic revision; no generic watchdog; R50/R51/R53/R54/root/yaw/pivot thresholds unchanged.");
        SKSE::log::info("[Hooks][R62_R53_PARENT_FLICKER_BRIDGE] source=R53 exact semantic RUN carrier recovery; same RUN revision/event plus lower-family carrier may bridge <=2 parent-inactive actor frames; active-edge normal runStart is deferred to bounded R53; no Speed/root/yaw/pivot/threshold writes added.");
        SKSE::log::info("[Hooks][R67_MOTION_TRAJECTORY] source=R66_EXACT_BASE; navigationSources=CONTINUOUS_STEERING_ONLY cannedPivotSource=COMBAT_TARGET; DistanceMatching=QUERY_CORE_PRESERVED_TURNLOOP_WRITES_ZERO; MotionWarping=MW1_NATIVE_FINITE_ACTION_XY_SCALE_INSIDE_OPEN_NATIVE; CapsuleV13=R35_PASS_NATIVE; ordinaryWalkTrotRun=UNCHANGED; R62/R61/R60/R53 preserved.");
        SKSE::log::info("[Hooks][R71_CONTINUOUS_FIRST] source=R70; pivotPolicy=NATIVE_STANDING_ONLY; activeLocomotionLargeHeading=CONTINUOUS_STEERING_NO_MOVESTOP_NO_TURNSTOP_NO_HOLD; AnimationPrioritySpeedWrites=ZERO; nativeLocomotionMayPreemptPendingPivot=true; R53/R60/R61/R62/root/yaw/DistanceMatching/MotionWarping=PRESERVED.");
        SKSE::log::info("[Hooks][R75_PROFILE_SCALEOUT] base=R72_NATIVE_SPEED_CONTINUITY profileDiscovery=Data/MotionDB/<Profile>/Intent.ini speciesHardcode=ZERO trollTopology=CONTINUOUS_NATIVE_GAIT GaitRouterOptional=true standingYawPolicy=R74_PROFILE_GRAPH_WITNESS rootAuthority=UNCHANGED");
        SKSE::log::info("[Hooks][R77_BEHAVIOR_EXTENSIONS] path=Data/MotionDB/<Profile>/Extensions/*.ini; primitiveTriggers=GraphBoolEdge,GraphIntTransition,GraphFloatCrossUp,GraphFloatCrossDown; arbitraryGraphVariable=true arbitraryEvent=true optionalBoolGuard=true retryBounded=true speciesHardcode=ZERO behaviorHardcode=ZERO writes=EVENTS_ONLY rootSpeedDirectionYawLocalTime=UNTOUCHED");
        SKSE::log::info("[Hooks][R78_PROFILE_CADENCE] path=Data/MotionDB/<Profile>/LocomotionPolicy.ini; Speed.CadencePolicy=NativeContinuity|StableFamilyGoal; default=NativeContinuity; StableFamilyGoal remains behind action bypass and R50/R51 carrier certification; speciesHardcode=ZERO behaviorHardcode=ZERO");
        SKSE::log::info("[Hooks][R99_ROTATION_1X_CLOCK] source=Data/MotionDB/<Profile>/LocomotionPolicy.ini::Playback; eligible=PROFILE_SCOPED_RuntimeYawClips+RotationClips; profileNodeRateOverrides=RECIPROCAL_NATIVE_TIMESTEP_BEFORE_HKB_CLIP_UPDATE; noPostNativeLocalTimeWrites=true; actorFrameDt=RE::GetSecondsSinceLastFrame_PRIMARY+HOOKS_CACHE_SECONDARY; nonOverriddenRotationClips=R97_UNCHANGED; rootXY=R85_HKX_WFM_ONLY speciesHardcode=ZERO behaviorHardcode=ZERO");
        SKSE::log::info("[Hooks][R95_STABLE_PHYSICAL_SAMPLE] source=Data/MotionDB/<Profile>/LocomotionPolicy.ini::StableSampleSelector; decision=PRE_NATIVE_CHARACTER_UPDATE; finalBoundary=REASSERT_ONLY_NO_SECOND_DECISION; selectors=SampledSpeed+Direction_PROFILE_NAMED; samples=EXACT_AUTHORED_POINTS_ONLY; normalContributors=ONE; transitionContributors=AT_MOST_TWO_BY_SERIALIZED_AXIS_SWITCH; Slow/FastAliasSelection=BLOCKED_BEFORE_NATIVE_SELECTION; authoredPivot=PROFILE_OPT_IN_LARGE_HEADING_STOP_THEN_90_180 standingWitness=PROFILE_GRAPH_INACTIVE_OR_PHYSICAL_CARRIER_ABSENT; rootXY=R85_HKX_WFM_ONLY; speciesHardcode=ZERO behaviorHardcode=ZERO");
        SKSE::log::info("[Hooks][R72_NATIVE_SPEED_CONTINUITY] source=R71; normalPreHavokSpeedIntent=PRIMARY_SEMANTIC_WRITER; R48/R54=NEAR_ZERO_STALL_RESCUE_ONLY; finiteNonZeroNativeSpeed=PRESERVE; actionBypass=ZERO_LOCOMOTION_SPEED_WRITES; R50/R51 lower-carrier RUN protection=PRESERVED; R71 ContinuousFirst pivots=PRESERVED; root/yaw/DM/MW1=UNCHANGED.");
        SKSE::log::info("[Hooks][R96_STABLE_GAIT_EDGE] source=GaitRouter.ini::ReassertSameSemanticOnActiveEdge; false=ACTIVE_PARENT_FLICKER_WITH_SAME_ROUTE_EVENT_REVISION_DOES_NOT_RESEND_GAIT_EVENT; true=PRESERVE_LEGACY_R35_EDGE_BEHAVIOR; structural R46/R49 recovery remains separate and bounded; root/yaw/speed/localTime=UNCHANGED.");
        SKSE::log::info("[Hooks][R100_180_PIVOT_LATCH] route=180 remains latched after transient NotifyAnimationGraph rejection for the full configured stop-time budget; no arbitrary frame-count expiry and no downgrade to 90 inside the same stop episode.");
        SKSE::log::info("[Hooks][R103_SINGLE_YAW_STRICT_90_PHASE] historicalBase=true; TurnDelta authority=DISABLED; StablePivot external witness suppresses RuntimeYaw R1/R2; turnStop closes route90 physical yaw during blend-out; R109 performs ZERO clip->localTime writes and does not use child alias state as physical authority; rootXY=UNCHANGED; canned180=R100_UNCHANGED.");
        SKSE::log::info("[Hooks][R110_VANILLA_90_STABLE_180] route90=PROFILE_CAN_DISABLE_STABLEPIVOT; disabled90 means NO_MOVE_STOP NO_TURN_EVENT NO_STABLEPIVOT_CAPSULE_CONTEXT NO_SELECTOR_CANDIDATE_HOLD; Bethesda TurnDelta/rotationSpeed/loop lifecycle remain native; route180=R100_LATCH_PRESERVED; rootXY/R35_CONTINUOUS_YAW otherwise unchanged.");
        SKSE::log::info("[Hooks][R114_START_STOP_QUARANTINE] native90=BethesdaSelectionAndYaw; largeHeadingCandidate=BLOCK_MOVESTART_AND_HOLD_OUTGOING_SAMPLE; moveStop=ONE_SHOT_THEN_STOP_DRAIN_HOLD; standingReady=CARRIER_FREE_CONFIRM; turnPhase=ZERO_SAMPLE_WRITES; postTurnStart=QUARANTINE_BEFORE_MOVESTART; RuntimeYawR1R2=SUPPRESSED; V13StandingYaw=NATIVE_OPEN; 90Cadence=TURN_PHASE_ONLY_NO_LOCALTIME; noPhysicalYawProgress=BOUNDED_FAIL_OPEN; route180=ONE_PASSIVE_LATCHED_SIDE_OFFER; rootXY unchanged.");
        SKSE::log::info("[Hooks][R116_CANNED_90_PARITY] behaviorContract=MT_Canned_Turn_Behavior_SINGLE_PLAY_90_AND_180; dispatch=ONE_EVENT_AFTER_STOP_DRAIN; route90=turnLeft_or_turnRight; route180=turnLeft180_or_turnRight180; completion=AUTHORED_ROTATIONCLIP_EXIT; PlaybackOverride=OFF; V13PhysicalRateOverride=OFF; RuntimeYawR1R2=SUPPRESSED_WHILE_CANNED; TurnDeltaWrites=ZERO; rotationSpeed.zWrites=ZERO; clipLocalTimeWrites=ZERO; Wolf=UNCHANGED.");
        SKSE::log::info("[Hooks][R41_TURNLOOP_HEALTH] R40 key-specific yaw progress retained; brief observed=false gaps up to 0.10s are bridged only while no RuntimeYaw locomotion carrier exists; unhealthy wall-clock reaches 0.18s => ONE turnStop; RotationClip new DB session bootstrap now requires INPLACE_TO_DB_ROOT provenance; R35 gait/root/XY unchanged.");
    }

    void Reset()
    {
        AnimationPriority::Clear();
        CapsuleSlerp::Clear();
        RotationIntentTranslator::Clear();
        SteeringIntentController::Clear();
        SyntheticMotionInjector::Clear();
        MotionTrajectoryController::Clear();
        detail::ClearGaitGraphRouter();
        detail::ClearBehaviorExtensions();
        detail::ClearTurnLoopHealthState();
        detail::ClearTranslationGateStats();
        TranslationAuthority::Clear();
        SpeedIntentTranslator::Clear();
        LocomotionSDK::Clear();
        AnimationMotionDatabase::Clear();
        MotionProfileDatabase::Clear();
        SKSE::log::info("[Hooks] Runtime state cleared; build=INCESSUS-R116-R85-CANNED-90-PARITY-2026-09-06");
    }
}

