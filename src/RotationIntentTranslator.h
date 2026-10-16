// BUILD_ID: ALR-UNIVERSAL-ROTATION-INTENT-R17-LIVE-ERROR-PLUS-BOOTSTRAP-GUARD-2026-08-27
#pragma once

namespace RotationIntentTranslator
{
    // True when the actor's profile has [RotationIntent] Enabled=true.
    bool SupportsActor(RE::Actor* actor);

    // Producer. MUST be called once per Character::Update, before the injector's
    // BeginActorFrame. Samples HighProcessData::pathingDesiredOrientation against the
    // actor's yaw and publishes both a live and a confirmed/snapped heading error.
    void BeforeCharacterUpdate(RE::Character* actor, float deltaTime);

    // Live, unfiltered heading error in degrees for the current frame.
    // Signed shortest angle. This is the signal the moving-turn blender consumes.
    bool GetLiveTurnError(RE::Actor* actor, float& outDegrees);

    // Confirmed heading error in degrees, gated by candidateConfirmFrames.
    // Intended for canned-turn arbitration, which needs a stable decision.
    bool GetLatchedRawTurnError(RE::Actor* actor, float& outDegrees);

    // Legacy graph-goal writer. Retained for compatibility; not called by R17,
    // because Hooks::ApplyMovingTurnBlend is the sole owner of TurnDelta.
    bool ApplyLatchedGraphGoalAtAnimationInput(RE::Actor* actor);

    void Clear();
}

