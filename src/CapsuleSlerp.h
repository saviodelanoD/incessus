#pragma once

// BUILD_ID: INCESSUS-R115-NATIVE-TURN-PHYSICAL-RATE-H-2026-09-06

#include "RE/Skyrim.h"

#include <cstdint>

namespace CapsuleSlerp
{
    // Compatibility entry point.
    void UpdateActor(RE::Character* actor, bool enabled);

    // Character::Update frame boundary. Registers/updates the per-controller V13
    // physical servo only for configured profiles. Procedural yaw itself additionally
    // requires bAnimationDriven + a certified RuntimeYawClip carrier.
    void BeginRuntimeYawFrame(RE::Character* actor, bool enabled, float deltaTime);

    // R28 compatibility installer: deliberately installs NO ProcessMotionData R2 hook.
    // Carrier/pivot witnesses come from SyntheticMotionInjector's existing safe state.
    [[nodiscard]] bool InstallRuntimeYawBridge();

    [[nodiscard]] bool HasAuthoredTurnWitnessForCurrentOrPreviousFrame(RE::Actor* actor);

    // R109: read the FINAL hkbBehaviorGraph generator output, after state machines and
    // blender generators have composed their children. The returned value is the physical
    // world-yaw delta for the current/most recently generated behavior output (the Havok
    // extracted-motion yaw uses the opposite sign convention from the capsule world yaw).
    // No clip/alias identity is involved. Returns false on absent/ambiguous/invalid output.
    [[nodiscard]] bool TryGetFinalBehaviorPhysicalYawDeltaDegrees(
        RE::Actor* actor,
        float& outPhysicalYawDeltaDegrees,
        std::int32_t* outGeneratorOutputIndex = nullptr);

    // R103: StablePivot can prove a native behavior RotationClip even when the clip is not
    // MotionDB-backed. active keeps the upstream single-owner witness alive;
    // allowNativeStandingYaw independently controls whether V13 may pass the native target.
    // This separation lets turnStop freeze physical yaw during native blend-out without
    // re-enabling procedural RuntimeYaw locomotion leaves.
    void SetExternalAuthoredTurnWitness(RE::Actor* actor, bool active, bool allowNativeStandingYaw = true);

    // R115: narrow physical-rate context for the R114 native standing-turn window.
    // Bethesda still selects the turn and owns TurnDelta/pose. Hooks supplies only the
    // latched physical direction/rate plus a READ-ONLY TurnDelta witness. When inactive,
    // CapsuleSlerp remains completely dormant for profiles that disabled CapsuleSlerp.
    void SetNativeStandingTurnPhysicalYawRate(
        RE::Actor* actor,
        bool active,
        float signedYawRateDegreesPerSecond = 0.0f,
        bool turnDeltaRead = false,
        float turnDelta = 0.0f);

    // R104 diagnostic-only context published by StablePivot before Character::Update.
    // This never changes yaw/root/graph state; it only lets the V13 sink emit one
    // causally aligned trace line with StablePivot phase, native HKB witness age,
    // latched route and read-only TurnDelta. phase: 0=IDLE,1=STOPPING,2=PENDING,3=ACTIVE.
    void SetStablePivotDiagnosticContext(
        RE::Actor* actor,
        std::uint8_t phase,
        std::uint32_t routeDegrees,
        bool nativeHkbWitness,
        float nativeHkbWitnessAgeSeconds,
        const char* nativeHkbKey,
        bool turnDeltaRead,
        float turnDelta,
        float physicalYawRateDegreesPerSecond);

    // R102: read the real bhkCharacterController yaw used by the V13 physical sink.
    // StablePivot uses this only to decide when a looping 90-degree turn has physically
    // reached its requested angle and when a visual-only/no-yaw turn must be aborted.
    [[nodiscard]] bool TryGetPhysicalYawRadians(RE::Actor* actor, float& outYawRadians);

    void Clear();
}

