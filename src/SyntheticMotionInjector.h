#pragma once

// BUILD_ID: INCESSUS-C1-NATIVE-EXTRACTED-ROOT-H-2026-10-03

#include "RE/Skyrim.h"

namespace SyntheticMotionInjector
{
    void BeginActorFrame(RE::Character* character, float deltaTime);
    void BeginAnimationEvaluation(RE::Character* character);
    void EndAnimationEvaluation(RE::Character* character);
    void ObserveClipUpdate(
        RE::hkbClipGenerator* clip,
        const RE::hkbContext& context,
        float deltaTime,
        RE::FormID actorFormID = 0);

    // R15.8 translation authority lifecycle:
    // - Acquisition remains strong: transition-local provenance or stable final DB root.
    // - After ACQUIRED, DB ownership is latched until an explicit release or bounded
    //   continuity timeout; a missing Generate callback is not a release signal.
    // - Matching PMD may refresh an already-acquired ownership, but can never acquire.
    // - INPLACE->ROOT => IGNORE_FROM; ROOT->INPLACE => IGNORE_TO; ROOT->ROOT => no flag.
    // - Actor-global LEAF_PREARM policy spraying remains removed.
    // - bAnimationDriven bootstrap is transport setup only, not acquisition authority.
    // - Native serialized IGNORE_FROM/IGNORE_TO remain native gate context.
    [[nodiscard]] bool HasExternalTranslationForCurrentFrame(RE::Actor* actor);
    [[nodiscard]] bool HasSyntheticWfmTranslationForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasNativeMotionWitnessForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasNativeAuthoredRotationForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasNativeAuthoredTurnForCurrentOrPreviousFrame(RE::Actor* actor);

    enum class UniversalRootResult : std::uint8_t
    {
        kNone = 0,
        kExact,
        kBridge,
        kHoldZero
    };
    [[nodiscard]] UniversalRootResult ResolveUniversalExtractedTranslation(
        RE::Actor* actor, float deltaTime, RE::NiPoint3& outTranslation);
    [[nodiscard]] bool HasUniversalExtractedRootAuthority(RE::Actor* actor);
    void ReleaseUniversalExtractedRootAuthority(RE::Actor* actor);
    [[nodiscard]] bool IsUniversalCreatureEligible(RE::Actor* actor);
    [[nodiscard]] bool HasNativeMotionSessionForCurrentFrame(RE::Actor* actor);
    void ConfirmNativeLocomotionRootAuthorityAtGate(RE::Actor* actor);
    [[nodiscard]] bool HasNativeLocomotionRootAuthorityForCurrentFrame(RE::Actor* actor);
    [[nodiscard]] bool IsNativeLocomotionBootstrapDrivenForCurrentFrame(RE::Actor* actor);
    [[nodiscard]] bool HasDbMotionSourceAuthorityForCurrentFrame(RE::Actor* actor);
    // R28 read-only yaw-carrier queries derived from the existing clip/root lifecycle.
    // No new ProcessMotionData hook or guessed Havok ABI is involved.
    [[nodiscard]] bool HasRuntimeYawLocomotionCarrierForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasAuthoredRotationClipForCurrentOrPreviousFrame(RE::Actor* actor);
    void ConfirmDbRootOwnershipAtGate(RE::Actor* actor);
    [[nodiscard]] bool IsDbBootstrapDrivenForCurrentFrame(RE::Actor* actor);
    [[nodiscard]] bool HasIgnoreFromTransitionForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasIgnoreToTransitionForCurrentOrPreviousFrame(RE::Actor* actor);
    [[nodiscard]] bool HasNativeRootMotionForCurrentFrame(RE::Actor* actor);  // telemetry only
    [[nodiscard]] bool Install();
    void Clear();
}

