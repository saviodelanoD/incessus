#pragma once

// BUILD_ID: INCESSUS-B6-STEERING-DEMAND-STABLE-SELECTOR-H-2026-10-02

#include "RE/Skyrim.h"

#include <cstdint>
#include <string_view>

namespace SpeedIntentTranslator
{
    enum class Family : std::uint8_t
    {
        kNone = 0,
        kWalkForward = 1,
        kTrotForward = 2,
        kRunForward = 3
    };

    [[nodiscard]] bool SupportsActor(RE::Actor* actor);

    // Pure semantic gait solver. Reads AI/ActorState intent only and maintains
    // one stable WALK/TROT/RUN latch. It does not inspect MotionDB, clips,
    // root motion, capsule motion, yaw, Direction or SpeedSampled.
    void BeforeUpdate(RE::Character* actor, float deltaTime);

    // B6 profileless-creature lane. Steering supplies MovementDemand; AI/package/ActorState
    // only classify semantic gait. AutoGraph resolves the active graph contract, drives
    // moveStart/walkStart/runStart where structurally present, and writes exact authored
    // sampled-speed/direction selector coordinates for continuous creature graphs.
    // Existing profiles never use this path and remain authoritative.
    void BeforeUniversalUpdate(RE::Character* actor, float deltaTime);
    void ApplyAutoGraphAdapterPreNative(RE::Character* actor, float deltaTime);
    void ApplyAutoGraphAdapterFinal(RE::Actor* actor);

    // B6 separates movement presence from gait classification. Only a live Skyrim
    // AI/path steering demand may start/maintain the profileless locomotion lane.
    [[nodiscard]] bool HasUniversalMovementDemand(RE::Actor* actor);
    [[nodiscard]] bool HasResolvedUniversalGraphContract(RE::Actor* actor);

    // The only graph write made by the legacy profile module: canonical Speed at 0x79 POST.
    // Movement presence remains vanilla, so a held gait can never manufacture
    // locomotion while the actor is idle.
    [[nodiscard]] bool ApplyGraphGoal(RE::Actor* actor);

    // Read-only handoff to SyntheticMotionInjector. This is the semantic family
    // only; it is not root-motion ownership and has no clip lifetime semantics.
    [[nodiscard]] Family LatchedFamily(RE::FormID formID);
    [[nodiscard]] std::uint64_t Revision(RE::FormID formID);

    // Compatibility only for the untouched SyntheticMotionInjector.
    [[nodiscard]] bool IsRuntimeEligible(RE::Actor* actor);
    void ObserveDbClip(RE::FormID formID, std::string_view key, std::string_view node, std::string_view animation);
    void ObserveExternalPassThrough(RE::FormID formID);

    void Clear();
}

