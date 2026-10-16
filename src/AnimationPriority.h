#pragma once

// BUILD_ID: ALR-ANIMATION-PRIORITY-R11-PREPARE-EVENT-ACTIVE-2026-08-20

#include "RE/Skyrim.h"

#include <cstdint>

namespace AnimationPriority
{
    enum class YawPolicy : std::uint8_t
    {
        kCapsuleSlerp = 0,
        kHoldForAnimation,
        kAnimationOwnsYaw
    };

    [[nodiscard]] YawPolicy GetYawPolicy(RE::FormID formID) noexcept;
    [[nodiscard]] bool IsBusy(RE::Actor* actor) noexcept;

    // Runs after RotationIntent produced TurnDelta and before Character::Update.
    // A configured large turn is converted into PREPARE -> EVENT -> ACTIVE.
    void UpdateActor(RE::Character* actor, bool enabled, float deltaTime);

    // Reasserts graph ownership at the animation-input boundary after
    // RotationIntent has written its normal graph goal.
    [[nodiscard]] bool ApplyGraphAuthority(RE::Actor* actor);

    void Clear();
}

