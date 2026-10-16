#pragma once

#include "RE/Skyrim.h"

#include <cstdint>

namespace SteeringIntentController
{
    enum class Source : std::uint8_t
    {
        kNone = 0,
        kPathDesiredPosition,
        kAIProcessTarget,
        kCombatTarget,
        kFollowTarget,
        kHeldLastHeading
    };

    struct FrameCommand
    {
        bool valid{ false };
        std::uint64_t frameSerial{ 0 };
        Source source{ Source::kNone };
        RE::FormID targetFormID{ 0 };

        RE::NiPoint3 actorPosition{};
        RE::NiPoint3 goalPosition{};
        float goalDistanceXY{ 0.0f };

        float actorYawRadians{ 0.0f };
        float rawHeadingRadians{ 0.0f };
        float filteredHeadingRadians{ 0.0f };
        float rawErrorRadians{ 0.0f };
        float filteredErrorRadians{ 0.0f };

        float targetYawRateRadPerSecond{ 0.0f };
        float currentYawRateRadPerSecond{ 0.0f };
        float yawStepRadians{ 0.0f };
        float deltaTime{ 0.0f };
    };

    [[nodiscard]] bool SupportsActor(RE::Actor* actor) noexcept;
    void BeginFrame(RE::Character* actor, float deltaTime);
    [[nodiscard]] bool GetFrameCommand(RE::Actor* actor, FrameCommand& out) noexcept;
    [[nodiscard]] const char* SourceName(Source source) noexcept;
    void Clear();
}

