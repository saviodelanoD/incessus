#pragma once

// BUILD_ID: ALR-R67-DM-SAFE-QUERY-CORE-MW1-NATIVE-ACTION-TRANSLATION-WARP-2026-09-01

#include "RE/Skyrim.h"

#include <cstdint>
#include <string>
#include <string_view>

namespace MotionTrajectoryController
{
    enum class MatchMetric : std::uint8_t
    {
        kProjectedDistance = 0,
        kYaw
    };

    struct MatchResult
    {
        bool valid{ false };
        bool clamped{ false };
        MatchMetric metric{ MatchMetric::kProjectedDistance };
        std::string profile;
        std::string key;
        float motionTime{ 0.0f };
        float normalizedTime{ 0.0f };
        float requested{ 0.0f };
        float achieved{ 0.0f };
        float error{ 0.0f };
        float totalAvailable{ 0.0f };
        float clipDuration{ 0.0f };
    };

    [[nodiscard]] MatchResult FindTimeForDistance(
        std::string_view profile,
        std::string_view animationName,
        float targetDistance);
    [[nodiscard]] MatchResult FindTimeForRemainingDistance(
        std::string_view profile,
        std::string_view animationName,
        float remainingDistance);
    [[nodiscard]] MatchResult FindTimeForRemainingYaw(
        std::string_view profile,
        std::string_view animationName,
        float remainingYawRadians);

    // Post-native clip observer. R67 preserves the inverse Distance Matching query core,
    // but performs ZERO localTime writes on looping RotationClips after the R66 spin
    // evidence. It also detects configured finite native combat actions for MW1.
    void ApplyAfterNativeClipUpdate(
        RE::FormID actorFormID,
        RE::hkbClipGenerator* clip,
        float inputDeltaTime);

    // MW1: target-aware translation-only Motion Warping for already-admitted native
    // finite action root. It never opens the native gate, never touches DB-root motion,
    // Z, secondaryMovement, capsule yaw, or graph variables. Returns true only when XY
    // was actually rescaled.
    [[nodiscard]] bool WarpNativeActionTranslation(
        RE::Actor* actor,
        float deltaTime,
        const RE::NiPoint3& authoredTranslation,
        RE::NiPoint3& outTranslation);

    void Clear();
}

