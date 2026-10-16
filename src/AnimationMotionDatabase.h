#pragma once

// BUILD_ID: ALR-UNIVERSAL-MOTIONDB-R2-DATA-ROOT-2026-08-19

#include <cstddef>
#include <string>
#include <string_view>

namespace AnimationMotionDatabase
{
    struct DeltaSample
    {
        bool valid{ false };
        bool yawValid{ false };
        std::string profile;
        std::string key;
        float startPhase{ 0.0f };
        float phaseAdvance{ 0.0f };
        float clipDuration{ 0.0f };
        float dx{ 0.0f };
        float dy{ 0.0f };
        float dz{ 0.0f };
        float deltaYaw{ 0.0f };
        float distanceXY{ 0.0f };
    };

    struct CumulativeSample
    {
        bool valid{ false };
        bool yawValid{ false };
        std::string profile;
        std::string key;
        float clipDuration{ 0.0f };
        float motionTime{ 0.0f };
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
        float yaw{ 0.0f };
    };

    struct ClipMetrics
    {
        bool valid{ false };
        bool yawValid{ false };
        std::string profile;
        std::string key;
        float duration{ 0.0f };
        float cycleX{ 0.0f };
        float cycleY{ 0.0f };
        float cycleZ{ 0.0f };
        float cycleYaw{ 0.0f };
        float distanceXY{ 0.0f };
    };

    struct Stats
    {
        bool loadAttempted{ false };
        bool directoryFound{ false };
        std::size_t profilesSeen{ 0 };
        std::size_t filesSeen{ 0 };
        std::size_t clipsLoaded{ 0 };
        std::size_t clipsRejected{ 0 };
        std::size_t duplicateKeys{ 0 };
        std::size_t queries{ 0 };
        std::size_t hits{ 0 };
        std::size_t misses{ 0 };
    };

    void EnsureLoaded();

    [[nodiscard]] DeltaSample SampleDelta(std::string_view profile, std::string_view animationName, float startPhase, float phaseAdvance);
    [[nodiscard]] DeltaSample SampleDeltaSeconds(std::string_view profile, std::string_view animationName, float startTime, float deltaTime);
    [[nodiscard]] bool HasClip(std::string_view profile, std::string_view animationName);
    [[nodiscard]] ClipMetrics GetClipMetrics(std::string_view profile, std::string_view animationName);
    [[nodiscard]] CumulativeSample SampleCumulativeSeconds(std::string_view profile, std::string_view animationName, float motionTime);

    // Legacy player-facing overloads intentionally resolve only inside Data/MotionDB/Player.
    [[nodiscard]] DeltaSample SampleDelta(std::string_view animationName, float startPhase, float phaseAdvance);
    [[nodiscard]] DeltaSample SampleDeltaSeconds(std::string_view animationName, float startTime, float deltaTime);
    [[nodiscard]] bool HasClip(std::string_view animationName);
    [[nodiscard]] ClipMetrics GetClipMetrics(std::string_view animationName);
    [[nodiscard]] CumulativeSample SampleCumulativeSeconds(std::string_view animationName, float motionTime);

    [[nodiscard]] Stats GetStats();
    [[nodiscard]] std::string DatabasePath();
    void Clear();
}

