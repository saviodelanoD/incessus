#pragma once

// Animal Locomotion Revolution SDK V1
// Read-only public ABI for external pose/warping consumers.
// Intentionally exposes no CommonLibSSE, Skyrim, Havok or internal ALR types.

#include <cstdint>
#include <type_traits>

#if defined(_WIN32) && defined(ALR_SDK_EXPORTS)
#  define ALR_SDK_API __declspec(dllexport)
#else
#  define ALR_SDK_API
#endif

namespace ALR_API
{
    inline constexpr std::uint32_t kInterfaceVersion1 = 1;

    // Runtime carrier mask. Bits intentionally mirror ALR's generated locomotion witness.
    inline constexpr std::uint8_t kCarrierWalk = 0x01;
    inline constexpr std::uint8_t kCarrierTrot = 0x02;
    inline constexpr std::uint8_t kCarrierRun  = 0x04;

    enum class Gait : std::uint8_t
    {
        kNone = 0,
        kWalk,
        kTrot,
        kRun
    };

    enum SnapshotFlags : std::uint32_t
    {
        kSupportedActor              = 1u << 0,
        kGraphLocomotionActive       = 1u << 1,
        kGraphAnimationDriven        = 1u << 2,
        kDirectionalMovement         = 1u << 3,
        kMovingForward               = 1u << 4,
        kMovingBackward              = 1u << 5,
        kMovingLeft                  = 1u << 6,
        kMovingRight                 = 1u << 7,
        kSteeringValid               = 1u << 8,
        kPivotBusy                   = 1u << 9,
        kActionBypassActive          = 1u << 10,
        kDbRootAuthority             = 1u << 11,
        kExternalTranslationWritten  = 1u << 12,
        kRuntimeYawCarrier           = 1u << 13,
        kAuthoredRotationObserved    = 1u << 14,
        kAuthoredRotationProgressing = 1u << 15,
        kNativeAuthoredMotion        = 1u << 16,
        kNativeRootAuthority         = 1u << 17,
        kFacingYawInjected           = 1u << 18,
        kNativeAuthoredYaw           = 1u << 19
    };

    // Snapshot contract:
    // - All angles are radians.
    // - Positions are Skyrim world units.
    // - desiredGait is semantic intent, runtimeCarrierMask is generated physical evidence.
    // - No field grants authority to write graph/root/capsule state.
    struct LocomotionSnapshotV1
    {
        std::uint32_t structSize{ sizeof(LocomotionSnapshotV1) };
        std::uint32_t interfaceVersion{ kInterfaceVersion1 };
        std::uint32_t actorFormID{ 0 };
        std::uint32_t targetFormID{ 0 };
        std::uint32_t flags{ 0 };
        std::uint64_t snapshotSerial{ 0 };
        std::uint64_t intentRevision{ 0 };
        std::uint64_t steeringFrameSerial{ 0 };

        Gait desiredGait{ Gait::kNone };
        std::uint8_t runtimeCarrierMask{ 0 };
        std::uint8_t steeringSourceRaw{ 0 };
        std::uint8_t reserved0{ 0 };

        float worldX{ 0.0f };
        float worldY{ 0.0f };
        float worldZ{ 0.0f };
        float worldYawRadians{ 0.0f };

        float goalX{ 0.0f };
        float goalY{ 0.0f };
        float goalZ{ 0.0f };
        float goalDistanceXY{ 0.0f };

        float graphSpeed{ 0.0f };
        float walkGoal{ 0.0f };
        float trotGoal{ 0.0f };
        float runGoal{ 0.0f };

        float rawHeadingRadians{ 0.0f };
        float filteredHeadingRadians{ 0.0f };
        float rawErrorRadians{ 0.0f };
        float filteredErrorRadians{ 0.0f };

        float targetYawRateRadPerSecond{ 0.0f };
        float currentYawRateRadPerSecond{ 0.0f };
        float yawStepRadians{ 0.0f };
        float steeringDeltaTime{ 0.0f };

        char profile[32]{};
        char steeringSourceName[24]{};

        // R127 native Havok telemetry. These are observed from the runtime motion
        // actually selected/evaluated by Skyrim; no baked MotionDB is involved.
        std::uint8_t nativeMotionValid{ 0 };
        std::uint8_t nativeAuthoredYawValid{ 0 };
        std::uint8_t nativeFacingYawValid{ 0 };
        std::uint8_t nativeLocomotionOwned{ 0 };
        float authoredDeltaX{ 0.0f };
        float authoredDeltaY{ 0.0f };
        float authoredDeltaZ{ 0.0f };
        float authoredSpeed{ 0.0f };
        float authoredYawStepRadians{ 0.0f };
        float facingYawStepRadians{ 0.0f };
        char activeNode[64]{};
        char activeAnimation[128]{};
    };

    struct ClipMetricsV1
    {
        std::uint32_t structSize{ sizeof(ClipMetricsV1) };
        std::uint32_t interfaceVersion{ kInterfaceVersion1 };
        std::uint8_t valid{ 0 };
        std::uint8_t yawValid{ 0 };
        std::uint16_t reserved0{ 0 };
        float duration{ 0.0f };
        float cycleX{ 0.0f };
        float cycleY{ 0.0f };
        float cycleZ{ 0.0f };
        float cycleYaw{ 0.0f };
        float distanceXY{ 0.0f };
        char profile[32]{};
        char key[64]{};
    };

    struct CumulativeSampleV1
    {
        std::uint32_t structSize{ sizeof(CumulativeSampleV1) };
        std::uint32_t interfaceVersion{ kInterfaceVersion1 };
        std::uint8_t valid{ 0 };
        std::uint8_t yawValid{ 0 };
        std::uint16_t reserved0{ 0 };
        float clipDuration{ 0.0f };
        float motionTime{ 0.0f };
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
        float yaw{ 0.0f };
        char profile[32]{};
        char key[64]{};
    };

    using QueryLocomotionFn = std::uint32_t (*)(std::uint32_t actorFormID, LocomotionSnapshotV1* out) noexcept;
    using QueryClipMetricsFn = std::uint32_t (*)(const char* profile, const char* animationName, ClipMetricsV1* out) noexcept;
    using SampleCumulativeFn = std::uint32_t (*)(const char* profile, const char* animationName, float motionTime, CumulativeSampleV1* out) noexcept;

    struct InterfaceV1
    {
        std::uint32_t structSize{ sizeof(InterfaceV1) };
        std::uint32_t version{ kInterfaceVersion1 };
        const char* buildId{ nullptr };
        QueryLocomotionFn QueryLocomotion{ nullptr };
        QueryClipMetricsFn QueryClipMetrics{ nullptr };
        SampleCumulativeFn SampleCumulative{ nullptr };
    };

    using RequestInterfaceFn = const InterfaceV1* (*)(std::uint32_t version) noexcept;

    static_assert(std::is_standard_layout_v<LocomotionSnapshotV1>);
    static_assert(std::is_standard_layout_v<ClipMetricsV1>);
    static_assert(std::is_standard_layout_v<CumulativeSampleV1>);
    static_assert(std::is_standard_layout_v<InterfaceV1>);
}

extern "C" ALR_SDK_API const ALR_API::InterfaceV1* ALR_RequestInterface(std::uint32_t version) noexcept;

