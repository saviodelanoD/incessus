#pragma once

#include "RE/Skyrim.h"

#include <cstdint>
#include <string>
#include <string_view>
#include <vector>

namespace MotionProfileDatabase
{
    struct SpeedIntentConfig
    {
        bool enabled{ false };

        // SpeedIntent owns only semantic gait selection and one graph Speed write.
        // graphMoving is an optional native movement-presence witness.
        std::string graphSpeed;
        std::string graphMoving;

        float walkGoal{ 0.0f };
        float trotGoal{ 0.0f };
        float runGoal{ 0.0f };
        float stateChangeConfirmSeconds{ 0.0f };

        // Semantic AI/package source -> abstract WALK/TROT/RUN family mapping.
        std::string fleeFamily;
        std::string combatFamily;
        std::string actorRunFamily;
        std::string packageWalkFamily;
        std::string packageJogFamily;
        std::string packageFastWalkFamily;
        std::string packageRunFamily;
        std::string alertFamily;
        std::string actorWalkFamily;
        std::string defaultPackageFamily;
    };

    struct RotationIntentConfig
    {
        bool enabled{ false };
        
        // Vari�vel cont�nua que ser� injetada no Graph
        std::string turnDeltaVariable;
        
        // Flags do Graph que suprimem a rota��o nativa da engine (selo da c�psula)
        std::vector<std::string> actionBypassVariables;
        
        // Timer el�stico: Quantos frames segurar o valor antes de confirmar
        std::uint8_t candidateConfirmFrames{ 0 };
        
        // Lista de �ngulos exatos suportados pela �rvore de anima��o (snapping)
        std::vector<float> allowedTurnAngles;
    };


    struct AnimationPriorityConfig
    {
        bool enabled{ false };

        // Behavior-state witnesses/events. These are profile data, not hardcoded species rules.
        std::string graphAnimationDrivenVariable;
        std::string graphLocomotionActiveVariable;
        std::string graphSpeedVariable;
        std::string moveStopEvent;
        std::string cannedTurnStopEvent;
        std::string positiveTurnDirection{ "Left" };

        // Raw (unsnapped) path-heading bands.
        float turn90MinAngle{ 75.0f };
        float turn90MaxAngle{ 125.0f };
        float turn180MinAngle{ 165.0f };
        std::string intermediatePolicy{ "Turn90ThenResidual" };

        // We stop locomotion first, confirm Standing through !isMoving, then dispatch canned.
        std::uint8_t standingConfirmFrames{ 2 };
        std::uint8_t cleanupConfirmFrames{ 2 };
        float stopTimeoutSeconds{ 0.50f };
        float eventConfirmTimeoutSeconds{ 0.40f };
        float activeTimeoutSeconds{ 2.50f };
        float retryCooldownSeconds{ 0.25f };

        // Ordered graph EVENT names. First event accepted by NotifyAnimationGraph wins.
        std::vector<std::string> turnLeft90Events;
        std::vector<std::string> turnRight90Events;
        std::vector<std::string> turnLeft180Events;
        std::vector<std::string> turnRight180Events;

        // Optional profile-specific graph variables used to suppress competing drivers.
        std::vector<std::string> suppressionVariables;

        // AnimationPriority never requests looping turns. When it takes ownership of a
        // large turn it can preempt an already-running vanilla TurnLoop with this event.
        // A true global StuckOnly interceptor/detector is intentionally outside this build.
        bool preemptTurnLoopOnCanned{ true };
        std::string turnLoopStopEvent;
    };

    struct CapsuleSlerpConfig
    {
        bool enabled{ false };

        // Exponential response controls ease-out. Max yaw rate is the hard physical
        // ceiling that makes large capsule corrections visibly slow and predictable.
        float responsePerSecond{ 4.0f };
        float maxYawRateDegreesPerSecond{ 75.0f };
        float minErrorDegrees{ 0.10f };
        std::uint32_t telemetryEveryFrames{ 60 };
    };

    struct Profile
    {
        std::string name;
        std::string folder;
        std::vector<std::string> raceEditorIDExact;
        std::vector<std::string> raceEditorIDContains;
        SpeedIntentConfig speed;
        RotationIntentConfig rotation;
        AnimationPriorityConfig animationPriority;

        // Physical root-yaw policy is profile data. RuntimeYawClips receive a live
        // animrotation value at the clip WORLD_FROM_MODEL root; RotationClips keep
        // their authored MotionDB animrotation. Unlisted clips preserve baseline behavior.
        std::vector<std::string> rotationClips;
        std::vector<std::string> runtimeYawClips;

        CapsuleSlerpConfig capsuleSlerp;
    };

    void EnsureLoaded();
    [[nodiscard]] const Profile* ResolveProfile(RE::Actor* actor);
    [[nodiscard]] const Profile* GetProfile(std::string_view nameOrFolder);
    [[nodiscard]] bool SupportsActor(RE::Actor* actor);
    [[nodiscard]] std::string DatabaseFolderForActor(RE::Actor* actor);
    [[nodiscard]] std::string RootPath();
    void Clear();
}
