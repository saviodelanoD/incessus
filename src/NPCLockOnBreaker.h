// BUILD_ID: ALR-NPC-LOCKON-BREAKER-45DEG-1S-2026-08-19
#pragma once

#include "PCH.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace NPCLockOnBreaker
{
    // Requested policy:
    // - NPC lock-on works normally while the combat target stays inside +/-45 degrees.
    // - If the target leaves that cone, break the NPC's current combat target immediately.
    // - For the next 1.0 second, prevent reacquisition/rotation tracking.
    // - After the cooldown, vanilla is free to acquire a target again.
    //
    // Default scope is NPC/creature -> Player only. This avoids changing NPC-vs-NPC combat.
    inline constexpr float kBreakAngleDegrees = 45.0f;
    inline constexpr float kCooldownSeconds = 1.0f;
    inline constexpr bool kOnlyBreakWhenTargetIsPlayer = true;

    namespace detail
    {
        struct State
        {
            float cooldownRemaining{ 0.0f };
            RE::ActorHandle brokenTarget{};
            bool suppressedReacquireLogged{ false };
        };

        inline std::unordered_map<RE::FormID, State> g_states;
        inline std::mutex g_mutex;

        [[nodiscard]] inline float SanitizeDeltaTime(float deltaTime) noexcept
        {
            if (!std::isfinite(deltaTime) || deltaTime <= 0.0f) {
                return 0.0f;
            }

            // Cooldown is a gameplay timer. Cap pathological single-frame spikes so a
            // hitch cannot consume the whole lockout instantly.
            return std::min(deltaTime, 0.100f);
        }

        [[nodiscard]] inline bool IsEligibleActor(RE::Character* actor) noexcept
        {
            return actor &&
                   !actor->IsPlayerRef() &&
                   !actor->IsDead() &&
                   actor->IsInCombat();
        }

        inline void SuppressLockOnNow(RE::Character* actor) noexcept
        {
            if (!actor) {
                return;
            }

            auto& runtime = actor->GetActorRuntimeData();

            // This is the actual Actor combat-target handle exposed by CommonLibSSE NG.
            // Clearing it breaks the current combat lock without stopping combat itself.
            runtime.currentCombatTarget.reset();

            // Also suppress Skyrim's actor tracking rotation during the cooldown. This is
            // deliberately limited to the cooldown and avoids touching actor yaw directly.
            runtime.boolBits.reset(RE::Actor::BOOL_BITS::kShouldRotateToTrack);
        }

        [[nodiscard]] inline bool TickAndSuppressCooldown(
            RE::Character* actor,
            float deltaTime,
            bool advanceTimer)
        {
            if (!actor) {
                return false;
            }

            bool cooling = false;
            bool ended = false;
            bool logReacquireSuppression = false;
            float remaining = 0.0f;
            RE::ActorHandle currentTarget{};

            {
                std::scoped_lock lock{ g_mutex };

                const auto it = g_states.find(actor->GetFormID());
                if (it == g_states.end()) {
                    return false;
                }

                auto& state = it->second;

                if (advanceTimer) {
                    state.cooldownRemaining =
                        std::max(0.0f, state.cooldownRemaining - SanitizeDeltaTime(deltaTime));
                }

                if (state.cooldownRemaining > 0.0f) {
                    cooling = true;
                    remaining = state.cooldownRemaining;

                    currentTarget = actor->GetActorRuntimeData().currentCombatTarget;
                    if (currentTarget && !state.suppressedReacquireLogged) {
                        state.suppressedReacquireLogged = true;
                        logReacquireSuppression = true;
                    }
                } else {
                    ended = true;
                    g_states.erase(it);
                }
            }

            if (cooling) {
                SuppressLockOnNow(actor);

                if (logReacquireSuppression) {
                    SKSE::log::info(
                        "[NPCLockOnBreaker][REACQUIRE_BLOCKED] actor={:08X} remaining={:.3f}s action=CLEAR_CURRENT_COMBAT_TARGET_PLUS_ROTATE_TRACK",
                        actor->GetFormID(), remaining);
                }
                return true;
            }

            if (ended) {
                SKSE::log::info(
                    "[NPCLockOnBreaker][COOLDOWN_END] actor={:08X} vanillaTargetAcquisition=RESTORED",
                    actor->GetFormID());
            }

            return false;
        }
    }

    inline void BeforeCharacterUpdate(RE::Character* actor, float deltaTime)
    {
        if (!detail::IsEligibleActor(actor)) {
            if (actor) {
                std::scoped_lock lock{ detail::g_mutex };
                detail::g_states.erase(actor->GetFormID());
            }
            return;
        }

        // While cooling down, keep the combat target empty before vanilla Actor::Update
        // and keep rotation tracking disabled.
        if (detail::TickAndSuppressCooldown(actor, deltaTime, true)) {
            return;
        }

        auto& runtime = actor->GetActorRuntimeData();
        const RE::ActorHandle targetHandle = runtime.currentCombatTarget;
        if (!targetHandle) {
            return;
        }

        const auto target = targetHandle.get();
        if (!target || target->IsDead()) {
            return;
        }

        if constexpr (kOnlyBreakWhenTargetIsPlayer) {
            if (!target->IsPlayerRef()) {
                return;
            }
        }

        const float angleDegrees =
            actor->GetHeadingAngle(target->GetPosition(), true);

        if (!std::isfinite(angleDegrees) || angleDegrees < kBreakAngleDegrees) {
            return;
        }

        {
            std::scoped_lock lock{ detail::g_mutex };
            auto& state = detail::g_states[actor->GetFormID()];
            state.cooldownRemaining = kCooldownSeconds;
            state.brokenTarget = targetHandle;
            state.suppressedReacquireLogged = false;
        }

        // Break before vanilla update can use the target to demand an arbitrary yaw.
        detail::SuppressLockOnNow(actor);

        SKSE::log::info(
            "[NPCLockOnBreaker][BREAK] actor={:08X} target={:08X} angle={:.2f}deg threshold={:.2f}deg cooldown={:.2f}s action=CLEAR_CURRENT_COMBAT_TARGET_PLUS_ROTATE_TRACK",
            actor->GetFormID(), target->GetFormID(), angleDegrees,
            kBreakAngleDegrees, kCooldownSeconds);
    }

    inline void AfterCharacterUpdate(RE::Character* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }

        // Vanilla may try to reacquire during Actor::Update. Clear it again on the
        // outgoing edge while the one-second cooldown is still active.
        (void)detail::TickAndSuppressCooldown(actor, 0.0f, false);
    }

    inline void Clear()
    {
        std::scoped_lock lock{ detail::g_mutex };
        detail::g_states.clear();

        SKSE::log::info(
            "[NPCLockOnBreaker][CLEAR] build=ALR-NPC-LOCKON-BREAKER-45DEG-1S-2026-08-19");
    }
}

