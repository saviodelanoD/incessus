#pragma once

// BUILD_ID: ALR-TRANSLATION-AUTHORITY-R1-2026-08-20

#include "RE/Skyrim.h"
#include "SKSE/SKSE.h"

#include <cstdint>
#include <mutex>
#include <unordered_map>

namespace TranslationAuthority
{
    enum class Owner : std::uint8_t
    {
        kVanilla = 0,
        kMotionDbPending,
        kMotionDb
    };

    namespace detail
    {
        struct State
        {
            Owner owner{ Owner::kVanilla };
            std::uint64_t frameSerial{ 0 };
            std::uint64_t claims{ 0 };
            std::uint64_t planarOverrides{ 0 };
            bool announced{ false };
            bool overrideAnnounced{ false };
        };

        inline std::unordered_map<RE::FormID, State> g_state;
        inline std::mutex g_mutex;
    }

    inline void BeginFrame(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }

        std::scoped_lock lock{ detail::g_mutex };
        auto& state = detail::g_state[actor->GetFormID()];
        ++state.frameSerial;
        state.owner = Owner::kMotionDbPending;
    }

    inline void ObserveMotionDbTranslation(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }

        bool announce = false;
        std::uint64_t frame = 0;
        std::uint64_t claims = 0;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_state.find(actor->GetFormID());
            if (it == detail::g_state.end() || it->second.owner == Owner::kVanilla) {
                return;
            }

            auto& state = it->second;
            state.owner = Owner::kMotionDb;
            ++state.claims;
            frame = state.frameSerial;
            claims = state.claims;
            if (!state.announced) {
                state.announced = true;
                announce = true;
            }
        }

        if (announce) {
            SKSE::log::info(
                "[TranslationAuthority][CLAIM] form={:08X} frame={} owner=MOTIONDB axes=XY vertical=VANILLA claims={}",
                actor->GetFormID(), frame, claims);
        }
    }

    [[nodiscard]] inline bool OwnsPlanarTranslation(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_state.find(actor->GetFormID());
        return it != detail::g_state.end() && it->second.owner == Owner::kMotionDb;
    }


    inline void ObservePlanarOverride(
        RE::Actor* actor,
        const RE::NiPoint3& authoredTranslation,
        const RE::NiPoint3& vanillaTranslation,
        const RE::NiPoint3& finalTranslation)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }

        bool announce = false;
        std::uint64_t frame = 0;
        std::uint64_t overrides = 0;
        {
            std::scoped_lock lock{ detail::g_mutex };
            const auto it = detail::g_state.find(actor->GetFormID());
            if (it == detail::g_state.end() || it->second.owner != Owner::kMotionDb) {
                return;
            }

            auto& state = it->second;
            ++state.planarOverrides;
            frame = state.frameSerial;
            overrides = state.planarOverrides;
            if (!state.overrideAnnounced) {
                state.overrideAnnounced = true;
                announce = true;
            }
        }

        if (announce) {
            SKSE::log::info(
                "[TranslationAuthority][APPLY] form={:08X} frame={} authoredXY=({:+.5f},{:+.5f}) vanillaXY=({:+.5f},{:+.5f}) finalXY=({:+.5f},{:+.5f}) vanillaZ={:+.5f} finalZ={:+.5f} overrides={}",
                actor->GetFormID(), frame,
                authoredTranslation.x, authoredTranslation.y,
                vanillaTranslation.x, vanillaTranslation.y,
                finalTranslation.x, finalTranslation.y,
                vanillaTranslation.z, finalTranslation.z, overrides);
        }
    }

    inline void EndFrame(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }

        std::scoped_lock lock{ detail::g_mutex };
        const auto it = detail::g_state.find(actor->GetFormID());
        if (it != detail::g_state.end()) {
            it->second.owner = Owner::kVanilla;
        }
    }

    inline void Clear()
    {
        std::scoped_lock lock{ detail::g_mutex };
        detail::g_state.clear();
    }
}

