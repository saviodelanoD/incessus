// BUILD_ID: ALR-R67-DM-SAFE-QUERY-CORE-MW1-NATIVE-ACTION-TRANSLATION-WARP-2026-09-01

#include "PCH.h"
#include "MotionTrajectoryController.h"
#include "AnimationMotionDatabase.h"
#include "MotionProfileDatabase.h"
#include "SteeringIntentController.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <cctype>
#include <limits>
#include <mutex>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <vector>
#include <cstdlib>
#include <string>
#include <string_view>
#include <unordered_map>

namespace MotionTrajectoryController
{
    namespace
    {
        constexpr float kEpsilon = 1.0e-5f;
        constexpr std::size_t kCoarseSegments = 64;
        constexpr std::size_t kRefineIterations = 16;

        struct Basis
        {
            bool valid{ false };
            bool yawValid{ false };
            AnimationMotionDatabase::ClipMetrics metrics{};
            AnimationMotionDatabase::CumulativeSample start{};
            AnimationMotionDatabase::CumulativeSample end{};
            float startTime{ 0.0f };
            float endTime{ 0.0f };
            float axisX{ 0.0f };
            float axisY{ 0.0f };
            float totalDistance{ 0.0f };
            float yawSign{ 1.0f };
            float totalYaw{ 0.0f };
        };

        std::mutex g_mutex;
        bool g_installLogged{ false };

        struct MotionWarpConfig
        {
            bool loaded{ false };
            bool enabled{ false };
            std::string actionVariable{ "IsAttacking" };
            std::vector<std::string> clipContains;
            float windowSeconds{ 0.30f };
            float targetDistance{ 110.0f };
            float maxFacingErrorDegrees{ 30.0f };
            float translationStrength{ 0.75f };
            float minTranslationScale{ 0.70f };
            float maxTranslationScale{ 1.35f };
            float maxPlanarSpeed{ 750.0f };
            bool ignoreZ{ true };
            bool warpRotation{ false };
        };

        struct MotionWarpState
        {
            bool active{ false };
            std::string profile;
            std::string animation;
            RE::FormID targetFormID{ 0 };
            float latestLocalTime{ 0.0f };
            float startDistance{ 0.0f };
            float lastDistance{ 0.0f };
            float lastFacingErrorDegrees{ 0.0f };
            std::uint64_t lastSeenFrameSerial{ 0 };
            std::uint64_t writes{ 0 };
            std::uint64_t skipFacing{ 0 };
            std::uint64_t skipNoRoot{ 0 };
        };

        std::mutex g_configMutex;
        std::unordered_map<std::string, MotionWarpConfig> g_warpConfigs;
        std::unordered_map<RE::FormID, MotionWarpState> g_warpState;

        [[nodiscard]] std::string LowerAscii(std::string_view value)
        {
            std::string out;
            out.reserve(value.size());
            for (const unsigned char c : value) {
                out.push_back(static_cast<char>(std::tolower(c)));
            }
            return out;
        }

        [[nodiscard]] std::string Trim(std::string value)
        {
            const auto notSpace = [](unsigned char c) { return !std::isspace(c); };
            value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
            value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
            return value;
        }

        [[nodiscard]] std::vector<std::string> SplitCsvLower(const std::string& value)
        {
            std::vector<std::string> out;
            std::stringstream stream(value);
            std::string item;
            while (std::getline(stream, item, ',')) {
                item = Trim(std::move(item));
                if (!item.empty()) out.push_back(LowerAscii(item));
            }
            return out;
        }

        [[nodiscard]] bool ParseBool(std::string value, bool fallback) noexcept
        {
            value = LowerAscii(Trim(std::move(value)));
            if (value == "true" || value == "1" || value == "yes" || value == "on") return true;
            if (value == "false" || value == "0" || value == "no" || value == "off") return false;
            return fallback;
        }

        [[nodiscard]] float ParseFloat(const std::string& value, float fallback) noexcept
        {
            char* end = nullptr;
            const float parsed = std::strtof(value.c_str(), &end);
            return end && end != value.c_str() && std::isfinite(parsed) ? parsed : fallback;
        }

        [[nodiscard]] MotionWarpConfig LoadMotionWarpConfig(std::string_view profileFolder)
        {
            MotionWarpConfig cfg{};
            cfg.loaded = true;
            namespace fs = std::filesystem;
            const fs::path path = fs::path("Data") / "MotionDB" / std::string(profileFolder) / "Intent.ini";
            std::ifstream input(path);
            if (!input) {
                SKSE::log::warn(R"([MotionTrajectory][MW1_CONFIG] profile="{}" path="{}" loaded=false action=DISABLED)",
                    profileFolder, path.string());
                return cfg;
            }

            std::string section;
            std::string line;
            while (std::getline(input, line)) {
                line = Trim(std::move(line));
                if (line.empty() || line[0] == ';' || line[0] == '#') continue;
                if (line.front() == '[' && line.back() == ']') {
                    section = LowerAscii(Trim(line.substr(1, line.size() - 2)));
                    continue;
                }
                if (section != "motionwarping") continue;
                const auto eq = line.find('=');
                if (eq == std::string::npos) continue;
                const std::string key = LowerAscii(Trim(line.substr(0, eq)));
                const std::string value = Trim(line.substr(eq + 1));
                if (key == "enabled") cfg.enabled = ParseBool(value, cfg.enabled);
                else if (key == "actionvariable") cfg.actionVariable = value;
                else if (key == "clipcontains") cfg.clipContains = SplitCsvLower(value);
                else if (key == "windowseconds") cfg.windowSeconds = ParseFloat(value, cfg.windowSeconds);
                else if (key == "targetdistance") cfg.targetDistance = ParseFloat(value, cfg.targetDistance);
                else if (key == "maxfacingerrordegrees") cfg.maxFacingErrorDegrees = ParseFloat(value, cfg.maxFacingErrorDegrees);
                else if (key == "translationstrength") cfg.translationStrength = ParseFloat(value, cfg.translationStrength);
                else if (key == "mintranslationscale") cfg.minTranslationScale = ParseFloat(value, cfg.minTranslationScale);
                else if (key == "maxtranslationscale") cfg.maxTranslationScale = ParseFloat(value, cfg.maxTranslationScale);
                else if (key == "maxplanarspeed") cfg.maxPlanarSpeed = ParseFloat(value, cfg.maxPlanarSpeed);
                else if (key == "ignorez") cfg.ignoreZ = ParseBool(value, cfg.ignoreZ);
                else if (key == "warprotation") cfg.warpRotation = ParseBool(value, cfg.warpRotation);
            }

            cfg.windowSeconds = std::clamp(cfg.windowSeconds, 0.05f, 1.50f);
            cfg.targetDistance = std::clamp(cfg.targetDistance, 0.0f, 1000.0f);
            cfg.maxFacingErrorDegrees = std::clamp(cfg.maxFacingErrorDegrees, 1.0f, 89.0f);
            cfg.translationStrength = std::clamp(cfg.translationStrength, 0.0f, 1.0f);
            cfg.minTranslationScale = std::clamp(cfg.minTranslationScale, 0.0f, 1.0f);
            cfg.maxTranslationScale = std::clamp(cfg.maxTranslationScale, 1.0f, 3.0f);
            if (cfg.maxTranslationScale < cfg.minTranslationScale) cfg.maxTranslationScale = cfg.minTranslationScale;
            cfg.maxPlanarSpeed = std::clamp(cfg.maxPlanarSpeed, 1.0f, 5000.0f);

            // MW1 is intentionally translation-only and planar. Refuse a profile that
            // asks this first candidate to take rotation or Z ownership.
            if (!cfg.ignoreZ || cfg.warpRotation || cfg.clipContains.empty() || cfg.actionVariable.empty()) {
                SKSE::log::warn(
                    R"([MotionTrajectory][MW1_CONFIG] profile="{}" path="{}" enabled=false reason=MW1_REQUIRES_IGNORE_Z_TRUE_WARP_ROTATION_FALSE_ACTION_AND_CLIPS ignoreZ={} warpRotation={} clips={})",
                    profileFolder, path.string(), cfg.ignoreZ, cfg.warpRotation, cfg.clipContains.size());
                cfg.enabled = false;
            } else {
                SKSE::log::info(
                    R"([MotionTrajectory][MW1_CONFIG] profile="{}" enabled={} actionVariable="{}" clips={} window={:.3f}s targetDistance={:.2f} maxFacing={:.1f}deg strength={:.2f} scale=[{:.2f},{:.2f}] maxPlanarSpeed={:.1f} ignoreZ=true warpRotation=false)",
                    profileFolder, cfg.enabled, cfg.actionVariable, cfg.clipContains.size(), cfg.windowSeconds,
                    cfg.targetDistance, cfg.maxFacingErrorDegrees, cfg.translationStrength,
                    cfg.minTranslationScale, cfg.maxTranslationScale, cfg.maxPlanarSpeed);
            }
            return cfg;
        }

        [[nodiscard]] MotionWarpConfig GetMotionWarpConfig(std::string_view profileFolder)
        {
            const std::string key = LowerAscii(profileFolder);
            std::scoped_lock lock{ g_configMutex };
            if (const auto it = g_warpConfigs.find(key); it != g_warpConfigs.end()) return it->second;
            auto cfg = LoadMotionWarpConfig(profileFolder);
            g_warpConfigs.emplace(key, cfg);
            return cfg;
        }

        [[nodiscard]] bool AnimationMatchesMotionWarp(
            const MotionWarpConfig& cfg,
            std::string_view animationName)
        {
            const std::string lower = LowerAscii(animationName);
            for (const auto& token : cfg.clipContains) {
                if (!token.empty() && lower.find(token) != std::string::npos) return true;
            }
            return false;
        }

        [[nodiscard]] bool ReadGraphBool(RE::Actor* actor, const std::string& variable) noexcept
        {
            if (!actor || variable.empty()) return false;
            bool value = false;
            return actor->GetGraphVariableBool(variable.c_str(), value) && value;
        }

        [[nodiscard]] float SmoothStep01(float x) noexcept
        {
            x = std::clamp(x, 0.0f, 1.0f);
            return x * x * (3.0f - 2.0f * x);
        }

        void EndMotionWarpLocked(RE::FormID formID, const char* reason)
        {
            const auto it = g_warpState.find(formID);
            if (it == g_warpState.end() || !it->second.active) return;
            const auto state = it->second;
            SKSE::log::info(
                R"([MotionTrajectory][MW1_SESSION_END] form={:08X} profile="{}" anim="{}" target={:08X} reason={} localTime={:.4f} distance[start={:.2f} last={:.2f}] writes={} rootOwnership=UNCHANGED)",
                formID, state.profile, state.animation, state.targetFormID, reason,
                state.latestLocalTime, state.startDistance, state.lastDistance, state.writes);
            g_warpState.erase(it);
        }

        [[nodiscard]] Basis BuildBasis(std::string_view profile, std::string_view animationName)
        {
            Basis basis{};
            basis.metrics = AnimationMotionDatabase::GetClipMetrics(profile, animationName);
            if (!basis.metrics.valid || !std::isfinite(basis.metrics.duration) || basis.metrics.duration <= kEpsilon) {
                return basis;
            }
            basis.start = AnimationMotionDatabase::SampleCumulativeSeconds(
                profile, animationName, -(std::numeric_limits<float>::max)());
            basis.end = AnimationMotionDatabase::SampleCumulativeSeconds(
                profile, animationName, (std::numeric_limits<float>::max)());
            if (!basis.start.valid || !basis.end.valid) return basis;
            basis.startTime = basis.start.motionTime;
            basis.endTime = basis.end.motionTime;
            if (!std::isfinite(basis.startTime) || !std::isfinite(basis.endTime) ||
                basis.endTime - basis.startTime <= kEpsilon) return basis;

            const float dx = basis.end.x - basis.start.x;
            const float dy = basis.end.y - basis.start.y;
            basis.totalDistance = std::hypot(dx, dy);
            if (std::isfinite(basis.totalDistance) && basis.totalDistance > kEpsilon) {
                basis.axisX = dx / basis.totalDistance;
                basis.axisY = dy / basis.totalDistance;
            } else {
                basis.totalDistance = 0.0f;
            }

            basis.yawValid = basis.start.yawValid && basis.end.yawValid;
            if (basis.yawValid) {
                const float authoredYaw = basis.end.yaw - basis.start.yaw;
                basis.totalYaw = std::fabs(authoredYaw);
                basis.yawSign = authoredYaw < 0.0f ? -1.0f : 1.0f;
                if (!std::isfinite(basis.totalYaw)) {
                    basis.totalYaw = 0.0f;
                    basis.yawValid = false;
                }
            }
            basis.valid = true;
            return basis;
        }

        [[nodiscard]] float ProjectDistance(
            const Basis& basis,
            const AnimationMotionDatabase::CumulativeSample& sample) noexcept
        {
            if (!basis.valid || !sample.valid || basis.totalDistance <= kEpsilon) return 0.0f;
            const float dx = sample.x - basis.start.x;
            const float dy = sample.y - basis.start.y;
            const float projected = dx * basis.axisX + dy * basis.axisY;
            return std::isfinite(projected) ? projected : 0.0f;
        }

        [[nodiscard]] float ProjectYaw(
            const Basis& basis,
            const AnimationMotionDatabase::CumulativeSample& sample) noexcept
        {
            if (!basis.valid || !basis.yawValid || !sample.valid || !sample.yawValid) return 0.0f;
            const float projected = (sample.yaw - basis.start.yaw) * basis.yawSign;
            return std::isfinite(projected) ? projected : 0.0f;
        }

        template <class ProgressFn>
        [[nodiscard]] MatchResult Solve(
            std::string_view profile,
            std::string_view animationName,
            const Basis& basis,
            MatchMetric metric,
            float requested,
            float totalAvailable,
            ProgressFn&& progressFn)
        {
            MatchResult out{};
            out.metric = metric;
            out.profile = basis.metrics.profile;
            out.key = basis.metrics.key;
            out.clipDuration = basis.metrics.duration;
            out.totalAvailable = totalAvailable;
            if (!basis.valid || !std::isfinite(requested) || !std::isfinite(totalAvailable) || totalAvailable <= kEpsilon) return out;

            const float target = std::clamp(requested, 0.0f, totalAvailable);
            out.requested = target;
            out.clamped = std::fabs(target - requested) > kEpsilon;

            struct Point { float time{}, progress{}; bool valid{}; };
            std::array<Point, kCoarseSegments + 1> points{};
            std::size_t bestIndex = 0;
            float bestError = std::numeric_limits<float>::infinity();
            std::size_t crossingIndex = points.size();
            float crossingScore = std::numeric_limits<float>::infinity();

            for (std::size_t i = 0; i < points.size(); ++i) {
                const float alpha = static_cast<float>(i) / static_cast<float>(kCoarseSegments);
                const float time = basis.startTime + (basis.endTime - basis.startTime) * alpha;
                const auto sample = AnimationMotionDatabase::SampleCumulativeSeconds(profile, animationName, time);
                if (!sample.valid) continue;
                const float progress = progressFn(sample);
                if (!std::isfinite(progress)) continue;
                points[i] = { time, progress, true };
                const float error = std::fabs(progress - target);
                if (error < bestError) { bestError = error; bestIndex = i; }
                if (i > 0 && points[i - 1].valid) {
                    const float a = points[i - 1].progress - target;
                    const float b = progress - target;
                    if ((a <= 0.0f && b >= 0.0f) || (a >= 0.0f && b <= 0.0f)) {
                        const float score = std::fabs(a) + std::fabs(b);
                        if (score < crossingScore) { crossingScore = score; crossingIndex = i - 1; }
                    }
                }
            }

            if (!points[bestIndex].valid) return out;
            float loTime = points[bestIndex].time, hiTime = points[bestIndex].time;
            float loProgress = points[bestIndex].progress, hiProgress = points[bestIndex].progress;
            if (crossingIndex < kCoarseSegments && points[crossingIndex].valid && points[crossingIndex + 1].valid) {
                loTime = points[crossingIndex].time; hiTime = points[crossingIndex + 1].time;
                loProgress = points[crossingIndex].progress; hiProgress = points[crossingIndex + 1].progress;
                for (std::size_t n = 0; n < kRefineIterations; ++n) {
                    const float midTime = 0.5f * (loTime + hiTime);
                    const auto sample = AnimationMotionDatabase::SampleCumulativeSeconds(profile, animationName, midTime);
                    if (!sample.valid) break;
                    const float midProgress = progressFn(sample);
                    if (!std::isfinite(midProgress)) break;
                    const bool inLoMid = (loProgress <= target && target <= midProgress) ||
                        (loProgress >= target && target >= midProgress);
                    if (inLoMid) { hiTime = midTime; hiProgress = midProgress; }
                    else { loTime = midTime; loProgress = midProgress; }
                }
                if (std::fabs(hiProgress - target) < std::fabs(loProgress - target)) {
                    out.motionTime = hiTime; out.achieved = hiProgress;
                } else {
                    out.motionTime = loTime; out.achieved = loProgress;
                }
            } else {
                out.motionTime = points[bestIndex].time;
                out.achieved = points[bestIndex].progress;
            }
            out.normalizedTime = std::clamp((out.motionTime - basis.startTime) / (basis.endTime - basis.startTime), 0.0f, 1.0f);
            out.error = out.achieved - target;
            out.valid = std::isfinite(out.motionTime) && std::isfinite(out.achieved) && std::isfinite(out.error);
            return out;
        }

    }

    MatchResult FindTimeForDistance(std::string_view profile, std::string_view animationName, float targetDistance)
    {
        const auto basis = BuildBasis(profile, animationName);
        if (!basis.valid || basis.totalDistance <= kEpsilon) return {};
        return Solve(profile, animationName, basis, MatchMetric::kProjectedDistance,
            targetDistance, basis.totalDistance,
            [&](const AnimationMotionDatabase::CumulativeSample& s) { return ProjectDistance(basis, s); });
    }

    MatchResult FindTimeForRemainingDistance(std::string_view profile, std::string_view animationName, float remainingDistance)
    {
        const auto basis = BuildBasis(profile, animationName);
        if (!basis.valid || basis.totalDistance <= kEpsilon || !std::isfinite(remainingDistance)) return {};
        const float remaining = std::clamp(remainingDistance, 0.0f, basis.totalDistance);
        return Solve(profile, animationName, basis, MatchMetric::kProjectedDistance,
            basis.totalDistance - remaining, basis.totalDistance,
            [&](const AnimationMotionDatabase::CumulativeSample& s) { return ProjectDistance(basis, s); });
    }

    MatchResult FindTimeForRemainingYaw(std::string_view profile, std::string_view animationName, float remainingYawRadians)
    {
        const auto basis = BuildBasis(profile, animationName);
        if (!basis.valid || !basis.yawValid || basis.totalYaw <= kEpsilon || !std::isfinite(remainingYawRadians)) return {};
        const float remaining = std::clamp(std::fabs(remainingYawRadians), 0.0f, basis.totalYaw);
        return Solve(profile, animationName, basis, MatchMetric::kYaw,
            basis.totalYaw - remaining, basis.totalYaw,
            [&](const AnimationMotionDatabase::CumulativeSample& s) { return ProjectYaw(basis, s); });
    }

    void ApplyAfterNativeClipUpdate(RE::FormID actorFormID, RE::hkbClipGenerator* clip, float inputDeltaTime)
    {
        (void)inputDeltaTime;
        if (actorFormID == 0 || !clip || !std::isfinite(clip->localTime)) return;
        auto* actor = RE::TESForm::LookupByID<RE::Actor>(actorFormID);
        if (!actor || actor->IsPlayerRef() || actor->IsDead() || !actor->Is3DLoaded()) return;
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->folder.empty()) return;

        if (!g_installLogged) {
            std::scoped_lock lock{ g_mutex };
            if (!g_installLogged) {
                g_installLogged = true;
                SKSE::log::info(
                    "[MotionTrajectory][R67_INSTALL] DistanceMatching=QUERY_CORE_PRESERVED TurnLoopLocalTimeWrites=ZERO MotionWarping=MW1_FUNCTIONAL_NATIVE_ACTION_TRANSLATION_ONLY NativeGate=REQUIRED DB_ROOT=UNTOUCHED CapsuleYaw=UNTOUCHED");
            }
        }

        const char* rawAnim = clip->animationName.c_str();
        const std::string animation = rawAnim ? rawAnim : "";
        if (animation.empty()) return;

        // R67 correction from the long R66 playtest: the post-update path performs
        // ZERO localTime writes. The inverse Distance Matching API remains available
        // for true finite trajectories, but looping RotationClips are no longer seeked.

        const MotionWarpConfig cfg = GetMotionWarpConfig(profile->folder);
        if (!cfg.enabled) return;

        const bool actionActive = ReadGraphBool(actor, cfg.actionVariable);
        if (!actionActive) {
            std::scoped_lock lock{ g_mutex };
            EndMotionWarpLocked(actorFormID, "ACTION_VARIABLE_FALSE");
            return;
        }

        if (!AnimationMatchesMotionWarp(cfg, animation)) return;

        SteeringIntentController::FrameCommand command{};
        if (!SteeringIntentController::GetFrameCommand(actor, command) || !command.valid ||
            command.source != SteeringIntentController::Source::kCombatTarget || command.targetFormID == 0) {
            return;
        }

        const float facingErrorDegrees = std::fabs(command.filteredErrorRadians) * 57.29577951308232f;
        const float effectiveLocalTime = std::max(0.0f, clip->localTime);
        if (effectiveLocalTime > cfg.windowSeconds + 0.05f) return;

        std::scoped_lock lock{ g_mutex };
        auto it = g_warpState.find(actorFormID);
        const bool needsNew = it == g_warpState.end() || !it->second.active ||
            it->second.animation != animation || it->second.targetFormID != command.targetFormID ||
            effectiveLocalTime + 0.05f < it->second.latestLocalTime;
        if (needsNew) {
            if (it != g_warpState.end() && it->second.active) EndMotionWarpLocked(actorFormID, "NEW_FINITE_ACTION");
            MotionWarpState state{};
            state.active = true;
            state.profile = profile->folder;
            state.animation = animation;
            state.targetFormID = command.targetFormID;
            state.latestLocalTime = effectiveLocalTime;
            state.startDistance = command.goalDistanceXY;
            state.lastDistance = command.goalDistanceXY;
            state.lastFacingErrorDegrees = facingErrorDegrees;
            state.lastSeenFrameSerial = command.frameSerial;
            g_warpState[actorFormID] = state;
            SKSE::log::info(
                R"([MotionTrajectory][MW1_SESSION_BEGIN] form={:08X} profile="{}" anim="{}" target={:08X} localTime={:.4f} distance={:.2f} targetDistance={:.2f} facingError={:.2f}deg window={:.3f}s mode=NATIVE_AUTHORED_XY_SCALE_ONLY)",
                actorFormID, profile->folder, animation, command.targetFormID, effectiveLocalTime,
                command.goalDistanceXY, cfg.targetDistance, facingErrorDegrees, cfg.windowSeconds);
            return;
        }

        auto& state = it->second;
        state.latestLocalTime = effectiveLocalTime;
        state.lastDistance = command.goalDistanceXY;
        state.lastFacingErrorDegrees = facingErrorDegrees;
        state.lastSeenFrameSerial = command.frameSerial;
    }

    bool WarpNativeActionTranslation(
        RE::Actor* actor,
        float deltaTime,
        const RE::NiPoint3& authoredTranslation,
        RE::NiPoint3& outTranslation)
    {
        outTranslation = authoredTranslation;
        if (!actor || actor->IsPlayerRef() || actor->IsDead() || !actor->Is3DLoaded()) return false;
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || profile->folder.empty()) return false;
        const MotionWarpConfig cfg = GetMotionWarpConfig(profile->folder);
        if (!cfg.enabled || !ReadGraphBool(actor, cfg.actionVariable)) return false;

        SteeringIntentController::FrameCommand command{};
        if (!SteeringIntentController::GetFrameCommand(actor, command) || !command.valid ||
            command.source != SteeringIntentController::Source::kCombatTarget || command.targetFormID == 0) {
            return false;
        }

        const float dt = (std::isfinite(deltaTime) && deltaTime > 1.0e-5f && deltaTime < 0.10f) ?
            deltaTime : (1.0f / 60.0f);
        const float authoredPlanar = std::hypot(authoredTranslation.x, authoredTranslation.y);

        std::scoped_lock lock{ g_mutex };
        auto it = g_warpState.find(actor->GetFormID());
        if (it == g_warpState.end() || !it->second.active) return false;
        auto& state = it->second;

        if (state.targetFormID != command.targetFormID) {
            EndMotionWarpLocked(actor->GetFormID(), "COMBAT_TARGET_CHANGED");
            return false;
        }
        if (command.frameSerial > state.lastSeenFrameSerial + 3) {
            EndMotionWarpLocked(actor->GetFormID(), "ACTION_CLIP_NOT_RECENT");
            return false;
        }
        if (state.latestLocalTime > cfg.windowSeconds) {
            EndMotionWarpLocked(actor->GetFormID(), "WARP_WINDOW_COMPLETE");
            return false;
        }

        const float facingErrorDegrees = std::fabs(command.filteredErrorRadians) * 57.29577951308232f;
        state.lastDistance = command.goalDistanceXY;
        state.lastFacingErrorDegrees = facingErrorDegrees;
        if (!std::isfinite(facingErrorDegrees) || facingErrorDegrees > cfg.maxFacingErrorDegrees) {
            ++state.skipFacing;
            if (state.skipFacing <= 3 || state.skipFacing % 30 == 0) {
                SKSE::log::info(
                    R"([MotionTrajectory][MW1_SKIP_FACING] form={:08X} anim="{}" target={:08X} facingError={:.2f}deg max={:.2f}deg localTime={:.4f} skips={} action=KEEP_NATIVE_ROOT)",
                    actor->GetFormID(), state.animation, state.targetFormID, facingErrorDegrees,
                    cfg.maxFacingErrorDegrees, state.latestLocalTime, state.skipFacing);
            }
            return false;
        }

        // Never synthesize root motion into an animation that authored none. MW1 only
        // rescales native planar root that already passed the existing native gate.
        if (!std::isfinite(authoredPlanar) || authoredPlanar <= 1.0e-4f) {
            ++state.skipNoRoot;
            if (state.skipNoRoot <= 2 || state.skipNoRoot % 30 == 0) {
                SKSE::log::info(
                    R"([MotionTrajectory][MW1_SKIP_NO_AUTHORED_ROOT] form={:08X} anim="{}" localTime={:.4f} skips={} action=NO_INJECTION)",
                    actor->GetFormID(), state.animation, state.latestLocalTime, state.skipNoRoot);
            }
            return false;
        }

        const float progress = std::clamp(state.latestLocalTime / cfg.windowSeconds, 0.0f, 1.0f);
        const float rampIn = SmoothStep01(progress / 0.15f);
        const float rampOut = 1.0f - SmoothStep01((progress - 0.85f) / 0.15f);
        const float windowWeight = std::clamp(rampIn * rampOut, 0.0f, 1.0f);
        if (windowWeight <= 1.0e-4f) return false;

        const float remainingTime = std::max(cfg.windowSeconds - state.latestLocalTime, dt);
        const float closingDistance = std::max(command.goalDistanceXY - cfg.targetDistance, 0.0f);
        const float authoredSpeed = authoredPlanar / dt;
        if (!std::isfinite(authoredSpeed) || authoredSpeed <= 1.0e-4f) return false;

        const float desiredClosingSpeed = closingDistance / remainingTime;
        float targetScale = desiredClosingSpeed / authoredSpeed;
        if (!std::isfinite(targetScale)) targetScale = 1.0f;
        targetScale = std::clamp(targetScale, cfg.minTranslationScale, cfg.maxTranslationScale);

        // MaxPlanarSpeed is an amplification ceiling only. If vanilla itself is already
        // faster than this diagnostic cap, MW1 never slows it merely because of the cap.
        const float maxAmplificationBySpeed = std::max(1.0f, cfg.maxPlanarSpeed / authoredSpeed);
        targetScale = std::min(targetScale, maxAmplificationBySpeed);

        const float strength = cfg.translationStrength * windowWeight;
        const float appliedScale = std::clamp(
            1.0f + (targetScale - 1.0f) * strength,
            cfg.minTranslationScale,
            cfg.maxTranslationScale);
        if (!std::isfinite(appliedScale) || std::fabs(appliedScale - 1.0f) < 0.005f) return false;

        outTranslation.x = authoredTranslation.x * appliedScale;
        outTranslation.y = authoredTranslation.y * appliedScale;
        outTranslation.z = authoredTranslation.z;  // MW1 IgnoreZ is mandatory.

        ++state.writes;
        if (state.writes <= 8 || state.writes % 30 == 0) {
            SKSE::log::info(
                R"([MotionTrajectory][MW1_TRANSLATION_WRITE] form={:08X} anim="{}" target={:08X} localTime={:.4f}/{:.4f} progress={:.3f} dist={:.2f} desiredStop={:.2f} facing={:.2f}deg authoredStep={:.4f} authoredSpeed={:.2f} scale[target={:.3f} applied={:.3f}] warpedStep={:.4f} writes={} channel=OPEN_NATIVE_PREVANILLA_XY_ONLY z=UNCHANGED yaw=UNCHANGED collisionDebt=NONE)",
                actor->GetFormID(), state.animation, state.targetFormID,
                state.latestLocalTime, cfg.windowSeconds, progress, command.goalDistanceXY,
                cfg.targetDistance, facingErrorDegrees, authoredPlanar, authoredSpeed,
                targetScale, appliedScale, std::hypot(outTranslation.x, outTranslation.y), state.writes);
        }
        return true;
    }

    void Clear()
    {
        {
            std::scoped_lock lock{ g_mutex };
            g_warpState.clear();
            g_installLogged = false;
        }
        {
            std::scoped_lock lock{ g_configMutex };
            g_warpConfigs.clear();
        }
    }
}

