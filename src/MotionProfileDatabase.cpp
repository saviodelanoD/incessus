// BUILD_ID: ALR-MOTION-PROFILE-RUNTIME-ANIMROTATION-CLASSES-R1-2026-08-27
#include "PCH.h"
#include "MotionProfileDatabase.h"

#include <algorithm>
#include <cctype>
#include <filesystem>
#include <cmath>
#include <memory>
#include <fstream>
#include <mutex>
#include <sstream>
#include <unordered_map>
#include <vector>

namespace MotionProfileDatabase
{
    namespace fs = std::filesystem;

    namespace
    {
        constexpr std::string_view kRootPath = "Data/MotionDB";
        constexpr std::string_view kIntentFileName = "Intent.ini";

        std::vector<Profile> g_profiles;
        std::unordered_map<std::string, std::size_t> g_profileByName;
        std::mutex g_mutex;
        bool g_loaded{ false };

        [[nodiscard]] std::string Trim(std::string value)
        {
            auto notSpace = [](unsigned char c) { return !std::isspace(c); };
            value.erase(value.begin(), std::find_if(value.begin(), value.end(), notSpace));
            value.erase(std::find_if(value.rbegin(), value.rend(), notSpace).base(), value.end());
            return value;
        }

        [[nodiscard]] std::string Lower(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            return value;
        }

        [[nodiscard]] std::vector<std::string> SplitCsv(const std::string& raw, bool lower = true)
        {
            std::vector<std::string> out;
            std::stringstream stream(raw);
            std::string part;
            while (std::getline(stream, part, ',')) {
                part = Trim(std::move(part));
                if (part.empty()) {
                    continue;
                }
                if (lower) {
                    part = Lower(std::move(part));
                }
                out.push_back(std::move(part));
            }
            return out;
        }

        [[nodiscard]] std::string NormalizeMotionDbKey(std::string value)
        {
            std::replace(value.begin(), value.end(), '\\', '/');
            const auto slash = value.find_last_of('/');
            if (slash != std::string::npos) {
                value.erase(0, slash + 1);
            }
            value = Lower(Trim(std::move(value)));

            constexpr std::string_view annotationSuffix = ".annotations.txt";
            constexpr std::string_view hkxSuffix = ".hkx";
            if (value.size() >= annotationSuffix.size() &&
                value.compare(value.size() - annotationSuffix.size(), annotationSuffix.size(), annotationSuffix) == 0) {
                value.resize(value.size() - annotationSuffix.size());
            } else if (value.size() >= hkxSuffix.size() &&
                       value.compare(value.size() - hkxSuffix.size(), hkxSuffix.size(), hkxSuffix) == 0) {
                value.resize(value.size() - hkxSuffix.size());
            }

            value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
                return c == '_' || c == ' ' || c == '-';
            }), value.end());
            return value;
        }

        [[nodiscard]] std::vector<std::string> SplitMotionDbKeys(const std::string& raw)
        {
            auto values = SplitCsv(raw, false);
            for (auto& value : values) {
                value = NormalizeMotionDbKey(std::move(value));
            }
            values.erase(std::remove_if(values.begin(), values.end(), [](const std::string& value) {
                return value.empty();
            }), values.end());
            std::sort(values.begin(), values.end());
            values.erase(std::unique(values.begin(), values.end()), values.end());
            return values;
        }

        [[nodiscard]] std::string JoinCsv(const std::vector<std::string>& values)
        {
            std::string out;
            for (const auto& value : values) {
                if (!out.empty()) {
                    out += ',';
                }
                out += value;
            }
            return out;
        }

        [[nodiscard]] bool ParseBool(std::string value, bool fallback)
        {
            value = Lower(Trim(std::move(value)));
            if (value == "1" || value == "true" || value == "yes" || value == "on") return true;
            if (value == "0" || value == "false" || value == "no" || value == "off") return false;
            return fallback;
        }

        [[nodiscard]] float ParseFloat(const std::string& raw, float fallback)
        {
            try {
                const float value = std::stof(Trim(raw));
                return std::isfinite(value) ? value : fallback;
            } catch (...) {
                return fallback;
            }
        }

        [[nodiscard]] std::int32_t ParseInt(const std::string& raw, std::int32_t fallback)
        {
            try {
                return static_cast<std::int32_t>(std::stol(Trim(raw)));
            } catch (...) {
                return fallback;
            }
        }

        [[nodiscard]] std::uint8_t ParseU8(const std::string& raw, std::uint8_t fallback)
        {
            const auto value = ParseInt(raw, fallback);
            return static_cast<std::uint8_t>(std::clamp<std::int32_t>(value, 0, 255));
        }

        using IniMap = std::unordered_map<std::string, std::unordered_map<std::string, std::string>>;

        [[nodiscard]] bool ReadIni(const fs::path& path, IniMap& out)
        {
            std::ifstream input(path);
            if (!input) return false;
            std::string section;
            std::string line;
            while (std::getline(input, line)) {
                line = Trim(std::move(line));
                if (line.empty() || line[0] == ';' || line[0] == '#') continue;
                if (line.front() == '[' && line.back() == ']') {
                    section = Lower(Trim(line.substr(1, line.size() - 2)));
                    continue;
                }
                const auto eq = line.find('=');
                if (eq == std::string::npos) continue;
                auto key = Lower(Trim(line.substr(0, eq)));
                auto value = Trim(line.substr(eq + 1));
                out[section][key] = std::move(value);
            }
            return true;
        }

        [[nodiscard]] std::string Get(const IniMap& ini, std::string_view section, std::string_view key, std::string fallback = {})
        {
            const auto s = ini.find(std::string{ section });
            if (s == ini.end()) return fallback;
            const auto k = s->second.find(std::string{ key });
            return k == s->second.end() ? fallback : k->second;
        }

        [[nodiscard]] bool Has(const IniMap& ini, std::string_view section, std::string_view key)
        {
            const auto s = ini.find(std::string{ section });
            return s != ini.end() && s->second.contains(std::string{ key });
        }

        [[nodiscard]] bool LoadProfile(const fs::path& folder, Profile& out)
        {
            IniMap ini;
            const auto configPath = folder / kIntentFileName;
            if (!ReadIni(configPath, ini)) return false;

            out = {};
            out.folder = folder.filename().string();
            out.name = Get(ini, "actor", "name", out.folder);
            out.raceEditorIDExact = SplitCsv(Get(ini, "actor", "raceeditoridexact", ""));
            out.raceEditorIDContains = SplitCsv(Get(ini, "actor", "raceeditoridcontains", ""));
            if (out.raceEditorIDExact.empty() && out.raceEditorIDContains.empty()) {
                SKSE::log::warn("[MotionProfile][REJECT] folder=\"{}\" reason=NO_RACE_MATCH_RULE", out.folder);
                return false;
            }

            auto& speed = out.speed;
            speed.enabled = ParseBool(Get(ini, "speedintent", "enabled", "false"), false);
            speed.graphSpeed = Get(ini, "speedintent", "graphspeedvariable", "");
            speed.graphMoving = Get(ini, "speedintent", "graphmovingvariable", "");
            speed.walkGoal = ParseFloat(Get(ini, "speedintent", "walkgoal", ""), 0.0f);
            speed.trotGoal = ParseFloat(Get(ini, "speedintent", "trotgoal", ""), 0.0f);
            speed.runGoal = ParseFloat(Get(ini, "speedintent", "rungoal", ""), 0.0f);
            speed.stateChangeConfirmSeconds = ParseFloat(Get(ini, "speedintent", "statechangeconfirmseconds", ""), 0.0f);
            
            speed.fleeFamily = Get(ini, "speedintent", "fleefamily", "");
            speed.combatFamily = Get(ini, "speedintent", "combatfamily", "");
            speed.actorRunFamily = Get(ini, "speedintent", "actorrunfamily", "");
            speed.packageWalkFamily = Get(ini, "speedintent", "packagewalkfamily", "");
            speed.packageJogFamily = Get(ini, "speedintent", "packagejogfamily", "");
            speed.packageFastWalkFamily = Get(ini, "speedintent", "packagefastwalkfamily", "");
            speed.packageRunFamily = Get(ini, "speedintent", "packagerunfamily", "");
            speed.alertFamily = Get(ini, "speedintent", "alertfamily", "");
            speed.actorWalkFamily = Get(ini, "speedintent", "actorwalkfamily", "");
            speed.defaultPackageFamily = Get(ini, "speedintent", "defaultpackagefamily", "");

            if (speed.enabled) {
                const bool allRequiredKeys =
                    Has(ini, "speedintent", "graphspeedvariable") &&
                    Has(ini, "speedintent", "graphmovingvariable") &&
                    Has(ini, "speedintent", "walkgoal") && Has(ini, "speedintent", "trotgoal") &&
                    Has(ini, "speedintent", "rungoal") && Has(ini, "speedintent", "statechangeconfirmseconds") &&
                    Has(ini, "speedintent", "fleefamily") && Has(ini, "speedintent", "combatfamily") &&
                    Has(ini, "speedintent", "actorrunfamily") && Has(ini, "speedintent", "packagewalkfamily") &&
                    Has(ini, "speedintent", "packagejogfamily") && Has(ini, "speedintent", "packagefastwalkfamily") &&
                    Has(ini, "speedintent", "packagerunfamily") && Has(ini, "speedintent", "alertfamily") &&
                    Has(ini, "speedintent", "actorwalkfamily") && Has(ini, "speedintent", "defaultpackagefamily");
                const bool valid = allRequiredKeys &&
                    !speed.graphSpeed.empty() && !speed.graphMoving.empty() &&
                    std::isfinite(speed.walkGoal) && std::isfinite(speed.trotGoal) && std::isfinite(speed.runGoal) &&
                    speed.stateChangeConfirmSeconds > 0.0f &&
                    !speed.fleeFamily.empty() && !speed.combatFamily.empty() && !speed.actorRunFamily.empty() &&
                    !speed.packageWalkFamily.empty() && !speed.packageJogFamily.empty() &&
                    !speed.packageFastWalkFamily.empty() && !speed.packageRunFamily.empty() &&
                    !speed.alertFamily.empty() && !speed.actorWalkFamily.empty() && !speed.defaultPackageFamily.empty();
                if (!valid) {
                    speed.enabled = false;
                    SKSE::log::warn(
                        "[MotionProfile][SPEED_DISABLED] folder=\"{}\" reason=INCOMPLETE_INTENT_DATA",
                        out.folder);
                }
            }

            auto& rotation = out.rotation;
            rotation.enabled = ParseBool(Get(ini, "rotationintent", "enabled", "false"), false);
            rotation.turnDeltaVariable = Get(ini, "rotationintent", "turndeltavariable", "");
            rotation.actionBypassVariables = SplitCsv(Get(ini, "rotationintent", "actionbypassvariables", ""), false);
            rotation.candidateConfirmFrames = ParseU8(Get(ini, "rotationintent", "candidateconfirmframes", ""), 0);
            
            rotation.allowedTurnAngles.clear();
            for (const auto& str : SplitCsv(Get(ini, "rotationintent", "allowedturnangles", ""))) {
                rotation.allowedTurnAngles.push_back(ParseFloat(str, 0.0f));
            }

            if (rotation.enabled) {
                const bool allRequiredKeys =
                    Has(ini, "rotationintent", "turndeltavariable") &&
                    Has(ini, "rotationintent", "candidateconfirmframes") &&
                    Has(ini, "rotationintent", "allowedturnangles");
                    
                const bool valid = allRequiredKeys &&
                    !rotation.turnDeltaVariable.empty() &&
                    rotation.candidateConfirmFrames > 0 &&
                    !rotation.allowedTurnAngles.empty();
                    
                if (!valid) {
                    rotation.enabled = false;
                    SKSE::log::warn(
                        "[MotionProfile][ROTATION_DISABLED] folder=\"{}\" reason=INCOMPLETE_INTENT_DATA_OR_MISSING_ANGLES",
                        out.folder);
                }
            }

            auto& animationPriority = out.animationPriority;
            animationPriority.enabled = ParseBool(Get(ini, "animationpriority", "enabled", "false"), false);
            animationPriority.graphAnimationDrivenVariable =
                Get(ini, "animationpriority", "graphanimationdrivenvariable", "bAnimationDriven");
            animationPriority.graphLocomotionActiveVariable =
                Get(ini, "animationpriority", "graphlocomotionactivevariable", "isMoving");
            animationPriority.graphSpeedVariable =
                Get(ini, "animationpriority", "graphspeedvariable", speed.graphSpeed);
            animationPriority.moveStopEvent =
                Get(ini, "animationpriority", "movestopevent", "moveStop");
            animationPriority.cannedTurnStopEvent =
                Get(ini, "animationpriority", "cannedturnstopevent", "cannedTurnStop");
            animationPriority.positiveTurnDirection =
                Get(ini, "animationpriority", "positiveturndirection", "Left");

            animationPriority.turn90MinAngle =
                ParseFloat(Get(ini, "animationpriority", "turn90minangle", "75.0"), 75.0f);
            animationPriority.turn90MaxAngle =
                ParseFloat(Get(ini, "animationpriority", "turn90maxangle", "125.0"), 125.0f);
            animationPriority.turn180MinAngle =
                ParseFloat(Get(ini, "animationpriority", "turn180minangle", "165.0"), 165.0f);
            animationPriority.intermediatePolicy =
                Get(ini, "animationpriority", "intermediatepolicy", "Turn90ThenResidual");

            animationPriority.standingConfirmFrames =
                ParseU8(Get(ini, "animationpriority", "standingconfirmframes", "2"), 2);
            animationPriority.cleanupConfirmFrames =
                ParseU8(Get(ini, "animationpriority", "cleanupconfirmframes", "2"), 2);
            animationPriority.stopTimeoutSeconds =
                ParseFloat(Get(ini, "animationpriority", "stoptimeoutseconds", "0.50"), 0.50f);
            animationPriority.eventConfirmTimeoutSeconds =
                ParseFloat(Get(ini, "animationpriority", "eventconfirmtimeoutseconds", "0.40"), 0.40f);
            animationPriority.activeTimeoutSeconds =
                ParseFloat(Get(ini, "animationpriority", "activetimeoutseconds", "2.50"), 2.50f);
            animationPriority.retryCooldownSeconds =
                ParseFloat(Get(ini, "animationpriority", "retrycooldownseconds", "0.25"), 0.25f);

            animationPriority.turnLeft90Events =
                SplitCsv(Get(ini, "animationpriority", "turnleft90events", ""), false);
            animationPriority.turnRight90Events =
                SplitCsv(Get(ini, "animationpriority", "turnright90events", ""), false);
            animationPriority.turnLeft180Events =
                SplitCsv(Get(ini, "animationpriority", "turnleft180events", ""), false);
            animationPriority.turnRight180Events =
                SplitCsv(Get(ini, "animationpriority", "turnright180events", ""), false);

            // Root-yaw classification is independent from the optional canned-turn router.
            // RuntimeYawClips replace authored animrotation with the live Skyrim yaw request.
            // RotationClips explicitly preserve authored animrotation and win on overlap.
            out.rotationClips =
                SplitMotionDbKeys(Get(ini, "animationpriority", "rotationclips", ""));
            out.runtimeYawClips =
                SplitMotionDbKeys(Get(ini, "animationpriority", "runtimeyawclips", ""));

            const auto runtimeCountBefore = out.runtimeYawClips.size();
            out.runtimeYawClips.erase(
                std::remove_if(
                    out.runtimeYawClips.begin(),
                    out.runtimeYawClips.end(),
                    [&](const std::string& key) {
                        return std::binary_search(out.rotationClips.begin(), out.rotationClips.end(), key);
                    }),
                out.runtimeYawClips.end());
            if (out.runtimeYawClips.size() != runtimeCountBefore) {
                SKSE::log::warn(
                    "[MotionProfile][YAW_CLASS_OVERLAP] folder=\"{}\" removedFromRuntimeYaw={} authority=ROTATION_CLIPS_WINS",
                    out.folder, runtimeCountBefore - out.runtimeYawClips.size());
            }

            animationPriority.suppressionVariables =
                SplitCsv(Get(ini, "animationpriority", "suppressionvariables", ""), false);

            animationPriority.preemptTurnLoopOnCanned =
                ParseBool(Get(ini, "animationpriority", "preemptturnlooponcanned", "true"), true);
            animationPriority.turnLoopStopEvent =
                Get(ini, "animationpriority", "turnloopstopevent", "turnStop");

            if (animationPriority.enabled) {
                const bool hasAllRoutes =
                    !animationPriority.turnLeft90Events.empty() &&
                    !animationPriority.turnRight90Events.empty() &&
                    !animationPriority.turnLeft180Events.empty() &&
                    !animationPriority.turnRight180Events.empty();

                const auto intermediate = Lower(animationPriority.intermediatePolicy);
                const bool validIntermediate =
                    intermediate == "turn90thenresidual" || intermediate == "capsuleslerp";

                const auto positiveDirection = Lower(animationPriority.positiveTurnDirection);
                const bool validPositiveDirection =
                    positiveDirection == "left" || positiveDirection == "right";

                const bool valid =
                    rotation.enabled &&
                    validPositiveDirection &&
                    !animationPriority.graphAnimationDrivenVariable.empty() &&
                    !animationPriority.graphLocomotionActiveVariable.empty() &&
                    !animationPriority.graphSpeedVariable.empty() &&
                    !animationPriority.moveStopEvent.empty() &&
                    !animationPriority.cannedTurnStopEvent.empty() &&
                    (!animationPriority.preemptTurnLoopOnCanned || !animationPriority.turnLoopStopEvent.empty()) &&
                    std::isfinite(animationPriority.turn90MinAngle) &&
                    std::isfinite(animationPriority.turn90MaxAngle) &&
                    std::isfinite(animationPriority.turn180MinAngle) &&
                    animationPriority.turn90MinAngle > 0.0f &&
                    animationPriority.turn90MinAngle < animationPriority.turn90MaxAngle &&
                    animationPriority.turn90MaxAngle < animationPriority.turn180MinAngle &&
                    animationPriority.turn180MinAngle <= 180.0f &&
                    animationPriority.standingConfirmFrames > 0 &&
                    animationPriority.cleanupConfirmFrames > 0 &&
                    std::isfinite(animationPriority.stopTimeoutSeconds) &&
                    animationPriority.stopTimeoutSeconds > 0.0f &&
                    std::isfinite(animationPriority.eventConfirmTimeoutSeconds) &&
                    animationPriority.eventConfirmTimeoutSeconds > 0.0f &&
                    std::isfinite(animationPriority.activeTimeoutSeconds) &&
                    animationPriority.activeTimeoutSeconds > 0.0f &&
                    std::isfinite(animationPriority.retryCooldownSeconds) &&
                    animationPriority.retryCooldownSeconds >= 0.0f &&
                    validIntermediate &&
                    hasAllRoutes;

                if (!valid) {
                    animationPriority.enabled = false;
                    SKSE::log::warn(
                        "[MotionProfile][ANIMATION_PRIORITY_DISABLED] folder=\"{}\" reason=INVALID_OR_INCOMPLETE_GRAPH_ROUTE_DATA",
                        out.folder);
                }
            }

            auto& capsuleSlerp = out.capsuleSlerp;
            capsuleSlerp.enabled = ParseBool(Get(ini, "capsuleslerp", "enabled", "false"), false);
            capsuleSlerp.responsePerSecond =
                ParseFloat(Get(ini, "capsuleslerp", "responsepersecond", "4.0"), 4.0f);
            capsuleSlerp.maxYawRateDegreesPerSecond =
                ParseFloat(Get(ini, "capsuleslerp", "maxyawratedegreespersecond", "75.0"), 75.0f);
            capsuleSlerp.minErrorDegrees =
                ParseFloat(Get(ini, "capsuleslerp", "minerrordegrees", "0.10"), 0.10f);
            capsuleSlerp.telemetryEveryFrames = static_cast<std::uint32_t>(std::clamp<std::int32_t>(
                ParseInt(Get(ini, "capsuleslerp", "telemetryeveryframes", "60"), 60), 1, 3600));

            if (capsuleSlerp.enabled) {
                const bool valid =
                    std::isfinite(capsuleSlerp.responsePerSecond) &&
                    capsuleSlerp.responsePerSecond > 0.0f &&
                    capsuleSlerp.responsePerSecond <= 60.0f &&
                    std::isfinite(capsuleSlerp.maxYawRateDegreesPerSecond) &&
                    capsuleSlerp.maxYawRateDegreesPerSecond > 0.0f &&
                    capsuleSlerp.maxYawRateDegreesPerSecond <= 720.0f &&
                    std::isfinite(capsuleSlerp.minErrorDegrees) &&
                    capsuleSlerp.minErrorDegrees >= 0.0f &&
                    capsuleSlerp.minErrorDegrees <= 10.0f;
                if (!valid) {
                    capsuleSlerp.enabled = false;
                    SKSE::log::warn(
                        "[MotionProfile][CAPSULE_SLERP_DISABLED] folder=\"{}\" reason=INVALID_CONFIG",
                        out.folder);
                }
            }

            return true;
        }

        void LoadLocked()
        {
            if (g_loaded) return;
            g_loaded = true;
            g_profiles.clear();
            g_profileByName.clear();

            const fs::path root{ kRootPath };
            std::error_code ec;
            if (!fs::exists(root, ec) || !fs::is_directory(root, ec)) {
                SKSE::log::warn("[MotionProfile][MISSING_ROOT] path=\"{}\"", root.generic_string());
                return;
            }

            for (fs::directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
                if (!it->is_directory()) continue;
                Profile profile{};
                if (!LoadProfile(it->path(), profile)) continue;
                const auto index = g_profiles.size();
                const auto nameKey = Lower(profile.name);
                const auto folderKey = Lower(profile.folder);
                g_profiles.push_back(std::move(profile));
                g_profileByName[nameKey] = index;
                g_profileByName[folderKey] = index;
                const auto& loaded = g_profiles.back();
                SKSE::log::info(
                    "[MotionProfile][LOAD] profile=\"{}\" folder=\"{}\" speed={} rotation={} animationPriority={} exactRules={} containsRules={}",
                    loaded.name, loaded.folder, loaded.speed.enabled, loaded.rotation.enabled, loaded.animationPriority.enabled,
                    loaded.raceEditorIDExact.size(), loaded.raceEditorIDContains.size());
                if (loaded.animationPriority.enabled) {
                    SKSE::log::info(
                        "[MotionProfile][ANIMATION_PRIORITY] profile=\"{}\" rawBands=90=({:.1f},{:.1f}) intermediate=[{:.1f},{:.1f}) policy={} 180>={:.1f} positiveDirection={} moveStop=\"{}\" cannedStop=\"{}\" movingWitness=\"{}\" animationWitness=\"{}\" standingConfirm={} cleanupConfirm={}",
                        loaded.name,
                        loaded.animationPriority.turn90MinAngle,
                        loaded.animationPriority.turn90MaxAngle,
                        loaded.animationPriority.turn90MaxAngle,
                        loaded.animationPriority.turn180MinAngle,
                        loaded.animationPriority.intermediatePolicy,
                        loaded.animationPriority.turn180MinAngle,
                        loaded.animationPriority.positiveTurnDirection,
                        loaded.animationPriority.moveStopEvent,
                        loaded.animationPriority.cannedTurnStopEvent,
                        loaded.animationPriority.graphLocomotionActiveVariable,
                        loaded.animationPriority.graphAnimationDrivenVariable,
                        loaded.animationPriority.standingConfirmFrames,
                        loaded.animationPriority.cleanupConfirmFrames);
                    SKSE::log::info(
                        "[MotionProfile][ANIMATION_EVENTS] profile=\"{}\" left90=\"{}\" right90=\"{}\" left180=\"{}\" right180=\"{}\" preemptTurnLoop={} turnLoopStop=\"{}\"",
                        loaded.name,
                        JoinCsv(loaded.animationPriority.turnLeft90Events),
                        JoinCsv(loaded.animationPriority.turnRight90Events),
                        JoinCsv(loaded.animationPriority.turnLeft180Events),
                        JoinCsv(loaded.animationPriority.turnRight180Events),
                        loaded.animationPriority.preemptTurnLoopOnCanned,
                        loaded.animationPriority.turnLoopStopEvent);
                }
                SKSE::log::info(
                    "[MotionProfile][ROOT_YAW_CLASSES] profile=\"{}\" authored={} runtime={} authoredKeys=\"{}\" runtimeKeys=\"{}\"",
                    loaded.name,
                    loaded.rotationClips.size(),
                    loaded.runtimeYawClips.size(),
                    JoinCsv(loaded.rotationClips),
                    JoinCsv(loaded.runtimeYawClips));
                if (loaded.capsuleSlerp.enabled) {
                    SKSE::log::info(
                        "[MotionProfile][CAPSULE_SLERP] profile=\"{}\" response={:.2f}/s maxYawRate={:.1f}deg/s minError={:.2f}deg telemetryEveryFrames={}",
                        loaded.name,
                        loaded.capsuleSlerp.responsePerSecond,
                        loaded.capsuleSlerp.maxYawRateDegreesPerSecond,
                        loaded.capsuleSlerp.minErrorDegrees,
                        loaded.capsuleSlerp.telemetryEveryFrames);
                }
            }
            if (ec) {
                SKSE::log::warn("[MotionProfile][SCAN_ERROR] path=\"{}\" code={} message=\"{}\"",
                    root.generic_string(), ec.value(), ec.message());
            }
            SKSE::log::info("[MotionProfile][READY] path=\"{}\" profiles={}", root.generic_string(), g_profiles.size());
        }

        [[nodiscard]] std::string RaceEditorID(RE::Actor* actor)
        {
            if (!actor || actor->IsPlayerRef()) return {};
            const auto* race = actor->GetRace();
            const char* id = race ? race->GetFormEditorID() : nullptr;
            return id && *id ? Lower(std::string{ id }) : std::string{};
        }
    }

    void EnsureLoaded()
    {
        std::scoped_lock lock{ g_mutex };
        LoadLocked();
    }

    const Profile* ResolveProfile(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) return nullptr;
        const auto race = RaceEditorID(actor);
        if (race.empty()) return nullptr;
        std::scoped_lock lock{ g_mutex };
        LoadLocked();

        for (const auto& profile : g_profiles) {
            for (const auto& exact : profile.raceEditorIDExact) {
                if (race == exact) return std::addressof(profile);
            }
        }

        const Profile* best = nullptr;
        std::size_t bestLength = 0;
        for (const auto& profile : g_profiles) {
            for (const auto& token : profile.raceEditorIDContains) {
                if (!token.empty() && race.find(token) != std::string::npos && token.size() > bestLength) {
                    best = std::addressof(profile);
                    bestLength = token.size();
                }
            }
        }
        return best;
    }

    const Profile* GetProfile(std::string_view nameOrFolder)
    {
        std::scoped_lock lock{ g_mutex };
        LoadLocked();
        const auto it = g_profileByName.find(Lower(std::string{ nameOrFolder }));
        return it == g_profileByName.end() ? nullptr : std::addressof(g_profiles[it->second]);
    }

    bool SupportsActor(RE::Actor* actor)
    {
        return actor && !actor->IsPlayerRef() && ResolveProfile(actor) != nullptr;
    }

    std::string DatabaseFolderForActor(RE::Actor* actor)
    {
        if (!actor) return {};
        if (actor->IsPlayerRef()) return "Player";
        const auto* profile = ResolveProfile(actor);
        return profile ? profile->folder : std::string{};
    }

    std::string RootPath()
    {
        return std::string{ kRootPath };
    }

    void Clear()
    {
        std::scoped_lock lock{ g_mutex };
        g_profiles.clear();
        g_profileByName.clear();
        g_loaded = false;
    }
}
