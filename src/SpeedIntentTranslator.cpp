// BUILD_ID: INCESSUS-B8-PROVEN-HANDOFF-GRAPH-OWNED-CLIPS-2026-10-02
#include "PCH.h"
#include "SpeedIntentTranslator.h"
#include "MotionProfileDatabase.h"
#include "SteeringIntentController.h"

#include "RE/A/AIProcess.h"
#include "RE/M/MiddleHighProcessData.h"
#include "RE/T/TESPackage.h"
#include "RE/B/BSAnimationGraphManager.h"
#include "RE/B/BShkbAnimationGraph.h"
#include "RE/H/hkbBehaviorGraph.h"
#include "RE/H/hkbBehaviorGraphData.h"
#include "RE/H/hkbBehaviorGraphStringData.h"

#include <algorithm>
#include <array>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <string>
#include <string_view>
#include <unordered_map>

namespace SpeedIntentTranslator
{
    using Clock = std::chrono::steady_clock;

    namespace
    {
        enum class IntentSource : std::uint8_t
        {
            kNone,
            kCombat,
            kFlee,
            kRuntimePackagePreferred,
            kAlert,
            kActorRunState,
            kActorWalkState,
            kDefaultPackageWalk
        };

        struct SemanticIntentSample
        {
            bool valid{ false };
            Family family{ Family::kNone };
            IntentSource source{ IntentSource::kNone };
            std::int32_t packagePreferredRaw{ -1 };
        };

        struct State
        {
            bool initialized{ false };
            Family latched{ Family::kNone };
            Family candidate{ Family::kNone };
            Clock::time_point candidateSince{};
            std::uint64_t revision{ 0 };
            std::uint64_t frames{ 0 };
            std::uint64_t changes{ 0 };
            std::uint64_t pendingResets{ 0 };
            std::uint64_t graphWrites{ 0 };
            std::uint64_t graphMotionGateSkips{ 0 };
            bool universalMovementDemand{ false };
            std::uint64_t universalDemandRevision{ 0 };
            std::uint32_t universalDemandLossFrames{ 0 };
            float universalGoalDistanceXY{ 0.0f };
            SteeringIntentController::Source universalDemandSource{ SteeringIntentController::Source::kNone };
            Family lastLoggedGraphFamily{ Family::kNone };
            IntentSource lastSource{ IntentSource::kNone };
            std::int32_t lastPreferredRaw{ -1 };
        };

        std::unordered_map<RE::FormID, State> g_state;
        std::mutex g_mutex;

        enum class AutoGraphTopology : std::uint8_t
        {
            kUnknown,
            kParentEventOnly,
            kContinuousSpeed,
            kDiscreteEvents,
            kHybrid
        };

        struct AutoGraphDescriptor
        {
            bool discovered{ false };
            bool structuralContract{ false };
            std::string signature;
            std::string projectKey;
            AutoGraphTopology topology{ AutoGraphTopology::kUnknown };
            std::string parentStartEvent;
            std::string stopEvent;
            std::string walkEvent;
            std::string trotEvent;
            std::string runEvent;
            std::string speedVariable;
            std::string sampledSpeedVariable;
            std::string directionVariable;
            std::string movingVariable;
            float structuralWalkSpeed{ 0.0f };
            float structuralTrotSpeed{ 0.0f };
            float structuralRunSpeed{ 0.0f };
            std::uint32_t directionSectors{ 0 };
            std::uint64_t drives{ 0 };
            std::uint64_t eventAccepted{ 0 };
            std::uint64_t eventRejected{ 0 };
            std::uint64_t speedWrites{ 0 };
        };

        struct AutoGraphActorState
        {
            std::string signature;
            Family lastFamily{ Family::kNone };
            std::uint64_t lastRevision{ 0 };
            bool lastDirectional{ false };
            bool parentEventAccepted{ false };
            Family familyEventAccepted{ Family::kNone };
            std::uint32_t parentRetryCooldownFrames{ 0 };
            std::uint32_t familyRetryCooldownFrames{ 0 };
            std::uint64_t frames{ 0 };
            std::uint64_t parentAttempts{ 0 };
            std::uint64_t familyAttempts{ 0 };
            std::uint64_t stopAttempts{ 0 };
            float lastObservedSpeed{ 0.0f };

            // B8: only continuous-selector graphs own a synthetic directional selector.
            // The selector is recomputed from current steering and may change while a
            // locomotion session is active; it is not a clip/state latch.
            bool directionInitialized{ false };
            std::int32_t directionSector{ 0 };
            std::int32_t pendingDirectionSector{ -1 };
            std::uint32_t pendingDirectionFrames{ 0 };
            float selectedDirectionValue{ 0.0f };
            bool selectorInitialized{ false };
            float selectedSelectorGoal{ 0.0f };

        };

        std::unordered_map<std::string, AutoGraphDescriptor> g_autoGraphDescriptors;
        std::unordered_map<RE::FormID, AutoGraphActorState> g_autoGraphActors;
        std::mutex g_autoGraphMutex;

        constexpr std::uint32_t kAutoGraphParentRetryFrames = 6;
        constexpr std::uint32_t kAutoGraphFamilyRetryFrames = 6;
        constexpr float kDirectionSwitchHysteresisDegrees = 10.0f;
        constexpr std::uint32_t kDirectionConfirmFrames = 4;
        constexpr float kRadToDeg = 57.29577951308232f;

        // Proven by the old functional Troll R93 selector. These are controller
        // stability rules, not creature tuning: exact graph anchors/sectors still come
        // from the behavior-derived structural contract.

        struct StructuralContractSeed
        {
            std::string_view key;
            float walk;
            float trot;
            float run;
            std::uint32_t directionSectors;
        };

        // Generated from the Nemesis LE creature behavior XML supplied by the user.
        // These are NOT physical capsule speeds and NOT per-race tuning. They are the
        // authored blend-parameter anchors already present in each behavior graph.
        // Runtime still discovers the actual event/variable names from the loaded graph;
        // a seed is used only when the loaded project exposes the expected locomotion
        // language (moveStart + Speed, plus walkStart/runStart when present).
        constexpr std::array<StructuralContractSeed, 32> kStructuralContracts{{
            { "bc", 192.000000f, 192.000000f, 384.000000f, 8 },
            { "bear", 59.820000f, 219.417999f, 542.359985f, 0 },
            { "benthiclurker", 122.150002f, 122.150002f, 305.459991f, 8 },
            { "boar", 59.820000f, 219.417999f, 542.359985f, 0 },
            { "chaurus", 142.509995f, 142.509995f, 350.266998f, 4 },
            { "chicken", 34.710999f, 34.710999f, 251.938004f, 0 },
            { "cow", 65.000000f, 211.000000f, 525.450012f, 0 },
            { "deer", 833.000000f, 833.000000f, 833.000000f, 0 },
            { "dog", 74.540001f, 287.320007f, 562.650024f, 0 },
            { "draugr", 76.970001f, 76.970001f, 339.220001f, 8 },
            { "falmer", 100.442001f, 100.442001f, 175.774002f, 8 },
            { "frostbitespider", 85.089996f, 85.089996f, 300.200989f, 4 },
            { "giant", 82.458000f, 82.458000f, 415.000000f, 8 },
            { "goat", 62.130001f, 62.130001f, 360.000000f, 0 },
            { "hare", 89.182800f, 89.182800f, 244.444000f, 0 },
            { "havgraven", 73.790001f, 73.790001f, 116.242996f, 8 },
            { "hmdaedra", 64.000000f, 64.000000f, 128.000000f, 0 },
            { "horker", 32.766998f, 32.766998f, 83.406998f, 0 },
            { "mammoth", 61.840000f, 61.840000f, 400.000000f, 0 },
            { "mudcrab", 72.739998f, 72.739998f, 145.869995f, 4 },
            { "riekling", 167.417007f, 167.417007f, 275.817993f, 4 },
            { "sabrecat", 113.088997f, 208.701996f, 563.560974f, 0 },
            { "sc", 192.000000f, 192.000000f, 384.000000f, 8 },
            { "scrib", 401.010986f, 401.010986f, 802.291016f, 0 },
            { "skeever", 36.142300f, 170.112000f, 486.233002f, 0 },
            { "spriggan", 65.318001f, 65.318001f, 358.869995f, 8 },
            { "steam", 96.000000f, 96.000000f, 192.000000f, 8 },
            { "troll", 102.699997f, 102.699997f, 269.489990f, 8 },
            { "vampirebrute", 118.924500f, 118.924500f, 403.722992f, 8 },
            { "vampirelord", 70.074997f, 70.074997f, 303.040009f, 8 },
            { "werewolf", 70.074997f, 70.074997f, 303.040009f, 8 },
            { "wolf", 74.540001f, 287.320007f, 562.650024f, 0 }
        }};

        [[nodiscard]] std::string Lower(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            return value;
        }

        [[nodiscard]] std::string NormalizeGraphSymbol(std::string_view value)
        {
            std::string out;
            out.reserve(value.size());
            for (const unsigned char c : value) {
                if (std::isalnum(c) != 0) {
                    out.push_back(static_cast<char>(std::tolower(c)));
                }
            }
            return out;
        }

        [[nodiscard]] bool HasToken(std::string_view normalized, std::string_view token) noexcept
        {
            return normalized.find(token) != std::string_view::npos;
        }

        [[nodiscard]] std::string NormalizeProjectKey(std::string_view project)
        {
            std::string key = NormalizeGraphSymbol(project);
            constexpr std::array<std::string_view, 3> suffixes{ "project", "behavior", "behaviour" };
            bool stripped = true;
            while (stripped) {
                stripped = false;
                for (const auto suffix : suffixes) {
                    if (key.size() >= suffix.size() &&
                        key.compare(key.size() - suffix.size(), suffix.size(), suffix) == 0) {
                        key.resize(key.size() - suffix.size());
                        stripped = true;
                        break;
                    }
                }
            }
            if (key == "hcow") key = "cow";
            if (key == "sabre") key = "sabrecat";
            if (key == "boars") key = "boar";
            return key;
        }

        [[nodiscard]] const StructuralContractSeed* FindStructuralContract(std::string_view projectKey) noexcept
        {
            for (const auto& seed : kStructuralContracts) {
                if (seed.key == projectKey) return &seed;
            }
            return nullptr;
        }

        [[nodiscard]] const char* AutoGraphTopologyName(AutoGraphTopology topology) noexcept
        {
            switch (topology) {
            case AutoGraphTopology::kParentEventOnly: return "PARENT_EVENT_ONLY";
            case AutoGraphTopology::kContinuousSpeed: return "CONTINUOUS_SPEED";
            case AutoGraphTopology::kDiscreteEvents: return "DISCRETE_EVENTS";
            case AutoGraphTopology::kHybrid: return "HYBRID_EVENT_PLUS_SPEED";
            default: return "UNKNOWN";
            }
        }

        [[nodiscard]] int ParentStartScore(std::string_view raw) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            if (key == "movestart") return 1000;
            if (key == "locomotionstart") return 950;
            if (HasToken(key, "move") && HasToken(key, "start") && !HasToken(key, "stop")) return 800;
            if (HasToken(key, "locomotion") && HasToken(key, "start")) return 750;
            return -1;
        }

        [[nodiscard]] int StopScore(std::string_view raw) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            if (key == "movestop") return 1000;
            if (key == "locomotionstop") return 950;
            if (HasToken(key, "move") && HasToken(key, "stop")) return 800;
            if (HasToken(key, "locomotion") && HasToken(key, "stop")) return 750;
            return -1;
        }

        [[nodiscard]] int FamilyEventScore(std::string_view raw, std::string_view family) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            const std::string exact = std::string(family) + "start";
            if (key == exact) return 1000;
            // Never promote directional/strafe helper events to the semantic forward
            // gait contract. B5 selected Horse StrafeStartWalkLeft merely because it
            // contained "walk" + "start". Unknown graphs now fail open instead.
            if (HasToken(key, "strafe") || HasToken(key, "left") || HasToken(key, "right") ||
                HasToken(key, "back")) return -1;
            if (HasToken(key, family) && HasToken(key, "start") &&
                !HasToken(key, "stop") && !HasToken(key, "turn")) return 800;
            return -1;
        }

        [[nodiscard]] int SpeedVariableScore(std::string_view raw) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            if (key == "speed") return 1000;
            if (key == "movespeed") return 900;
            if (HasToken(key, "speed") && !HasToken(key, "sample") &&
                !HasToken(key, "turn") && !HasToken(key, "attack") && !HasToken(key, "rate")) return 600;
            return -1;
        }

        [[nodiscard]] int SampledSpeedVariableScore(std::string_view raw) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            if (key == "sampledspeed" || key == "speedsampled") return 1100;
            if (HasToken(key, "speed") && HasToken(key, "sample")) return 800;
            return -1;
        }

        [[nodiscard]] int DirectionVariableScore(std::string_view raw) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            // Exact only. Creature graphs contain many unrelated direction variables
            // (staggerDirection, DirectionOffset, DirectionBlendA/B, etc.).
            return key == "direction" ? 1100 : -1;
        }

        [[nodiscard]] int MovingVariableScore(std::string_view raw) noexcept
        {
            const auto key = NormalizeGraphSymbol(raw);
            if (key == "ismoving" || key == "bismoving") return 1000;
            if (key == "moving") return 900;
            if (HasToken(key, "moving") && !HasToken(key, "attack")) return 500;
            return -1;
        }

        template <class ScoreFn>
        void SelectBestSymbol(const RE::hkArray<RE::hkStringPtr>& names, ScoreFn&& scoreFn, std::string& out)
        {
            int bestScore = -1;
            for (const auto& hkName : names) {
                const char* cstr = hkName.c_str();
                if (!cstr || !*cstr) continue;
                const int score = scoreFn(cstr);
                if (score > bestScore) {
                    bestScore = score;
                    out = cstr;
                }
            }
        }

        [[nodiscard]] bool HasActorDirectionalIntent(RE::Actor* actor) noexcept
        {
            if (!actor) return false;
            if (auto* actorState = actor->AsActorState()) {
                return actorState->actorState1.movingForward != 0 ||
                    actorState->actorState1.movingBack != 0 ||
                    actorState->actorState1.movingLeft != 0 ||
                    actorState->actorState1.movingRight != 0;
            }
            return false;
        }

        [[nodiscard]] bool HasUniversalActionBypass(RE::Actor* actor) noexcept
        {
            if (!actor) return true;
            for (const char* name : { "IsAttacking", "IsStaggering", "IsRecoiling" }) {
                bool value = false;
                if (actor->GetGraphVariableBool(name, value) && value) {
                    return true;
                }
            }
            return false;
        }

        struct UniversalDemandSample
        {
            bool demand{ false };
            bool commandValid{ false };
            bool actorDirectional{ false };
            SteeringIntentController::Source source{ SteeringIntentController::Source::kNone };
            float goalDistanceXY{ 0.0f };
        };

        [[nodiscard]] UniversalDemandSample SampleUniversalMovementDemand(RE::Actor* actor) noexcept
        {
            UniversalDemandSample out{};
            if (!actor) return out;

            SteeringIntentController::FrameCommand command{};
            out.commandValid = SteeringIntentController::GetFrameCommand(actor, command) &&
                command.valid && std::isfinite(command.goalDistanceXY);
            out.actorDirectional = HasActorDirectionalIntent(actor);
            if (!out.commandValid) return out;

            out.source = command.source;
            out.goalDistanceXY = command.goalDistanceXY;

            // B8 correctness-first movement proof. A spatial target/waypoint alone is not
            // locomotion authority: combat, facing and package logic may expose a target
            // while the behavior intentionally remains standing. Before handoff the vanilla
            // capsule is still fully authoritative, so wait until Skyrim itself raises a
            // directional movement request while steering/pathing is valid. Once established,
            // BeforeUniversalUpdate provides a short loss-grace so root-driven motion does not
            // collapse merely because those vanilla directional bits disappear after handoff.
            out.demand = out.actorDirectional;
            return out;
        }

        [[nodiscard]] Family UniversalPackagePreferredFamily(RE::AIProcess* process, std::int32_t& rawPreferred) noexcept
        {
            rawPreferred = -1;
            if (!process || !process->currentPackage.package) return Family::kNone;
            auto* package = process->currentPackage.package;
            constexpr std::uint32_t kPreferredSpeedFlag =
                static_cast<std::uint32_t>(RE::PACKAGE_DATA::GeneralFlag::kPreferredSpeed);
            const bool preferredActive =
                package->packData.packFlags.any(RE::PACKAGE_DATA::GeneralFlag::kPreferredSpeed) ||
                (process->currentPackage.modifiedPackageFlag & kPreferredSpeedFlag) != 0;
            if (!preferredActive) return Family::kNone;

            rawPreferred = static_cast<std::int32_t>(process->currentPackage.preferredSpeed);
            switch (rawPreferred) {
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kWalk):
                return Family::kWalkForward;
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kJog):
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kFastWalk):
                return Family::kTrotForward;
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kRun):
                return Family::kRunForward;
            default:
                return Family::kNone;
            }
        }

        [[nodiscard]] SemanticIntentSample SampleUniversalSemanticIntent(RE::Character* actor) noexcept
        {
            SemanticIntentSample out{};
            if (!actor) return out;

            bool actorStateValid = false;
            bool walking = false;
            bool running = false;
            bool sprinting = false;
            bool forceRun = false;
            if (auto* actorState = actor->AsActorState()) {
                actorStateValid = true;
                walking = actorState->actorState1.walking != 0;
                running = actorState->actorState1.running != 0;
                sprinting = actorState->actorState1.sprinting != 0;
                forceRun = actorState->actorState2.forceRun != 0;
            }

            auto* process = actor->GetActorRuntimeData().currentProcess;
            const bool fleeing = process && process->middleHigh && process->middleHigh->isFleeing;
            const bool combat = actor->IsInCombat();
            const bool alert = process &&
                process->lowProcessFlags.any(RE::AIProcess::LowProcessFlags::kAlert);

            // B5 restores the proven semantic precedence used by the profile solver.
            // B4 accidentally allowed ActorState WALK to mask COMBAT/FLEE, permanently
            // trapping profileless creatures such as Troll in the WALK family.
            if (fleeing) {
                return { true, Family::kRunForward, IntentSource::kFlee, -1 };
            }
            if (combat) {
                return { true, Family::kRunForward, IntentSource::kCombat, -1 };
            }
            if (actorStateValid && (sprinting || forceRun || running)) {
                return { true, Family::kRunForward, IntentSource::kActorRunState, -1 };
            }
            if (process) {
                std::int32_t preferredRaw = -1;
                const Family preferred = UniversalPackagePreferredFamily(process, preferredRaw);
                if (preferred != Family::kNone) {
                    return { true, preferred, IntentSource::kRuntimePackagePreferred, preferredRaw };
                }
            }
            if (alert) {
                return { true, Family::kTrotForward, IntentSource::kAlert, -1 };
            }
            if (actorStateValid && walking) {
                return { true, Family::kWalkForward, IntentSource::kActorWalkState, -1 };
            }
            if (process && process->currentPackage.package) {
                return { true, Family::kWalkForward, IntentSource::kDefaultPackageWalk, -1 };
            }
            return out;
        }

        [[nodiscard]] bool ResolveAutoGraphDescriptor(
            RE::Actor* actor,
            std::string& signature,
            AutoGraphDescriptor& descriptor)
        {
            signature.clear();
            descriptor = {};
            if (!actor) return false;

            RE::BSTSmartPointer<RE::BSAnimationGraphManager> manager;
            if (!actor->GetAnimationGraphManager(manager) || !manager) return false;
            const auto activeGraph = manager->GetRuntimeData().activeGraph;
            if (activeGraph >= manager->graphs.size()) return false;
            auto* graph = manager->graphs[activeGraph].get();
            if (!graph || !graph->behaviorGraph) return false;
            auto* data = graph->behaviorGraph->data.get();
            if (!data || !data->stringData) return false;
            auto* strings = data->stringData.get();
            if (!strings) return false;

            const char* project = graph->projectName.c_str();
            if (project && *project) {
                signature = project;
                descriptor.projectKey = NormalizeProjectKey(project);
            } else {
                signature = "behavior@" + std::to_string(reinterpret_cast<std::uintptr_t>(graph->behaviorGraph));
            }
            signature += "#e" + std::to_string(strings->eventNames.size());
            signature += "#v" + std::to_string(strings->variableNames.size());

            {
                std::scoped_lock lock{ g_autoGraphMutex };
                const auto it = g_autoGraphDescriptors.find(signature);
                if (it != g_autoGraphDescriptors.end()) {
                    descriptor = it->second;
                    return true;
                }
            }

            descriptor.discovered = true;
            descriptor.signature = signature;
            SelectBestSymbol(strings->eventNames, ParentStartScore, descriptor.parentStartEvent);
            SelectBestSymbol(strings->eventNames, StopScore, descriptor.stopEvent);
            SelectBestSymbol(strings->eventNames,
                [](std::string_view n) { return FamilyEventScore(n, "walk"); }, descriptor.walkEvent);
            SelectBestSymbol(strings->eventNames,
                [](std::string_view n) { return FamilyEventScore(n, "trot"); }, descriptor.trotEvent);
            SelectBestSymbol(strings->eventNames,
                [](std::string_view n) { return FamilyEventScore(n, "run"); }, descriptor.runEvent);

            int bestSpeedScore = -1;
            int bestSampledSpeedScore = -1;
            int bestDirectionScore = -1;
            int bestMovingScore = -1;
            for (const auto& hkName : strings->variableNames) {
                const char* cstr = hkName.c_str();
                if (!cstr || !*cstr) continue;
                const int speedScore = SpeedVariableScore(cstr);
                if (speedScore > bestSpeedScore) {
                    bestSpeedScore = speedScore;
                    descriptor.speedVariable = cstr;
                }
                const int sampledScore = SampledSpeedVariableScore(cstr);
                if (sampledScore > bestSampledSpeedScore) {
                    bestSampledSpeedScore = sampledScore;
                    descriptor.sampledSpeedVariable = cstr;
                }
                const int directionScore = DirectionVariableScore(cstr);
                if (directionScore > bestDirectionScore) {
                    bestDirectionScore = directionScore;
                    descriptor.directionVariable = cstr;
                }
                const int movingScore = MovingVariableScore(cstr);
                if (movingScore > bestMovingScore) {
                    bestMovingScore = movingScore;
                    descriptor.movingVariable = cstr;
                }
            }

            if (const auto* seed = FindStructuralContract(descriptor.projectKey);
                seed && !descriptor.speedVariable.empty() && !descriptor.parentStartEvent.empty()) {
                descriptor.structuralContract = true;
                descriptor.structuralWalkSpeed = seed->walk;
                descriptor.structuralTrotSpeed = seed->trot > 0.001f ? seed->trot : seed->walk;
                descriptor.structuralRunSpeed = seed->run > 0.001f ? seed->run : descriptor.structuralTrotSpeed;
                descriptor.directionSectors = !descriptor.directionVariable.empty()
                    ? seed->directionSectors : 0;
            }

            const bool hasFamilyEvents = !descriptor.walkEvent.empty() ||
                !descriptor.trotEvent.empty() || !descriptor.runEvent.empty();
            const bool hasSpeed = !descriptor.speedVariable.empty();
            if (hasFamilyEvents && hasSpeed) descriptor.topology = AutoGraphTopology::kHybrid;
            else if (hasFamilyEvents) descriptor.topology = AutoGraphTopology::kDiscreteEvents;
            else if (hasSpeed) descriptor.topology = AutoGraphTopology::kContinuousSpeed;
            else if (!descriptor.parentStartEvent.empty()) descriptor.topology = AutoGraphTopology::kParentEventOnly;
            else descriptor.topology = AutoGraphTopology::kUnknown;

            {
                std::scoped_lock lock{ g_autoGraphMutex };
                auto [it, inserted] = g_autoGraphDescriptors.emplace(signature, descriptor);
                descriptor = it->second;
                if (inserted) {
                    SKSE::log::info(
                        "[AutoGraph][DISCOVER] form={:08X} signature=\"{}\" projectKey=\"{}\" topology={} events[parent=\"{}\" walk=\"{}\" trot=\"{}\" run=\"{}\" stop=\"{}\"] vars[speed=\"{}\" sampled=\"{}\" direction=\"{}\" moving=\"{}\"] structural[resolved={} walk={:.3f} trot={:.3f} run={:.3f} sectors={}] source=ACTIVE_HKB_STRING_TABLE+NEMESIS_LE_XML_STRUCTURAL_WEIGHTS profileRequirement=NONE",
                        actor->GetFormID(), signature, descriptor.projectKey, AutoGraphTopologyName(descriptor.topology),
                        descriptor.parentStartEvent, descriptor.walkEvent, descriptor.trotEvent,
                        descriptor.runEvent, descriptor.stopEvent,
                        descriptor.speedVariable, descriptor.sampledSpeedVariable, descriptor.directionVariable,
                        descriptor.movingVariable, descriptor.structuralContract,
                        descriptor.structuralWalkSpeed, descriptor.structuralTrotSpeed,
                        descriptor.structuralRunSpeed, descriptor.directionSectors);
                }
            }
            return true;
        }

        [[nodiscard]] std::string EventForAutoFamily(const AutoGraphDescriptor& descriptor, Family family)
        {
            switch (family) {
            case Family::kWalkForward: return descriptor.walkEvent;
            case Family::kTrotForward:
                // Quadruped-style graphs keep TROT inside the WALK state and use the
                // sampled-speed anchor to choose the authored trot clips.
                return !descriptor.trotEvent.empty() ? descriptor.trotEvent : descriptor.walkEvent;
            case Family::kRunForward: return descriptor.runEvent;
            default: return {};
            }
        }

        [[nodiscard]] float StructuralSpeedForFamily(const AutoGraphDescriptor& descriptor, Family family) noexcept
        {
            if (!descriptor.structuralContract) return 0.0f;
            switch (family) {
            case Family::kWalkForward: return descriptor.structuralWalkSpeed;
            case Family::kTrotForward: return descriptor.structuralTrotSpeed;
            case Family::kRunForward: return descriptor.structuralRunSpeed;
            default: return 0.0f;
            }
        }

        [[nodiscard]] float WrapSignedDegrees(float degrees) noexcept
        {
            if (!std::isfinite(degrees)) return 0.0f;
            degrees = std::fmod(degrees + 180.0f, 360.0f);
            if (degrees < 0.0f) degrees += 360.0f;
            return degrees - 180.0f;
        }

        [[nodiscard]] std::int32_t DirectionSectorForError(
            float errorDegrees,
            std::uint32_t directionSectors) noexcept
        {
            const auto sectors = static_cast<std::int32_t>(
                std::max<std::uint32_t>(1, directionSectors));
            // Supplied vanilla creature graphs serialize cyclic Direction as
            // F -> Right -> Back -> Left. Steering's positive error uses the opposite sign.
            float raw = -(WrapSignedDegrees(errorDegrees) / 360.0f);
            while (raw < 0.0f) raw += 1.0f;
            while (raw >= 1.0f) raw -= 1.0f;
            auto sector = static_cast<std::int32_t>(
                std::lround(raw * static_cast<float>(sectors))) % sectors;
            if (sector < 0) sector += sectors;
            return sector;
        }

        [[nodiscard]] float DirectionValueForSector(
            std::int32_t sector,
            std::uint32_t directionSectors) noexcept
        {
            const auto sectors = static_cast<std::int32_t>(
                std::max<std::uint32_t>(1, directionSectors));
            sector %= sectors;
            if (sector < 0) sector += sectors;
            return static_cast<float>(sector) / static_cast<float>(sectors);
        }

        [[nodiscard]] float SteeringCenterDegreesForSector(
            std::int32_t sector,
            std::uint32_t directionSectors) noexcept
        {
            const auto sectors = static_cast<std::int32_t>(
                std::max<std::uint32_t>(1, directionSectors));
            sector %= sectors;
            if (sector < 0) sector += sectors;
            return WrapSignedDegrees(
                -(360.0f / static_cast<float>(sectors)) * static_cast<float>(sector));
        }

        [[nodiscard]] Family FamilyFromConfig(std::string_view value) noexcept
        {
            std::string key{ value };
            key = Lower(std::move(key));
            key.erase(std::remove_if(key.begin(), key.end(), [](unsigned char c) {
                return std::isspace(c) != 0 || c == '_' || c == '-';
            }), key.end());

            if (key == "walk" || key == "walkforward" || key == "walkforwardfamily") {
                return Family::kWalkForward;
            }
            if (key == "trot" || key == "trotforward" || key == "trotforwardfamily") {
                return Family::kTrotForward;
            }
            if (key == "run" || key == "runforward" || key == "runforwardfamily") {
                return Family::kRunForward;
            }
            return Family::kNone;
        }

        [[nodiscard]] const char* FamilyName(Family family) noexcept
        {
            switch (family) {
            case Family::kWalkForward: return "WALK";
            case Family::kTrotForward: return "TROT";
            case Family::kRunForward: return "RUN";
            default: return "NONE";
            }
        }

        [[nodiscard]] const char* IntentSourceName(IntentSource source) noexcept
        {
            switch (source) {
            case IntentSource::kCombat: return "COMBAT_HIGH_URGENCY";
            case IntentSource::kFlee: return "FLEE_HIGH_URGENCY";
            case IntentSource::kRuntimePackagePreferred: return "RUNTIME_PACKAGE_PREFERRED_SPEED";
            case IntentSource::kAlert: return "AI_ALERT_MEDIUM_URGENCY";
            case IntentSource::kActorRunState: return "ACTOR_STATE_RUN";
            case IntentSource::kActorWalkState: return "ACTOR_STATE_WALK";
            case IntentSource::kDefaultPackageWalk: return "PACKAGE_DEFAULT_LOW_URGENCY";
            default: return "AMBIGUOUS_HOLD";
            }
        }

        [[nodiscard]] float SecondsSince(Clock::time_point from, Clock::time_point now) noexcept
        {
            if (from == Clock::time_point{}) {
                return 999.0f;
            }
            return std::chrono::duration<float>(now - from).count();
        }

        [[nodiscard]] std::uint8_t SolverToken(Family family) noexcept
        {
            switch (family) {
            case Family::kWalkForward: return 1;
            case Family::kTrotForward: return 2;
            case Family::kRunForward: return 3;
            default: return 0;
            }
        }

        [[nodiscard]] float GraphGoal(
            Family family,
            const MotionProfileDatabase::SpeedIntentConfig& config) noexcept
        {
            switch (family) {
            case Family::kWalkForward: return config.walkGoal;
            case Family::kTrotForward: return config.trotGoal;
            case Family::kRunForward: return config.runGoal;
            default: return 0.0f;
            }
        }

        [[nodiscard]] Family RuntimePackagePreferredFamily(
            RE::AIProcess* process,
            const MotionProfileDatabase::SpeedIntentConfig& config,
            std::int32_t& rawPreferred) noexcept
        {
            rawPreferred = -1;
            if (!process || !process->currentPackage.package) {
                return Family::kNone;
            }

            auto* package = process->currentPackage.package;
            constexpr std::uint32_t kPreferredSpeedFlag =
                static_cast<std::uint32_t>(RE::PACKAGE_DATA::GeneralFlag::kPreferredSpeed);
            const bool preferredActive =
                package->packData.packFlags.any(RE::PACKAGE_DATA::GeneralFlag::kPreferredSpeed) ||
                (process->currentPackage.modifiedPackageFlag & kPreferredSpeedFlag) != 0;
            if (!preferredActive) {
                return Family::kNone;
            }

            rawPreferred = static_cast<std::int32_t>(process->currentPackage.preferredSpeed);
            switch (rawPreferred) {
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kWalk):
                return FamilyFromConfig(config.packageWalkFamily);
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kJog):
                return FamilyFromConfig(config.packageJogFamily);
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kFastWalk):
                return FamilyFromConfig(config.packageFastWalkFamily);
            case static_cast<std::int32_t>(RE::PACKAGE_DATA::PreferredSpeed::kRun):
                return FamilyFromConfig(config.packageRunFamily);
            default:
                return Family::kNone;
            }
        }

        // Exact semantic precedence proven in R1-69/R1-71:
        // flee > combat > explicit run state > package PreferredSpeed > alert >
        // walking > ordinary package > ambiguous hold.
        [[nodiscard]] SemanticIntentSample SampleSemanticIntent(
            RE::Character* actor,
            const MotionProfileDatabase::SpeedIntentConfig& config) noexcept
        {
            SemanticIntentSample out{};
            if (!actor) {
                return out;
            }

            bool actorStateValid = false;
            bool walking = false;
            bool running = false;
            bool sprinting = false;
            bool forceRun = false;
            if (auto* actorState = actor->AsActorState()) {
                actorStateValid = true;
                walking = actorState->actorState1.walking != 0;
                running = actorState->actorState1.running != 0;
                sprinting = actorState->actorState1.sprinting != 0;
                forceRun = actorState->actorState2.forceRun != 0;
            }

            auto* process = actor->GetActorRuntimeData().currentProcess;
            const bool combat = actor->IsInCombat();
            const bool fleeing = process && process->middleHigh && process->middleHigh->isFleeing;
            const bool alert = process &&
                process->lowProcessFlags.any(RE::AIProcess::LowProcessFlags::kAlert);

            if (fleeing) {
                return { true, FamilyFromConfig(config.fleeFamily), IntentSource::kFlee, -1 };
            }
            if (combat) {
                return { true, FamilyFromConfig(config.combatFamily), IntentSource::kCombat, -1 };
            }
            if (actorStateValid && (sprinting || forceRun || running)) {
                return { true, FamilyFromConfig(config.actorRunFamily), IntentSource::kActorRunState, -1 };
            }

            if (process) {
                std::int32_t preferredRaw = -1;
                if (const Family preferred = RuntimePackagePreferredFamily(process, config, preferredRaw);
                    preferred != Family::kNone) {
                    return { true, preferred, IntentSource::kRuntimePackagePreferred, preferredRaw };
                }
            }

            if (alert) {
                return { true, FamilyFromConfig(config.alertFamily), IntentSource::kAlert, -1 };
            }
            if (actorStateValid && walking) {
                return { true, FamilyFromConfig(config.actorWalkFamily), IntentSource::kActorWalkState, -1 };
            }
            if (process && process->currentPackage.package) {
                return { true, FamilyFromConfig(config.defaultPackageFamily), IntentSource::kDefaultPackageWalk, -1 };
            }
            return out;
        }

        // Exact R1-69 latch behavior: bootstrap immediately; every replacement must
        // remain continuously identical for 5 seconds. Ambiguous samples hold the
        // current state and erase no state by themselves.
        [[nodiscard]] bool UpdateLatch(
            State& state,
            Family rawCandidate,
            Clock::time_point now,
            float confirmationSeconds,
            float& confirmedFor) noexcept
        {
            confirmedFor = 0.0f;

            if (!state.initialized) {
                if (rawCandidate == Family::kNone) {
                    return false;
                }
                state.initialized = true;
                state.latched = rawCandidate;
                state.candidate = rawCandidate;
                state.candidateSince = Clock::time_point{};
                ++state.revision;
                ++state.changes;
                return true;
            }

            if (rawCandidate == Family::kNone) {
                rawCandidate = state.latched;
            }

            if (rawCandidate == state.latched) {
                if (state.candidate != state.latched || state.candidateSince != Clock::time_point{}) {
                    ++state.pendingResets;
                }
                state.candidate = state.latched;
                state.candidateSince = Clock::time_point{};
                return false;
            }

            if (rawCandidate != state.candidate) {
                state.candidate = rawCandidate;
                state.candidateSince = now;
                return false;
            }

            confirmedFor = SecondsSince(state.candidateSince, now);
            if (confirmedFor >= confirmationSeconds) {
                state.latched = rawCandidate;
                state.candidateSince = Clock::time_point{};
                ++state.revision;
                ++state.changes;
                return true;
            }
            return false;
        }

    }

    bool SupportsActor(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        return profile && profile->speed.enabled;
    }

    void BeforeUpdate(RE::Character* actor, float)
    {
        if (!actor || actor->IsPlayerRef()) {
            return;
        }
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || !profile->speed.enabled) {
            return;
        }

        const auto formID = actor->GetFormID();
        const auto now = Clock::now();
        const auto semantic = SampleSemanticIntent(actor, profile->speed);

        bool changed = false;
        bool bootstrap = false;
        Family oldFamily = Family::kNone;
        Family newFamily = Family::kNone;
        std::uint64_t revision = 0;
        float confirmedFor = 0.0f;

        {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_state[formID];
            ++state.frames;
            state.lastSource = semantic.source;
            state.lastPreferredRaw = semantic.packagePreferredRaw;

            oldFamily = state.latched;
            const Family rawCandidate = semantic.valid ? semantic.family : state.latched;
            changed = UpdateLatch(state, rawCandidate, now, profile->speed.stateChangeConfirmSeconds, confirmedFor);
            newFamily = state.latched;
            revision = state.revision;
            bootstrap = changed && oldFamily == Family::kNone && confirmedFor <= 0.0f;
        }

        if (changed) {
            SKSE::log::info(
                "[SpeedIntent][LATCH] form={:08X} rev={} from={} to={} token={} source={} packagePref={} confirmation={}",
                formID, revision, FamilyName(oldFamily), FamilyName(newFamily),
                SolverToken(newFamily), IntentSourceName(semantic.source),
                semantic.packagePreferredRaw, bootstrap ? "BOOTSTRAP" : "PROFILE_CONTINUOUS");
        }
    }

    void BeforeUniversalUpdate(RE::Character* actor, float)
    {
        if (!actor || actor->IsPlayerRef() || MotionProfileDatabase::SupportsActor(actor)) return;

        const auto formID = actor->GetFormID();
        const auto demandSample = SampleUniversalMovementDemand(actor);
        const auto semantic = SampleUniversalSemanticIntent(actor);

        Family oldFamily = Family::kNone;
        Family newFamily = Family::kNone;
        std::uint64_t revision = 0;
        bool familyChanged = false;
        bool demandChanged = false;
        bool demandNow = false;
        SteeringIntentController::Source demandSource = SteeringIntentController::Source::kNone;
        float goalDistance = 0.0f;
        {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_state[formID];
            ++state.frames;
            state.lastSource = semantic.source;
            state.lastPreferredRaw = semantic.packagePreferredRaw;
            oldFamily = state.latched;

            bool nextDemand = demandSample.demand;
            if (nextDemand) {
                state.universalDemandLossFrames = 0;
            } else if (state.universalMovementDemand) {
                // B8 anti-churn: a proven locomotion demand is not destroyed by one
                // transient steering-source gap. ActorState may SUSTAIN a proven demand
                // briefly, but can never create one from idle.
                constexpr std::uint32_t kDemandLossGraceFrames = 8;
                const bool heldHeading = demandSample.commandValid &&
                    demandSample.source == SteeringIntentController::Source::kHeldLastHeading;
                const bool transientLoss = !demandSample.commandValid || heldHeading ||
                    !demandSample.actorDirectional;
                if (transientLoss &&
                    state.universalDemandLossFrames < kDemandLossGraceFrames) {
                    ++state.universalDemandLossFrames;
                    nextDemand = true;
                } else {
                    state.universalDemandLossFrames = 0;
                }
            } else {
                state.universalDemandLossFrames = 0;
            }
            demandChanged = nextDemand != state.universalMovementDemand;
            if (demandChanged) ++state.universalDemandRevision;
            state.universalMovementDemand = nextDemand;
            state.universalGoalDistanceXY = demandSample.goalDistanceXY;
            state.universalDemandSource = demandSample.source;
            demandNow = state.universalMovementDemand;
            demandSource = state.universalDemandSource;
            goalDistance = state.universalGoalDistanceXY;

            // Gait classification is intentionally separate from movement presence.
            // ActorState/package/combat may choose WALK/TROT/RUN, but they can never
            // manufacture locomotion. When no semantic signal exists, keep the last
            // family; a new movement demand with no prior family defaults to WALK.
            Family desired = semantic.valid ? semantic.family : state.latched;
            if (desired == Family::kNone && demandNow) desired = Family::kWalkForward;

            if (!state.initialized && desired != Family::kNone) {
                state.initialized = true;
                state.latched = desired;
                state.candidate = desired;
                ++state.revision;
                ++state.changes;
                familyChanged = true;
            } else if (state.initialized && desired != Family::kNone && desired != state.latched) {
                state.latched = desired;
                state.candidate = desired;
                state.candidateSince = Clock::time_point{};
                ++state.revision;
                ++state.changes;
                familyChanged = true;
            }
            newFamily = state.latched;
            revision = state.revision;
        }

        if (demandChanged) {
            SKSE::log::info(
                "[UniversalIntent][DEMAND] form={:08X} active={} source={} goalDist={:.2f} actorDirectional={} commandValid={} authority=AI_PATHING_NOT_ACTORSTATE",
                formID, demandNow, SteeringIntentController::SourceName(demandSource), goalDistance,
                demandSample.actorDirectional, demandSample.commandValid);
        }
        if (familyChanged) {
            SKSE::log::info(
                "[UniversalIntent][LATCH] form={:08X} rev={} from={} to={} token={} source={} packagePref={} movementDemand={} translation=SEMANTIC_ONLY graphValue=AUTOGRAPH",
                formID, revision, FamilyName(oldFamily), FamilyName(newFamily),
                SolverToken(newFamily), IntentSourceName(semantic.source), semantic.packagePreferredRaw, demandNow);
        }
    }

    void ApplyAutoGraphAdapterPreNative(RE::Character* actor, float)
    {
        if (!actor || actor->IsPlayerRef() || MotionProfileDatabase::SupportsActor(actor)) return;

        const auto formID = actor->GetFormID();
        const bool movementDemand = HasUniversalMovementDemand(actor);
        const Family family = LatchedFamily(formID);
        const std::uint64_t revision = Revision(formID);

        std::string signature;
        AutoGraphDescriptor descriptor{};
        if (!ResolveAutoGraphDescriptor(actor, signature, descriptor) ||
            !descriptor.structuralContract || descriptor.speedVariable.empty()) {
            return;
        }

        if (HasUniversalActionBypass(actor)) {
            std::scoped_lock lock{ g_autoGraphMutex };
            auto& state = g_autoGraphActors[formID];
            if (state.signature != signature) {
                state = {};
                state.signature = signature;
            }
            state.lastDirectional = false;
            state.parentEventAccepted = false;
            state.familyEventAccepted = Family::kNone;
            state.parentAttempts = 0;
            state.familyAttempts = 0;
            state.directionInitialized = false;
            state.pendingDirectionSector = -1;
            state.pendingDirectionFrames = 0;
            state.selectorInitialized = false;
            state.selectedSelectorGoal = 0.0f;
            return;
        }

        bool graphMoving = false;
        const bool movingRead = !descriptor.movingVariable.empty() &&
            actor->GetGraphVariableBool(descriptor.movingVariable.c_str(), graphMoving);

        float priorSpeed = 0.0f;
        const bool speedRead =
            actor->GetGraphVariableFloat(descriptor.speedVariable.c_str(), priorSpeed) &&
            std::isfinite(priorSpeed);

        SteeringIntentController::FrameCommand steering{};
        const bool steeringUsable =
            SteeringIntentController::GetFrameCommand(actor, steering) && steering.valid &&
            std::isfinite(steering.filteredErrorRadians);

        bool sendParent = false;
        bool sendFamily = false;
        bool sendStop = false;
        bool writeSelector = false;
        bool writeStopSelector = false;
        bool writeDirection = false;
        bool edgeStart = false;
        bool edgeStop = false;
        bool familyChanged = false;
        bool directionChanged = false;
        float selectorGoal = 0.0f;
        float directionGoal = 0.0f;
        std::int32_t directionSector = 0;
        std::string selectorVariable;
        std::string parentEvent;
        std::string familyEvent;
        std::string stopEvent;

        {
            std::scoped_lock lock{ g_autoGraphMutex };
            auto& shared = g_autoGraphDescriptors[signature];
            auto& state = g_autoGraphActors[formID];
            ++state.frames;
            if (state.signature != signature) {
                state = {};
                state.signature = signature;
            }

            if (state.parentRetryCooldownFrames > 0) --state.parentRetryCooldownFrames;
            if (state.familyRetryCooldownFrames > 0) --state.familyRetryCooldownFrames;

            edgeStart = movementDemand && !state.lastDirectional;
            edgeStop = !movementDemand && state.lastDirectional;
            familyChanged = movementDemand &&
                (revision != state.lastRevision || family != state.lastFamily);

            parentEvent = shared.parentStartEvent;
            familyEvent = EventForAutoFamily(shared, family);
            stopEvent = shared.stopEvent;

            if (movingRead && graphMoving) {
                state.parentEventAccepted = true;
            }

            constexpr std::uint64_t kMaxParentAttempts = 3;
            constexpr std::uint64_t kMaxFamilyAttempts = 3;
            const bool parentInactive = movingRead ? !graphMoving : !state.parentEventAccepted;
            if (movementDemand && !parentEvent.empty() &&
                (edgeStart ||
                 (parentInactive && !state.parentEventAccepted &&
                  state.parentRetryCooldownFrames == 0 &&
                  state.parentAttempts < kMaxParentAttempts))) {
                sendParent = true;
                state.parentRetryCooldownFrames = 12;
                ++state.parentAttempts;
            }

            const bool parentReady = parentEvent.empty() ||
                (movingRead && graphMoving) ||
                state.parentEventAccepted ||
                state.parentAttempts >= kMaxParentAttempts;
            if (movementDemand && parentReady && !sendParent && !familyEvent.empty() &&
                (familyChanged ||
                 (state.familyEventAccepted != family &&
                  state.familyRetryCooldownFrames == 0 &&
                  state.familyAttempts < kMaxFamilyAttempts))) {
                sendFamily = true;
                state.familyRetryCooldownFrames = 12;
                ++state.familyAttempts;
            }

            if (edgeStop) {
                if (!stopEvent.empty()) sendStop = true;
                state.parentEventAccepted = false;
                state.familyEventAccepted = Family::kNone;
                state.parentAttempts = 0;
                state.familyAttempts = 0;
                state.directionInitialized = false;
                state.pendingDirectionSector = -1;
                state.pendingDirectionFrames = 0;
                state.selectorInitialized = false;
                state.selectedSelectorGoal = 0.0f;
            }

            // B8 contract split:
            // - EVENT/HYBRID graphs: state events own the gait family; sampled speed picks the
            //   authored point inside that family.
            // - CONTINUOUS graphs: sampled speed itself selects the gait family.
            // This is the structural distinction proven by the old Wolf and Troll implementations.
            const bool continuousSelector =
                shared.topology == AutoGraphTopology::kContinuousSpeed;
            // Structural anchors were extracted from hkbBlenderGenerator child weights,
            // therefore they live in the sampled/blender domain. Writing those anchors
            // into upstream Speed (B5/B7) mixed two different domains. Prefer the actual
            // sampled selector whenever the loaded graph exposes it, including hybrid
            // walk/run state graphs such as Wolf/Bear.
            selectorVariable = !shared.sampledSpeedVariable.empty()
                ? shared.sampledSpeedVariable : shared.speedVariable;
            selectorGoal = StructuralSpeedForFamily(shared, family);
            writeSelector = movementDemand && !selectorVariable.empty() &&
                std::isfinite(selectorGoal) && selectorGoal > 0.001f;
            writeStopSelector = edgeStop && !selectorVariable.empty();
            if (writeSelector) {
                state.selectorInitialized = true;
                state.selectedSelectorGoal = selectorGoal;
            }

            // Only continuous graphs whose structural contract explicitly exposes a 4/8-way
            // cyclic Direction selector receive synthetic Direction. No hybrid/event graph is
            // forced into the Troll directional model.
            const bool directionalContinuous = continuousSelector &&
                shared.directionSectors >= 4 && !shared.directionVariable.empty();
            if (movementDemand && directionalContinuous && steeringUsable) {
                const float steeringErrorDeg =
                    WrapSignedDegrees(steering.filteredErrorRadians * kRadToDeg);
                const auto desiredSector =
                    DirectionSectorForError(steeringErrorDeg, shared.directionSectors);

                if (!state.directionInitialized) {
                    state.directionInitialized = true;
                    state.directionSector = desiredSector;
                    state.selectedDirectionValue =
                        DirectionValueForSector(desiredSector, shared.directionSectors);
                    state.pendingDirectionSector = -1;
                    state.pendingDirectionFrames = 0;
                    directionChanged = true;
                } else if (desiredSector != state.directionSector) {
                    const float sectorWidth =
                        360.0f / static_cast<float>(shared.directionSectors);
                    const float currentCenter =
                        SteeringCenterDegreesForSector(state.directionSector, shared.directionSectors);
                    const float errorFromCurrentCenter = std::abs(
                        WrapSignedDegrees(steeringErrorDeg - currentCenter));
                    const bool leftEnvelope = errorFromCurrentCenter >
                        (sectorWidth * 0.5f + kDirectionSwitchHysteresisDegrees);

                    if (leftEnvelope) {
                        if (state.pendingDirectionSector == desiredSector) {
                            if (state.pendingDirectionFrames < 255) ++state.pendingDirectionFrames;
                        } else {
                            state.pendingDirectionSector = desiredSector;
                            state.pendingDirectionFrames = 1;
                        }
                        if (state.pendingDirectionFrames >= kDirectionConfirmFrames) {
                            state.directionSector = desiredSector;
                            state.selectedDirectionValue =
                                DirectionValueForSector(desiredSector, shared.directionSectors);
                            state.pendingDirectionSector = -1;
                            state.pendingDirectionFrames = 0;
                            directionChanged = true;
                        }
                    } else {
                        state.pendingDirectionSector = -1;
                        state.pendingDirectionFrames = 0;
                    }
                } else {
                    state.pendingDirectionSector = -1;
                    state.pendingDirectionFrames = 0;
                }

                directionSector = state.directionSector;
                directionGoal = state.selectedDirectionValue;
                writeDirection = state.directionInitialized;
            }

            state.lastDirectional = movementDemand;
            state.lastFamily = family;
            state.lastRevision = revision;
            if (speedRead) state.lastObservedSpeed = priorSpeed;
            ++shared.drives;
            descriptor = shared;
        }

        bool selectorWrote = false;
        bool directionWrote = false;
        if (writeSelector) {
            selectorWrote = actor->SetGraphVariableFloat(selectorVariable.c_str(), selectorGoal);
        } else if (writeStopSelector) {
            selectorWrote = actor->SetGraphVariableFloat(selectorVariable.c_str(), 0.0f);
        }
        if (writeDirection && !descriptor.directionVariable.empty()) {
            directionWrote = actor->SetGraphVariableFloat(
                descriptor.directionVariable.c_str(), directionGoal);
        }

        bool parentAccepted = false;
        bool familyAccepted = false;
        bool stopAccepted = false;
        if (sendParent) parentAccepted = actor->NotifyAnimationGraph(parentEvent.c_str());
        if (sendFamily) familyAccepted = actor->NotifyAnimationGraph(familyEvent.c_str());
        if (sendStop) stopAccepted = actor->NotifyAnimationGraph(stopEvent.c_str());

        {
            std::scoped_lock lock{ g_autoGraphMutex };
            auto& state = g_autoGraphActors[formID];
            if (sendParent && parentAccepted) state.parentEventAccepted = true;
            if (sendFamily && familyAccepted) state.familyEventAccepted = family;
            if (sendStop) {
                state.parentEventAccepted = false;
                state.familyEventAccepted = Family::kNone;
            }

            auto& shared = g_autoGraphDescriptors[signature];
            const std::uint64_t accepted =
                static_cast<std::uint64_t>(parentAccepted) +
                static_cast<std::uint64_t>(familyAccepted) +
                static_cast<std::uint64_t>(stopAccepted);
            const std::uint64_t attempted =
                static_cast<std::uint64_t>(sendParent) +
                static_cast<std::uint64_t>(sendFamily) +
                static_cast<std::uint64_t>(sendStop);
            shared.eventAccepted += accepted;
            shared.eventRejected += attempted - accepted;
            if (selectorWrote || directionWrote) ++shared.speedWrites;
        }

        if (edgeStart || edgeStop || familyChanged || sendParent || sendFamily || sendStop ||
            directionChanged) {
            SKSE::log::info(
                "[AutoGraph][B8_DRIVE] form={:08X} signature=\"{}\" topology={} demand={} family={} rev={} graph[movingRead={} moving={}] nativeSpeed[input=\"{}\" priorRead={} prior={:.3f}] selector[var=\"{}\" goal={:.3f} wrote={}] direction[var=\"{}\" sector={} value={:.4f} wrote={} changed={}] events[parent=\"{}\":{} family=\"{}\":{} stop=\"{}\":{}] handoff=REQUIRES_CERTIFIED_LOCOMOTION_CARRIER",
                formID, signature, AutoGraphTopologyName(descriptor.topology), movementDemand,
                FamilyName(family), revision, movingRead, graphMoving,
                descriptor.speedVariable, speedRead, priorSpeed,
                selectorVariable, writeStopSelector ? 0.0f : selectorGoal, selectorWrote,
                descriptor.directionVariable, directionSector, directionGoal,
                directionWrote, directionChanged,
                parentEvent, parentAccepted, familyEvent, familyAccepted,
                stopEvent, stopAccepted);
        }
    }

    void ApplyAutoGraphAdapterFinal(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef() || MotionProfileDatabase::SupportsActor(actor) ||
            !HasUniversalMovementDemand(actor) || HasUniversalActionBypass(actor)) {
            return;
        }

        std::string signature;
        AutoGraphDescriptor descriptor{};
        if (!ResolveAutoGraphDescriptor(actor, signature, descriptor) ||
            !descriptor.structuralContract || descriptor.sampledSpeedVariable.empty()) {
            return;
        }

        bool selectorInitialized = false;
        float selectorGoal = 0.0f;
        bool directionInitialized = false;
        float directionGoal = 0.0f;
        {
            std::scoped_lock lock{ g_autoGraphMutex };
            const auto it = g_autoGraphActors.find(actor->GetFormID());
            if (it == g_autoGraphActors.end() || it->second.signature != signature) return;
            selectorInitialized = it->second.selectorInitialized;
            selectorGoal = it->second.selectedSelectorGoal;
            directionInitialized = it->second.directionInitialized;
            directionGoal = it->second.selectedDirectionValue;
        }

        const std::string& selectorVariable = descriptor.sampledSpeedVariable;
        if (selectorInitialized && std::isfinite(selectorGoal) && selectorGoal > 0.001f) {
            actor->SetGraphVariableFloat(selectorVariable.c_str(), selectorGoal);
        }
        if (descriptor.topology == AutoGraphTopology::kContinuousSpeed &&
            directionInitialized && descriptor.directionSectors >= 4 &&
            !descriptor.directionVariable.empty() && std::isfinite(directionGoal)) {
            actor->SetGraphVariableFloat(descriptor.directionVariable.c_str(), directionGoal);
        }
    }

    bool ApplyGraphGoal(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef()) {
            return false;
        }
        const auto* profile = MotionProfileDatabase::ResolveProfile(actor);
        if (!profile || !profile->speed.enabled) {
            return false;
        }

        const auto formID = actor->GetFormID();
        Family held = Family::kNone;
        bool initialized = false;
        bool logAdapter = false;
        std::uint64_t revision = 0;
        {
            std::scoped_lock lock{ g_mutex };
            const auto it = g_state.find(formID);
            if (it == g_state.end()) {
                return false;
            }
            initialized = it->second.initialized;
            held = it->second.latched;
            revision = it->second.revision;
            logAdapter = initialized && it->second.lastLoggedGraphFamily != held;
        }

        if (!initialized || held == Family::kNone) {
            return false;
        }

        // Gait and movement presence are intentionally separate. The selector may
        // remember RUN while the wolf is idle, but this module cannot start movement.
        bool directional = false;
        if (auto* actorState = actor->AsActorState()) {
            directional = actorState->actorState1.movingForward != 0 ||
                actorState->actorState1.movingBack != 0 ||
                actorState->actorState1.movingLeft != 0 ||
                actorState->actorState1.movingRight != 0;
        }
        bool graphMoving = false;
        const bool graphMovingRead = actor->GetGraphVariableBool(profile->speed.graphMoving.c_str(), graphMoving);
        if (!directional && !(graphMovingRead && graphMoving)) {
            std::scoped_lock lock{ g_mutex };
            ++g_state[formID].graphMotionGateSkips;
            return false;
        }

        const float goal = GraphGoal(held, profile->speed);
        const bool wrote = actor->SetGraphVariableFloat(profile->speed.graphSpeed.c_str(), goal);
        if (wrote) {
            std::scoped_lock lock{ g_mutex };
            auto& state = g_state[formID];
            ++state.graphWrites;
            if (logAdapter) {
                state.lastLoggedGraphFamily = held;
            }
        }

        if (wrote && logAdapter) {
            SKSE::log::info(
                "[SpeedIntent][GRAPH] form={:08X} rev={} held={} token={} Speed={:.3f} SpeedSampled=NATIVE Direction=NATIVE",
                formID, revision, FamilyName(held), SolverToken(held), goal);
        }
        return wrote;
    }

    bool HasUniversalMovementDemand(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef() || MotionProfileDatabase::SupportsActor(actor)) return false;
        std::scoped_lock lock{ g_mutex };
        const auto it = g_state.find(actor->GetFormID());
        return it != g_state.end() && it->second.universalMovementDemand;
    }

    bool HasResolvedUniversalGraphContract(RE::Actor* actor)
    {
        if (!actor || actor->IsPlayerRef() || MotionProfileDatabase::SupportsActor(actor)) return false;
        std::string signature;
        AutoGraphDescriptor descriptor{};
        if (!ResolveAutoGraphDescriptor(actor, signature, descriptor)) return false;
        return descriptor.structuralContract && !descriptor.speedVariable.empty() &&
            !descriptor.parentStartEvent.empty();
    }

    Family LatchedFamily(RE::FormID formID)
    {
        if (formID == 0) {
            return Family::kNone;
        }
        std::scoped_lock lock{ g_mutex };
        const auto it = g_state.find(formID);
        return it != g_state.end() && it->second.initialized ? it->second.latched : Family::kNone;
    }

    std::uint64_t Revision(RE::FormID formID)
    {
        std::scoped_lock lock{ g_mutex };
        const auto it = g_state.find(formID);
        return it != g_state.end() ? it->second.revision : 0;
    }

    bool IsRuntimeEligible(RE::Actor* actor)
    {
        return actor && actor->GetParentCell() && actor->Is3DLoaded() &&
            !actor->IsDisabled() && !actor->IsDead() &&
            !actor->IsInRagdollState() && !actor->IsInKillMove() &&
            !actor->IsInMidair() && !actor->IsInWater() &&
            !actor->IsBeingRidden();
    }

    // Compatibility only: the untouched SyntheticMotionInjector still calls
    // these three entry points. They do not participate in gait selection.
    void ObserveDbClip(RE::FormID, std::string_view, std::string_view, std::string_view)
    {}

    void ObserveExternalPassThrough(RE::FormID)
    {}

    void Clear()
    {
        std::scoped_lock lock{ g_mutex };
        for (const auto& [formID, state] : g_state) {
            SKSE::log::info(
                "[SpeedIntent][SUMMARY] form={:08X} frames={} rev={} latched={} pending={} changes={} resets={} graphWrites={} idleSkips={} demand={} demandRev={} goalDist={:.2f} demandSource={}",
                formID, state.frames, state.revision, FamilyName(state.latched), FamilyName(state.candidate),
                state.changes, state.pendingResets, state.graphWrites, state.graphMotionGateSkips,
                state.universalMovementDemand, state.universalDemandRevision, state.universalGoalDistanceXY,
                SteeringIntentController::SourceName(state.universalDemandSource));
        }
        g_state.clear();
        {
            std::scoped_lock autoLock{ g_autoGraphMutex };
            for (const auto& [signature, descriptor] : g_autoGraphDescriptors) {
                SKSE::log::info(
                    "[AutoGraph][SUMMARY] signature=\"{}\" projectKey=\"{}\" topology={} events[parent=\"{}\" walk=\"{}\" trot=\"{}\" run=\"{}\" stop=\"{}\"] vars[speed=\"{}\" sampled=\"{}\" direction=\"{}\" moving=\"{}\"] structural[resolved={} walk={:.3f} trot={:.3f} run={:.3f} sectors={}] drives={} eventAccepted={} eventRejected={} speedWrites={}",
                    signature, descriptor.projectKey, AutoGraphTopologyName(descriptor.topology),
                    descriptor.parentStartEvent, descriptor.walkEvent, descriptor.trotEvent,
                    descriptor.runEvent, descriptor.stopEvent, descriptor.speedVariable, descriptor.sampledSpeedVariable,
                    descriptor.directionVariable, descriptor.movingVariable, descriptor.structuralContract,
                    descriptor.structuralWalkSpeed, descriptor.structuralTrotSpeed,
                    descriptor.structuralRunSpeed, descriptor.directionSectors, descriptor.drives,
                    descriptor.eventAccepted, descriptor.eventRejected, descriptor.speedWrites);
            }
            g_autoGraphActors.clear();
            g_autoGraphDescriptors.clear();
        }
    }
}

