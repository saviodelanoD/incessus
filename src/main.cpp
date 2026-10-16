#include "PCH.h"
#include "Hooks.h"

namespace logger = SKSE::log;

namespace
{
    void SetupLog()
    {
        const auto logDirectory = SKSE::log::log_directory();
        if (!logDirectory) {
            return;
        }

        const auto pluginName = SKSE::PluginDeclaration::GetSingleton()->GetName();
        const auto logPath = *logDirectory / std::format("{}.log", pluginName);

        auto sink = std::make_shared<spdlog::sinks::basic_file_sink_mt>(
            logPath.string(),
            true);
        auto log = std::make_shared<spdlog::logger>("global", std::move(sink));

        spdlog::set_default_logger(std::move(log));
        spdlog::set_level(spdlog::level::info);
        spdlog::flush_on(spdlog::level::info);
    }

    void MessageHandler(SKSE::MessagingInterface::Message* a_message)
    {
        if (!a_message) {
            return;
        }

        switch (a_message->type) {
        case SKSE::MessagingInterface::kDataLoaded:
            Hooks::Install();
            break;

        case SKSE::MessagingInterface::kPostLoadGame:
        case SKSE::MessagingInterface::kNewGame:
            Hooks::Reset();
            break;

        default:
            break;
        }
    }
}

SKSEPluginLoad(const SKSE::LoadInterface* a_skse)
{
    SKSE::Init(a_skse);
    SetupLog();

    logger::info("Animal Locomotion Revolution 0.3.0 Authority Probe initializing...");

    auto* messaging = SKSE::GetMessagingInterface();
    if (!messaging) {
        logger::critical("Could not acquire the SKSE messaging interface.");
        return false;
    }

    messaging->RegisterListener(MessageHandler);
    logger::info("Plugin loaded; waiting for DataLoaded.");
    return true;
}

