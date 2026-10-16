// BUILD_ID: INCESSUS-R116.2-MOTIONDB-LEXICAL-PROFILE-GUARD-2026-09-06
#include "PCH.h"
#include "AnimationMotionDatabase.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace AnimationMotionDatabase
{
    namespace fs = std::filesystem;

    struct MotionPoint
    {
        float time{ 0.0f };
        float x{ 0.0f };
        float y{ 0.0f };
        float z{ 0.0f };
        float yaw{ 0.0f };
        bool hasMotion{ false };
        bool hasRotation{ false };
    };

    struct MotionClip
    {
        std::string profile;
        std::string key;
        fs::path sourcePath;
        std::vector<MotionPoint> points;
        float duration{ 0.0f };
        float cycleX{ 0.0f };
        float cycleY{ 0.0f };
        float cycleZ{ 0.0f };
        float cycleYaw{ 0.0f };
        bool hasMotion{ false };
        bool hasRotation{ false };
    };

    namespace detail
    {
        constexpr std::string_view kRelativeDatabasePath = "Data/MotionDB";
        constexpr std::string_view kLegacyPlayerProfile = "Player";
        constexpr float kEpsilon = 1.0e-6f;
        constexpr float kPi = 3.14159265358979323846f;
        constexpr float kTwoPi = 6.28318530717958647692f;

        std::unordered_map<std::string, MotionClip> g_clips;
        Stats g_stats{};
        std::mutex g_mutex;
        bool g_loaded{ false };

        [[nodiscard]] std::string Lower(std::string value)
        {
            std::transform(value.begin(), value.end(), value.begin(), [](unsigned char c) {
                return static_cast<char>(std::tolower(c));
            });
            return value;
        }

        void ReplaceAll(std::string& value, char from, char to)
        {
            std::replace(value.begin(), value.end(), from, to);
        }

        [[nodiscard]] bool EndsWith(std::string_view value, std::string_view suffix) noexcept
        {
            return value.size() >= suffix.size() && value.substr(value.size() - suffix.size()) == suffix;
        }

        [[nodiscard]] std::string NormalizeProfile(std::string value)
        {
            ReplaceAll(value, '\\', '/');
            value = Lower(std::move(value));
            value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
                return c == ' ' || c == '-' || c == '_';
            }), value.end());
            return value;
        }

        [[nodiscard]] std::string NormalizeKey(std::string value, std::string_view profile)
        {
            ReplaceAll(value, '\\', '/');
            const auto slash = value.find_last_of('/');
            if (slash != std::string::npos) value.erase(0, slash + 1);
            value = Lower(std::move(value));

            constexpr std::string_view annotationSuffix = ".annotations.txt";
            constexpr std::string_view hkxSuffix = ".hkx";
            if (EndsWith(value, annotationSuffix)) value.resize(value.size() - annotationSuffix.size());
            else if (EndsWith(value, hkxSuffix)) value.resize(value.size() - hkxSuffix.size());

            // A cache file may optionally carry its owning profile folder as a suffix.
            // The suffix is derived from data, never from a species list in code.
            std::string rawProfile = Lower(std::string{ profile });
            std::string suffix = "_" + rawProfile;
            if (!rawProfile.empty() && EndsWith(value, suffix)) value.resize(value.size() - suffix.size());

            value.erase(std::remove_if(value.begin(), value.end(), [](unsigned char c) {
                return c == '_' || c == ' ' || c == '-';
            }), value.end());
            return value;
        }

        [[nodiscard]] std::string CompositeKey(std::string_view profile, std::string_view animationName)
        {
            const auto p = NormalizeProfile(std::string{ profile });
            const auto k = NormalizeKey(std::string{ animationName }, profile);
            if (p.empty() || k.empty()) return {};
            return p + "\x1F" + k;
        }

        [[nodiscard]] float UnwrapYaw(float previous, float raw) noexcept
        {
            float value = raw;
            while (value - previous > kPi) value -= kTwoPi;
            while (value - previous < -kPi) value += kTwoPi;
            return value;
        }

        bool ParseClipFile(const fs::path& path, std::string_view profile, MotionClip& out)
        {
            std::ifstream input(path);
            if (!input) return false;
            out = {};
            out.profile = std::string{ profile };
            out.key = NormalizeKey(path.filename().string(), profile);
            out.sourcePath = path;
            if (out.profile.empty() || out.key.empty()) return false;

            std::unordered_map<long long, std::size_t> pointByMicrosecond;
            std::string line;
            while (std::getline(input, line)) {
                if (line.empty()) continue;
                std::istringstream stream(line);
                float time = 0.0f;
                std::string tag;
                float a = 0.0f, b = 0.0f, c = 0.0f;
                if (!(stream >> time >> tag >> a >> b >> c) || !std::isfinite(time)) continue;
                tag = Lower(std::move(tag));
                const long long timeKey = static_cast<long long>(std::llround(static_cast<double>(time) * 1000000.0));
                auto it = pointByMicrosecond.find(timeKey);
                if (it == pointByMicrosecond.end()) {
                    MotionPoint point{};
                    point.time = time;
                    out.points.push_back(point);
                    const auto index = out.points.size() - 1;
                    pointByMicrosecond.emplace(timeKey, index);
                    it = pointByMicrosecond.find(timeKey);
                }
                auto& point = out.points[it->second];
                if (tag == "animmotion") {
                    point.x = a; point.y = b; point.z = c;
                    point.hasMotion = true; out.hasMotion = true;
                } else if (tag == "animrotation") {
                    point.yaw = c;
                    point.hasRotation = true; out.hasRotation = true;
                }
            }

            std::sort(out.points.begin(), out.points.end(), [](const MotionPoint& l, const MotionPoint& r) {
                return l.time < r.time;
            });
            if (out.points.size() < 2 || (!out.hasMotion && !out.hasRotation)) return false;

            MotionPoint lastResolved{};
            bool haveLast = false;
            bool haveYaw = false;
            float unwrappedYaw = 0.0f;
            for (auto& point : out.points) {
                if (!point.hasMotion && haveLast) {
                    point.x = lastResolved.x; point.y = lastResolved.y; point.z = lastResolved.z;
                }
                if (point.hasRotation) {
                    if (!haveYaw) { unwrappedYaw = point.yaw; haveYaw = true; }
                    else unwrappedYaw = UnwrapYaw(unwrappedYaw, point.yaw);
                    point.yaw = unwrappedYaw;
                } else if (haveYaw) {
                    point.yaw = unwrappedYaw;
                }
                lastResolved = point;
                haveLast = true;
            }

            const auto& first = out.points.front();
            const auto& last = out.points.back();
            out.duration = last.time - first.time;
            if (!std::isfinite(out.duration) || out.duration <= kEpsilon) return false;
            out.cycleX = last.x - first.x;
            out.cycleY = last.y - first.y;
            out.cycleZ = last.z - first.z;
            out.cycleYaw = out.hasRotation ? last.yaw - first.yaw : 0.0f;
            return true;
        }

        struct Accumulated { float x{}, y{}, z{}, yaw{}; };

        [[nodiscard]] Accumulated SampleWrapped(const MotionClip& clip, float phase) noexcept
        {
            phase = std::clamp(phase, 0.0f, 1.0f);
            const float target = clip.points.front().time + phase * clip.duration;
            if (target <= clip.points.front().time) {
                const auto& p = clip.points.front(); return { p.x, p.y, p.z, p.yaw };
            }
            if (target >= clip.points.back().time) {
                const auto& p = clip.points.back(); return { p.x, p.y, p.z, p.yaw };
            }
            const auto upper = std::upper_bound(clip.points.begin(), clip.points.end(), target,
                [](float t, const MotionPoint& point) { return t < point.time; });
            if (upper == clip.points.begin() || upper == clip.points.end()) {
                const auto& p = clip.points.back(); return { p.x, p.y, p.z, p.yaw };
            }
            const auto& b = *upper;
            const auto& a = *(upper - 1);
            const float span = b.time - a.time;
            const float alpha = span > kEpsilon ? std::clamp((target - a.time) / span, 0.0f, 1.0f) : 0.0f;
            const auto lerp = [alpha](float l, float r) { return l + (r - l) * alpha; };
            return { lerp(a.x, b.x), lerp(a.y, b.y), lerp(a.z, b.z), lerp(a.yaw, b.yaw) };
        }

        [[nodiscard]] Accumulated SampleUnwrapped(const MotionClip& clip, float phaseRaw) noexcept
        {
            if (!std::isfinite(phaseRaw)) return {};
            const float cyclesFloat = std::floor(phaseRaw);
            const auto cycles = static_cast<long long>(cyclesFloat);
            float wrapped = phaseRaw - cyclesFloat;
            if (wrapped < 0.0f) wrapped += 1.0f;
            const auto sample = SampleWrapped(clip, wrapped);
            const auto& first = clip.points.front();
            return {
                sample.x - first.x + static_cast<float>(cycles) * clip.cycleX,
                sample.y - first.y + static_cast<float>(cycles) * clip.cycleY,
                sample.z - first.z + static_cast<float>(cycles) * clip.cycleZ,
                sample.yaw - first.yaw + static_cast<float>(cycles) * clip.cycleYaw
            };
        }

        void LoadDatabaseLocked()
        {
            if (g_loaded) return;
            g_loaded = true;
            g_stats = {};
            g_stats.loadAttempted = true;
            g_clips.clear();

            const fs::path root{ kRelativeDatabasePath };
            std::error_code ec;
            g_stats.directoryFound = fs::exists(root, ec) && fs::is_directory(root, ec);
            if (!g_stats.directoryFound) {
                SKSE::log::warn(R"([MotionDB][MISSING_DIR] path="{}")", root.generic_string());
                return;
            }

            std::unordered_set<std::string> profiles;
            for (fs::recursive_directory_iterator it(root, ec), end; !ec && it != end; it.increment(ec)) {
                if (!it->is_regular_file()) continue;
                const auto filename = Lower(it->path().filename().string());
                if (!EndsWith(filename, ".annotations.txt")) continue;

                // R116.2: derive the profile from the lexical path that the iterator used to
                // reach the file. std::filesystem::relative() canonicalizes paths and on
                // Windows can resolve directory junctions/reparse points outside MotionDB,
                // producing ".." as the first component. That must never become a profile.
                fs::path relative = it->path().lexically_relative(root);
                if (relative.empty() || relative.is_absolute()) {
                    std::error_code relEc;
                    relative = fs::relative(it->path(), root, relEc);
                    if (relEc || relative.empty() || relative.is_absolute()) continue;
                }

                auto component = relative.begin();
                if (component == relative.end()) continue;
                const std::string profile = component->string();
                const bool invalidProfileComponent =
                    profile.empty() || profile == "." || profile == "..";
                if (invalidProfileComponent || relative.parent_path().empty()) {
                    ++g_stats.clipsRejected;
                    SKSE::log::warn(
                        R"([MotionDB][REJECT] file="{}" relative="{}" profile="{}" reason=INVALID_PROFILE_FOLDER)",
                        it->path().generic_string(), relative.generic_string(), profile);
                    continue;
                }

                ++g_stats.filesSeen;
                profiles.insert(NormalizeProfile(profile));
                MotionClip clip{};
                if (!ParseClipFile(it->path(), profile, clip)) {
                    ++g_stats.clipsRejected;
                    SKSE::log::warn(R"([MotionDB][REJECT] profile="{}" file="{}")", profile, it->path().generic_string());
                    continue;
                }

                const auto composite = CompositeKey(profile, clip.key);
                if (composite.empty()) {
                    ++g_stats.clipsRejected;
                    continue;
                }
                if (g_clips.contains(composite)) {
                    ++g_stats.duplicateKeys;
                    SKSE::log::warn(R"([MotionDB][DUPLICATE] profile="{}" key="{}" replacing="{}" with="{}")",
                        profile, clip.key, g_clips[composite].sourcePath.generic_string(), clip.sourcePath.generic_string());
                }
                const auto samples = clip.points.size();
                const auto duration = clip.duration;
                const auto dx = clip.cycleX, dy = clip.cycleY, dz = clip.cycleZ, yaw = clip.cycleYaw;
                const auto key = clip.key;
                g_clips[composite] = std::move(clip);
                ++g_stats.clipsLoaded;
                SKSE::log::info(
                    R"([MotionDB][LOAD] profile="{}" key="{}" samples={} duration={:.6f} cycle=({:+.4f},{:+.4f},{:+.4f}) distanceXY={:.4f} cycleYaw={:+.6f}rad/{:+.2f}deg)",
                    profile, key, samples, duration, dx, dy, dz, std::hypot(dx, dy), yaw, yaw * 57.29577951308232f);
            }
            g_stats.profilesSeen = profiles.size();
            if (ec) {
                SKSE::log::warn(R"([MotionDB][SCAN_ERROR] path="{}" code={} message="{}")",
                    root.generic_string(), ec.value(), ec.message());
            }
            SKSE::log::info(
                R"([MotionDB][READY] path="{}" profiles={} filesSeen={} loaded={} rejected={} duplicateKeys={})",
                root.generic_string(), g_stats.profilesSeen, g_stats.filesSeen, g_stats.clipsLoaded,
                g_stats.clipsRejected, g_stats.duplicateKeys);
        }

        [[nodiscard]] const MotionClip* FindClipLocked(std::string_view profile, std::string_view animationName)
        {
            const auto composite = CompositeKey(profile, animationName);
            if (composite.empty()) return nullptr;
            const auto it = g_clips.find(composite);
            return it == g_clips.end() ? nullptr : std::addressof(it->second);
        }
    }

    void EnsureLoaded()
    {
        std::scoped_lock lock{ detail::g_mutex };
        detail::LoadDatabaseLocked();
    }

    DeltaSample SampleDelta(std::string_view profile, std::string_view animationName, float startPhase, float phaseAdvance)
    {
        DeltaSample out{};
        if (!std::isfinite(startPhase) || !std::isfinite(phaseAdvance) || phaseAdvance < 0.0f) return out;
        std::scoped_lock lock{ detail::g_mutex };
        detail::LoadDatabaseLocked();
        ++detail::g_stats.queries;
        const auto* clip = detail::FindClipLocked(profile, animationName);
        if (!clip) { ++detail::g_stats.misses; return out; }
        const auto a = detail::SampleUnwrapped(*clip, startPhase);
        const auto b = detail::SampleUnwrapped(*clip, startPhase + phaseAdvance);
        out.valid = true; out.yawValid = clip->hasRotation; out.profile = clip->profile; out.key = clip->key;
        out.startPhase = startPhase; out.phaseAdvance = phaseAdvance; out.clipDuration = clip->duration;
        out.dx = b.x - a.x; out.dy = b.y - a.y; out.dz = b.z - a.z; out.deltaYaw = b.yaw - a.yaw;
        out.distanceXY = std::hypot(out.dx, out.dy); ++detail::g_stats.hits; return out;
    }

    DeltaSample SampleDeltaSeconds(std::string_view profile, std::string_view animationName, float startPhase, float deltaSeconds)
    {
        DeltaSample out{};
        if (!std::isfinite(startPhase) || !std::isfinite(deltaSeconds) || deltaSeconds < 0.0f) return out;
        std::scoped_lock lock{ detail::g_mutex };
        detail::LoadDatabaseLocked();
        ++detail::g_stats.queries;
        const auto* clip = detail::FindClipLocked(profile, animationName);
        if (!clip || !std::isfinite(clip->duration) || clip->duration <= detail::kEpsilon) {
            ++detail::g_stats.misses; return out;
        }
        const float phaseAdvance = deltaSeconds / clip->duration;
        const auto a = detail::SampleUnwrapped(*clip, startPhase);
        const auto b = detail::SampleUnwrapped(*clip, startPhase + phaseAdvance);
        out.valid = true; out.yawValid = clip->hasRotation; out.profile = clip->profile; out.key = clip->key;
        out.startPhase = startPhase; out.phaseAdvance = phaseAdvance; out.clipDuration = clip->duration;
        out.dx = b.x - a.x; out.dy = b.y - a.y; out.dz = b.z - a.z; out.deltaYaw = b.yaw - a.yaw;
        out.distanceXY = std::hypot(out.dx, out.dy); ++detail::g_stats.hits; return out;
    }

    bool HasClip(std::string_view profile, std::string_view animationName)
    {
        std::scoped_lock lock{ detail::g_mutex };
        detail::LoadDatabaseLocked();
        return detail::FindClipLocked(profile, animationName) != nullptr;
    }

    ClipMetrics GetClipMetrics(std::string_view profile, std::string_view animationName)
    {
        ClipMetrics out{};
        std::scoped_lock lock{ detail::g_mutex };
        detail::LoadDatabaseLocked();
        const auto* clip = detail::FindClipLocked(profile, animationName);
        if (!clip || clip->points.empty() || !std::isfinite(clip->duration) || clip->duration <= detail::kEpsilon) return out;
        out.valid = true; out.yawValid = clip->hasRotation; out.profile = clip->profile; out.key = clip->key;
        out.duration = clip->duration; out.cycleX = clip->cycleX; out.cycleY = clip->cycleY; out.cycleZ = clip->cycleZ;
        out.cycleYaw = clip->cycleYaw; out.distanceXY = std::hypot(clip->cycleX, clip->cycleY); return out;
    }

    CumulativeSample SampleCumulativeSeconds(std::string_view profile, std::string_view animationName, float motionTime)
    {
        CumulativeSample out{};
        if (!std::isfinite(motionTime)) return out;
        std::scoped_lock lock{ detail::g_mutex };
        detail::LoadDatabaseLocked();
        ++detail::g_stats.queries;
        const auto* clip = detail::FindClipLocked(profile, animationName);
        if (!clip || clip->points.empty() || !std::isfinite(clip->duration) || clip->duration <= detail::kEpsilon) {
            ++detail::g_stats.misses; return out;
        }
        const float firstTime = clip->points.front().time;
        const float lastTime = clip->points.back().time;
        const float t = std::clamp(motionTime, firstTime, lastTime);
        const float phase = std::clamp((t - firstTime) / clip->duration, 0.0f, 1.0f);
        const auto sample = detail::SampleWrapped(*clip, phase);
        out.valid = true; out.yawValid = clip->hasRotation; out.profile = clip->profile; out.key = clip->key;
        out.clipDuration = clip->duration; out.motionTime = t; out.x = sample.x; out.y = sample.y; out.z = sample.z; out.yaw = sample.yaw;
        ++detail::g_stats.hits; return out;
    }

    DeltaSample SampleDelta(std::string_view animationName, float startPhase, float phaseAdvance)
    { return SampleDelta(detail::kLegacyPlayerProfile, animationName, startPhase, phaseAdvance); }
    DeltaSample SampleDeltaSeconds(std::string_view animationName, float startTime, float deltaTime)
    { return SampleDeltaSeconds(detail::kLegacyPlayerProfile, animationName, startTime, deltaTime); }
    bool HasClip(std::string_view animationName)
    { return HasClip(detail::kLegacyPlayerProfile, animationName); }
    ClipMetrics GetClipMetrics(std::string_view animationName)
    { return GetClipMetrics(detail::kLegacyPlayerProfile, animationName); }
    CumulativeSample SampleCumulativeSeconds(std::string_view animationName, float motionTime)
    { return SampleCumulativeSeconds(detail::kLegacyPlayerProfile, animationName, motionTime); }

    Stats GetStats()
    {
        std::scoped_lock lock{ detail::g_mutex };
        return detail::g_stats;
    }

    std::string DatabasePath() { return std::string{ detail::kRelativeDatabasePath }; }

    void Clear()
    {
        std::scoped_lock lock{ detail::g_mutex };
        detail::g_clips.clear(); detail::g_stats = {}; detail::g_loaded = false;
    }
}

