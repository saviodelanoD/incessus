#pragma once

#include "RE/Skyrim.h"

namespace LocomotionSDK
{
    // Internal producer side. Public consumers never include this header.
    // Publish after the ALR animal Character::Update pipeline has resolved the frame.
    void PublishActorFrame(RE::Character* actor) noexcept;
    void Clear() noexcept;
}

