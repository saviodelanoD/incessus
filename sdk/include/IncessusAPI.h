#pragma once
#include "AnimalLocomotionRevolutionAPI.h"

namespace Incessus {
namespace API = ALR_API;
namespace SDK {
inline constexpr const char* kRelease = "INCESSUS-SDK-U9-2026-10-05";
inline bool Compatible(const API::InterfaceV1* api) noexcept {
    return api && api->structSize>=sizeof(API::InterfaceV1) &&
        api->version==API::kInterfaceVersion1 && api->QueryLocomotion;
}
inline bool Query(const API::InterfaceV1* api,std::uint32_t actor,API::LocomotionSnapshotV1& out) noexcept {
    out={};
    if(!actor || !Compatible(api))return false;
    if(!api->QueryLocomotion(actor,&out) || out.structSize!=sizeof(out) ||
        out.interfaceVersion!=API::kInterfaceVersion1 || out.actorFormID!=actor || !out.snapshotSerial ||
        !(out.flags & API::kSupportedActor)) {out={};return false;}
    return true;
}
}
}

