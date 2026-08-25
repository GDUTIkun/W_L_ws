#pragma once

#include "wheel_leg_core/types.hpp"

namespace wheel_leg {

// Only for legacy position/linear-velocity fields, never rotations or wrenches.
inline Vector3 canonicalFluToLegacyForwardRightUp(const Vector3 &value_n) {
  return {value_n[0], -value_n[1], value_n[2]};
}

inline Vector3 legacyForwardRightUpToCanonicalFlu(const Vector3 &legacy) {
  return {legacy[0], -legacy[1], legacy[2]};
}

}  // namespace wheel_leg
