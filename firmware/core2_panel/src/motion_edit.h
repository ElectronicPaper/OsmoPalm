#pragma once
#include "motion_program.h"
namespace osmo {
// A delete keeps later frames and their incoming transition settings.
// The existing clear-tail operation remains a separate explicit action.
inline bool eraseMotionPoint(MotionProgram &program, uint8_t index) {
    if (program.count > MOTION_POINT_CAPACITY || index >= program.count) return false;
    for (uint8_t i=index; i+1<program.count; ++i) program.points[i]=program.points[i+1];
    program.points[--program.count]=MotionPoint{};
    return true;
}
}
