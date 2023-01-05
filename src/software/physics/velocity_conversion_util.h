#pragma once
#include "software/geom/vector.h"

/**
 *
 * @param global_vector
 * @param global_orientation
 * @return
 */
inline Vector globalToLocal(const Vector &global_vector, const Angle &global_orientation)
{
    return global_vector.rotate(-global_orientation);
}

inline Vector localToGlobal(const Vector &global_vector, const Angle &global_orientation)
{
    return global_vector.rotate(global_orientation);
}