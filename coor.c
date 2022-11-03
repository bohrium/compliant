#include "math.h"

#include "coor.h"

coor_t czero()
{
    return ( coor_t ) {.0, .0, .0};
}

coor_t c_add(coor_t v, coor_t w)
{
    return ( coor_t ){.x = v.x + w.x,
                      .y = v.y + w.y,
                      .z = v.z + w.z };
}

coor_t c_sub(coor_t v, coor_t w)
{
    return ( coor_t ){.x = v.x - w.x,
                      .y = v.y - w.y,
                      .z = v.z - w.z };
}

coor_t scale(float c, coor_t v)
{
    return ( coor_t ){.x = c * v.x,
                      .y = c * v.y,
                      .z = c * v.z };
}


/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~  Metric Structure  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

float norm2(coor_t v)
{
    return v.x*v.x +
           v.y*v.y +
           v.z*v.z ;
}

float dist2(coor_t v, coor_t w)
{
    return norm2(c_sub(w, v));
}

float norm(coor_t v)
{
    return sqrt(norm2(v));
}

float dist(coor_t v, coor_t w)
{
    return norm(c_sub(w, v));
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
/*~~  Dynamics  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/

void reset_forces(cloud_t parts)
{
    for (int i = 0; i != parts.len; ++i) {
        parts.force_accum[i] = czero();
    }
}

void update_pos(float dt, cloud_t parts)
{
    for (int i = 0; i != parts.len; ++i) {
        parts.pos[i] = c_add(parts.pos[i],
                             scale(dt/parts.mass[i], parts.mom[i]));
    }
}

void update_mom(float dt, cloud_t parts)
{
    for (int i = 0; i != parts.len; ++i) {
        parts.mom[i] = c_add(parts.mom[i],
                             scale(dt, parts.force_accum[i]));
    }
}

void sim_springs(cloud_t parts, springs_t springs)
{
    for (int i = 0; i != springs.len; ++i) {
        int src = springs.src[i];
        int dst = springs.dst[i];
        coor_t rel_pos = c_sub(parts.pos[dst], parts.pos[src]);
        float stretch = 1. - ( springs.nat_len[i] / norm(rel_pos) );
        coor_t force = scale(springs.stiffness[i] * stretch, rel_pos);
        parts.force_accum[src] = c_add(parts.force_accum[src], force);
        parts.force_accum[dst] = c_sub(parts.force_accum[dst], force);
    }
}

void sim_drag(cloud_t parts, float viscosity)
{
    for (int i = 0; i != parts.len; ++i) {
        parts.force_accum[i] = c_add(parts.force_accum[i],
                                     scale(-viscosity, parts.mom[i]));
    }
}
