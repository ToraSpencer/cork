#pragma once

#include "vec.h"

namespace Empty3d 
{

struct TriIn
{
    Vec3d p[3];
};

struct EdgeIn
{
    Vec3d p[2];
};


struct TriEdgeIn
{
    TriIn   tri;
    EdgeIn  edge;
};

bool isEmpty(const TriEdgeIn &input);
Vec3d coords(const TriEdgeIn &input);
bool emptyExact(const TriEdgeIn &input);
Vec3d coordsExact(const TriEdgeIn &input);

struct TriTriTriIn
{
    TriIn tri[3];
};

bool isEmpty(const TriTriTriIn &input);
Vec3d coords(const TriTriTriIn &input);
bool emptyExact(const TriTriTriIn &input);
Vec3d coordsExact(const TriTriTriIn &input);

extern int degeneracy_count;            // count degeneracies encountered
extern int exact_count;                     // count of filter calls failed
extern int callcount;           // total call count


/*
// exact versions


bool emptyExact(const Cell3d0 &c0,
                const Complex3d2 &complex,
                const Metric3d2 &metric);

void cell3d0toPointExact(SmVector3 &point,
                         const Cell3d0 &c0,
                         const Complex3d2 &complex,
                         const Metric3d2 &metric);
*/
}  


