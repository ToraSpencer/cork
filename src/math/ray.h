#pragma once

// Ray3�ࡪ�����ߣ�

#include "vec.h"



template <class N>
struct Ray3
{
    Vec3<N> p; // point of origin
    Vec3<N> r; // ray direction
    
    Ray3<N>() {}
    Ray3<N>(const Vec3<N> &point, const Vec3<N> &dir) :
        p(point), r(dir)
    {}
    template<class T>
    Ray3<N>(const Ray3<T> &cp) : p(cp.p), r(cp.r) {}
};


template<class N>
inline std::ostream& operator<<(std::ostream &out, const Ray3<N> &ray) {
    return out << '[' << ray.p << ';' << ray.r << ']';
}


typedef Ray3<float> Ray3f;
typedef Ray3<double> Ray3d;

