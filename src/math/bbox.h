#pragma once

// BBox*�ࡪ����ͬά�ȵİ�Χ���ࣻ

#include "vec.h"
#include <cfloat>


// ע�⣺
/*
 NOTE on usage of BBoxes
  all BBoxes are initialized so that
      convex(BBox(), bb) == bb
  for any bb
 */



//////////////////////////////////////////////////////////////////////////// ��ά��������Χ����
template<class N>
class BBox2 
{
public: // data
    Vec2<N> minp, maxp;

public: 
    BBox2(const Vec2<N> &minpp, const Vec2<N> &maxpp) :
        minp(minpp), maxp(maxpp) {}

    inline BBox2(const BBox2<N> &bb) : minp(bb.minp), maxp(bb.maxp) {}
    inline BBox2(); // specialized implementations for float/double only
};


template<> inline
BBox2<float>::BBox2() :
    minp( FLT_MAX, FLT_MAX),
    maxp(-FLT_MAX,-FLT_MAX)
{}


template<> inline
BBox2<double>::BBox2() :
    minp( DBL_MAX, DBL_MAX),
    maxp(-DBL_MAX,-DBL_MAX)
{}


template<class N>
inline bool isEmpty(const BBox2<N> &bb) 
{
    return bb.maxp[0] < bb.minp[0] ||
           bb.maxp[1] < bb.minp[1];
}


template<class N>
inline bool isIn(const Vec2<N> &p, const BBox2<N> &bb) {
    return bb.minp[0] <= p[0] && p[0] <= bb.maxp[0] &&
           bb.minp[1] <= p[1] && p[1] <= bb.maxp[1];
}


template<class N>
inline bool hasIsct(const BBox2<N> &lhs, const BBox2<N> &rhs) 
{
    return lhs.minp[0] <= rhs.maxp[0] && lhs.maxp[0] >= rhs.minp[0] &&
           lhs.minp[1] <= rhs.maxp[1] && lhs.maxp[1] >= rhs.minp[1];
}


template<class N>
inline BBox2<N> convex(const BBox2<N> &lhs, const BBox2<N> &rhs) {
    return BBox2<N>(min(lhs.minp, rhs.minp), max(lhs.maxp, rhs.maxp));
}


template<class N>
inline BBox2<N> isct(const BBox2<N> &lhs, const BBox2<N> &rhs) {
    return BBox2<N>(max(lhs.minp, rhs.minp), min(lhs.maxp, rhs.maxp));
}

template<class N>
inline Vec2<N> dim(const BBox2<N> &bb) 
{
    return bb.maxp - bb.minp;
}


template<class N>
inline N perimeter(const BBox2<N> &bb) 
{
    Vec2<N> d = dim(bb);
    return 2*(d[0] + d[1]);
}

template<class N>
inline std::ostream& operator<<(std::ostream &out, const BBox2<N> &bb) 
{
    return out << "[min" << bb.minp << ";max" << bb.maxp << ']';
}


//////////////////////////////////////////////////////////////////////////// ��ά�������Χ����
template<class N>
class BBox3 
{
public: // data
    Vec3<N> minp, maxp;     // minpΪ��Χ�о���Զ��������Ǹ���������ꣻmaxpΪ��Զ���Ǹ���

public:  
    BBox3(const Vec3<N> &minpp, const Vec3<N> &maxpp) :
        minp(minpp), maxp(maxpp) {}

    inline BBox3(const BBox3<N> &bb) : minp(bb.minp), maxp(bb.maxp) {}

    inline BBox3(); // specialized implementations for float/double only
};


template<> inline
BBox3<float>::BBox3() :
    minp( FLT_MAX, FLT_MAX, FLT_MAX),
    maxp(-FLT_MAX,-FLT_MAX,-FLT_MAX)
{}


template<> inline
BBox3<double>::BBox3() :
    minp( DBL_MAX, DBL_MAX, DBL_MAX),
    maxp(-DBL_MAX,-DBL_MAX,-DBL_MAX)
{}

template<class N>
inline bool isEmpty(const BBox3<N> &bb) {
    return bb.maxp[0] < bb.minp[0] ||
           bb.maxp[1] < bb.minp[1] ||
           bb.maxp[2] < bb.minp[2];
}


template<class N>
inline bool isIn(const Vec3<N> &p, const BBox3<N> &bb) 
{
    return bb.minp[0] <= p[0] && p[0] <= bb.maxp[0] &&
           bb.minp[1] <= p[1] && p[1] <= bb.maxp[1] &&
           bb.minp[2] <= p[2] && p[2] <= bb.maxp[2];
}


// hasIsct()�����ж�������Χ���Ƿ��н��棻
template<class N>
inline bool hasIsct(const BBox3<N> &lhs, const BBox3<N> &rhs) 
{
    return lhs.minp[0] <= rhs.maxp[0] && lhs.maxp[0] >= rhs.minp[0] &&
           lhs.minp[1] <= rhs.maxp[1] && lhs.maxp[1] >= rhs.minp[1] &&
           lhs.minp[2] <= rhs.maxp[2] && lhs.maxp[2] >= rhs.minp[2];
}


// convex()�������ذ���������Χ��lhs, rhs����С��Χ�У�
template<class N>
inline BBox3<N> convex(const BBox3<N> &lhs, const BBox3<N> &rhs) 
{
    return BBox3<N>(min(lhs.minp, rhs.minp), max(lhs.maxp, rhs.maxp));
}


// isct()����������������İ�Χ�н���Ĺ������ֹ��ɵİ�Χ�У�
template<class N>
inline BBox3<N> isct(const BBox3<N> &lhs, const BBox3<N> &rhs) 
{
    return BBox3<N>(max(lhs.minp, rhs.minp), min(lhs.maxp, rhs.maxp));
}


// dim()�������ذ�Χ�еĶԽ���������
template<class N>
inline Vec3<N> dim(const BBox3<N> &bb) 
{
    return bb.maxp - bb.minp;
}


// surfaceArea()���������Χ�еı������
template<class N>
inline N surfaceArea(const BBox3<N> &bb) 
{
    Vec3<N> d = dim(bb);
    return 2*(d[1]*d[2] + d[0]*d[2] + d[0]*d[1]);
}


template<class N>
inline std::ostream& operator<<(std::ostream &out, const BBox3<N> &bb) 
{
    return out << "[min" << bb.minp << ";max" << bb.maxp << ']';
}


// multiply()�������Ű�Χ�С���by Tora
template<class N>
inline BBox3<N> multiply(const BBox3<N>& box, const double scale) 
{
    if (scale <= 0)
        return BBox3<N>();

    Vec3<N> n = box.minp;
    Vec3<N> m = box.maxp;
    Vec3<N> center = m + n;
    center.x /= 2;
    center.y /= 2;
    center.z /= 2;
    Vec3<N> arrow = m - center;
    arrow.x *= scale;
    arrow.y *= scale;
    arrow.z *= scale;
    Vec3<N> mm = center + arrow;
    Vec3<N> nn = center - arrow;

    return BBox3<N>(nn, mm);
}



//////////////////////////////////////////////////////////////////////////// ��ά�������Χ����
template<class N>
class BBox4 {
public: // data
    Vec4<N> minp, maxp;
public: // constructors
    BBox4(const Vec4<N> &minpp, const Vec4<N> &maxpp) :
        minp(minpp), maxp(maxpp) {}
    inline BBox4(const BBox4<N> &bb) : minp(bb.minp), maxp(bb.maxp) {}
    inline BBox4();  // specialized implementations for float/double only
};

template<> inline
BBox4<float>::BBox4() :
    minp( FLT_MAX, FLT_MAX, FLT_MAX, FLT_MAX),
    maxp(-FLT_MAX,-FLT_MAX,-FLT_MAX,-FLT_MAX)
{}
template<> inline
BBox4<double>::BBox4() :
    minp( DBL_MAX, DBL_MAX, DBL_MAX, DBL_MAX),
    maxp(-DBL_MAX,-DBL_MAX,-DBL_MAX,-DBL_MAX)
{}

template<class N>
inline bool isEmpty(const BBox4<N> &bb) {
    return bb.maxp[0] < bb.minp[0] ||
           bb.maxp[1] < bb.minp[1] ||
           bb.maxp[2] < bb.minp[2] ||
           bb.maxp[3] < bb.minp[3];
}
template<class N>
inline bool isIn(const Vec4<N> &p, const BBox4<N> &bb) {
    return bb.minp[0] <= p[0] && p[0] <= bb.maxp[0] &&
           bb.minp[1] <= p[1] && p[1] <= bb.maxp[1] &&
           bb.minp[2] <= p[2] && p[2] <= bb.maxp[2] &&
           bb.minp[3] <= p[3] && p[3] <= bb.maxp[3];
}
template<class N>
inline bool hasIsct(const BBox4<N> &lhs, const BBox4<N> &rhs) {
    return lhs.minp[0] <= rhs.maxp[0] && lhs.maxp[0] >= rhs.minp[0] &&
           lhs.minp[1] <= rhs.maxp[1] && lhs.maxp[1] >= rhs.minp[1] &&
           lhs.minp[2] <= rhs.maxp[2] && lhs.maxp[2] >= rhs.minp[2] &&
           lhs.minp[3] <= rhs.maxp[3] && lhs.maxp[3] >= rhs.minp[3];
}
template<class N>
inline BBox4<N> convex(const BBox4<N> &lhs, const BBox4<N> &rhs) {
    return BBox4<N>(min(lhs.minp, rhs.minp), max(lhs.maxp, rhs.maxp));
}
template<class N>
inline BBox4<N> isct(const BBox4<N> &lhs, const BBox4<N> &rhs) {
    return BBox4<N>(max(lhs.minp, rhs.minp), min(lhs.maxp, rhs.maxp));
}

template<class N>
inline Vec4<N> dim(const BBox4<N> &bb) {
    return bb.maxp - bb.minp;
}

template<class N>
inline std::ostream& operator<<(std::ostream &out, const BBox4<N> &bb) {
    return out << "[min" << bb.minp << ";max" << bb.maxp << ']';
}



// define some common useful versions of the boxes
typedef BBox2<float> BBox2f;
typedef BBox3<float> BBox3f;
typedef BBox4<float> BBox4f;

typedef BBox2<double> BBox2d;
typedef BBox3<double> BBox3d;
typedef BBox4<double> BBox4d;


