#pragma once

#include <functional>
#include "vec.h"
#include <vector>


// vertex and triangle data must satisfy the following minimal specifications! These exact field names must be used!
struct MinimalVertexData
{
    Vec3d pos;      // required for both RawMesh and Mesh
};


struct MinimalTriangleData
{
    int a, b, c;
};


// RawMesh��ģ��
template<class VertData, class TriData>
struct RawMesh
{
    std::vector<VertData> vertices;
    std::vector<TriData> triangles;
};



// ��ͬ���͵�RawMesh�����ת����
template<class VertDataOut, class TriDataOut,  class VertDataIn,  class TriDataIn>
inline RawMesh<VertDataOut,TriDataOut> transduce(const RawMesh<VertDataIn,TriDataIn> &input, \
    std::function<void(VertDataOut &, const VertDataIn &)> vertTransduce,\
    std::function<void(TriDataOut  &, const TriDataIn  &)> triTransduce
);


#include "rawMesh.tpp"
