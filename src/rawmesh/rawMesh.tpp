#pragma once


template<class VertDataOut, class TriDataOut,
         class VertDataIn,  class TriDataIn>
inline RawMesh<VertDataOut,TriDataOut> transduce(
    const RawMesh<VertDataIn,TriDataIn> &input,
    std::function<void(VertDataOut &, const VertDataIn &)> vertTransduce,
    std::function<void(TriDataOut  &, const TriDataIn  &)> triTransduce
) {
    RawMesh<VertDataOut, TriDataOut> output;
    
    uint nVert = input.vertices.size();
    uint nTri  = input.triangles.size();
    output.vertices.resize(nVert);
    output.triangles.resize(nTri);
    
    for(uint i=0; i<nVert; i++)
        vertTransduce(output.vertices[i], input.vertices[i]);
    for(uint i=0; i<nTri; i++)
        triTransduce(output.triangles[i], input.triangles[i]);
    
    return output;
}

