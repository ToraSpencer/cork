#pragma once

// Mesh�ࡪ������ִ�м�����������ͣ�using CorkMesh = Mesh<CorkVertex, CorkTriangle>;

#include "rawMesh.h"
#include "prelude.h"
#include <algorithm>
#include <vector>
#include <set>
#include <queue>
#include <cfloat>
#include <cmath>
#include <sstream>
#include <iostream>
#include <fstream>
#include <memory>


#include "files.h"
#include "vec.h"
#include "ray.h"
#include "shortVec.h"
#include "iterPool.h"

#include "unsafeRayTriIsct.h"
#include "unionFind.h"
#include "bbox.h"
#include "quantization.h"
#include "empty3d.h"
#include "aabvh.h"

using std::cout;
using std::cerr;
using std::endl;
using std::stringstream;
using std::string;
using std::ostream;


#define REAL double

extern "C"
{
#include "triangle.h"
}

/////////////////////////////////////////////////////// ǰ������
struct GenericVertType;
struct IsctVertType;
struct OrigVertType;
struct GenericEdgeType;
struct IsctEdgeType;
struct OrigEdgeType;
struct SplitEdgeType;
struct GenericTriType;
struct GluePointMarker;


// ���½ṹ��ֻ�����ڲ����û�����ʹ�� 
struct TopoVert;
struct TopoEdge;
struct TopoTri;



/////////////////////////////////////////////////////////////////// һ��ģ������������в��ֵ�ʵ�ַ��ڸ���.tpp�ļ��У�
#ifndef uint
typedef unsigned int uint;
#endif


static std::string g_debugPath = "G:/gitRepositories/cork/win/wincork/data/";


// ����IO���������ͣ������������㣻����ʹ�õ���CorkMesh����
/*
    if a mesh is taken as input, the client must manage the memory
     if a mesh is given as output, please use the provided
    function to free the allocated memory.
*/
struct CorkTriMesh
{
    uint    n_triangles;
    uint    n_vertices;
    uint* triangles;
    float* vertices;
};

struct BoolVertexData {};


// ����Ƭ���
struct BoolTriangleData
{
    byte bool_alg_data;                 // ����������mesh1��bool_alg_data���Ϊ0��mesh2�ı��Ϊ1������Ƭ���ָ��ʱ��ı����Ҫ��������
};


template<class VertData, class TriData>
struct IsctVertEdgeTriInput
{
    VertData* e[2];
    VertData* t[3];
};


template<class VertData, class TriData>
struct IsctVertTriTriTriInput
{
    VertData* t[3][3];
};


template<class VertData, class TriData>
struct SubdivideTriInput
{
    TriData* pt;
    VertData* pv[3];
    VertData* v[3];
};


// �ա�������Ƭ�󽻶�����
struct IsctVertexData
{

};


// �ա�������Ƭ������Ƭ��
struct IsctTriangleData
{

};


// �����ػ��ֶ����ࣻ
struct RemeshVertexData
{
    bool manifold;               // ��Ǹö����Ƿ������ε㣻
};


// �ա��������ػ�������Ƭ��
struct RemeshTriangleData
{
};


// �����ػ��ֲ�����
struct RemeshOptions
{
    double maxEdgeLength;
    double minEdgeLength;
    double minAngle;
    double maxAngle;

    RemeshOptions() : maxEdgeLength(1.0), minEdgeLength(0.3), minAngle(5.0), maxAngle(170.0)
    {}
};



// TopoVert�ࡪ�����˶�����
struct TopoVert
{
    uint                    ref;                              // index to actual data
    void* data;                                             // algorithm specific handle ������ò�����󽻹���������洢���Ƕ����Ŷ�֮������ꣿ����
    ShortVec<TopoTri*, 8>       tris;                   // �ö������������Ƭ
    ShortVec<TopoEdge*, 8>       edges;               // �ö�������ı�
};


// TopoEdge�ࡪ�����˱���
struct TopoEdge
{
    void* data;                                     // algorithm specific handle
    TopoVert* verts[2];                         // endpoint vertices
    ShortVec<TopoTri*, 2>       tris;           // incident triangles

    // by Tora��������̨�ϴ�ӡ�ߵ��������㣺
    void printEdge()   const
    {
        std::cout << "(" << verts[0]->ref << ", " << verts[1]->ref << ")" << std::endl;
    }
};


// TopoTri�ࡪ����������Ƭ�࣬��Ŀ�����õ�reinterpret_cast()��TopoTri::dataǿ��ת��ΪtriangleProblem����
struct TopoTri
{
    uint                    ref;                         // index to actual data
    void* data;                                     // ��ͬ������ָ��ͬ�Ķ���������Ƭ����ָ������Ƭ������triangleProblem����
    TopoVert* verts[3];                     // vertices of this triangle
    TopoEdge* edges[3];               // edges of this triangle, opposite to the given vertex
};




// Mesh�ࡪ������ִ�м�����������ͣ�using CorkMesh = Mesh<CorkVertex, CorkTriangle>;
template<class VertData, class TriData>
class Mesh
{
// ֧������
public:
    struct Tri
    {
        TriData data;
        union
        {
            struct
            {
                uint a, b, c; // vertex ids
            };
            uint v[3];
        };

        inline Tri() {}
        inline Tri(const uint a0, const uint b0, const uint c0) : a(a0), b(b0), c(c0) {}
    };

// ǰ������
public:
    class BoolProblem;

// ��������
public:
    std::vector<Tri>        tris;
    std::vector<VertData>   verts;

public:
    Mesh();
    Mesh(Mesh &&src);
    Mesh(const RawMesh<VertData,TriData> &raw);
    virtual ~Mesh();
    
    void operator=(Mesh &&src);

    void meshCopy(const Mesh& mesh);          // by Tora�������������
   
    bool valid() const;      //      �Ϸ��Լ�⡪�� - all numbers are well-defined and finite;  - all triangle vertex indices are in the right range
    
    RawMesh<VertData,TriData> raw() const;
    inline int numVerts() const { return verts.size(); }
    inline int numTris() const { return tris.size(); }
    inline void clear() { this->verts.clear(); this->tris.clear(); }
    void disjointUnion(const Mesh& cp);          // ���������е��������񲻿��ǽ����غϲ���һ�𣬳�ʼ���������ݴ��뵽Mesh�ĳ�Ա����verts��tris�У�


    // ��������Ľ����Χ��
#ifdef TEST_CROSS_BOX_ACCE
    BBox3d boxIsct;
#endif

// ������������
public:
    inline void for_verts(std::function<void(VertData&)> func);
    inline void for_tris(std::function<void(TriData&, VertData&, VertData&, VertData&)> func);
    inline void for_edges(std::function<void(VertData&, VertData&)> start, \
        std::function<void(TriData& t, VertData&, VertData&, VertData&)> each_tri);


// �����������ģ��
public:     
    void boolUnion(Mesh& rhs);
    void boolDiff(Mesh& rhs);
    void boolIsct(Mesh& rhs);
    void boolXor(Mesh& rhs);

    
// ����Ƭ��ģ�顪��ISCT (intersections) 
public:
    struct Isct 
    {
        Ray3d   ray;
        bool    exists;
        uint    tri_id;
        Vec3d   isct;
        Vec3d   bary;
    };
    class  IsctProblem;                              // ����Ƭ�ཻ������������
    class TriangleProblem;                        // support type for IsctProblem
    struct TriangleProblemInfo;


    Isct pick(Ray3d ray);
    inline void accessIsct(const Isct &isct, std::function<void(TriData &, VertData &, VertData &, VertData &)> func);
    bool isClosed();                                                        // checks if the mesh is closed
    void resolveIntersections();         // ����Ƭ�󽻣��ҳ�ÿ���ཻ����Ƭ�Ľ��߶Σ�makes all intersections explicit
    bool isSelfIntersecting();              // is the mesh self-intersecting?
    void testingComputeStaticIsctPoints(std::vector<Vec3d>* points);
    void testingComputeStaticIsct(std::vector<Vec3d>* points,
        std::vector< std::pair<Vec3d, Vec3d> >* edges);


// �ڲ����ģ��
public:    
    inline void merge_tris(uint tid_result, uint tid0, uint tid1);
    inline void split_tris(uint t0ref, uint t1ref, uint t_orig_ref);
    inline void move_tri(Tri &t_new, Tri &t_old);
    inline void subdivide_tri(uint t_piece_ref, uint t_parent_ref);


// ����������Ϣ
public:    
    struct TopoCache;

    struct NeighborEntry 
    {
        uint vid;
        ShortVec<uint, 2> tids;
        inline NeighborEntry() {}
        inline NeighborEntry(uint vid_) : vid(vid_) {}
    };

    struct NeighborCache 
    {
        std::vector< ShortVec<NeighborEntry, 8> > skeleton;
        inline NeighborEntry& operator()(uint i, uint j) 
        {
            uint N = skeleton[i].size();
            for(uint k = 0; k < N; k++)
            {
                if(skeleton[i][k].vid == j)
                    return skeleton[i][k];
            }
            skeleton[i].push_back(NeighborEntry(j));
            return skeleton[i][N];
        }
    };

    // һ������Ϣ��
    template<class Edata>
    struct EGraphEntry
    {
        uint                vid;                        // �ñ�β�����������
        ShortVec<uint, 2>   tids;               // �ñ����ڵ���������Ƭ������
        Edata               data;                       // ʹ��ʱ�����Edata����ΪBoolEdata����ʾ�ñ��Ƿ�������Ƭ�ཻ�ߣ�

        inline EGraphEntry() {}
        inline EGraphEntry(uint vid_) : vid(vid_) {}
    };

    // ����ı���Ϣ
    template<class Edata>
    struct EGraphCache
    {
        std::vector< ShortVec<EGraphEntry<Edata>, 8> > skeleton;            // �������飻skeleton[0]������Ϊ0�Ķ�����������бߣ�

        // ����()�������(i, j)Ϊ���������������������������������㹹�ɵıߣ����ظñ���Ϣ����û�У��ڳ�Ա��������������һ������Ϣ��Ȼ�󷵻أ�
        inline EGraphEntry<Edata> & operator()(uint i, uint j)
        {
            uint N = skeleton[i].size();
            for(uint k = 0; k < N; k++)
            {
                if(skeleton[i][k].vid == j)
                    return skeleton[i][k];
            }
            skeleton[i].push_back(EGraphEntry<Edata>(j));
            return skeleton[i][N];
        }


        // ʹ�ô���ĺ�����action����������ÿһ������Ϣskeleton[i][j];
        inline void for_each(std::function<void(uint i, uint j, EGraphEntry<Edata> &entry)> action)
        {
            for(uint i=0; i<skeleton.size(); i++)
            {
                for(auto &entry : skeleton[i]) 
                    action(i, entry.vid, entry);
            }
        }
    };

    NeighborCache createNeighborCache();

    std::vector<uint> getComponentIds();                        // parallel to vertex array

    template<class Edata>
    EGraphCache<Edata> createEGraphCache();

// �����ػ���ģ��
public:
    RemeshOptions remesh_options;
    struct RemeshScratchpad;
    void remesh();  // �����ػ��֣� REQUIRES:  - MinimalData  - RemeshData

    TopoEdge* allocateRemeshEdge(RemeshScratchpad &);
    void deallocateRemeshEdge(RemeshScratchpad &, TopoEdge*);
    
    void edgeSplit(RemeshScratchpad &,  TopoEdge* e_split);
    void edgeCollapse(RemeshScratchpad &,  TopoEdge* e_collapse,  bool collapsing_tetrahedra_disappear);
    
    // Need edge scoring routines...
    void scoreAndEnqueue(std::set< std::pair<double, TopoEdge*> > &queue, TopoEdge* edge);
    void dequeue(std::set< std::pair<double, TopoEdge*> > &queue, TopoEdge* edge);
    double computeEdgeScore(TopoEdge* edge);
    
    // support functions
    void populateTriFromTopoTri(TopoTri* t);

    // calls the first function once, then the second once for each triangle
    inline void edgeNeighborhood(TopoEdge* edge, std::function<void(VertData &v0, VertData &v1)> once, \
        std::function<void(VertData &v0, VertData &v1, VertData &vopp, TriData &t)> each_tri);
};



/////////////////////////////////////////////////////////////////////////////// inline����ʵ�֣�

// ��������
template<class VertData, class TriData>
inline void Mesh<VertData,TriData>::for_verts(std::function<void(VertData &v)> func)
{
    for(auto &v : verts)
        func(v);
}


// ��������Ƭ
template<class VertData, class TriData>
inline void Mesh<VertData,TriData>::for_tris(std::function<void(TriData &, VertData &, VertData &, VertData &)> func) 
{
    for(auto &tri : tris) 
    {
        auto &a = verts[tri.a];
        auto &b = verts[tri.b];
        auto &c = verts[tri.c];
        func(tri.data, a, b, c);
    }
}


// ������
template<class VertData, class TriData>
inline void Mesh<VertData,TriData>::for_edges(std::function<void(VertData &, VertData &)> start, \
    std::function<void(TriData &t, VertData &, VertData &, VertData &)> each_tri) 
{
    NeighborCache cache = createNeighborCache();
    for(uint i=0; i<cache.skeleton.size(); i++) 
    {
        for(auto &entry : cache.skeleton[i]) 
        {
            uint j = entry.vid;
            start(verts[i], verts[j]);
            for(uint tid : entry.tids) 
            {
                Tri &tri = tris[tid];
                each_tri(tri.data, verts[tri.a], verts[tri.b], verts[tri.c]);
            }
        }
    }
}



template<class VertData, class TriData>
inline void Mesh<VertData,TriData>::accessIsct(const Isct &isct,  std::function<void(TriData &, VertData &, VertData &, VertData &)> func)
{
    Tri &tri = tris[isct.tri_id];
    auto &a = verts[tri.a];
    auto &b = verts[tri.b];
    auto &c = verts[tri.c];
    func(tri.data, a, b, c);
}

 


///////////////////////////////////////////////////////////////////////////////////////////////////// Mesh<>ģ����ػ���

// ǰ������
class CorkTriangle;


// CorkVertex����������
class CorkVertex :
    public MinimalVertexData,
    public RemeshVertexData,
    public IsctVertexData,
    public BoolVertexData
{
public:
    CorkVertex() {}
    CorkVertex(const double x, const double y, const double z)
    {
        this->pos.x = x;
        this->pos.y = y;
        this->pos.z = z;
    }
    CorkVertex(const Vec3<double>& pos0)
    {
        this->pos = pos0;
    }
    ~CorkVertex() {};

    void merge(const CorkVertex& v0, const CorkVertex& v1)
    {
        double                              a0 = 0.5;
        if (v0.manifold && !v1.manifold)     a0 = 0.0;
        if (!v0.manifold && v1.manifold)     a0 = 1.0;
        double a1 = 1.0 - a0;

        pos = a0 * v0.pos + a1 * v1.pos;
    }


    void interpolate(const CorkVertex& v0, const CorkVertex& v1)
    {
        double a0 = 0.5;
        double a1 = 0.5;
        pos = a0 * v0.pos + a1 * v1.pos;
    }


    void isct(IsctVertEdgeTriInput<CorkVertex, CorkTriangle> input)
    {
        Vec2d       a_e = Vec2d(1, 1) / 2.0;
        Vec3d       a_t = Vec3d(1, 1, 1) / 3.0;
        a_e /= 2.0;
        a_t /= 2.0;
    }


    void isct(IsctVertTriTriTriInput<CorkVertex, CorkTriangle> input)
    {
        Vec3d       a[3];
        for (uint k = 0; k < 3; k++) {
            a[k] = Vec3d(1, 1, 1) / 3.0;
            a[k] /= 3.0;
        }
        for (uint i = 0; i < 3; i++) {
            for (uint j = 0; j < 3; j++) {
            }
        }
    }


    void isctInterpolate(const CorkVertex& v0, const CorkVertex& v1)
    {
        double a0 = len(v1.pos - pos);
        double a1 = len(v0.pos - pos);
        if (a0 + a1 == 0.0) a0 = a1 = 0.5; // safety
        double sum = a0 + a1;
        a0 /= sum;
        a1 /= sum;
    }

    void disp() 
    {
        std::cout << "(" << this->pos.x << ", " << this->pos.y <<", " << this->pos.z << ")" << std::endl;
    }
};


// CorkTriangle�ࡪ������Ƭ�ࣻ
class CorkTriangle :
    public MinimalTriangleData,
    public RemeshTriangleData,
    public IsctTriangleData,
    public BoolTriangleData
{
public:
    CorkTriangle() {};
    CorkTriangle(const int a0, const int b0, const int c0)
    {
        this->a = a0;
        this->b = b0;
        this->c = c0;
    }
    ~CorkTriangle() {}

    void merge(const CorkTriangle&, const CorkTriangle&) {}

    static void split(CorkTriangle&, CorkTriangle&,
        const CorkTriangle&) {}

    void move(const CorkTriangle&) {}

    void subdivide(SubdivideTriInput<CorkVertex, CorkTriangle> input)
    {
        bool_alg_data = input.pt->bool_alg_data;
    }
};

using RawCorkMesh = RawMesh<CorkVertex, CorkTriangle>;
using CorkMesh = Mesh<CorkVertex, CorkTriangle>;                                // �������ʱʹ�õ������ࣻ

using edgeInfo = std::tuple<unsigned, unsigned, std::pair<unsigned, unsigned>, bool>;    // ���׶��㡢��β���㡢tids���Ƿ�������Ƭ���ߣ�����by Tora
 



///////////////////////////////////////////////////////////////////////////////////////////////////////// һЩIO�ӿ�������
void file2corktrimesh(const Files::FileMesh& in, CorkTriMesh* out);
void corktrimesh2file(CorkTriMesh in, Files::FileMesh& out);
void loadMesh(string filename, CorkTriMesh* out);
void saveMesh(string filename, CorkTriMesh in);
void freeCorkTriMesh(CorkTriMesh* mesh);        // �ͷ�CorkTriMesh����Ķ��ڴ棻



/////////////////////////////////////////////////////////////////////////////////////////// by Tora����һЩdebugʱ�õ����ͣ�
struct TopoVertInfo
{
    uint                    ref;
    void* data;
    std::vector<TopoTri>       tris_rela;                         // �ö������������Ƭ
    std::vector<TopoEdge>       edges_rela;               // �ö�������ı�

    TopoVertInfo() {}
    TopoVertInfo(const TopoVert& tv)
    {
        this->ref = tv.ref;
        this->data = tv.data;

        this->tris_rela.reserve(tv.tris.user_size);
        this->edges_rela.reserve(tv.edges.user_size);
        for (unsigned i = 0; i < tv.tris.user_size; ++i)
            this->tris_rela.push_back(*tv.tris[i]);
        for (unsigned i = 0; i < tv.edges.user_size; ++i)
            this->edges_rela.push_back(*tv.edges[i]);
    }
};


struct TopoEdgeInfo
{
    void* data;
    std::vector<TopoVert>   verts_rela;                      // ԭ�������Ϊnullptr�򲻿��룻   
    std::vector<TopoTri>       tris_rela;

    TopoEdgeInfo() {}
    TopoEdgeInfo(const TopoEdge& te)
    {
        this->data = te.data;
        this->verts_rela.reserve(2);
        this->tris_rela.reserve(te.tris.user_size);
        for (unsigned i = 0; i < te.tris.user_size; ++i)
            this->tris_rela.push_back(*te.tris[i]);
        for (unsigned i = 0; i < 2; ++i)
        {
            if (nullptr != te.verts[i])
                this->verts_rela.push_back(*te.verts[i]);
        }
    }
};


struct TopoTriInfo
{
    uint ref;
    void* data;
    std::vector<TopoVert> verts_rela;        // ԭ�������Ϊnullptr�򲻿��룻   
    std::vector<TopoEdge> edges_rela;     // ԭ�������Ϊnullptr�򲻿��룻   

    TopoTriInfo() {}

    TopoTriInfo(const TopoTri& tt)
    {
        this->ref = tt.ref;
        this->data = tt.data;
        for (unsigned i = 0; i < 3; ++i)
        {
            if (nullptr != tt.verts[i])
                this->verts_rela.push_back(*tt.verts[i]);
            if (nullptr != tt.edges[i])
                this->edges_rela.push_back(*tt.edges[i]);
        }
    }
};

// �����������
class CorkMeshCombainer :public CorkMesh
{
public:
    CorkMeshCombainer() :CorkMesh() {}
    ~CorkMeshCombainer() {}

    void add(const CorkMesh& mesh)
    {
        unsigned versCountOri = this->verts.size();

        // ���������񶥵㣺
        this->verts.insert(verts.end(), mesh.verts.begin(), mesh.verts.end());

        // ����������Ƭ���޸ģ�����
        auto newTris = mesh.tris;
        for (auto& tri : newTris)
        {
            tri.a += versCountOri;
            tri.b += versCountOri;
            tri.c += versCountOri;
        }
        this->tris.insert(tris.end(), newTris.begin(), newTris.end());
    }
};


class CorkEdge
{
public:
    CorkVertex va;
    CorkVertex vb;

public:
    CorkEdge() {}
    CorkEdge(const CorkVertex& va0, const CorkVertex& vb0) : va(va0), vb(vb0)
    {}
};


class CorkEdges
{
public:
    std::vector<CorkEdge> edgeVec;

public:
    CorkEdges() {}
    CorkEdges(const std::vector<CorkEdge>& edgeVec0) : edgeVec(edgeVec0)
    {    }

    void add(const CorkEdge& e)
    {
        this->edgeVec.push_back(e);
    }
};


///////////////////////////////////////////////////////////////////////////////////////////////////////// ���������Ľӿڣ�
bool isSolid(CorkTriMesh mesh);
void computeUnion(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void computeDifference(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void computeIntersection(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void computeSymmetricDifference(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void resolveIntersections(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);


///////////////////////////////////////////////////////////////////////////////////////////////  by Tora����debug�ࡢ�ӿ�������
void OBJWriteMesh(const char* pszFileName, const CorkMesh& mesh);
void OBJWriteVers(const char* pszFileName, const std::vector<CorkVertex>& vers);
void OBJWriteVersEdges(const char* pszFileName, const std::vector<CorkVertex>& vers, \
    const std::vector<std::pair<unsigned, unsigned>>& edges);              // ���������ݵ�OBJ�ļ���
void OBJWriteVersEdges(const char* pszFileName, const std::vector<edgeInfo>& eInfos, \
    const CorkMesh& mesh);
void OBJWriteVersEdges(const char* pszFileName, const std::vector<TopoEdge>& tpEdges, const CorkMesh& mesh);


void debugWriteMesh(const char* fileName, const CorkTriMesh& mesh);
void debugWriteMesh(const char* fileName, CorkMesh& mesh);
void debugWriteTriMesh(const char* fileName, const CorkMesh& mesh, const unsigned triIdx);

void debugWriteVers(const char* fileName, std::vector<CorkVertex>& vers);

void debugWriteEdge(const char* fileName, const CorkEdge& e);
void debugWriteEdge(const char* fileName, const CorkMesh& mesh, const TopoEdge& te);
void debugWriteEdge(const char* fileName, const CorkVertex& ver1, const CorkVertex& ver2);
void debugWriteEdges(const char* fileName, const CorkEdges& es);
void debugWriteEdges(const char* fileName, std::vector<CorkVertex>& vers, \
    const std::vector<std::pair<unsigned, unsigned>>& edges);
void debugWriteEdges(const char* fileName, const std::vector<edgeInfo>& eInfos, \
    const CorkMesh& mesh);
void debugWriteEdges(const char* fileName, const std::vector<TopoEdge>& tpEdges, const CorkMesh& mesh);


/////////////////////////////////////////////////////////////////////////////////////////////// by Tora����support�ӿ�������
bool myIsInside(const CorkVertex& va, const CorkVertex& vb, const CorkVertex& vc, const CorkMesh& mesh);
void selectedTris2Mesh(CorkMesh& newMesh, const CorkMesh& oriMesh, \
    const std::vector<unsigned>& selectedTriIdx);                   // ����ѡȡ������Ƭ�����������Ӧ��������
void corkTriMesh2CorkMesh(CorkTriMesh in, CorkMesh* mesh_out);
void corkMesh2CorkTriMesh(CorkMesh* mesh_in, CorkTriMesh* out);
void drawCube(CorkMesh& cubeMesh, const double xlen, const double ylen, const double zlen, \
    const CorkVertex& center = CorkVertex(0, 0, 0));                 // �������������񣬰˸����㣻 


BBox3d getAABB(TopoEdge* e, const CorkMesh& mesh);
BBox3d getAABB(TopoTri* t, const CorkMesh& mesh);
BBox3d getAABB(const CorkMesh& mesh);
GeomBlob<TopoEdge*> getEdgeBlob(TopoEdge* e, const CorkMesh& mesh);
void getAABBmesh(CorkMesh& meshOut, const BBox3d& bbox);

template<typename   T>
bool isIn(const std::set<T*>& set, T* elemPtr) 
{
    auto iter = set.find(elemPtr);
    if (iter == set.end())
        return false;
    else
        return true;
}



///////////////////////////////////////////////////////////////////////////////////////////////////// һЩdebug�õ����ݣ�

// �Զ����ʱ����ʹ��WINDOWS��ʱAPI
class tiktok
{
private:
    tiktok() = default;
    tiktok(const tiktok&) {}
    ~tiktok() = default;

public:
    DWORD startTik;
    DWORD endTik;
    unsigned recordCount;
    std::vector<DWORD> records;

    static tiktok& getInstance()
    {
        static tiktok tt_instance;
        return tt_instance;
    }

    void start()
    {
        this->startTik = GetTickCount();
        this->recordCount = 0;
        this->records.clear();
    }

    void endCout(const char* str)
    {
        this->endTik = GetTickCount();
        std::cout << str << endTik - startTik << std::endl;
    }

    bool endWrite(const char* fileName, const char* str)
    {
        this->endTik = GetTickCount();
        std::ofstream file(fileName, std::ios_base::out | std::ios_base::app);
        if (!file)
            return false;

        file << str << endTik - startTik << std::endl;
        file.close();
        return true;
    }

    void takeArecord()
    {
        this->records.push_back(GetTickCount());
        recordCount++;
    }
};

