#pragma once

// Mesh类——用于执行计算的网格类型；using CorkMesh = Mesh<CorkVertex, CorkTriangle>;

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

/////////////////////////////////////////////////////// 前置声明
struct GenericVertType;
struct IsctVertType;
struct OrigVertType;
struct GenericEdgeType;
struct IsctEdgeType;
struct OrigEdgeType;
struct SplitEdgeType;
struct GenericTriType;
struct GluePointMarker;


// 以下结构体只用于内部，用户请勿使用 
struct TopoVert;
struct TopoEdge;
struct TopoTri;



/////////////////////////////////////////////////////////////////// 一堆模板的声明；其中部分的实现放在各个.tpp文件中；
#ifndef uint
typedef unsigned int uint;
#endif


static std::string g_debugPath = "G:/gitRepositories/cork/win/wincork/data/";


// 用于IO的网格类型，不能用于运算；运算使用的是CorkMesh类型
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


// 三角片标记
struct BoolTriangleData
{
    byte bool_alg_data;                 // 布尔运算中mesh1的bool_alg_data标记为0，mesh2的标记为1；三角片被分割的时候改标记需要被拷贝；
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


// 空——三角片求交顶点类
struct IsctVertexData
{

};


// 空——三角片求交三角片类
struct IsctTriangleData
{

};


// 网格重划分顶点类；
struct RemeshVertexData
{
    bool manifold;               // 标记该顶点是否是流形点；
};


// 空——网格重划分三角片类
struct RemeshTriangleData
{
};


// 网格重划分参数；
struct RemeshOptions
{
    double maxEdgeLength;
    double minEdgeLength;
    double minAngle;
    double maxAngle;

    RemeshOptions() : maxEdgeLength(1.0), minEdgeLength(0.3), minAngle(5.0), maxAngle(170.0)
    {}
};



// TopoVert类——拓扑顶点类
struct TopoVert
{
    uint                    ref;                              // index to actual data
    void* data;                                             // algorithm specific handle ？？？貌似在求交过程中这里存储的是顶点扰动之后的坐标？？？
    ShortVec<TopoTri*, 8>       tris;                   // 该顶点关联的三角片
    ShortVec<TopoEdge*, 8>       edges;               // 该顶点关联的边
};


// TopoEdge类——拓扑边类
struct TopoEdge
{
    void* data;                                     // algorithm specific handle
    TopoVert* verts[2];                         // endpoint vertices
    ShortVec<TopoTri*, 2>       tris;           // incident triangles

    // by Tora——控制台上打印边的两个顶点：
    void printEdge()   const
    {
        std::cout << "(" << verts[0]->ref << ", " << verts[1]->ref << ")" << std::endl;
    }
};


// TopoTri类——拓扑三角片类，项目中有用到reinterpret_cast()将TopoTri::data强制转换为triangleProblem对象；
struct TopoTri
{
    uint                    ref;                         // index to actual data
    void* data;                                     // 不同程序中指向不同的对象，如三角片求交中指向本三角片关联的triangleProblem对象；
    TopoVert* verts[3];                     // vertices of this triangle
    TopoEdge* edges[3];               // edges of this triangle, opposite to the given vertex
};




// Mesh类——用于执行计算的网格类型；using CorkMesh = Mesh<CorkVertex, CorkTriangle>;
template<class VertData, class TriData>
class Mesh
{
// 支持类型
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

// 前置申明
public:
    class BoolProblem;

// 网格数据
public:
    std::vector<Tri>        tris;
    std::vector<VertData>   verts;

public:
    Mesh();
    Mesh(Mesh &&src);
    Mesh(const RawMesh<VertData,TriData> &raw);
    virtual ~Mesh();
    
    void operator=(Mesh &&src);

    void meshCopy(const Mesh& mesh);          // by Tora——网格深拷贝：
   
    bool valid() const;      //      合法性检测—— - all numbers are well-defined and finite;  - all triangle vertex indices are in the right range
    
    RawMesh<VertData,TriData> raw() const;
    inline int numVerts() const { return verts.size(); }
    inline int numTris() const { return tris.size(); }
    inline void clear() { this->verts.clear(); this->tris.clear(); }
    void disjointUnion(const Mesh& cp);          // 布尔运算中的两个网格不考虑交互地合并到一起，初始操作；数据存入到Mesh的成员数据verts和tris中；


    // 两个网格的交叉包围盒
#ifdef TEST_CROSS_BOX_ACCE
    BBox3d boxIsct;
#endif

// 遍历网格数据
public:
    inline void for_verts(std::function<void(VertData&)> func);
    inline void for_tris(std::function<void(TriData&, VertData&, VertData&, VertData&)> func);
    inline void for_edges(std::function<void(VertData&, VertData&)> start, \
        std::function<void(TriData& t, VertData&, VertData&, VertData&)> each_tri);


// 布尔运算操作模块
public:     
    void boolUnion(Mesh& rhs);
    void boolDiff(Mesh& rhs);
    void boolIsct(Mesh& rhs);
    void boolXor(Mesh& rhs);

    
// 三角片求交模块——ISCT (intersections) 
public:
    struct Isct 
    {
        Ray3d   ray;
        bool    exists;
        uint    tri_id;
        Vec3d   isct;
        Vec3d   bary;
    };
    class  IsctProblem;                              // 三角片相交问题的求解器；
    class TriangleProblem;                        // support type for IsctProblem
    struct TriangleProblemInfo;


    Isct pick(Ray3d ray);
    inline void accessIsct(const Isct &isct, std::function<void(TriData &, VertData &, VertData &, VertData &)> func);
    bool isClosed();                                                        // checks if the mesh is closed
    void resolveIntersections();         // 三角片求交，找出每对相交三角片的交线段；makes all intersections explicit
    bool isSelfIntersecting();              // is the mesh self-intersecting?
    void testingComputeStaticIsctPoints(std::vector<Vec3d>* points);
    void testingComputeStaticIsct(std::vector<Vec3d>* points,
        std::vector< std::pair<Vec3d, Vec3d> >* edges);


// 内部设计模块
public:    
    inline void merge_tris(uint tid_result, uint tid0, uint tid1);
    inline void split_tris(uint t0ref, uint t1ref, uint t_orig_ref);
    inline void move_tri(Tri &t_new, Tri &t_old);
    inline void subdivide_tri(uint t_piece_ref, uint t_parent_ref);


// 网格拓扑信息
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

    // 一条边信息；
    template<class Edata>
    struct EGraphEntry
    {
        uint                vid;                        // 该边尾顶点的索引；
        ShortVec<uint, 2>   tids;               // 该边相邻的两个三角片索引；
        Edata               data;                       // 使用时这里的Edata类型为BoolEdata，表示该边是否是三角片相交边；

        inline EGraphEntry() {}
        inline EGraphEntry(uint vid_) : vid(vid_) {}
    };

    // 网格的边信息
    template<class Edata>
    struct EGraphCache
    {
        std::vector< ShortVec<EGraphEntry<Edata>, 8> > skeleton;            // 二重数组；skeleton[0]是索引为0的顶点关联的所有边；

        // 重载()运算符；(i, j)为输入的两个顶点的索引，若有这两个顶点构成的边，返回该边信息；若没有，在成员数据中生成这样一条边信息，然后返回；
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


        // 使用传入的函数子action遍历作用于每一条边信息skeleton[i][j];
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

// 网格重划分模块
public:
    RemeshOptions remesh_options;
    struct RemeshScratchpad;
    void remesh();  // 网格重划分； REQUIRES:  - MinimalData  - RemeshData

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



/////////////////////////////////////////////////////////////////////////////// inline函数实现：

// 遍历顶点
template<class VertData, class TriData>
inline void Mesh<VertData,TriData>::for_verts(std::function<void(VertData &v)> func)
{
    for(auto &v : verts)
        func(v);
}


// 遍历三角片
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


// 遍历边
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

 


///////////////////////////////////////////////////////////////////////////////////////////////////// Mesh<>模板的特化：

// 前置声明
class CorkTriangle;


// CorkVertex——顶点类
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


// CorkTriangle类——三角片类；
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
using CorkMesh = Mesh<CorkVertex, CorkTriangle>;                                // 具体计算时使用的网格类；

using edgeInfo = std::tuple<unsigned, unsigned, std::pair<unsigned, unsigned>, bool>;    // 边首顶点、边尾顶点、tids、是否是三角片交线；——by Tora
 



///////////////////////////////////////////////////////////////////////////////////////////////////////// 一些IO接口声明：
void file2corktrimesh(const Files::FileMesh& in, CorkTriMesh* out);
void corktrimesh2file(CorkTriMesh in, Files::FileMesh& out);
void loadMesh(string filename, CorkTriMesh* out);
void saveMesh(string filename, CorkTriMesh in);
void freeCorkTriMesh(CorkTriMesh* mesh);        // 释放CorkTriMesh对象的堆内存；



/////////////////////////////////////////////////////////////////////////////////////////// by Tora——一些debug时用的类型：
struct TopoVertInfo
{
    uint                    ref;
    void* data;
    std::vector<TopoTri>       tris_rela;                         // 该顶点关联的三角片
    std::vector<TopoEdge>       edges_rela;               // 该顶点关联的边

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
    std::vector<TopoVert>   verts_rela;                      // 原数据如果为nullptr则不拷入；   
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
    std::vector<TopoVert> verts_rela;        // 原数据如果为nullptr则不拷入；   
    std::vector<TopoEdge> edges_rela;     // 原数据如果为nullptr则不拷入；   

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

// 网格组合器：
class CorkMeshCombainer :public CorkMesh
{
public:
    CorkMeshCombainer() :CorkMesh() {}
    ~CorkMeshCombainer() {}

    void add(const CorkMesh& mesh)
    {
        unsigned versCountOri = this->verts.size();

        // 插入新网格顶点：
        this->verts.insert(verts.end(), mesh.verts.begin(), mesh.verts.end());

        // 拷贝新三角片，修改，插入
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


///////////////////////////////////////////////////////////////////////////////////////////////////////// 布尔操作的接口：
bool isSolid(CorkTriMesh mesh);
void computeUnion(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void computeDifference(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void computeIntersection(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void computeSymmetricDifference(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);
void resolveIntersections(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out);


///////////////////////////////////////////////////////////////////////////////////////////////  by Tora——debug类、接口声明：
void OBJWriteMesh(const char* pszFileName, const CorkMesh& mesh);
void OBJWriteVers(const char* pszFileName, const std::vector<CorkVertex>& vers);
void OBJWriteVersEdges(const char* pszFileName, const std::vector<CorkVertex>& vers, \
    const std::vector<std::pair<unsigned, unsigned>>& edges);              // 输出点边数据到OBJ文件中
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


/////////////////////////////////////////////////////////////////////////////////////////////// by Tora——support接口声明：
bool myIsInside(const CorkVertex& va, const CorkVertex& vb, const CorkVertex& vc, const CorkMesh& mesh);
void selectedTris2Mesh(CorkMesh& newMesh, const CorkMesh& oriMesh, \
    const std::vector<unsigned>& selectedTriIdx);                   // 输入选取的三角片索引，输出对应的新网格；
void corkTriMesh2CorkMesh(CorkTriMesh in, CorkMesh* mesh_out);
void corkMesh2CorkTriMesh(CorkMesh* mesh_in, CorkTriMesh* out);
void drawCube(CorkMesh& cubeMesh, const double xlen, const double ylen, const double zlen, \
    const CorkVertex& center = CorkVertex(0, 0, 0));                 // 生成立方体网格，八个顶点； 


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



///////////////////////////////////////////////////////////////////////////////////////////////////// 一些debug用的内容：

// 自定义计时器，使用WINDOWS计时API
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

