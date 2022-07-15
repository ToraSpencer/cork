#pragma once

////////////////////////////////////////////////////////////////////////////////////////Mesh<>类模板的接口实现；

// constructors
template<class VertData, class TriData>
Mesh<VertData, TriData>::Mesh() {}


template<class VertData, class TriData>
Mesh<VertData, TriData>::Mesh(Mesh&& cp)
    : tris(cp.tris), verts(cp.verts)
{}


template<class VertData, class TriData>
Mesh<VertData, TriData>::Mesh(const RawMesh<VertData, TriData>& raw) :
    tris(raw.triangles.size()), verts(raw.vertices)
{
    // fill out the triangles
    for (uint i = 0; i < raw.triangles.size(); i++) 
    {
        tris[i].data = raw.triangles[i];
        tris[i].a = raw.triangles[i].a;
        tris[i].b = raw.triangles[i].b;
        tris[i].c = raw.triangles[i].c;
    }
}


template<class VertData, class TriData>
Mesh<VertData, TriData>::~Mesh() {}


template<class VertData, class TriData>
void Mesh<VertData, TriData>::operator=(Mesh&& src)
{
    tris = src.tris;
    verts = src.verts;
}


// by Tora——网格深拷贝：
template<class VertData, class TriData>
void Mesh<VertData, TriData>::meshCopy(const Mesh& mesh)
{
    this->remesh_options = mesh.remesh_options;
    this->verts = mesh.verts;
    this->tris = mesh.tris;
}


template<class VertData, class TriData>
bool Mesh<VertData, TriData>::valid() const
{
    for (uint i = 0; i < verts.size(); i++)
    {
        if (!std::isfinite(verts[i].pos.x) ||
            !std::isfinite(verts[i].pos.y) ||
            !std::isfinite(verts[i].pos.z)) {
            std::ostringstream message;
            message << "vertex #" << i << " has non-finite coordinates: "
                << verts[i].pos;
            CORK_ERROR(message.str());
            return false;
        }
    }

    for (uint i = 0; i < tris.size(); i++)
    {
        if (tris[i].a >= verts.size() ||
            tris[i].b >= verts.size() ||
            tris[i].c >= verts.size()) {
            std::ostringstream message;
            message << "triangle #" << i << " should have indices in "
                << "the range 0 to " << (verts.size() - 1)
                << ", but it has invalid indices: "
                << tris[i].a << ", " << tris[i].b << ", " << tris[i].c;
            CORK_ERROR(message.str());
            return false;
        }
    }

    return true;
}


template<class VertData, class TriData>
RawMesh<VertData, TriData> Mesh<VertData, TriData>::raw() const
{
    RawMesh<VertData, TriData> result;
    result.vertices = verts;
    result.triangles.resize(tris.size());
    for (uint i = 0; i < tris.size(); i++) 
    {
        result.triangles[i] = tris[i].data;
        result.triangles[i].a = tris[i].a;
        result.triangles[i].b = tris[i].b;
        result.triangles[i].c = tris[i].c;
    }

    return result;
}


// 布尔运算中的两个网格不考虑交互地合并到一起，初始操作；数据存入到Mesh的成员数据verts和tris中；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::disjointUnion(const Mesh& cp)
{
    uint oldVsize = verts.size();
    uint oldTsize = tris.size();
    uint cpVsize = cp.verts.size();
    uint cpTsize = cp.tris.size();
    uint newVsize = oldVsize + cpVsize;
    uint newTsize = oldTsize + cpTsize;

    std::vector<int> v_remap(cpVsize);          // oh this is obvious...
    verts.resize(newVsize);
    tris.resize(newTsize);

    for (uint i = 0; i < cpVsize; i++)
        verts[oldVsize + i] = cp.verts[i];

    for (uint i = 0; i < cpTsize; i++)
    {
        auto& tri = tris[oldTsize + i];
        tri = cp.tris[i];
        tri.a += oldVsize;
        tri.b += oldVsize;
        tri.c += oldVsize;
    }
}


// Picking.
// Dumb Implementation just passes over all triangles w/o any precomputed
// acceleration structure
template<class VertData, class TriData>
typename Mesh<VertData, TriData>::Isct
Mesh<VertData, TriData>::pick(Ray3d ray)
{
    Isct result;
    result.ray = ray;
    result.exists = false;

    double mint = DBL_MAX;

    // pass all triangles over ray
    for (uint i = 0; i < tris.size(); i++) 
    {
        const Tri& tri = tris[i];

        uint   a = tri.a;
        uint   b = tri.b;
        uint   c = tri.c;
        Vec3d va = verts[a].pos;
        Vec3d vb = verts[b].pos;
        Vec3d vc = verts[c].pos;

        // normalize vertex order (to prevent leaks)
        if (a > b) { std::swap(a, b); std::swap(va, vb); }
        if (b > c) { std::swap(b, c); std::swap(vb, vc); }
        if (a > b) { std::swap(a, b); std::swap(va, vb); }

        double t;
        Vec3d  bary;
        if (isct_ray_triangle(ray, va, vb, vc, &t, &bary)) 
        {
            if (t > 0 && t < mint) 
            {
                result.exists = true;
                mint = t;
                result.tri_id = i;
                result.isct = ray.p + t * ray.r;
                result.bary = bary;
            }
        }
    }

    return result;
}


template<class VertData, class TriData>
bool Mesh<VertData, TriData>::isClosed()
{
    EGraphCache<int> chains = createEGraphCache<int>();
    chains.for_each([&](uint i, uint j, EGraphEntry<int>& entry) 
        {
          entry.data = 0;
        });
    // count up how many times each edge is encountered in one
    // orientation vs. the other
    for (Tri& tri : tris) {
        chains(tri.a, tri.b).data++;
        chains(tri.b, tri.a).data--;

        chains(tri.b, tri.c).data++;
        chains(tri.c, tri.b).data--;

        chains(tri.c, tri.a).data++;
        chains(tri.a, tri.c).data--;
    }
    // now go through and see if any of these are non-zero
    bool closed = true;
    chains.for_each([&](uint i, uint j, EGraphEntry<int>& entry) {
        if (entry.data != 0)
            closed = false;
        });
    return closed;
}


static inline
bool contains(const ShortVec<uint, 8>& list, uint item)
{
    for (uint k : list)
        if (k == item)
            return true;
    return false;
}


template<class VertData, class TriData>
typename Mesh<VertData, TriData>::NeighborCache
Mesh<VertData, TriData>::createNeighborCache()
{
    NeighborCache result;
    result.skeleton.resize(verts.size());

    for (uint tid = 0; tid < tris.size(); tid++) 
    {
        const Tri& tri = tris[tid];

        result(tri.a, tri.b).tids.push_back(tid);
        result(tri.b, tri.a).tids.push_back(tid);

        result(tri.a, tri.c).tids.push_back(tid);
        result(tri.c, tri.a).tids.push_back(tid);

        result(tri.b, tri.c).tids.push_back(tid);
        result(tri.c, tri.b).tids.push_back(tid);
    }

    return result;
}


// This function signature is an amazing disaster...
#ifdef _WIN32
template<class VertData, class TriData>
template<class Edata>
typename Mesh<VertData, TriData>::EGraphCache<Edata>
#else
template<class VertData, class TriData>
template<class Edata>
typename Mesh<VertData, TriData>::template EGraphCache<Edata>
#endif

//template<class Edata>
//Mesh<VertData, TriData>::EGraphCache<Edata> Mesh<VertData, TriData>::createEGraphCache();
Mesh<VertData, TriData>::createEGraphCache()
{
    EGraphCache<Edata> result;
    result.skeleton.resize(verts.size());

    for (uint tid = 0; tid < tris.size(); tid++) 
    {
        const Tri& tri = tris[tid];

        result(tri.a, tri.b).tids.push_back(tid);
        result(tri.b, tri.a).tids.push_back(tid);

        result(tri.a, tri.c).tids.push_back(tid);
        result(tri.c, tri.a).tids.push_back(tid);

        result(tri.b, tri.c).tids.push_back(tid);
        result(tri.c, tri.b).tids.push_back(tid);
    }

    return result;
}


template<class VertData, class TriData>
std::vector<uint> Mesh<VertData, TriData>::getComponentIds()
{
    UnionFind uf(verts.size());
    for (const Tri& tri : tris) 
    {
        uf.unionIds(tri.a, tri.b);
        uf.unionIds(tri.a, tri.c);
    }

    return uf.dump();
}






////////////////////////////////////////////////////////////////////////////////////////////// 网格数据类接口实现；

// 释放CorkTriMesh对象的堆内存；
void freeCorkTriMesh(CorkTriMesh* mesh)
{
    delete[] mesh->triangles;
    delete[] mesh->vertices;
    mesh->n_triangles = 0;
    mesh->n_vertices = 0;
}


// IO网格类型转换为计算用的网格类型
void corkTriMesh2CorkMesh(CorkTriMesh in, CorkMesh* mesh_out)
{
    RawCorkMesh raw;
    raw.vertices.resize(in.n_vertices);
    raw.triangles.resize(in.n_triangles);
    if (in.n_vertices == 0 || in.n_triangles == 0)
    {
        CORK_ERROR("empty mesh input to Cork routine.");
        *mesh_out = CorkMesh(raw);
        return;
    }

    uint max_ref_idx = 0;
    for (uint i = 0; i < in.n_triangles; i++)
    {
        raw.triangles[i].a = in.triangles[3 * i + 0];
        raw.triangles[i].b = in.triangles[3 * i + 1];
        raw.triangles[i].c = in.triangles[3 * i + 2];
        max_ref_idx = std::max(
            std::max(max_ref_idx,
                in.triangles[3 * i + 0]),
            std::max(in.triangles[3 * i + 1],
                in.triangles[3 * i + 2])
        );
    }
    if (max_ref_idx > in.n_vertices)
    {
        CORK_ERROR("mesh input to Cork routine has an out of range reference "
            "to a vertex.");
        raw.vertices.clear();
        raw.triangles.clear();
        *mesh_out = CorkMesh(raw);
        return;
    }

    for (uint i = 0; i < in.n_vertices; i++)
    {
        raw.vertices[i].pos.x = in.vertices[3 * i + 0];
        raw.vertices[i].pos.y = in.vertices[3 * i + 1];
        raw.vertices[i].pos.z = in.vertices[3 * i + 2];
    }

    *mesh_out = CorkMesh(raw);
}


void corkMesh2CorkTriMesh(CorkMesh* mesh_in, CorkTriMesh* out)
{
    RawCorkMesh raw = mesh_in->raw();

    out->n_triangles = raw.triangles.size();
    out->n_vertices = raw.vertices.size();

    out->triangles = new uint[(out->n_triangles) * 3];
    out->vertices = new float[(out->n_vertices) * 3];

    for (uint i = 0; i < out->n_triangles; i++)
    {
        (out->triangles)[3 * i + 0] = raw.triangles[i].a;
        (out->triangles)[3 * i + 1] = raw.triangles[i].b;
        (out->triangles)[3 * i + 2] = raw.triangles[i].c;
    }

    for (uint i = 0; i < out->n_vertices; i++)
    {
        (out->vertices)[3 * i + 0] = raw.vertices[i].pos.x;
        (out->vertices)[3 * i + 1] = raw.vertices[i].pos.y;
        (out->vertices)[3 * i + 2] = raw.vertices[i].pos.z;
    }
}


// 检测网格是否是solid的——不可以有自交，必须封闭；    ！！！isSolid()接口貌似有问题，把cylinder2.obj网格判定为非solid的；
/*
     布尔操作的输入网格必须是"solid"的:
      -   closed (aka. watertight; see comment at bottom)
      -   无自交（non-self-intersecting

     additionally, inputs should use a counter-clockwise convention
     for triangle facing.  If the triangles are presented in clockwise
     orientation, the object is interpreted as its unbounded complement
*/
bool isSolid(CorkTriMesh cmesh)
{
    CorkMesh mesh;
    corkTriMesh2CorkMesh(cmesh, &mesh);

    bool solid = true;

    if (mesh.isSelfIntersecting())
    {
        CORK_ERROR("isSolid() was given a self-intersecting mesh");
        solid = false;
    }

    if (!mesh.isClosed())
    {
        CORK_ERROR("isSolid() was given a non-closed mesh");
        solid = false;
    }

    return solid;
}


// 布尔并运算
void computeUnion(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out)
{
    CorkMesh cmIn0, cmIn1;
    corkTriMesh2CorkMesh(in0, &cmIn0);
    corkTriMesh2CorkMesh(in1, &cmIn1);

    cmIn0.boolUnion(cmIn1);

    corkMesh2CorkTriMesh(&cmIn0, out);
}


// 布尔减法运算
void computeDifference(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out)
{
    CorkMesh cmIn0, cmIn1;
    corkTriMesh2CorkMesh(in0, &cmIn0);
    corkTriMesh2CorkMesh(in1, &cmIn1);

    cmIn0.boolDiff(cmIn1);

    corkMesh2CorkTriMesh(&cmIn0, out);
}


// 布尔交运算
void computeIntersection(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out)
{
    CorkMesh cmIn0, cmIn1;
    corkTriMesh2CorkMesh(in0, &cmIn0);
    corkTriMesh2CorkMesh(in1, &cmIn1);

    cmIn0.boolIsct(cmIn1);

    corkMesh2CorkTriMesh(&cmIn0, out);
}


void computeSymmetricDifference(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out)
{
    CorkMesh cmIn0, cmIn1;
    corkTriMesh2CorkMesh(in0, &cmIn0);
    corkTriMesh2CorkMesh(in1, &cmIn1);

    cmIn0.boolXor(cmIn1);

    corkMesh2CorkTriMesh(&cmIn0, out);
}


/*
     Not a Boolean operation, but related:
      No portion of either surface is deleted.  However, the
      curve of intersection between the two surfaces is made explicit,
      such that the two surfaces are now connected.
*/
void resolveIntersections(CorkTriMesh in0, CorkTriMesh in1, CorkTriMesh* out) {
    CorkMesh cmIn0, cmIn1;
    corkTriMesh2CorkMesh(in0, &cmIn0);
    corkTriMesh2CorkMesh(in1, &cmIn1);

    cmIn0.disjointUnion(cmIn1);
    cmIn0.resolveIntersections();

    corkMesh2CorkTriMesh(&cmIn0, out);
}


void file2corktrimesh(const Files::FileMesh& in, CorkTriMesh* out)
{
    out->n_vertices = in.vertices.size();
    out->n_triangles = in.triangles.size();

    out->triangles = new uint[(out->n_triangles) * 3];
    out->vertices = new float[(out->n_vertices) * 3];

    for (uint i = 0; i < out->n_triangles; i++)
    {
        (out->triangles)[3 * i + 0] = in.triangles[i].a;
        (out->triangles)[3 * i + 1] = in.triangles[i].b;
        (out->triangles)[3 * i + 2] = in.triangles[i].c;
    }

    for (uint i = 0; i < out->n_vertices; i++)
    {
        (out->vertices)[3 * i + 0] = in.vertices[i].pos.x;
        (out->vertices)[3 * i + 1] = in.vertices[i].pos.y;
        (out->vertices)[3 * i + 2] = in.vertices[i].pos.z;
    }
}


void corktrimesh2file(CorkTriMesh in, Files::FileMesh& out)
{
    out.vertices.resize(in.n_vertices);
    out.triangles.resize(in.n_triangles);

    for (uint i = 0; i < in.n_triangles; i++)
    {
        out.triangles[i].a = in.triangles[3 * i + 0];
        out.triangles[i].b = in.triangles[3 * i + 1];
        out.triangles[i].c = in.triangles[3 * i + 2];
    }

    for (uint i = 0; i < in.n_vertices; i++)
    {
        out.vertices[i].pos.x = in.vertices[3 * i + 0];
        out.vertices[i].pos.y = in.vertices[3 * i + 1];
        out.vertices[i].pos.z = in.vertices[3 * i + 2];
    }
}


void loadMesh(string filename, CorkTriMesh* out)
{
    Files::FileMesh filemesh;

    if (Files::readTriMesh(filename, &filemesh) > 0) 
    {
        cerr << "Unable to load in " << filename << endl;
        exit(1);
    }

    file2corktrimesh(filemesh, out);
}


void saveMesh(string filename, CorkTriMesh in)
{
    Files::FileMesh filemesh;

    corktrimesh2file(in, filemesh);

    if (Files::writeTriMesh(filename, &filemesh) > 0)
    {
        cerr << "Unable to write to " << filename << endl;
        exit(1);
    }
}





//////////////////////////////////////////////////////////////////////////////////////////////  by Tora——debug接口实现

void OBJWriteMesh(const char* pszFileName, const CorkMesh& mesh)
{
    std::ofstream dstFile(pszFileName);

    unsigned nSize = mesh.verts.size();

    for (unsigned j = 0; j < nSize; j++)
    {
        char szBuf[256] = { 0 };
        CorkVertex vert = mesh.verts[j];
        sprintf_s(szBuf, 256, "v %f %f %f", vert.pos.x, vert.pos.y, vert.pos.z);
        dstFile << szBuf << "\n";
    }

    nSize = mesh.tris.size();
    for (unsigned j = 0; j < nSize; ++j)
    {
        char szBuf[256] = { 0 };
        const auto& tri = mesh.tris[j];
        sprintf_s(szBuf, 256, "f %d %d %d", tri.a + 1, tri.b + 1, tri.c + 1);
        dstFile << szBuf << "\n";
    }
}


void OBJWriteVers(const char* pszFileName, const std::vector<CorkVertex>& vers)
{
    std::ofstream dstFile(pszFileName);

    unsigned nSize = vers.size();

    for (unsigned j = 0; j < nSize; j++)
    {
        char szBuf[256] = { 0 };
        CorkVertex vert = vers[j];
        sprintf_s(szBuf, 256, "v %f %f %f", vert.pos.x, vert.pos.y, vert.pos.z);
        dstFile << szBuf << "\n";
    }
}


void OBJWriteVersEdges(const char* pszFileName, const std::vector<CorkVertex>& vers, \
    const std::vector<std::pair<unsigned, unsigned>>& edges)
{
    std::ofstream dstFile(pszFileName);

    unsigned nSize = vers.size();

    for (unsigned j = 0; j < nSize; j++)
    {
        char szBuf[256] = { 0 };
        CorkVertex vert = vers[j];
        sprintf_s(szBuf, 256, "v %f %f %f", vert.pos.x, vert.pos.y, vert.pos.z);
        dstFile << szBuf << "\n";
    }

    nSize = edges.size();
    for (unsigned j = 0; j < nSize; ++j)
    {
        char szBuf[256] = { 0 };
        const std::pair<unsigned, unsigned>& e = edges[j];
        sprintf_s(szBuf, 256, "l %d %d", e.first + 1, e.second + 1);
        dstFile << szBuf << "\n";
    }
}


void OBJWriteVersEdges(const char* pszFileName, const std::vector<edgeInfo>& eInfos, \
    const CorkMesh& mesh)
{
    std::vector<CorkVertex> vers;
    std::vector<std::pair<unsigned, unsigned>> edges;
    vers.reserve(2 * eInfos.size());
    edges.resize(eInfos.size());

    for (const auto& info : eInfos)
    {
        vers.push_back(mesh.verts[std::get<0>(info)]);
        vers.push_back(mesh.verts[std::get<1>(info)]);
    }

    for (unsigned i = 0; i < eInfos.size(); ++i)            // (0, 1) (2,3) (4, 5) (6, 7)
        edges[i] = std::make_pair(2 * i, 2 * i + 1);

    OBJWriteVersEdges(pszFileName, vers, edges);
}


void OBJWriteVersEdges(const char* pszFileName, const std::vector<TopoEdge>& tpEdges, const CorkMesh& mesh)
{
    std::vector<CorkVertex> vers;
    std::vector<std::pair<unsigned, unsigned>> edges;
    vers.reserve(2 * tpEdges.size());
    edges.resize(tpEdges.size());
    for (const auto& tpE : tpEdges)
    {
        uint index1 = tpE.verts[0]->ref;
        uint index2 = tpE.verts[1]->ref;
        vers.push_back(mesh.verts[index1]);
        vers.push_back(mesh.verts[index2]);
    }

    for (unsigned i = 0; i < tpEdges.size(); ++i)       // (0, 1) (2,3) (4, 5) (6, 7)
        edges[i] = std::make_pair(2 * i, 2 * i + 1);

    OBJWriteVersEdges(pszFileName, vers, edges);
}


void debugWriteMesh(const char* fileName, const CorkTriMesh& mesh)
{
    char str[512] = { 0 };
    CorkMesh mesh0;
    corkTriMesh2CorkMesh(mesh, &mesh0);
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteMesh(str, mesh0);
}


void debugWriteMesh(const char* fileName, CorkMesh& mesh)
{
    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteMesh(str, mesh);
}


// 打印单个三角片网格
void debugWriteTriMesh(const char* fileName, const CorkMesh& mesh, const unsigned triIdx)
{
    CorkMesh triMesh;
    const auto& theTri = mesh.tris[triIdx];
    triMesh.verts.reserve(3);
    triMesh.verts.push_back(mesh.verts[theTri.a]);
    triMesh.verts.push_back(mesh.verts[theTri.b]);
    triMesh.verts.push_back(mesh.verts[theTri.c]);
    triMesh.tris.push_back(CorkMesh::Tri(0, 1, 2));

    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteMesh(str, triMesh);
}


// 打印一条边
void debugWriteEdge(const char* fileName, const CorkEdge& e)
{
    std::vector<CorkVertex> vers;
    std::vector<std::pair<unsigned, unsigned>> edge;
    vers.push_back(e.va);
    vers.push_back(e.vb);
    edge.push_back(std::make_pair(0, 1));

    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, vers, edge);
}


void debugWriteEdge(const char* fileName, const CorkMesh& mesh, const TopoEdge& te)
{
    std::vector<CorkVertex> vers;
    std::vector<std::pair<unsigned, unsigned>> edge;

    CorkVertex ver0(mesh.verts[te.verts[0]->ref]);
    CorkVertex ver1(mesh.verts[te.verts[1]->ref]);
    vers.push_back(ver0);
    vers.push_back(ver1);
    edge.push_back(std::make_pair(0, 1));

    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, vers, edge);
}


void debugWriteEdge(const char* fileName, const CorkVertex& ver1, const CorkVertex& ver2)
{
    std::vector<CorkVertex> vers;
    std::vector<std::pair<unsigned, unsigned>> edge;

    vers.push_back(ver1);
    vers.push_back(ver2);
    edge.push_back(std::make_pair(0, 1));

    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, vers, edge);
}

void debugWriteVers(const char* fileName, std::vector<CorkVertex>& vers)
{
    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVers(str, vers);
}


void debugWriteEdges(const char* fileName, const CorkEdges& es)
{
    std::vector<CorkVertex> vers;
    std::vector<std::pair<unsigned, unsigned>> edges;
    vers.reserve(es.edgeVec.size() *2);
    edges.reserve(es.edgeVec.size());

    for (const auto& e: es.edgeVec) 
    {
        vers.push_back(e.va);
        vers.push_back(e.vb);
    }

    for (unsigned i = 0; i < es.edgeVec.size(); ++i)
        edges.push_back(std::make_pair(2*i , 2*i+1));           // (0, 1), (2, 3), (4, 5)....
    
    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, vers, edges);
}


void debugWriteEdges(const char* fileName, std::vector<CorkVertex>& vers, const std::vector<std::pair<unsigned, unsigned>>& edges)
{
    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, vers, edges);
}

void debugWriteEdges(const char* fileName, const std::vector<edgeInfo>& eInfos, \
    const CorkMesh& mesh)
{
    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, eInfos, mesh);
}


void debugWriteEdges(const char* fileName, const std::vector<TopoEdge>& tpEdges, const CorkMesh& mesh)
{
    char str[512] = { 0 };
    sprintf_s(str, "%s%s.obj", g_debugPath.c_str(), fileName);
    OBJWriteVersEdges(str, tpEdges, mesh);
}




////////////////////////////////////////////////////////////////////////////////////////////// by Tora——support接口实现

// 输入选取的三角片索引，输出对应的新网格；
void selectedTris2Mesh(CorkMesh& newMesh, const CorkMesh& oriMesh, const std::vector<unsigned>& selectedTriIdx)
{
    std::set<unsigned> newOldIdxInfo;
    for (const auto& triIdx : selectedTriIdx)
    {
        const auto& tri = oriMesh.tris[triIdx];
        newOldIdxInfo.insert(tri.a);
        newOldIdxInfo.insert(tri.b);
        newOldIdxInfo.insert(tri.c);
    }

    unsigned newVersCount = newOldIdxInfo.size();
    unsigned newTrisCount = selectedTriIdx.size();
    newMesh.verts.resize(newVersCount);
    newMesh.tris.reserve(oriMesh.tris.size());
    std::vector<int> oldNewIdxInfo(oriMesh.verts.size(), -1);
    auto iter = newOldIdxInfo.begin();

    // 得到新老顶点索引表；生成新网格的点云；
    for (unsigned i = 0; i < newVersCount; ++i)
    {
        unsigned oldIdx = *iter;
        oldNewIdxInfo[oldIdx] = static_cast<int>(i);
        newMesh.verts[i] = oriMesh.verts[oldIdx];
        iter++;
    }

    // 生成新网格的三角片数据：
    for (const auto& triIdx : selectedTriIdx)
    {
        newMesh.tris.push_back(oriMesh.tris[triIdx]);
        auto& currentTri = *newMesh.tris.rbegin();
        currentTri.a = static_cast<unsigned>(oldNewIdxInfo[currentTri.a]);
        currentTri.b = static_cast<unsigned>(oldNewIdxInfo[currentTri.b]);
        currentTri.c = static_cast<unsigned>(oldNewIdxInfo[currentTri.c]);
    }

    newMesh.verts.shrink_to_fit();
    newMesh.tris.shrink_to_fit();
}


// 生成立方体网格，八个顶点； 
void drawCube(CorkMesh& cubeMesh, const double xlen, const double ylen, const double zlen, const CorkVertex& center)
{
    cubeMesh.verts.reserve(8);
    cubeMesh.tris.reserve(12);

    double ox = center.pos.x - xlen / 2;
    double oy = center.pos.y - ylen / 2;
    double oz = center.pos.z - zlen / 2;
    cubeMesh.verts.push_back(CorkVertex(ox, oy, oz));
    cubeMesh.verts.push_back(CorkVertex(ox, oy + ylen, oz));
    cubeMesh.verts.push_back(CorkVertex(ox + xlen, oy, oz));
    cubeMesh.verts.push_back(CorkVertex(ox + xlen, oy + ylen, oz));
    cubeMesh.verts.push_back(CorkVertex(ox + xlen, oy, oz + zlen));
    cubeMesh.verts.push_back(CorkVertex(ox + xlen, oy + ylen, oz + zlen));
    cubeMesh.verts.push_back(CorkVertex(ox, oy, oz + zlen));
    cubeMesh.verts.push_back(CorkVertex(ox, oy + ylen, oz + zlen));

    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(1, 2, 0));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(1, 3, 2));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(3, 4, 2));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(3, 5, 4));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(0, 4, 6));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(0, 2, 4));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(7, 3, 1));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(7, 5, 3));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(7, 0, 6));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(7, 1, 0));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(5, 6, 4));
    cubeMesh.tris.push_back(Mesh<CorkVertex, CorkTriangle>::Tri(5, 7, 6));
}


// 检测某个三角片是否在某个网格内部；
bool myIsInside(const CorkVertex& va, const CorkVertex& vb, const CorkVertex& vc, const CorkMesh& mesh)
{
    // 测试某一个三角片是否在网格内：
    Vec3d p(0, 0, 0);               // 三角片重心
    p += va.pos;
    p += vb.pos;
    p += vc.pos;
    p /= 3.0;

    // 1. 生成射线
    Ray3d r;
    r.p = p;
    r.r = Vec3d(drand(0.5, 1.5), drand(0.5, 1.5), drand(0.5, 1.5));         // 射线方向是随机的，在第一象限内；
    int winding = 0;                  // 缠绕数winding number

    // 2. 射线对网格所有三角片求交；
    for (auto& tri : mesh.tris)
    {
        double flip = 1.0;
        uint   a = tri.a;
        uint   b = tri.b;
        uint   c = tri.c;
        Vec3d mva = mesh.verts[a].pos;
        Vec3d mvb = mesh.verts[b].pos;
        Vec3d mvc = mesh.verts[c].pos;

        // 升序排列当前三角片的三个顶点索引；
        if (a > b)
        {
            std::swap(a, b);
            std::swap(mva, mvb);
            flip = -flip;
        }
        if (b > c)
        {
            std::swap(b, c);
            std::swap(mvb, mvc);
            flip = -flip;
        }
        if (a > b)
        {
            std::swap(a, b);
            std::swap(mva, mvb);
            flip = -flip;
        }

        double t;
        Vec3d bary;
        if (isct_ray_triangle(r, mva, mvb, mvc, &t, &bary))
        {
            Vec3d normal = flip * cross(mvb - mva, mvc - mva);

            if (dot(normal, r.r) > 0.0)
                winding++;          // UNSAFE
            else
                winding--;
        }
    }

    // 3. 缠绕数大于0则返回true；
    return winding > 0;
}


// 生成边的包围盒——项目中交叉过程使用的是扰动之后的顶点，这里使用原顶点；
BBox3d getAABB(TopoEdge* e, const CorkMesh& mesh)
{
    Vec3d p0 = mesh.verts[e->verts[0]->ref].pos;
    Vec3d p1 = mesh.verts[e->verts[1]->ref].pos;
    return BBox3d(min(p0, p1), max(p0, p1));
}


// 生成三角片的包围盒——项目中交叉过程使用的是扰动之后的顶点，这里使用原顶点；
BBox3d getAABB(TopoTri* t, const CorkMesh& mesh)
{
    Vec3d p0 = mesh.verts[t->verts[0]->ref].pos;
    Vec3d p1 = mesh.verts[t->verts[1]->ref].pos;
    Vec3d p2 = mesh.verts[t->verts[2]->ref].pos;
    return BBox3d(min(p0, min(p1, p2)), max(p0, max(p1, p2)));
}


// 生成一整个网格对象的包围盒：
BBox3d getAABB(const CorkMesh& mesh) 
{
    unsigned versCount = mesh.verts.size();
    std::vector<double> xValues, yValues, zValues;
    xValues.reserve(versCount);
    yValues.reserve(versCount);
    zValues.reserve(versCount);

    for (const auto& ver : mesh.verts)
    {
        xValues.push_back(ver.pos.x);
        yValues.push_back(ver.pos.y);
        zValues.push_back(ver.pos.z);
    }

    Vec3d minp, maxp;
    minp.x = *std::min_element(xValues.begin(), xValues.end());
    minp.y = *std::min_element(yValues.begin(), yValues.end());
    minp.z = *std::min_element(zValues.begin(), zValues.end());
    maxp.x = *std::max_element(xValues.begin(), xValues.end());
    maxp.y = *std::max_element(yValues.begin(), yValues.end());
    maxp.z = *std::max_element(zValues.begin(), zValues.end());

    return BBox3d(minp, maxp);
}



GeomBlob<TopoEdge*> getEdgeBlob(TopoEdge* e, const CorkMesh& mesh)
{
    GeomBlob<TopoEdge*>  blob;
    blob.bbox = getAABB(e, mesh);
    blob.point = (blob.bbox.minp + blob.bbox.maxp) / 2.0;
    blob.id = e;
    return blob;
}


// 生成包围盒的网格
void getAABBmesh(CorkMesh& meshOut, const BBox3d& bbox)
{
    meshOut.verts.clear();
    meshOut.tris.clear();
    double xlen = std::abs(bbox.maxp.x - bbox.minp.x);
    double ylen = std::abs(bbox.maxp.y - bbox.minp.y);
    double zlen = std::abs(bbox.maxp.z - bbox.minp.z);
    CorkVertex center;
    center.pos.x = (bbox.maxp.x + bbox.minp.x) / 2;
    center.pos.y = (bbox.maxp.y + bbox.minp.y) / 2;
    center.pos.z = (bbox.maxp.z + bbox.minp.z) / 2;

    drawCube(meshOut, xlen, ylen, zlen, center);
}

