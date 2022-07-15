#pragma once

#include <iostream>

#define INVALID_ID uint(-1)


// 带有拓扑信息的一些数据结构，及其实现；




// TopoCache类——网格的拓扑信息
template<class VertData, class TriData>
struct Mesh<VertData, TriData>::TopoCache 
{
    // 成员数据：
    IterPool<TopoVert>    verts;
    IterPool<TopoEdge>    edges;
    IterPool<TopoTri>     tris;
    Mesh* mesh;


    TopoCache(Mesh* owner);
    virtual ~TopoCache() {}

    // until commit() is called, the Mesh::verts and Mesh::tris arrays will still contain garbage entries
    void commit();
    bool isValid();
    void print();

    // helpers to create bits and pieces
    inline TopoVert* newVert();
    inline TopoEdge* newEdge();
    inline TopoTri* newTri();

    // 删除数据的接口
    inline void freeVert(TopoVert*);
    inline void freeEdge(TopoEdge*);
    inline void freeTri(TopoTri*);

    // 删除三角片
    inline void deleteTri(TopoTri*);

    // 翻转三角片
    inline void flipTri(TopoTri*);

private:
    void init();
};


// TopoEdgePrototype类——support structure for cache construction
struct TopoEdgePrototype
{
    uint vid;
    ShortVec<TopoTri*, 2> tris;
    TopoEdgePrototype() {}
    TopoEdgePrototype(uint v) : vid(v) {}
};


template<class VertData, class TriData> inline
TopoVert* Mesh<VertData, TriData>::TopoCache::newVert()
{
    uint        ref = mesh->verts.size();
    mesh->verts.push_back(VertData());
    TopoVert*        v = verts.alloc(); // cache.verts
    v->ref = ref;
    return v;
}


template<class VertData, class TriData> inline
TopoEdge* Mesh<VertData, TriData>::TopoCache::newEdge()
{
    TopoEdge*        e = edges.alloc(); // cache.edges
    return e;
}


// 生成新三角片，插入到原融合网格中；
template<class VertData, class TriData> inline
TopoTri* Mesh<VertData, TriData>::TopoCache::newTri()
{
    uint        ref = mesh->tris.size();
    mesh->tris.push_back(Tri());
    TopoTri*        t = tris.alloc();                // cache.tris
    t->ref = ref;
    return t;
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::TopoCache::freeVert(TopoVert* v)
{
    verts.free(v);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::TopoCache::freeEdge(TopoEdge* e)
{
    edges.free(e);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::TopoCache::freeTri(TopoTri* t)
{
    tris.free(t);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::TopoCache::deleteTri(TopoTri* tri)
{
    // first, unhook the triangle from its faces
    for (uint k = 0; k < 3; k++) {
        TopoVert*            v = tri->verts[k];
        v->tris.erase(tri);
        TopoEdge*            e = tri->edges[k];
        e->tris.erase(tri);
    }
    // now, let's check for any edges which no longer border triangles
    for (uint k = 0; k < 3; k++) {
        TopoEdge*            e = tri->edges[k];
        if (e->tris.size() == 0) {
            // delete edge
            // unhook from vertices
            TopoVert*        v0 = e->verts[0];
            v0->edges.erase(e);
            TopoVert*        v1 = e->verts[1];
            v1->edges.erase(e);
            freeEdge(e);
        }
    }
    // now, let's check for any vertices which no longer border triangles
    for (uint k = 0; k < 3; k++) {
        TopoVert*            v = tri->verts[k];
        if (v->tris.size() == 0) {
            freeVert(v);
        }
    }
    // finally, release the triangle
    freeTri(tri);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::TopoCache::flipTri(TopoTri* t)
{
    std::swap(t->verts[0], t->verts[1]);
    std::swap(t->edges[0], t->edges[1]);
    std::swap(mesh->tris[t->ref].v[0], mesh->tris[t->ref].v[1]);
}


template<class VertData, class TriData>
Mesh<VertData, TriData>::TopoCache::TopoCache(Mesh* owner) : mesh(owner)
{
    init();
}


inline TopoEdgePrototype& getTopoEdgePrototype(uint a, uint b,  std::vector< ShortVec<TopoEdgePrototype, 8>>& prototypes)
{
    uint N = prototypes[a].size();
    for (uint i = 0; i < N; i++)
    {
        if (prototypes[a][i].vid == b)
            return prototypes[a][i];
    }
    prototypes[a].push_back(TopoEdgePrototype(b));
    return prototypes[a][N];
}


// 读取网格数据构造TopoCache对象，构造函数中调用；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::TopoCache::init()
{
    // 创建拓扑顶点对象；
    std::vector<TopoVert*> temp_verts(mesh->verts.size());      // need temp. reference
    for (uint i = 0; i < mesh->verts.size(); i++) 
    {
        TopoVert* vert = this->verts.alloc();             // cache.verts.alloc()
        vert->ref = i;
        temp_verts[i] = vert;
    }

    // We need to still do the following
    /*
      * Generate TopoTris
      * Generate TopoEdges
 
     ---- Hook up references between
      * Triangles and Vertices
      * Triangles and Edges
      * Vertices and Edges
    */


    // 创建拓扑三角片对象，
    /*
         We handle two of these items in a pass over the triangles,
          * Generate TopoTris
          * Hook up Triangles and Vertices
         building a structure to handle the edges as we go:
     */
    std::vector< ShortVec<TopoEdgePrototype, 8> > edgeacc(mesh->verts.size());          // 按顶点索引排列的边信息；
    for (uint i = 0; i < mesh->tris.size(); i++)                // 对三角片的遍历；
    {
        TopoTri* tri = this->tris.alloc();            // cache.tris.alloc()
        tri->ref = i;
        const Tri& ref_tri = mesh->tris[i];

        // 点关联的三角片信息、三角片关联的点信息；
        uint vids[3];
        for (uint k = 0; k < 3; k++) 
        {
            uint vid = vids[k] = ref_tri.v[k];
            tri->verts[k] = temp_verts[vid];
            temp_verts[vid]->tris.push_back(tri);
        }

        // then, put these in arbitrary but globally consistent order
        if (vids[0] > vids[1])   std::swap(vids[0], vids[1]);
        if (vids[1] > vids[2])   std::swap(vids[1], vids[2]);
        if (vids[0] > vids[1])   std::swap(vids[0], vids[1]);

        // and accrue in structure
        getTopoEdgePrototype(vids[0], vids[1], edgeacc).tris.push_back(tri);
        getTopoEdgePrototype(vids[0], vids[2], edgeacc).tris.push_back(tri);
        getTopoEdgePrototype(vids[1], vids[2], edgeacc).tris.push_back(tri);
    }

    // 创建拓扑边数据；关联边和点，关联边和三角片；
    /*
     Now, we can unpack the edge accumulation to
      * Generate TopoEdges
      * Hook up Triangles and Edges
      * Hook up Vertices and Edges
      */
    for (uint vid0 = 0; vid0 < edgeacc.size(); vid0++)          // 对顶点的遍历
    {
        for (TopoEdgePrototype& proto : edgeacc[vid0])          // 对该顶点发出的所有边的遍历；
        {
            uint vid1 = proto.vid;
            TopoVert* v0 = temp_verts[vid0];
            TopoVert* v1 = temp_verts[vid1];

            TopoEdge* edge = this->edges.alloc();         

            // 边关联的点信息、 点关联的边信息；
            edge->verts[0] = v0;
            v0->edges.push_back(edge);
            edge->verts[1] = v1;
            v1->edges.push_back(edge);

            // 边关联的三角片信息、三角片关联的边信息
            for (TopoTri* tri : proto.tris) 
            {
                edge->tris.push_back(tri);
                for (uint k = 0; k < 3; k++) 
                {
                    if (v0 != tri->verts[k] && v1 != tri->verts[k]) 
                    {
                        tri->edges[k] = edge;
                        break;
                    }
                }
            }
        }
    }
 
}


// ？？？去除重复点和重复三角片？？？
template<class VertData, class TriData>
void Mesh<VertData, TriData>::TopoCache::commit()
{
    // （非必要）检测当前TopoCatch::verts中是否有顶点
    std::vector<bool> live_verts(mesh->verts.size(), false);
    verts.for_each([&](TopoVert* vert) 
        { 
            live_verts[vert->ref] = true;
        });

    // record which triangles are live, and record connectivity
    std::vector<bool> live_tris(mesh->tris.size(), false);
    tris.for_each([&](TopoTri* tri) 
        {  
            live_tris[tri->ref] = true;
            for (uint k = 0; k < 3; k++)
                mesh->tris[tri->ref].v[k] = tri->verts[k]->ref;
        });

    // compact the vertices and build a remapping function
    std::vector<uint> vmap(mesh->verts.size());
    uint write = 0;
    for (uint read = 0; read < mesh->verts.size(); read++) 
    {
        if (live_verts[read]) 
        {
            vmap[read] = write;
            mesh->verts[write] = mesh->verts[read];
            write++;
        }
        else 
            vmap[read] = INVALID_ID;
    }
    mesh->verts.resize(write);

    // rewrite the vertex reference ids
    verts.for_each([&](TopoVert* vert) 
        { // cache.verts
            vert->ref = vmap[vert->ref];
        });

    std::vector<uint> tmap(mesh->tris.size());
    write = 0;
    for (uint read = 0; read < mesh->tris.size(); read++) 
    {
        if (live_tris[read]) 
        {
            tmap[read] = write;
            mesh->tris[write] = mesh->tris[read];
            for (uint k = 0; k < 3; k++)
                mesh->tris[write].v[k] = vmap[mesh->tris[write].v[k]];
            write++;
        }
        else 
            tmap[read] = INVALID_ID;
    }
    mesh->tris.resize(write);

    // rewrite the triangle reference ids
    tris.for_each([&](TopoTri* tri)
        { // cache.tris
             tri->ref = tmap[tri->ref];
        });
}


// support functions for validity check
template<class T, class Container> inline
bool count(const Container& contain, const T& val) {
    uint c = 0;
    for (const T& t : contain)
        if (t == val)    c++;
    return c;
}


template<class T> inline
bool count2(const T arr[], const T& val) {
    return ((arr[0] == val) ? 1 : 0) + ((arr[1] == val) ? 1 : 0);
}


template<class T> inline
bool count3(const T arr[], const T& val) {
    return ((arr[0] == val) ? 1 : 0) + ((arr[1] == val) ? 1 : 0)
        + ((arr[2] == val) ? 1 : 0);
}


template<class VertData, class TriData>
bool Mesh<VertData, TriData>::TopoCache::isValid()
{
    //print();
    std::set<TopoVert*> vaddr;
    std::set<TopoEdge*> eaddr;
    std::set<TopoTri*> taddr;
    verts.for_each([&vaddr](TopoVert* v) { vaddr.insert(v); });
    edges.for_each([&eaddr](TopoEdge* e) { eaddr.insert(e); });
    tris.for_each([&taddr](TopoTri* t) { taddr.insert(t); });

    // check verts
    verts.for_each([&](TopoVert* v) {
        ENSURE(v->ref < mesh->verts.size());
        // make sure each edge pointer goes somewhere and that
        // the pointed-to site also points back correctly
        for (TopoEdge* e : v->edges) {
            ENSURE(eaddr.count(e) > 0); // pointer is good
            ENSURE(count2(e->verts, v) == 1); // back-pointer is good
        }
        for (TopoTri* t : v->tris) {
            ENSURE(taddr.count(t) > 0);
            ENSURE(count3(t->verts, v) == 1);
        }
        });

    // check edges
    edges.for_each([&](TopoEdge* e) {
        // check for non-degeneracy
        ENSURE(e->verts[0] != e->verts[1]);
        for (uint k = 0; k < 2; k++) {
            TopoVert* v = e->verts[k];
            ENSURE(vaddr.count(v) > 0);
            ENSURE(count(v->edges, e) == 1);
        }
        for (TopoTri* t : e->tris) {
            ENSURE(taddr.count(t) > 0);
            ENSURE(count3(t->edges, e) == 1);
        }
        });

    // check triangles
    tris.for_each([&](TopoTri* t) {
        // check for non-degeneracy
        ENSURE(t->verts[0] != t->verts[1] && t->verts[1] != t->verts[2]
            && t->verts[0] != t->verts[2]);
        for (uint k = 0; k < 3; k++) {
            TopoVert* v = t->verts[k];
            ENSURE(vaddr.count(v) > 0);
            ENSURE(count(v->tris, t) == 1);

            TopoEdge* e = t->edges[k];
            ENSURE(eaddr.count(e) == 1);
            ENSURE(count(e->tris, t) == 1);

            // also need to ensure that the edges are opposite the
            // vertices as expected
            TopoVert* v0 = e->verts[0];
            TopoVert* v1 = e->verts[1];
            ENSURE((v0 == t->verts[(k + 1) % 3] && v1 == t->verts[(k + 2) % 3])
                || (v0 == t->verts[(k + 2) % 3] && v1 == t->verts[(k + 1) % 3]));
        }
        });

    return true;
}


std::ostream& operator<<(std::ostream& out, const TopoVert& vert)
{
    out << "ref(" << vert.ref << ") "
        << "e(" << vert.edges.size() << "):";
    for (TopoEdge* e : vert.edges)
        out << e << ";";
    out << " "
        << "t(" << vert.tris.size() << "):";
    for (TopoTri* t : vert.tris)
        out << t << ";";
    return out;
}

std::ostream& operator<<(std::ostream& out, const TopoEdge& edge)

{
    out << "v(2):" << edge.verts[0] << "(" << edge.verts[0]->ref << ");"
        << edge.verts[1] << "(" << edge.verts[1]->ref << ");";
    out << " "
        << "t(" << edge.tris.size() << "):";
    for (TopoTri* t : edge.tris)
        out << t << ";";
    return out;
}

std::ostream& operator<<(std::ostream& out, const TopoTri& tri)
{
    out << "ref(" << tri.ref << ") ";
    out << "v(3):" << tri.verts[0] << "(" << tri.verts[0]->ref << ");"
        << tri.verts[1] << "(" << tri.verts[1]->ref << ");"
        << tri.verts[2] << "(" << tri.verts[2]->ref << ");";
    out << " ";
    out << "e(3):" << tri.edges[0] << ";"
        << tri.edges[1] << ";"
        << tri.edges[2] << ";";
    return out;
}


template<class VertData, class TriData>
void Mesh<VertData, TriData>::TopoCache::print()
{
    using std::cout;
    using std::endl;

    cout << "dumping remeshing cache for debug..." << endl;
    cout << "TRIS" << endl;
    int tri_count = 0;
    tris.for_each([&](TopoTri* t) {
        cout << " " << t << ": " << *t << endl;
        tri_count++;
        });
    cout << "There were " << tri_count << " TRIS" << endl;
    cout << "EDGES" << endl;
    int edge_count = 0;
    edges.for_each([&](TopoEdge* e) {
        cout << " " << e << ": " << endl;
        cout << "  v " << e->verts[0] << "; "
            << e->verts[1] << endl;
        cout << "  t (" << e->tris.size() << ")" << endl;
        for (TopoTri* t : e->tris)
            cout << "    " << t << endl;
        edge_count++;
        });
    cout << "There were " << edge_count << " EDGES" << endl;
    cout << "VERTS" << endl;
    int vert_count = 0;
    verts.for_each([&](TopoVert* v) {
        cout << " " << v << ": ref(" << v->ref << ")" << endl;
        cout << "  e (" << v->edges.size() << ")" << endl;
        for (TopoEdge* e : v->edges)
            cout << "    " << e << endl;
        cout << "  t (" << v->tris.size() << ")" << endl;
        for (TopoTri* t : v->tris)
            cout << "    " << t << endl;
        vert_count++;
        });
    cout << "There were " << vert_count << " VERTS" << endl;
}

