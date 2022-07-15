#pragma once
 
// 三角片求交

/////////////////////////////////////////////////////////////////////////////////////////一些支持类



// GenericVertType类——顶点基类；
struct GenericVertType
{
    virtual ~GenericVertType() {}
    TopoVert*                    concrete;       // concret不为空指针，则说明此点在融合网格中已真实存在；为空指针的话可能该点是待新增的边，还没有添加到融合网格中；
    Vec3d                   coord;              // 点坐标；

    bool                    boundary;           //  true——该新增顶点在边上，false——该新增顶点在三角片内部；
    uint                    idx;                         // temporary for triangulation marshalling

    ShortVec<GenericEdgeType*, 2>       edges;      // 该点关联的边；

public:
    GenericVertType() {}
};


struct OrigVertType : public GenericVertType {};


// IsctVertType类——新插的顶点，有两种，一种在三角片内部，一种在边上；
struct IsctVertType : public GenericVertType
{
    GluePointMarker*                  glue_marker;
 
};


// GluePointMarker类——？？？
struct GluePointMarker
{
    ShortVec<IsctVertType*, 3>      copies;    // list of all the vertices to be glued...？？？貌似是待新插入的点？？？
    bool                    split_type;                      // splits are introduced manually, not via intersection,  and therefore use only e pointer
    bool                    edge_tri_type;               // true if edge-tri intersection, false if tri-tri-tri
    TopoEdge* e;
    TopoTri* t[3];
};


// GenericEdgeType类——边基类；
struct GenericEdgeType
{
    virtual ~GenericEdgeType() {}
    TopoEdge*                    concrete;          // 若concret不为空指针，则说明此边在融合网格中已真实存在；为空指针的话可能该边是待新增的边，还没有添加到融合网格中；

    bool                    boundary;
    uint                    idx;                            // temporary for triangulation marshalling

    GenericVertType*                   ends[2];
    ShortVec<IsctVertType*, 1>      interior;               // 如果该边不是交线段，是原始边数据，将需要插入该边内的顶点存入；
};


struct OrigEdgeType : public GenericEdgeType {};


// SplitEdgeType类——？？？边类；
struct SplitEdgeType : public GenericEdgeType {};


// IsctEdgeType类——新增点形成的新增边，一条IsctEdgeType边表示两个三角片的交线段；
struct IsctEdgeType : public GenericEdgeType
{
    // IsctEdgeType对象会保存在一个triangleProblem对象中，对应着某三角片t0；
public:
    //          use to detect duplicate instances within a triangle
    TopoTri*                    other_tri_key;     // 若本IsctEdgeType边是两交片t0,t1的交线段，且t0为当前对象所属tp对象对应的三角片，other_tri_key指向t1;
};


// GenericTriType类——三角片类
struct GenericTriType
{
    TopoTri*                    concrete;                       // 新生成的三角片的指针；
    GenericVertType*                   verts[3];
};




/////////////////////////////////////////////////////////////////////////////////////////// by Tora——一些debug时用的类型，便于监视：
#ifdef MY_DEBUG
struct GenericEdgeInfo
{
    TopoEdge* concrete;                       // ？？？该边在网格中真实地生成之后对应的TE对象？？？
    bool                    boundary;
    uint                    idx;                                         // temporary for triangulation marshalling
    std::vector<GenericVertType>   edgeVers;    // 可能只确认了一个端点；
    std::vector<IsctVertType>     ivOnEdge;

    GenericEdgeInfo() {}
    GenericEdgeInfo(const GenericEdgeType& ie)
    {
        this->concrete = ie.concrete;
        this->boundary = ie.boundary;
        this->idx = ie.idx;
        for (unsigned i = 0; i < 2; ++i)
        {
            if (nullptr != ie.ends[i])
                this->edgeVers.push_back(*ie.ends[i]);
        }
        for (unsigned i = 0; i < ie.interior.user_size; ++i)
        {
            if (nullptr != ie.interior[i])
                this->ivOnEdge.push_back(*ie.interior[i]);
        }
    }

};

struct IsctVertInfo
{
    GluePointMarker* glue_marker;
    TopoVert* concrete;                 // concret不为空指针，则说明此点在融合网格中已真实存在；为空指针的话可能该点是待新增的边，还没有添加到融合网格中；
    Vec3d                   coord;              // 点坐标；

    bool                    boundary;           //  true——该新增顶点在边上，false——该新增顶点在三角片内部；
    uint                    idx;                         // temporary for triangulation marshalling
    std::vector<GenericEdgeInfo>       edgeInfos;      // 该点关联的边；
    std::vector<TopoVert>       nbrVers;        // by Tora——若该交点在边上，即boundry == true，此数组填充该边的两顶点；

    IsctVertInfo() {}
    IsctVertInfo(const IsctVertType& iv)
    {
        this->glue_marker = iv.glue_marker;
        this->concrete = iv.concrete;
        this->coord = iv.coord;
        this->boundary = iv.boundary;
        this->idx = iv.idx;
        for (const auto& elem : iv.edges)
            this->edgeInfos.push_back(GenericEdgeInfo(*elem));

 
    }
};

struct IsctEdgeInfo
{
    TopoEdge* concrete;                       // ？？？该边在网格中真实地生成之后对应的TE对象？？？
    bool                    boundary;
    uint                    idx;                                         // temporary for triangulation marshalling
    std::vector<GenericVertType>   edgeVers;    // 可能只确认了一个端点；
    std::vector<IsctVertType>     ivOnEdge;
    TopoTri* other_tri_key;                    // ？？？被穿刺的三角片？

    IsctEdgeInfo() {}
    IsctEdgeInfo(const IsctEdgeType& ie)
    {
        this->concrete = ie.concrete;
        this->boundary = ie.boundary;
        this->idx = ie.idx;
        for (unsigned i = 0; i < 2; ++i)
        {
            if (nullptr != ie.ends[i])
                this->edgeVers.push_back(*ie.ends[i]);
        }
        for (unsigned i = 0; i < ie.interior.user_size; ++i)
        {
            if (nullptr != ie.interior[i])
                this->ivOnEdge.push_back(*ie.interior[i]);
        }
        this->other_tri_key = ie.other_tri_key;
    }

};


template<class VertData, class TriData>
struct Mesh<VertData, TriData>::TriangleProblemInfo
{
    std::vector<IsctVertInfo>      isct_versInfo;             // 该三角片需要新加的顶点；有两种，一种在三角片内部，一种在边上；
    std::vector<IsctEdgeInfo>      isct_edgesInfo;         // 该三角片需要新加的边；有两种，一种关联两个IsctVertType对象，一种关联一个IsctVertType顶点和一个原顶点；
    std::vector<GenericTriType>      gtris;
    std::vector<OrigVertType>   overts;                  // 该三角片的原始点数据
    std::vector<OrigEdgeType>  oedges;              // 该三角片的原始边数据
    TopoTri  the_tri;                                 // 对应三角片

    TriangleProblemInfo() {}
    TriangleProblemInfo(const Mesh<VertData, TriData>::TriangleProblem& tp)
    {
        this->the_tri = *tp.the_tri;
        for (unsigned i = 0; i < tp.iverts.user_size; ++i)
        {
            if (nullptr != tp.iverts[i])
            {
                const IsctVertType& iv = *tp.iverts[i];
                this->isct_versInfo.push_back(IsctVertInfo(iv));
            }
        }
        for (unsigned i = 0; i < tp.iedges.user_size; ++i)
        {
            if (nullptr != tp.iedges[i])
            {
                const IsctEdgeType& ie = *tp.iedges[i];
                this->isct_edgesInfo.push_back(IsctEdgeInfo(ie));
            }
        }
        for (unsigned i = 0; i < tp.gtris.user_size; ++i)
        {
            if (nullptr != tp.gtris[i])
                this->gtris.push_back(*tp.gtris[i]);
        }

        for (unsigned i = 0; i < 3; ++i)
        {
            if (nullptr != tp.overts[i])
                this->overts.push_back(*tp.overts[i]);
            if (nullptr != tp.oedges[i])
                this->oedges.push_back(*tp.oedges[i]);
        }
    }

};
#endif

//////////////////////////////////////////////////////////////////////////////解决三角片求交问题的功能类——TriangleProblem类、lsctProblem类

// TriangleProblem类——三角片求交和剖分的求解器；一个对象对应一个剖分前的三角片；
template<class VertData, class TriData>
class Mesh<VertData, TriData>::TriangleProblem
{
    // 成员数据
public:
    ShortVec<IsctVertType*, 4>      iverts;             // 该三角片需要新加的顶点；有两种，一种在三角片内部，一种在边上；
    ShortVec<IsctEdgeType*, 2>      iedges;         // 该三角片需要新加的边，即该三角片和其他三角片的交线段；
    ShortVec<GenericTriType*, 8>      gtris;        // 三角剖分的输出三角片；
    OrigVertType* overts[3];                  // 该三角片的原始点数据
    OrigEdgeType* oedges[3];              // 该三角片的原始边数据
    TopoTri* the_tri;                         // 该三角片的指针；

    IsctProblem* isctProbPtr;                      // by Tora——当前tp对象所从属的isctProblem对象的指针；

public:
    TriangleProblem() {}
    ~TriangleProblem() {}

    // 初始化；
    inline void init(IsctProblem* iprob, TopoTri* t)
    {
        the_tri = t;

        // 提取原始的点边数据
        for (uint k = 0; k < 3; k++)
            overts[k] = iprob->newOrigVert(the_tri->verts[k]);

        for (uint k = 0; k < 3; k++)
            oedges[k] = iprob->newOrigEdge(the_tri->edges[k], overts[(k + 1) % 3], overts[(k + 2) % 3]);

        // by Tora
        this->isctProbPtr = iprob;
    }


private:
 

    // 传入新插的顶点iv，该顶点在某条穿刺边上，tri_key为该穿刺边所在的三角片，生成需要新加的边已存在；如果该新加的边已存在则不生成；
    inline void addEdge(IsctProblem* iprob, IsctVertType* iv, TopoTri* t1)
    {
        /*
            情形I:
                当前tp对象三角片t0被穿刺，iv是待新加的t0内部的点；t1是穿刺边所在的三角片；

            情形2：
                当前tp对象三角片t0包含穿刺边，iv是待t0穿刺边上需要新加的点；t1是被穿刺三角片；
        */

        IsctEdgeType* ie = find_edge(this->iedges, t1);      // 当前三角片交线段数组中查找和三角片t1的交线段，有的话返回其指针；

        if (ie)
        {
            // I: 若本tp对象中与t1三角片的交线段*ie已经生成过了，将该交线段尾端点设为iv
            ie->ends[1] = iv;
            iv->edges.push_back(ie);            // 交点关联的边数组加上本交线段*ie；
        }
        else
        {
            // I：若本tp对象中没有保存该交线段*ie，生成之并保存；
            ie = iprob->newIsctEdge(iv, t1);
            iedges.push_back(ie);
        }
    }


    // addBoundaryHelper()——生成当前tp对象的某条边上的新顶点
    void addBoundaryHelper(TopoEdge* edge, IsctVertType* iv)
    {
        // 当前的tp对象对应的三角片是shield三角片，edge是其一条边，该边穿刺了别的三角片，iv是由穿刺点生成的需要新增的顶点；
        iv->boundary = true;            // true表示新顶点在边上；
        iverts.push_back(iv);

        // hook up point to boundary edge interior!
        for (uint k = 0; k < 3; k++)            // 遍历对当前triangleProblem对象三角片的三条边 
        {
            OrigEdgeType* oe = this->oedges[k];             // 原始的边数据；
            if (oe->concrete == edge)
            {
                oe->interior.push_back(iv);         // 原始边数据中记录下中间要插入的顶点；
                iv->edges.push_back(oe);
 
                break;
            }
        }
    }

public:

    // consolidate()——遍历所有交线段，检查是否有尾端点缺失的情况，尝试修复；
    void consolidate(IsctProblem* iprob)
    {
        // 如果一条交线尾端点没有确定，这种情形只可能是三角片自交（如果没有使用交叉包围盒的话）
        for (IsctEdgeType* ie : this->iedges)
        {
             if (ie->ends[1] == nullptr)
            {
                // 1. 当前tp对象的三角片t0，和交线段关联的三角片t1是否有共享顶点；
                TopoVert* vert = commonVert(the_tri, ie->other_tri_key);        // 如果当前sheild三角片和needle三角片有共享顶点，使用该顶点；

                // 2. 若没有共享顶点，报错，退出程序；
                ENSURE(vert);       // bad if we can't find a common vertex

                // 找到上面的sick_edge对应的原始数据，修正：
                for (uint k = 0; k < 3; k++)
                {
                    if (overts[k]->concrete == vert)
                    {
                        ie->ends[1] = overts[k];
                        overts[k]->edges.push_back(ie);
                        break;
                    }
                }
            }
         }

        ENSURE(isValid());
    }


    // addInteriorEndpoint()——计算边和三角片的交点，并根据此交点生成待加入新顶点IsctVertType对象然后返回；
    IsctVertType* addInteriorEndpoint(IsctProblem* iprob, TopoEdge* edge, GluePointMarker* glue)
    {
        // 当前tp对象的三角片是被穿刺的三角片；新增顶点在该三角片内部；

        // 1. 计算边和三角片交点的位置，生成需要新增的点对象iv，并保存到当前triangleProblem对象中；
        IsctVertType*       iv = iprob->newIsctVert(edge, the_tri, glue);       // 穿刺边上需要新加的顶点；
        iv->boundary = false;                       // false表示新增顶点在三角片内部；
        iverts.push_back(iv);                               // 被穿刺三角片内部也需要加入该新顶点；两个新加的顶点重合；
        
        // 2. 尝试为新增的顶点iv生成新的边数据——两个IsctEdgeType对象，不一定会生成；
        for (TopoTri* spearTriPtr : edge->tris)         // 遍历穿刺边所在的三角片；
            addEdge(iprob, iv, spearTriPtr);                    // 穿刺片所在的所有三角片生成需要新加的边；

        return iv;
    }


    // addBoundaryEndpoint()——在当前tp对象的三角片的边上新增顶点、边数据；
    void addBoundaryEndpoint(IsctProblem* iprob, TopoTri* shieldTri, TopoEdge* spearEdge, IsctVertType* iv)
    {
        // this指向的是穿刺边spearEdge所在的triangleProblem对象，shieldTri是被穿刺的三角片，spearEdge是穿刺该三角片的边，iv是边和三角片的交点；

        // 1. ？？？
        iv = iprob->copyIsctVert(iv);
        addBoundaryHelper(spearEdge, iv);            // 生成边上的新顶点；

        // 2. 新增点iv在当前三角片的边上，需要将其和当前三角片的其他新增点连成新边；
        addEdge(iprob, iv, shieldTri);
    }


    // 处理ttt情形中使用的生成交点接口；
    void addInteriorPoint(IsctProblem* iprob, TopoTri* t1, TopoTri* t2, GluePointMarker* glue)
    {
        // 当前tp对象对应的三角片是t0，t0t1t2三个三角片有ttt情形，即两两相交；

        // note this generates wasted re-computation of coordinates 3X
        IsctVertType* iv = iprob->newIsctVert(the_tri, t1, t2, glue);               // iv是三角片内部的点；
        iv->boundary = false;           
        iverts.push_back(iv);

        // find the 2 interior edges
        for (IsctEdgeType* ie : iedges)
        {
            if (ie->other_tri_key == t1 || ie->other_tri_key == t2)
            {
                ie->interior.push_back(iv);
                iv->edges.push_back(ie);
            }
        }
    }


    bool isValid() const 
    {
        ENSURE(the_tri);

        return true;
    }


    // subdivide()——对当前tp对象的三角片执行三角剖分：
    void subdivide(IsctProblem* iprob)
    {
        // 1. 整合当前三角片所有顶点，包括原始的三个顶点，还有之前生成的交点；
        ShortVec<GenericVertType*, 7> points;
        for (uint k = 0; k < 3; k++)
            points.push_back(overts[k]);

        for (IsctVertType* iv : iverts)
            points.push_back(iv);

        for (uint i = 0; i < points.size(); i++)
            points[i]->idx = i;
 
        // split edges and marshall data, for safety, we zero out references to pre-subdivided edges, which may have been destroyed

        // 2. ？？？整合边信息：
        ShortVec<GenericEdgeType*, 8> edges;
        for (uint k = 0; k < 3; k++)
        {
            subdivideEdge(iprob, oedges[k], edges);
            oedges[k] = nullptr;
        }
 
        for (IsctEdgeType*& ie : iedges)
        {
            subdivideEdge(iprob, ie, edges);
            ie = nullptr;               // 写为空指针；
        }
        for (uint i = 0; i < edges.size(); i++)
            edges[i]->idx = i;

        // 3. 生成三角片投影的二维平面，计算其法向量：
        Vec3d normal = cross(overts[1]->coord - overts[0]->coord,
            overts[2]->coord - overts[0]->coord);
        uint normdim = maxDim(abs(normal));
        uint dim0 = (normdim + 1) % 3;
        uint dim1 = (normdim + 2) % 3;
        double sign_flip = (normal.v[normdim] < 0.0) ? -1.0 : 1.0;

        // 4. 执行三角剖分：
        struct triangulateio in, out;
        {
            /* Define input points. */
            in.numberofpoints = points.size();
            in.numberofpointattributes = 0;
            in.pointlist = new REAL[in.numberofpoints * 2];
            in.pointattributelist = nullptr;
            in.pointmarkerlist = new int[in.numberofpoints];
            for (int k = 0; k < in.numberofpoints; k++)
            {
                in.pointlist[k * 2 + 0] = points[k]->coord.v[dim0];
                in.pointlist[k * 2 + 1] = points[k]->coord.v[dim1] * sign_flip;
                in.pointmarkerlist[k] = (points[k]->boundary) ? 1 : 0;
            }

            /* Define the input segments */
            in.numberofsegments = edges.size();
            in.numberofholes = 0;// yes, zero
            in.numberofregions = 0;// not using regions
            in.segmentlist = new int[in.numberofsegments * 2];
            in.segmentmarkerlist = new int[in.numberofsegments];
            for (int k = 0; k < in.numberofsegments; k++)
            {
                in.segmentlist[k * 2 + 0] = edges[k]->ends[0]->idx;
                in.segmentlist[k * 2 + 1] = edges[k]->ends[1]->idx;
                in.segmentmarkerlist[k] = (edges[k]->boundary) ? 1 : 0;
            }

            // to be safe... declare 0 triangle attributes on input
            in.numberoftriangles = 0;
            in.numberoftriangleattributes = 0;

            /* set for flags.... */
            out.pointlist = nullptr;
            out.pointattributelist = nullptr; // not necessary if using -N or 0 attr
            out.pointmarkerlist = nullptr;
            out.trianglelist = nullptr; // not necessary if using -E
            //out.triangleattributelist = null; // not necessary if using -E or 0 attr
            //out.trianglearealist = // only needed with -r and -a
            //out.neighborlist = null; // only neccesary if -n is used
            out.segmentlist = nullptr; // NEED THIS; output segments go here
            out.segmentmarkerlist = nullptr; // NEED THIS for OUTPUT SEGMENTS
            //out.edgelist = null; // only necessary if -e is used
            //out.edgemarkerlist = null; // only necessary if -e is used

            //       solve the triangulation problem
            char* params = (char*)("pzQYY");
            triangulate(params, &in, &out, nullptr);
        }


        // 5. 错误处理：
        if (out.numberofpoints != in.numberofpoints) 
        {
            {
                std::cout << "三角剖分异常！" << std::endl;
                std::cout << "out.numberofpoints: "
                    << out.numberofpoints << std::endl;
                std::cout << "points.size(): " << points.size() << std::endl;
                std::cout << "dumping out the points' coordinates" << std::endl;
                for (uint k = 0; k < points.size(); k++)
                {
                    GenericVertType* gv = points[k];
                    std::cout << "  " << gv->coord
                        << "  " << gv->idx << std::endl;
                }

                std::cout << "dumping out the segments" << std::endl;
                for (int k = 0; k < in.numberofsegments; k++)
                    std::cout << "  " << in.segmentlist[k * 2 + 0]
                    << "; " << in.segmentlist[k * 2 + 1]
                    << " (" << in.segmentmarkerlist[k]
                    << ") " << std::endl;

                std::cout << "dumping out the solved for triangles now..."
                    << std::endl;
                for (int k = 0; k < out.numberoftriangles; k++)
                {
                    std::cout << "  "
                        << out.trianglelist[(k * 3) + 0] << "; "
                        << out.trianglelist[(k * 3) + 1] << "; "
                        << out.trianglelist[(k * 3) + 2] << std::endl;
                }
            }

            // for debug
#ifdef MY_DEBUG
            uint t0Index = this->the_tri->ref;

            TriangleProblemInfo tpInfo(*this);
            std::vector<CorkVertex> allVers;
            std::vector<CorkVertex> sickEdgesVers;
            CorkEdges allEdges;
            for (const auto& v : points)
                allVers.push_back(CorkVertex(v->coord));

            for (const auto& e: edges) 
            {
                GenericEdgeInfo eInfo(*e);
                if (eInfo.edgeVers.size() == 2) 
                {
                    CorkVertex va(e->ends[0]->coord);
                    CorkVertex vb(e->ends[1]->coord);
                    CorkEdge edge(va, vb);
                    allEdges.add(edge);
                }
                else if (eInfo.edgeVers.size() == 2)
                {
                    CorkVertex va(e->ends[0]->coord);
                    sickEdgesVers.push_back(va);
                }
                else
                    std::cout << "当前边无端点信息；" << std::endl;
            }

            debugWriteTriMesh("tri0", *this->isctProbPtr->mesh, tpInfo.the_tri.ref);
            debugWriteVers("allVers", allVers);
            debugWriteEdges("allEdges", allEdges);
            debugWriteVers("sickEdgeVers", sickEdgesVers);

            std::vector<CorkVertex> allVersIn, allVersOut;
            for (int k = 0; k < in.numberofpoints; k++)
            {
                CorkVertex ver(in.pointlist[k * 2 + 0], in.pointlist[k * 2 + 1], 0);
                allVersIn.push_back(ver);
            }
            for (int k = 0; k < out.numberofpoints; k++)
            {
                CorkVertex ver(out.pointlist[k * 2 + 0], out.pointlist[k * 2 + 1], 0);
                allVersOut.push_back(ver);
            }
            debugWriteVers("allVersOut", allVersOut);

 

            getchar();
#endif
        }

        ENSURE(out.numberofpoints == in.numberofpoints);
 

        // 6.保存新生成的三角片：
        this->gtris.resize(out.numberoftriangles);
        for (int k = 0; k < out.numberoftriangles; k++) 
        {
            GenericVertType* gv0 = points[out.trianglelist[(k * 3) + 0]];
            GenericVertType* gv1 = points[out.trianglelist[(k * 3) + 1]];
            GenericVertType* gv2 = points[out.trianglelist[(k * 3) + 2]];
            gtris[k] = iprob->newGenericTri(gv0, gv1, gv2);
        }

        // for debug
#if 0
        if (6742 == this->the_tri->ref)
        {
            CorkMeshCombainer allNewTris;
            TriangleProblemInfo tpInfo(*this);
            for (const auto& tri : tpInfo.gtris)
            {
                CorkMesh triMesh;
                triMesh.tris.push_back(CorkMesh::Tri(0, 1, 2));
                triMesh.verts.push_back(CorkVertex(tri.verts[0]->coord));
                triMesh.verts.push_back(CorkVertex(tri.verts[1]->coord));
                triMesh.verts.push_back(CorkVertex(tri.verts[2]->coord));
                allNewTris.add(triMesh);
            }
            unsigned trisCount = tpInfo.gtris.size();
            debugWriteMesh("allNewTris", allNewTris);
            
            getchar();
        }
#endif

        // 7. clean up after triangulate...
        free(in.pointlist);
        free(in.pointmarkerlist);
        free(in.segmentlist);
        free(in.segmentmarkerlist);
        free(out.pointlist);
        free(out.pointmarkerlist);
        free(out.trianglelist);
        free(out.segmentlist);
        free(out.segmentmarkerlist);
    }


    void subdivideEdge(IsctProblem* iprob, GenericEdgeType* ge, ShortVec<GenericEdgeType*, 8>& edges)
    {
        if (ge->interior.size() == 0) 
            edges.push_back(ge);
        else if (ge->interior.size() == 1) 
        { 
            // common case
            SplitEdgeType* se0 = iprob->newSplitEdge(ge->ends[0], ge->interior[0], ge->boundary);
            SplitEdgeType* se1 = iprob->newSplitEdge(ge->interior[0], ge->ends[1], ge->boundary);
            edges.push_back(se0);
            edges.push_back(se1);

            // get rid of old edge
            iprob->releaseEdge(ge);
        }
        else 
        { 
            Vec3d       dir = ge->ends[1]->coord - ge->ends[0]->coord;
            uint        dim = (fabs(dir.x) > fabs(dir.y)) ?
                ((fabs(dir.x) > fabs(dir.z)) ? 0 : 2) :
                ((fabs(dir.y) > fabs(dir.z)) ? 1 : 2);
            double      sign = (dir.v[dim] > 0.0) ? 1.0 : -1.0;

            // pack the interior vertices into a vector for sorting
            std::vector< std::pair<double, IsctVertType*> > verts;
            for (IsctVertType* iv : ge->interior) 
            {
                // if the sort is ascending, then we're good...
                verts.push_back(std::make_pair(sign * iv->coord.v[dim], iv));
            }

            // ... and sort the vector
            std::sort(verts.begin(), verts.end());

            // then, write the verts into a new container with the endpoints
            std::vector<GenericVertType*>  allv(verts.size() + 2);
            allv[0] = ge->ends[0];
            allv[allv.size() - 1] = ge->ends[1];
            for (uint k = 0; k < verts.size(); k++)
                allv[k + 1] = verts[k].second;

            // now create and accumulate new split edges
            for (uint i = 1; i < allv.size(); i++) 
            {
                SplitEdgeType* se = iprob->newSplitEdge(allv[i - 1],  allv[i], ge->boundary);
                edges.push_back(se);
            }

            // get rid of old edge
            iprob->releaseEdge(ge);
        }
    }


    // 未使用：
#if 0
    IsctVertType* addBoundaryEndpoint(IsctProblem* iprob, TopoTri* tri_key, TopoEdge* edge, Vec3d coord, GluePointMarker* glue)
    {
        IsctVertType*       iv = iprob->newSplitIsctVert(coord, glue);
        addBoundaryHelper(edge, iv);
        // handle edge extending into interior
        addEdge(iprob, iv, tri_key);
        return iv;
    }


    // Should only happen for manually inserted split points on edges, not for points computed via intersection...
    IsctVertType* addBoundaryPointAlone(IsctProblem* iprob, TopoEdge* edge, Vec3d coord, GluePointMarker* glue)
    {
        IsctVertType*       iv = iprob->newSplitIsctVert(coord, glue);
        addBoundaryHelper(edge, iv);
        return iv;
    }
#endif
};


// lsctProblem类——三角片相交问题的求解器；
template<class VertData, class TriData>
class Mesh<VertData, TriData>::IsctProblem : public TopoCache
{
    // 成员数据
public:
    IterPool<GluePointMarker>   glue_pts;
    IterPool<TriangleProblem>   tprobs;                     // 存储了融合网格中三角片和边交点的信息；
    IterPool<IsctVertType>      ivpool;
    IterPool<OrigVertType>      ovpool;
    IterPool<IsctEdgeType>      iepool;
    IterPool<OrigEdgeType>      oepool;
    IterPool<SplitEdgeType>     sepool;
    IterPool<GenericTriType>    gtpool;
    std::vector<Vec3d>          quantized_coords;           // ？？？拷贝了原网格的顶点数据，貌似扰动过程在这些顶点上进行，并非会改写原有的网格顶点；

    AABVH<TopoEdge*> bvhObj;                   // by Tora——网格所有边的层次包围盒；调用bvh_edge_tri()时生成该对象；
    std::vector< GeomBlob<TopoEdge*> > edge_geoms;       // by Tora——网格所有边的GB对象，包含每条边的包围盒；调用bvh_edge_tri()时生成该对象

#ifdef TEST_CROSS_BOX_ACCE
    BBox3d boxIsct;                                                  // 两网格包围盒交叉形成的包围盒；
    std::vector<uint> selectedTrisIdx;
    std::vector<unsigned> selectedTriFlags;
#endif

public:

    // step3.0 构造IsctProblem对象；
    IsctProblem(Mesh* owner) : TopoCache(owner)
    {
        // 1. 初始化所有拓扑三角片数据，不关联任何一个triangleProblem对象；
        TopoCache::tris.for_each([](TopoTri* t)
            {
                t->data = nullptr;
            });

        // 2. ？？？Callibrate the quantization unit...
        double maxMag = 0.0;
        for (VertData& v : TopoCache::mesh->verts)
            maxMag = std::max(maxMag, max(abs(v.pos)));
        Quantization::callibrate(maxMag);

        // 顶点数据存入辅助数组quantized_coords;
        uint N = TopoCache::mesh->verts.size();
        quantized_coords.resize(N);

        uint write = 0;
        TopoCache::verts.for_each([&](TopoVert* v)
            {
#ifdef _WIN32
                Vec3d raw = TopoCache::mesh->verts[v->ref].pos;
#else
                Vec3d raw = TopoCache::mesh->verts[v->ref].pos;
#endif
                quantized_coords[write].x = Quantization::quantize(raw.x);
                quantized_coords[write].y = Quantization::quantize(raw.y);
                quantized_coords[write].z = Quantization::quantize(raw.z);
                v->data = &(quantized_coords[write]);
                write++;
            });


#if 0
        std::vector<CorkVertex> quantiVersOri;
        quantiVersOri.reserve(this->quantized_coords.size());
        for (const auto& pos : this->quantized_coords)
        {
            CorkVertex ver(pos.x, pos.y, pos.z);
            quantiVersOri.push_back(ver);
        }
        debugWriteVers("quantiVersOri", quantiVersOri);
#endif

    }


    virtual ~IsctProblem() 
    {
    }


    // vPos()——取拓扑顶点对象的坐标
    inline Vec3d vPos(TopoVert* v) const
    {
        return *(reinterpret_cast<Vec3d*>(v->data));
    }


    // 生成三角求解器triangleProblem对象；
    TriangleProblem* getTprob(TopoTri* t)
    {
        TriangleProblem* prob = reinterpret_cast<TriangleProblem*>(t->data);
        if (!prob)
        {
            t->data = prob = tprobs.alloc();
            prob->init(this, t);            // 三角求解器初始化；
        }

        return prob;
    }


    GluePointMarker* newGluePt()
    {
        GluePointMarker* glue = glue_pts.alloc();
        glue->split_type = false;
        return glue;
    }


    // newIsctVert()——计算边e和三角片t的交点，由此交点生成IsctVertType对象返回；
    inline IsctVertType* newIsctVert(TopoEdge* e, TopoTri* t, GluePointMarker* glue)
    {
        IsctVertType*       iv = ivpool.alloc();
        iv->concrete = nullptr;
        iv->coord = computeCoords(e, t);
        iv->glue_marker = glue;
        glue->copies.push_back(iv);

        return      iv;
    }


    // 处理ttt情形时，使用的生成交点的接口；
    inline IsctVertType* newIsctVert(TopoTri* t0, TopoTri* t1, TopoTri* t2, GluePointMarker* glue)
    {
        // t0t1t2三个三角片有ttt情形，即两两相交；

        IsctVertType*       iv = ivpool.alloc();
        iv->concrete = nullptr;
        iv->coord = computeCoords(t0, t1, t2);
        iv->glue_marker = glue;
        glue->copies.push_back(iv);

        return      iv;
    }


    inline IsctVertType* newSplitIsctVert(Vec3d coords, GluePointMarker* glue)
    {
        IsctVertType*       iv = ivpool.alloc();
        iv->concrete = nullptr;
        iv->coord = coords;
        iv->glue_marker = glue;
        glue->copies.push_back(iv);

        return      iv;
    }


    inline IsctVertType* copyIsctVert(IsctVertType* orig)
    {
        IsctVertType*       iv = ivpool.alloc();
        iv->concrete = nullptr;
        iv->coord = orig->coord;
        iv->glue_marker = orig->glue_marker;
        orig->glue_marker->copies.push_back(iv);
 

        return      iv;
    }


    inline IsctEdgeType* newIsctEdge(IsctVertType* endpoint, TopoTri* tri_key)
    {
        IsctEdgeType*       ie = iepool.alloc();
        ie->concrete = nullptr;
        ie->boundary = false;
        ie->ends[0] = endpoint;
        endpoint->edges.push_back(ie);
        ie->ends[1] = nullptr;              // other end null
        ie->other_tri_key = tri_key;
        return      ie;
    }


    inline OrigVertType* newOrigVert(TopoVert* v) 
    {
        OrigVertType*       ov = ovpool.alloc();
        ov->concrete = v;
        ov->coord = vPos(v);
        ov->boundary = true;
        return      ov;
    }


    inline OrigEdgeType* newOrigEdge(TopoEdge* e, OrigVertType* v0, OrigVertType* v1) 
    {
        OrigEdgeType*       oe = oepool.alloc();
        oe->concrete = e;
        oe->boundary = true;
        oe->ends[0] = v0;
        oe->ends[1] = v1;
        v0->edges.push_back(oe);
        v1->edges.push_back(oe);
        return      oe;
    }


    inline SplitEdgeType* newSplitEdge(GenericVertType* v0, GenericVertType* v1, bool boundary) 
    {
        SplitEdgeType*       se = sepool.alloc();
        se->concrete = nullptr;
        se->boundary = boundary;
        se->ends[0] = v0;
        se->ends[1] = v1;
        v0->edges.push_back(se);
        v1->edges.push_back(se);
        return      se;
    }


    inline GenericTriType* newGenericTri(GenericVertType* v0, GenericVertType* v1, GenericVertType* v2) {
        GenericTriType*       gt = gtpool.alloc();
        gt->verts[0] = v0;
        gt->verts[1] = v1;
        gt->verts[2] = v2;
        gt->concrete = nullptr;
        return      gt;
    }


    inline void releaseEdge(GenericEdgeType* ge) 
    {
        disconnectGE(ge);
        IsctEdgeType*       ie = dynamic_cast<IsctEdgeType*>(ge);
        if (ie) {
            iepool.free(ie);
        }
        else {
            OrigEdgeType*   oe = dynamic_cast<OrigEdgeType*>(ge);
            ENSURE(oe);
            oepool.free(oe);
        }
    }


    inline void killIsctVert(IsctVertType* iv)
    {
        iv->glue_marker->copies.erase(iv);
        if (iv->glue_marker->copies.size() == 0)
            glue_pts.free(iv->glue_marker);

        for (GenericEdgeType* ge : iv->edges) 
        {
            // disconnect
            ge->interior.erase(iv);
            if (ge->ends[0] == iv)   
                ge->ends[0] = nullptr;
            if (ge->ends[1] == iv)   
                ge->ends[1] = nullptr;
        }

        ivpool.free(iv);
    }


    // 删除一条交线段；
    inline void killIsctEdge(IsctEdgeType* ie) 
    {
        // an endpoint may be an original vertex
        if (ie->ends[1])
            ie->ends[1]->edges.erase(ie);
        iepool.free(ie);
    }


    inline void killOrigVert(OrigVertType* ov) {
        ovpool.free(ov);
    }


    inline void killOrigEdge(OrigEdgeType* oe) {
        oepool.free(oe);
    }


    bool hasIntersections();            // test for iscts, exit if one is found


    void findIntersections();


    void resolveAllIntersections();


public:

#ifdef TEST_CROSS_BOX_ACCE
    void getSelectedTrisIdx(const BBox3d& boxIsct);
#endif

    bool tryToFindIntersections();         

    // In that case, we can perturb the positions of points——给一对共面三角片施加小的扰动，使其不共面；
    void perturbPositions();

    // in order to give things another try, discard partial work
    void reset();


    // debug接口
public:
    void dumpIsctPoints(std::vector<Vec3d>* points);                                        // 输出相交的点
    void dumpIsctEdges(std::vector< std::pair<Vec3d, Vec3d> >* edges);          // 输出相交的边；

public:
    inline GeomBlob<TopoEdge*> edge_blob(TopoEdge* e);                  // 生成某条边的gb对象；

    inline void for_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)>);
    inline void bvh_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)>);


    inline BBox3d bboxFromTptr(TopoTri* t);

    inline BBox3d buildBox(TopoEdge* e) const;
    inline BBox3d buildBox(TopoTri* t) const;

    inline void marshallArithmeticInput(Empty3d::TriIn& input, TopoTri* t) const;
    inline void marshallArithmeticInput(Empty3d::EdgeIn& input, TopoEdge* e) const;
    inline void marshallArithmeticInput(Empty3d::TriEdgeIn& input, TopoEdge* e, TopoTri* t) const;
    inline void marshallArithmeticInput(Empty3d::TriTriTriIn& input, TopoTri* t0, TopoTri* t1, TopoTri* t2) const;

    bool checkIsct(TopoEdge* e, TopoTri* t) const;                           // 检查边e和三角片t是否有相交
    bool checkIsct(TopoTri* t0, TopoTri* t1, TopoTri* t2) const;

    Vec3d computeCoords(TopoEdge* e, TopoTri* t) const;
    Vec3d computeCoords(TopoTri* t0, TopoTri* t1, TopoTri* t2) const;

    void fillOutVertData(GluePointMarker* glue, VertData& data);
    void fillOutTriData(TopoTri* tri, TopoTri* parent);

private:
    class EdgeCache;

private:        // functions here to get around a GCC bug...
    void createRealPtFromGluePt(GluePointMarker* glue);
    void createRealTriangles(TriangleProblem* tprob, EdgeCache& ecache);
};


// EdgeCache类——IsctProblem的支持类，保存融合网格的边信息；
template<class VertData, class TriData>
class Mesh<VertData, TriData>::IsctProblem::EdgeCache
{
    // 支持类型
public:
    struct EdgeEntry
    {
        EdgeEntry(uint id) : vid(id) {}
        EdgeEntry() {}
        uint vid;
        TopoEdge* e;
    };

    // 成员数据：
public:
    IsctProblem* iprob;
    std::vector< ShortVec<EdgeEntry, 8> >   edges;

public:
    EdgeCache(IsctProblem* ip) : iprob(ip), edges(ip->mesh->verts.size()) {}

    TopoEdge* operator()(TopoVert* v0, TopoVert* v1) {
        uint i = v0->ref;
        uint j = v1->ref;
        if (i > j) std::swap(i, j);

        uint N = edges[i].size();
        for (uint k = 0; k < N; k++)
            if (edges[i][k].vid == j)
                return edges[i][k].e;
        // if not existing, create it
        edges[i].push_back(EdgeEntry(j));
        TopoEdge* e = edges[i][N].e = iprob->newEdge();
        e->verts[0] = v0;
        e->verts[1] = v1;
        v0->edges.push_back(e);
        v1->edges.push_back(e);

        return e;
    }

    TopoEdge* getTriangleEdge(GenericTriType* gt, uint k, TopoTri* big_tri)         // k = 0, 1, or 2
    {
        GenericVertType*   gv0 = gt->verts[(k + 1) % 3];
        GenericVertType*   gv1 = gt->verts[(k + 2) % 3];
        TopoVert*    v0 = gv0->concrete;
        TopoVert*    v1 = gv1->concrete;
        // if neither of these are intersection points,
        // then this is a pre-existing edge...
        TopoEdge*    e = nullptr;
        if (typeid(gv0) == typeid(OrigVertType*) &&
            typeid(gv1) == typeid(OrigVertType*)
            ) {
            // search through edges of original triangle...
            for (uint c = 0; c < 3; c++) {
                TopoVert* corner0 = big_tri->verts[(c + 1) % 3];
                TopoVert* corner1 = big_tri->verts[(c + 2) % 3];
                if ((corner0 == v0 && corner1 == v1) ||
                    (corner0 == v1 && corner1 == v0)) {
                    e = big_tri->edges[c];
                }
            }
            ENSURE(e); // Yell if we didn't find an edge
        }
        // otherwise, we need to check the cache to find this edge
        else
        {
            e = operator()(v0, v1);
        }
        return e;
    }

    TopoEdge* maybeEdge(GenericEdgeType* ge)
    {
        uint i = ge->ends[0]->concrete->ref;
        uint j = ge->ends[1]->concrete->ref;
        if (i > j) std::swap(i, j);

        uint N = edges[i].size();
        for (uint k = 0; k < N; k++)
            if (edges[i][k].vid == j)
                return edges[i][k].e;
        // if we can't find it
        return nullptr;
    }
};


// TriTripleTemp类——处理ttt相交情形的辅助类；
struct TriTripleTemp
{
    TopoTri* t0;
    TopoTri* t1;
    TopoTri* t2;

    TriTripleTemp(TopoTri* tp0, TopoTri* tp1, TopoTri* tp2) :
        t0(tp0), t1(tp1), t2(tp2)
    {}
};



//////////////////////////////////////////////////////////////////////////////////函数实现：

// 查找当前tp对象的iedges数据，遍历已生成的需要新增的边数据，如果某边在
template<uint LEN> inline
IsctEdgeType* find_edge(ShortVec<IsctEdgeType*, LEN>& vec, TopoTri* key)
{
    // 项目中具体使用时，vec为当前tp对象的iedges数组，保存的是需要新增的边
    for (IsctEdgeType* ie : vec) 
    {
        if (ie->other_tri_key == key)
            return ie;
    }
    return nullptr;
}


// commonVert()——若t0, t1有共享顶点，返回该顶点指针；
inline TopoVert* commonVert(TopoTri* t0, TopoTri* t1)
{
    for (uint i = 0; i < 3; i++) 
    {
        for (uint j = 0; j < 3; j++)
        {
            if (t0->verts[i] == t1->verts[j])
                return t0->verts[i];
        }
    }
    return nullptr;
}

inline bool hasCommonVert(TopoTri* t0, TopoTri* t1)
{
    return (t0->verts[0] == t1->verts[0] ||
        t0->verts[0] == t1->verts[1] ||
        t0->verts[0] == t1->verts[2] ||
        t0->verts[1] == t1->verts[0] ||
        t0->verts[1] == t1->verts[1] ||
        t0->verts[1] == t1->verts[2] ||
        t0->verts[2] == t1->verts[0] ||
        t0->verts[2] == t1->verts[1] ||
        t0->verts[2] == t1->verts[2]);
}

inline bool hasCommonVert(TopoEdge* e, TopoTri* t)
{
    return (e->verts[0] == t->verts[0] ||
        e->verts[0] == t->verts[1] ||
        e->verts[0] == t->verts[2] ||
        e->verts[1] == t->verts[0] ||
        e->verts[1] == t->verts[1] ||
        e->verts[1] == t->verts[2]);
}

inline void disconnectGE(GenericEdgeType* ge)
{
    ge->ends[0]->edges.erase(ge);
    ge->ends[1]->edges.erase(ge);
    for (IsctVertType* iv : ge->interior)
        iv->edges.erase(ge);
}


// 遍历shortVec中相邻元素组成的元素对；
template<class T, uint LEN> inline
void for_pairs(ShortVec<T, LEN>& vec,   std::function<void(T&, T&)> func) 
{
    for (uint i = 0; i < vec.size(); i++)
        for (uint j = i + 1; j < vec.size(); j++)
            func(vec[i], vec[j]);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::for_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)> func) 
{
    bool aborted = false;
    TopoCache::tris.for_each(\
        [&](TopoTri* t) 
        {
            TopoCache::edges.for_each([&](TopoEdge* e) 
                {
                    if (!aborted) 
                    {
                        if (!func(e, t))
                            aborted = true;
                    }
                });
        });
}


// edge_blob()——生成某条边的GeomBlob对象；
template<class VertData, class TriData> inline
GeomBlob<TopoEdge*> Mesh<VertData, TriData>::IsctProblem::edge_blob(TopoEdge* e) 
{
    GeomBlob<TopoEdge*>  blob;
    blob.bbox = buildBox(e);
    blob.point = (blob.bbox.minp + blob.bbox.maxp) / 2.0;
    blob.id = e;             
    return blob;
}


// bvh_edge_tri()——生成网格所有边的层次包围盒，传入函数子func，遍历作用于层次包围盒所有节点，若函数子func返回false，则终止遍历；
template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::bvh_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)> func) 
{
    this->edge_geoms.clear();

#ifdef TEST_CROSS_BOX_ACCE            // 三角片是否在交叉包围盒内的标记；
    this->selectedTriFlags.resize(this->mesh->tris.size(), 0);
    for (const auto& index : this->selectedTrisIdx)
        this->selectedTriFlags[index] = 1;
#endif

    // 1. 生成所有边的GeomBlob信息；
    TopoCache::edges.for_each(\
        [&](TopoEdge* e) 
        {
            this->edge_geoms.push_back(edge_blob(e));
        });
 

    // 2. 生成所有边的AABVH对象； 
    this->bvhObj.build(this->edge_geoms);                      // by Tora——层次包围盒对象存储到成员数据中，方便debug;
 

    // 3. 遍历所有拓扑三角片；
    bool aborted = false;
    TopoCache::tris.for_each([&](TopoTri* t)                // 对所有三角片的遍历
        {
#ifdef TEST_CROSS_BOX_ACCE
            if (1 == this->selectedTriFlags[t->ref])
            {
#endif
                BBox3d bbox = buildBox(t);              // 当前三角片的包围盒；
                if (!aborted)
                {
                    this->bvhObj.for_each_in_box(bbox, [&](TopoEdge* e)
                        {
                            if (!func(e, t))
                                aborted = true;
                        });
                }
#ifdef TEST_CROSS_BOX_ACCE
            }
#endif
        });
}


// step3.1.2. 检查网格中是否有三角片相交的情形，生成被穿刺的三角片对应的triangleProblem对象，return true; 碰到退化情形则返回false;
template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::tryToFindIntersections()
{
    //  注意：若出现了退化情形，则返回false
    Empty3d::degeneracy_count = 0;

    // 1. 生成融合网格所有边的层次包围盒aabvh，依次使用每一个三角片遍历aabvh，寻找边和三角片的交点；
    tiktok& tt = tiktok::getInstance();
    tt.takeArecord();
    bvh_edge_tri([&](TopoEdge* eisct, TopoTri* tisct)->bool 
        {
            // 1.1 检测当前三角片是否被穿刺；
            if (checkIsct(eisct, tisct)) 
            {
#ifdef TEST_CROSS_BOX_ACCE
                // for test——若当前穿刺边所在的三角片，全都在包围盒之外，则跳过；
                bool allSpearTriOut = true;
                for (unsigned i = 0; i < eisct->tris.user_size; ++i)
                {
                    TopoTri* spearTriPtr = eisct->tris[i];
                    if (nullptr == spearTriPtr)
                        continue;
                    if (1 == this->selectedTriFlags[spearTriPtr->ref])
                    {
                        allSpearTriOut = false;
                        break;
                    }
                }
                if (allSpearTriOut)
                    return true;
#endif

                // 1.1.1 创建GluePointMarker对象，在计算三角片和边交点时有用；
                GluePointMarker*      glue = newGluePt();
                glue->edge_tri_type = true;
                glue->e = eisct;
                glue->t[0] = tisct;

                // 1.1.2 当前被穿刺的三角片生成triangleProblem对象，生成需要新加的点；
                /*
                    当前三角片被穿刺 → 生成该三角片对应的tiangleProblem对象，其内部生成1个新增顶点iv，2条新增边ie0, ie1；
                    iv为该三角片被穿刺处的点，在三角片内部；
                    此时两条新增边的状态完全相同，端点都是未定状态，其中一个端点取为iv，另一个端点的指针是nullptr;
                */
                TriangleProblem* tpPtr = getTprob(tisct);
                IsctVertType* iv = tpPtr->addInteriorEndpoint(this, eisct, glue);           


                // 1.1.3 生成当前三角片需要新加的边数据；
                for (TopoTri* spearTri : eisct->tris)        // 遍历包含穿刺边的所有三角片；这些三角片也需要新加点和边；
                {
#ifdef TEST_CROSS_BOX_ACCE
                    // for test——tisct一定是交叉包围盒内的三角片，eisct关联的所有三角片中，可能存在交叉包围盒外的三角片，尝试跳过；                   
                    if (0 == this->selectedTriFlags[spearTri->ref])
                        continue;
#endif
                    TriangleProblem* spearTPptr = getTprob(spearTri);
                    spearTPptr->addBoundaryEndpoint(this, tisct, eisct, iv);
                }
            }

            // 1.2 
            if (Empty3d::degeneracy_count > 0)
                return false;               // 对层次包围盒的遍历终止
            else
                return true;                // continue
        });
    tt.takeArecord();
    auto rIter = tt.records.rbegin();
    DWORD time2 = *rIter++;
    DWORD time1 = *rIter;
    std::cout << "遍历BVH耗时：" << time2 - time1 << std::endl;
 
    // 2. ？？？退化处理？
    if (Empty3d::degeneracy_count > 0) 
        return false;                     // restart / abort

    // 3. 遍历tp对象，寻找ttt相交情形——即三个三角片t0, t1, t2；两两都相交
    tt.takeArecord();
    std::vector<TriTripleTemp> triples;
    tprobs.for_each([&](TriangleProblem* tprob) 
        {
            TopoTri* t0 = tprob->the_tri;

            for_pairs<IsctEdgeType*, 2>(tprob->iedges, \
                [&](IsctEdgeType*& ie1, IsctEdgeType*& ie2)                 // 对当前tp对象交线段的遍历，是成对地遍历；
                {
                    TopoTri* t1 = ie1->other_tri_key;               // 交线段关联的另一个三角片；
                    TopoTri* t2 = ie2->other_tri_key;
 
                    if (t0 < t1 && t0 < t2)             //      ！！！感觉这个if没必要；
                    {
#ifdef TEST_CROSS_BOX_ACCE
                        // 如果t1是交叉盒外的三角片，跳过；
                        if (nullptr == t1->data)
                            return;
#endif

                        TriangleProblem* prob1 = reinterpret_cast<TriangleProblem*>(t1->data);              // t1的tp对象；

                        for (IsctEdgeType* ie : prob1->iedges)      // 对三角片t1的交线段的遍历；
                        {
                            if (ie->other_tri_key == t2)            // 若t1和t2有交线段，则说明t0, t1, t2两两相交；
                            {
                                triples.push_back(TriTripleTemp(t0, t1, t2));   

                                // for debug_
#if 0
                                TriangleProblemInfo tpInfo(*tprob);
                                debugWriteTriMesh("ttt_tpTri", *this->mesh, tpInfo.the_tri.ref);

                                std::vector<CorkVertex> tpAllIsctVers;
                                for (const auto& iv : tpInfo.isct_versInfo)
                                    tpAllIsctVers.push_back(CorkVertex(iv.coord));
                                debugWriteVers("tpAllIsctVers", tpAllIsctVers);

                                CorkEdges isctEdges;
                                std::vector<CorkVertex> sickEdgeVers;
                                for (const auto& ie : tpInfo.isct_edgesInfo)
                                {
                                    if (ie.edgeVers.size() == 2) 
                                    {
                                        CorkVertex va(ie.edgeVers[0].coord);
                                        CorkVertex vb(ie.edgeVers[1].coord);
                                        CorkEdge iedge(va, vb);
                                        isctEdges.add(iedge);
                                    }
                                    else
                                    {
                                        CorkVertex va(ie.edgeVers[0].coord);
                                        sickEdgeVers.push_back(va);
                                    }
                                }
                                debugWriteEdges("tpAllIsctEdges", isctEdges);
                                debugWriteVers("tpSickEdgeVers", sickEdgeVers);

                                CorkMesh tttMesh;
                                selectedTris2Mesh(tttMesh, *this->mesh, std::vector<uint>{t0->ref, t1->ref, t2->ref});
                                debugWriteTriMesh("ttt_t0", *this->mesh, t0->ref);
                                debugWriteTriMesh("ttt_t1", *this->mesh, t1->ref);
                                debugWriteTriMesh("ttt_t2", *this->mesh, t2->ref);
                                debugWriteMesh("tttMesh", tttMesh);
                                getchar();
#endif
                            }
                        }
                    }
                });
        });
    tt.takeArecord();
    rIter = tt.records.rbegin();
    time2 = *rIter++;
    time1 = *rIter;
    std::cout << "查找ttt相交耗时：" << time2 - time1 << std::endl;

    // 4. 验证ttt相交情形是真实存在，还是退化情形；有退化情形返回false，终止程序；
    for (TriTripleTemp t : triples) 
    {
        if (!checkIsct(t.t0, t.t1, t.t2))               // 重新检查一遍三个三角片的交叉情况
            continue;

        // Abort if we encounter a degeneracy
        if (Empty3d::degeneracy_count > 0)   
            break;

        GluePointMarker*      glue = newGluePt();
        glue->edge_tri_type = false;
        glue->t[0] = t.t0;
        glue->t[1] = t.t1;
        glue->t[2] = t.t2;
        getTprob(t.t0)->addInteriorPoint(this, t.t1, t.t2, glue);
        getTprob(t.t1)->addInteriorPoint(this, t.t0, t.t2, glue);
        getTprob(t.t2)->addInteriorPoint(this, t.t0, t.t1, glue);
 
    }

    if (Empty3d::degeneracy_count > 0) 
        return false;   


    return true;
}


// step3.1.1.  perturbPositions()——给一对共面三角片施加小的扰动，使其不共面；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::perturbPositions()
{
    const double EPSILON = 1.0e-5;                          // 很小的扰动数；
    for (Vec3d& coord : quantized_coords) 
    {
        Vec3d perturbation(\
            Quantization::quantize(drand(-EPSILON, EPSILON)),
            Quantization::quantize(drand(-EPSILON, EPSILON)),
            Quantization::quantize(drand(-EPSILON, EPSILON)));          // 很小的扰动向量；

        coord += perturbation;
    }
}


template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::reset()
{
    // the data pointer in the triangles points to tproblems
    // that we're about to destroy,
    // so zero out all those pointers first!
    tprobs.for_each([](TriangleProblem* tprob) 
        {
            TopoTri* t = tprob->the_tri;
            t->data = nullptr;
        });

    glue_pts.clear();
    tprobs.clear();

    ivpool.clear();
    ovpool.clear();
    iepool.clear();
    oepool.clear();
    sepool.clear();
    gtpool.clear();
}


// step3.1. 寻找边和三角片的交点
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::findIntersections()
{
    // 3.1.1 初始扰动处理
    int nTrys = 5;
    perturbPositions();                 // always perturb for safety...

    // 3.1.2 交叉检测，生成层次包围盒，生成交叉三角片的triangleProblem对象；
    while (nTrys > 0) 
    {
        if (!tryToFindIntersections())  // 若返回false，说明有退化情形，施加扰动，重新检测；
        {
            reset();
            perturbPositions();       
            nTrys--;
        }
        else 
            break;
    }


    // 3.1.3 错误处理——若尝试次数用尽后依然有退化情形，退出程序；
    if (nTrys <= 0)
    {
        CORK_ERROR("Ran out of tries to perturb the mesh");
        exit(1);
    }

    // 查找、删除病态交线段、病态交点；
#ifdef IGNORE_SELF_ISCT
    tiktok& tt = tiktok::getInstance();
    tt.takeArecord();

        // 交线段ie对象存在this->iepool中，tp对象中存有这些ie的指针；
    std::set<IsctVertType*> allSickVers;
    std::set<IsctEdgeType*> allSickEdges;
    std::set<IsctVertType*> sickVersPtrSet;
    std::set<IsctEdgeType*> sickEdgesPtrSet;

    // 查找病态交线段——存在缺失端点则判定为病态交线段；
    for (auto& ie : this->iepool)
    {
        if (nullptr == ie.ends[1] || nullptr == ie.ends[0])
            sickEdgesPtrSet.insert(&ie);
    }

    // 汇入总信息：
    for (auto& iePtr : sickEdgesPtrSet)
        allSickEdges.insert(iePtr);

    do
    {
        // 查找病态交点——若该交点关联的边全部都是病态的边，则该点为病态交点；
        sickVersPtrSet.clear();
        this->ivpool.for_each([&](IsctVertType* iv)                // 对交点池的遍历：
            {
                auto relaEdges = iv->edges.toVec();
                for (auto& oePtr : relaEdges)       // 对该交点关联的边的遍历：
                {
                    IsctEdgeType* iePtr = reinterpret_cast<IsctEdgeType*>(oePtr);
                    if (isIn(allSickEdges, iePtr))       // 若该边是sickedge，从该交点的edges数组中删除该边 
                        iv->edges.erase(oePtr);
                }
                if (0 == iv->edges.size())
                    sickVersPtrSet.insert(iv);                  // 若该交点关联的边全部都是病态的边，则该点为病态交点；
            });

        // 汇入总信息：
        for (auto& ivPtr : sickVersPtrSet)
            allSickVers.insert(ivPtr);

        // 删除上一轮查找出的病态交线段、本轮查找出的病态交点
        for (auto& iePtr : sickEdgesPtrSet)
            this->killIsctEdge(iePtr);
        for (auto& ivPtr : sickVersPtrSet)
            this->killIsctVert(ivPtr);

        // 查找病态交线段：
        sickEdgesPtrSet.clear();
        this->iepool.for_each([&](IsctEdgeType* iePtr)
            {
                // 若缺失端点，判定为病态交线段；
                if (nullptr == iePtr->ends[0] || nullptr == iePtr->ends[1])
                    sickEdgesPtrSet.insert(iePtr);
            });

        // 汇入总信息；
        for (auto& iePtr : sickEdgesPtrSet)
            allSickEdges.insert(iePtr);

    } while (!sickEdgesPtrSet.empty());

    unsigned ivCount = this->ivpool.size();
    unsigned ieCount = this->iepool.size();

    // 每一个tp对象中删除掉病态的交点和交线段的指针，： 
    this->tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
        {
            auto ivertsVec = tpPtr->iverts.toVec();
            for (auto& ivPtr : ivertsVec)
            {
                if (isIn(allSickVers, ivPtr))
                    tpPtr->iverts.erase(ivPtr);
                tpPtr->iverts.erase(nullptr);
            }

            // ！！！shortVec使用erase()时容量会改变，在for循环中使用erase()貌似容易出错；
            auto copyVec = tpPtr->iedges.toVec();
            for (auto& iePtr : copyVec)
            {
                if (isIn(allSickEdges, iePtr))
                    tpPtr->iedges.erase(iePtr);
                tpPtr->iedges.erase(nullptr);
            }
        });

    // 若tp对象中已经没有交线段，或所有ie指针都为空，删除该tp对象；
    std::set<CorkMesh::TriangleProblem*> sickTpPtrs;
    this->tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
        {
            auto ieVec = tpPtr->iedges.toVec();
            for (auto& iePtr : ieVec)
            {
                if (nullptr != iePtr)
                    return;
            }
            sickTpPtrs.insert(tpPtr);
        });

    for (auto& tpPtr : sickTpPtrs)
        this->tprobs.free(tpPtr);

    tt.takeArecord();
    auto iter = tt.records.rbegin();
    DWORD time2 = *iter++;
    DWORD time1 = *iter;
    std::cout << "查找、删除病态交线段、交点、tp对象耗时：" << time2 - time1 << std::endl;

#else

    // 3.1.4  遍历triangleProblem对象；若存在某边缺少端点，尝试纠正数据；
    /*
         ok all points put together, all triangle problems assembled.
         Some intersection edges may have original vertices as endpoints
         we consolidate the problems to check for cases like these.
    */
    tprobs.for_each([&](Mesh<VertData, TriData>::TriangleProblem* tprob)
        {
            tprob->consolidate(this);
        });
#endif

}


// 判断网格是否有自交；
template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::hasIntersections()
{
    bool foundIsct = false;             // 是否发现有相交的标志；
    Empty3d::degeneracy_count = 0;

    // Find some edge-triangle intersection point...
    bvh_edge_tri([&](TopoEdge* eisct, TopoTri* tisct)->bool
        {
            if (checkIsct(eisct, tisct)) 
            {
                foundIsct = true;
                return false;       // break;
            }

            if (Empty3d::degeneracy_count > 0) 
                return false;       // break;

            return true;            // continue
         });


    if (Empty3d::degeneracy_count > 0 || foundIsct) 
    {
        std::cout << "This self-intersection might be spurious. "
            "Degeneracies were detected." << std::endl;
        return true;
    }
    else 
        return false;
}


// 生成某条边对应的轴对象包围盒BBox3d对象；
template<class VertData, class TriData> inline
BBox3d Mesh<VertData, TriData>::IsctProblem::buildBox(TopoEdge* e) const
{
    Vec3d p0 = vPos(e->verts[0]);
    Vec3d p1 = vPos(e->verts[1]);
    return BBox3d(min(p0, p1), max(p0, p1));
}


// 生成某个三角片对应的BBox3d对象；
template<class VertData, class TriData> inline
BBox3d Mesh<VertData, TriData>::IsctProblem::buildBox(TopoTri* t) const
{
    Vec3d p0 = vPos(t->verts[0]);
    Vec3d p1 = vPos(t->verts[1]);
    Vec3d p2 = vPos(t->verts[2]);
    return BBox3d(min(p0, min(p1, p2)), max(p0, max(p1, p2)));
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::marshallArithmeticInput(Empty3d::EdgeIn& input, TopoEdge* e) const 
{
    input.p[0] = vPos(e->verts[0]);
    input.p[1] = vPos(e->verts[1]);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::marshallArithmeticInput(Empty3d::TriIn& input, TopoTri* t) const 
{
    input.p[0] = vPos(t->verts[0]);
    input.p[1] = vPos(t->verts[1]);
    input.p[2] = vPos(t->verts[2]);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::marshallArithmeticInput(Empty3d::TriEdgeIn& input, TopoEdge* e, TopoTri* t) const 
{
    marshallArithmeticInput(input.edge, e);
    marshallArithmeticInput(input.tri, t);
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::marshallArithmeticInput(Empty3d::TriTriTriIn& input, TopoTri* t0, TopoTri* t1, TopoTri* t2) const 
{
    marshallArithmeticInput(input.tri[0], t0);
    marshallArithmeticInput(input.tri[1], t1);
    marshallArithmeticInput(input.tri[2], t2);
}


// checkIsct()——检测一条边和一个三角片是否有相交；
template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::checkIsct(TopoEdge* e, TopoTri* t) const
{
    // 生成该三角片和边的AABB;
    BBox3d      ebox = buildBox(e);
    BBox3d      tbox = buildBox(t);

    // 若判定没有相交，return false;
    if (!hasIsct(ebox, tbox))
        return      false;

    // 若三角片和边有公共顶点，则认为此种情形不属于相交；
    if (hasCommonVert(e, t))
        return      false;

    Empty3d::TriEdgeIn input;
    marshallArithmeticInput(input, e, t);
    bool empty = Empty3d::emptyExact(input);

    // by Tora——若碰到退化情形，打印出第一个退化情形相关的三角片和顶点：
    if (Empty3d::degeneracy_count > 0)
    {
        std::vector<CorkVertex> degEdgeVers(2);
        CorkMesh degTriMesh;
        selectedTris2Mesh(degTriMesh, *this->mesh, std::vector<uint>{t->ref});
        debugWriteMesh("退化三角片", degTriMesh);
        const auto& degTri = this->mesh->tris[t->ref];      // debugTri.a == 3617，点坐标为：(-16.672457, -0.54398, 13.377321)
        uint index0 = e->verts[0]->ref;                                  // index0 == 1030, 点坐标为：(-16.672457, -0.54398, 13.377321);  
        uint index1 = e->verts[1]->ref;                                  // 三角片和边交叉于三角片的一个顶点，该边一端点和该三角片顶点坐标相同，却是网格中有不同索引值的两个顶点；
        degEdgeVers[0] = this->mesh->verts[index0];
        degEdgeVers[1] = this->mesh->verts[index1];
        debugWriteVers("退化边的顶点", degEdgeVers);
        
        std::cout << "三角片的三个顶点：" << degTri.a << ", " << degTri.b << ", " << degTri.c << std::endl;
        this->mesh->verts[degTri.a].disp();
        this->mesh->verts[degTri.b].disp();
        this->mesh->verts[degTri.c].disp();

        std::cout << "边的两个端点：" << index0 << ", " << index1 << std::endl;
        this->mesh->verts[index0].disp();
        this->mesh->verts[index1].disp();
 
        std::cout << "degeneracy occured!!!" << std::endl;
    }

    return !empty;
}


template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::checkIsct(TopoTri* t0, TopoTri* t1, TopoTri* t2) const {
    // This function should only be called if we've already
    // identified that the intersection edges
    //      (t0,t1), (t0,t2), (t1,t2)
    // exist.
    // From this, we can conclude that each pair of triangles
    // shares no more than a single vertex in common.
    //  If each of these shared vertices is different from each other,
    // then we could legitimately have a triple intersection point,
    // but if all three pairs share the same vertex in common, then
    // the intersection of the three triangles must be that vertex.
    // So, we must check for such a single vertex in common amongst
    // the three triangles
    TopoVert* common = commonVert(t0, t1);
    if (common) {
        for (uint i = 0; i < 3; i++)
            if (common == t2->verts[i])
                return      false;
    }

    Empty3d::TriTriTriIn input;
    marshallArithmeticInput(input, t0, t1, t2);
    //bool empty = Empty3d::isEmpty(input);
    bool empty = Empty3d::emptyExact(input);
    return !empty;
}


template<class VertData, class TriData>
Vec3d Mesh<VertData, TriData>::IsctProblem::computeCoords(TopoEdge* e, TopoTri* t) const
{
    Empty3d::TriEdgeIn input;
    marshallArithmeticInput(input, e, t);
    Vec3d coords = Empty3d::coordsExact(input);
    return coords;
}


template<class VertData, class TriData>
Vec3d Mesh<VertData, TriData>::IsctProblem::computeCoords(TopoTri* t0, TopoTri* t1, TopoTri* t2) const 
{
    Empty3d::TriTriTriIn input;
    marshallArithmeticInput(input, t0, t1, t2);
    Vec3d coords = Empty3d::coordsExact(input);
    return coords;
}


template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::fillOutVertData(GluePointMarker* glue, VertData& data)
{
    if (glue->split_type)
    {
        // manually inserted split point
        uint v0i = glue->e->verts[0]->ref;
        uint v1i = glue->e->verts[1]->ref;
        data.isctInterpolate(TopoCache::mesh->verts[v0i], TopoCache::mesh->verts[v1i]);
    }
    else
    {
        if (glue->edge_tri_type)
        { 
            // edge-tri type
            IsctVertEdgeTriInput<VertData, TriData>      input;
            for (uint k = 0; k < 2; k++)
            {
                uint    vid = glue->e->verts[k]->ref;
                input.e[k] = &(TopoCache::mesh->verts[vid]);
            }
            for (uint k = 0; k < 3; k++)
            {
                uint    vid = glue->t[0]->verts[k]->ref;
                input.t[k] = &(TopoCache::mesh->verts[vid]);
            }
            data.isct(input);
        }
        else 
        { 
            // tri-tri-tri type
            IsctVertTriTriTriInput<VertData, TriData>    input;
            for (uint i = 0; i < 3; i++) 
            {
                for (uint j = 0; j < 3; j++) 
                {
                    uint    vid = glue->t[i]->verts[j]->ref;
                    input.t[i][j] = &(TopoCache::mesh->verts[vid]);
                }
            }
            data.isct(input);
        }
    }
}


template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::subdivide_tri(uint t_piece_ref, uint t_parent_ref) 
{
    SubdivideTriInput<VertData, TriData>     input;
    input.pt = &(tris[t_parent_ref].data);
    for (uint k = 0; k < 3; k++) 
    {
        input.pv[k] = &(verts[tris[t_parent_ref].v[k]]);
        input.v[k] = &(verts[tris[t_piece_ref].v[k]]);
    }

    tris[t_piece_ref].data.subdivide(input);        // 执行三角剖分：
}


template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::fillOutTriData(TopoTri* piece, TopoTri* parent) 
{
    TopoCache::mesh->subdivide_tri(piece->ref, parent->ref);
}


template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::createRealPtFromGluePt(GluePointMarker* glue) 
{
    ENSURE(glue->copies.size() > 0);

    TopoVert*        v = TopoCache::newVert();          // 生成新顶点；
    VertData& data = TopoCache::mesh->verts[v->ref];
    data.pos = glue->copies[0]->coord;

    fillOutVertData(glue, data);
    for (IsctVertType* iv : glue->copies)
        iv->concrete = v;
}


// createRealTriangles()——根据tp对象中的三角剖分信息，生成新三角片存入网格；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::createRealTriangles(TriangleProblem* tprob, EdgeCache& ecache) 
{
    // for debug——用于监视的引用：
    const auto& meshTris = this->mesh->tris;

    for (GenericTriType* gt : tprob->gtris)             // 遍历对象中的gt对象
    {
        TopoTri*        t = TopoCache::newTri();                // 原融合网格中生成新三角片；
        gt->concrete = t;           
        Tri& tri = TopoCache::mesh->tris[t->ref];           

        for (uint k = 0; k < 3; k++)            // 对新生成三角片顶点的遍历；
        {
            TopoVert*    v = gt->verts[k]->concrete;
            t->verts[k] = v;
            v->tris.push_back(t);
            tri.v[k] = v->ref;

            TopoEdge*    e = ecache.getTriangleEdge(gt, k, tprob->the_tri);
            e->tris.push_back(t);
            t->edges[k] = e;
        }

        fillOutTriData(t, tprob->the_tri);                          // 原融合网格中生成新三角片；
    }

    // Once all the pieces are hooked up, let's kill the old triangle!
    TopoCache::deleteTri(tprob->the_tri);
}


// step3.2——执行插点、三角剖分、插入新三角片的操作；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::resolveAllIntersections()
{
    // for debug——一些用于监视的引用：
    const auto& meshTris = this->mesh->tris;
    const auto& meshVers = this->mesh->verts;
    unsigned versCountOri = meshVers.size();

    // 3.2.1.执行三角剖分，新三角片信息存入各个TP对象中，此时没有存入原网格；
    this->tprobs.for_each([&](TriangleProblem* tprob) 
        {
            tprob->subdivide(this);
        });

    // 3.2.2 遍历所有glue对象，生成新的顶点插入到原融合网格中：
    this->glue_pts.for_each([&](GluePointMarker* glue) 
        {
            createRealPtFromGluePt(glue);
        });

    // Now that we have concrete vertices plugged in, we can go through the diced triangle pieces and create concrete triangles  for each of those.

    // Along the way, let's go ahead and hook up edges as appropriate

    // 3.2.3 根据tp对象中的三角剖分信息，生成新三角片存入网格；
    EdgeCache ecache(this);
    this->tprobs.for_each([&](TriangleProblem* tprob) 
        {
            createRealTriangles(tprob, ecache);     // 会进行三角剖分：
        });

    // 3.2.4 mark all edges as normal by zero-ing out the data pointer
    TopoCache::edges.for_each([](TopoEdge* e) 
        {
            e->data = 0;
        });

    // then iterate over the edges formed by intersections (i.e. those edges without the boundary flag set in each triangle)
     
    // and mark those by setting the data pointer
    
    // 3.2.5
    iepool.for_each([&](IsctEdgeType* ie) 
        {
            // every ie must be non-boundary
            TopoEdge* e = ecache.maybeEdge(ie);
            ENSURE(e);
            e->data = (void*)1;
        });

    // 3.2.6
    sepool.for_each([&](SplitEdgeType* se) 
        {
            TopoEdge* e = ecache.maybeEdge(se);
            ENSURE(e);
            e->data = (void*)1;
        });


    // This basically takes care of everything EXCEPT one detail
    // *) The base mesh data structures still need to be compacted

    // This detail should be handled by the calling code...
}


// 输出交点；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::dumpIsctPoints(std::vector<Vec3d>* points) 
{
    points->resize(glue_pts.size());
    uint write = 0;
    glue_pts.for_each([&](GluePointMarker* glue) 
        {
            ENSURE(glue->copies.size() > 0);
            IsctVertType*       iv = glue->copies[0];
            (*points)[write] = iv->coord;
            write++;
        });
}


// 输出和三角片相交的边；
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::dumpIsctEdges(std::vector< std::pair<Vec3d, Vec3d> >* edges) 
{
    edges->clear();
    tprobs.for_each([&](TriangleProblem* tprob) 
        {
            for (IsctEdgeType* ie : tprob->iedges)
            {
                GenericVertType* gv0 = ie->ends[0];
                GenericVertType* gv1 = ie->ends[1];
                edges->push_back(std::make_pair(gv0->coord, gv1->coord));
            }
        });
}


// 测试接口——
template<class VertData, class TriData>
void Mesh<VertData, TriData>::testingComputeStaticIsctPoints(std::vector<Vec3d>* points) 
{
    IsctProblem iproblem(this);

    iproblem.findIntersections();
    iproblem.dumpIsctPoints(points);
}


// 测试接口——
template<class VertData, class TriData>
void Mesh<VertData, TriData>::testingComputeStaticIsct(std::vector<Vec3d>* points,  std::vector< std::pair<Vec3d, Vec3d> >* edges) 
{
    IsctProblem iproblem(this);

    iproblem.findIntersections();
    iproblem.dumpIsctPoints(points);
    iproblem.dumpIsctEdges(edges);
}


// 三角片求交，找出每对相交三角片的交线段，插入新顶点，执行三角剖分
template<class VertData, class TriData>
void Mesh<VertData, TriData>::resolveIntersections()
{
    // 三角剖分过程中：先将插入新点的三角片删除，然后将三角剖分产生的新三角片尾部插入；

    IsctProblem iproblem(this);

#ifdef TEST_CROSS_BOX_ACCE
    iproblem.getSelectedTrisIdx(this->boxIsct);
#endif

    iproblem.findIntersections();                       // 寻找三角片交线；

    iproblem.resolveAllIntersections();

    iproblem.commit();
}


template<class VertData, class TriData>
bool Mesh<VertData, TriData>::isSelfIntersecting()
{
    IsctProblem iproblem(this);

    return iproblem.hasIntersections();
}


#ifdef TEST_CROSS_BOX_ACCE

// by Tora——确定交叉包围盒内的三角片
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::getSelectedTrisIdx(const BBox3d& boxIsct)
{
    this->boxIsct = boxIsct;
    this->selectedTrisIdx.clear();
    this->selectedTrisIdx.reserve(this->mesh->tris.size());
    for (uint i = 0; i < this->mesh->tris.size(); ++i)
    {
        const auto& tri = mesh->tris[i];
        CorkVertex ver0 = mesh->verts[tri.a];
        CorkVertex ver1 = mesh->verts[tri.b];
        CorkVertex ver2 = mesh->verts[tri.c];

        // 三角片三个顶点中只要有一个在此盒内就被选中：
        bool flag0 = isIn(ver0.pos, boxIsct);
        bool flag1 = isIn(ver1.pos, boxIsct);
        bool flag2 = isIn(ver2.pos, boxIsct);

        if (flag0 | flag1 | flag2)
            this->selectedTrisIdx.push_back(i);
    }
}
#endif
