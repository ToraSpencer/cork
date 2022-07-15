#pragma once
 
// ����Ƭ��

/////////////////////////////////////////////////////////////////////////////////////////һЩ֧����



// GenericVertType�ࡪ��������ࣻ
struct GenericVertType
{
    virtual ~GenericVertType() {}
    TopoVert*                    concrete;       // concret��Ϊ��ָ�룬��˵���˵����ں�����������ʵ���ڣ�Ϊ��ָ��Ļ����ܸõ��Ǵ������ıߣ���û����ӵ��ں������У�
    Vec3d                   coord;              // �����ꣻ

    bool                    boundary;           //  true���������������ڱ��ϣ�false��������������������Ƭ�ڲ���
    uint                    idx;                         // temporary for triangulation marshalling

    ShortVec<GenericEdgeType*, 2>       edges;      // �õ�����ıߣ�

public:
    GenericVertType() {}
};


struct OrigVertType : public GenericVertType {};


// IsctVertType�ࡪ���²�Ķ��㣬�����֣�һ��������Ƭ�ڲ���һ���ڱ��ϣ�
struct IsctVertType : public GenericVertType
{
    GluePointMarker*                  glue_marker;
 
};


// GluePointMarker�ࡪ��������
struct GluePointMarker
{
    ShortVec<IsctVertType*, 3>      copies;    // list of all the vertices to be glued...������ò���Ǵ��²���ĵ㣿����
    bool                    split_type;                      // splits are introduced manually, not via intersection,  and therefore use only e pointer
    bool                    edge_tri_type;               // true if edge-tri intersection, false if tri-tri-tri
    TopoEdge* e;
    TopoTri* t[3];
};


// GenericEdgeType�ࡪ���߻��ࣻ
struct GenericEdgeType
{
    virtual ~GenericEdgeType() {}
    TopoEdge*                    concrete;          // ��concret��Ϊ��ָ�룬��˵���˱����ں�����������ʵ���ڣ�Ϊ��ָ��Ļ����ܸñ��Ǵ������ıߣ���û����ӵ��ں������У�

    bool                    boundary;
    uint                    idx;                            // temporary for triangulation marshalling

    GenericVertType*                   ends[2];
    ShortVec<IsctVertType*, 1>      interior;               // ����ñ߲��ǽ��߶Σ���ԭʼ�����ݣ�����Ҫ����ñ��ڵĶ�����룻
};


struct OrigEdgeType : public GenericEdgeType {};


// SplitEdgeType�ࡪ�����������ࣻ
struct SplitEdgeType : public GenericEdgeType {};


// IsctEdgeType�ࡪ���������γɵ������ߣ�һ��IsctEdgeType�߱�ʾ��������Ƭ�Ľ��߶Σ�
struct IsctEdgeType : public GenericEdgeType
{
    // IsctEdgeType����ᱣ����һ��triangleProblem�����У���Ӧ��ĳ����Ƭt0��
public:
    //          use to detect duplicate instances within a triangle
    TopoTri*                    other_tri_key;     // ����IsctEdgeType��������Ƭt0,t1�Ľ��߶Σ���t0Ϊ��ǰ��������tp�����Ӧ������Ƭ��other_tri_keyָ��t1;
};


// GenericTriType�ࡪ������Ƭ��
struct GenericTriType
{
    TopoTri*                    concrete;                       // �����ɵ�����Ƭ��ָ�룻
    GenericVertType*                   verts[3];
};




/////////////////////////////////////////////////////////////////////////////////////////// by Tora����һЩdebugʱ�õ����ͣ����ڼ��ӣ�
#ifdef MY_DEBUG
struct GenericEdgeInfo
{
    TopoEdge* concrete;                       // �������ñ�����������ʵ������֮���Ӧ��TE���󣿣���
    bool                    boundary;
    uint                    idx;                                         // temporary for triangulation marshalling
    std::vector<GenericVertType>   edgeVers;    // ����ֻȷ����һ���˵㣻
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
    TopoVert* concrete;                 // concret��Ϊ��ָ�룬��˵���˵����ں�����������ʵ���ڣ�Ϊ��ָ��Ļ����ܸõ��Ǵ������ıߣ���û����ӵ��ں������У�
    Vec3d                   coord;              // �����ꣻ

    bool                    boundary;           //  true���������������ڱ��ϣ�false��������������������Ƭ�ڲ���
    uint                    idx;                         // temporary for triangulation marshalling
    std::vector<GenericEdgeInfo>       edgeInfos;      // �õ�����ıߣ�
    std::vector<TopoVert>       nbrVers;        // by Tora�������ý����ڱ��ϣ���boundry == true�����������ñߵ������㣻

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
    TopoEdge* concrete;                       // �������ñ�����������ʵ������֮���Ӧ��TE���󣿣���
    bool                    boundary;
    uint                    idx;                                         // temporary for triangulation marshalling
    std::vector<GenericVertType>   edgeVers;    // ����ֻȷ����һ���˵㣻
    std::vector<IsctVertType>     ivOnEdge;
    TopoTri* other_tri_key;                    // �����������̵�����Ƭ��

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
    std::vector<IsctVertInfo>      isct_versInfo;             // ������Ƭ��Ҫ�¼ӵĶ��㣻�����֣�һ��������Ƭ�ڲ���һ���ڱ��ϣ�
    std::vector<IsctEdgeInfo>      isct_edgesInfo;         // ������Ƭ��Ҫ�¼ӵıߣ������֣�һ�ֹ�������IsctVertType����һ�ֹ���һ��IsctVertType�����һ��ԭ���㣻
    std::vector<GenericTriType>      gtris;
    std::vector<OrigVertType>   overts;                  // ������Ƭ��ԭʼ������
    std::vector<OrigEdgeType>  oedges;              // ������Ƭ��ԭʼ������
    TopoTri  the_tri;                                 // ��Ӧ����Ƭ

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

//////////////////////////////////////////////////////////////////////////////�������Ƭ������Ĺ����ࡪ��TriangleProblem�ࡢlsctProblem��

// TriangleProblem�ࡪ������Ƭ�󽻺��ʷֵ��������һ�������Ӧһ���ʷ�ǰ������Ƭ��
template<class VertData, class TriData>
class Mesh<VertData, TriData>::TriangleProblem
{
    // ��Ա����
public:
    ShortVec<IsctVertType*, 4>      iverts;             // ������Ƭ��Ҫ�¼ӵĶ��㣻�����֣�һ��������Ƭ�ڲ���һ���ڱ��ϣ�
    ShortVec<IsctEdgeType*, 2>      iedges;         // ������Ƭ��Ҫ�¼ӵıߣ���������Ƭ����������Ƭ�Ľ��߶Σ�
    ShortVec<GenericTriType*, 8>      gtris;        // �����ʷֵ��������Ƭ��
    OrigVertType* overts[3];                  // ������Ƭ��ԭʼ������
    OrigEdgeType* oedges[3];              // ������Ƭ��ԭʼ������
    TopoTri* the_tri;                         // ������Ƭ��ָ�룻

    IsctProblem* isctProbPtr;                      // by Tora������ǰtp������������isctProblem�����ָ�룻

public:
    TriangleProblem() {}
    ~TriangleProblem() {}

    // ��ʼ����
    inline void init(IsctProblem* iprob, TopoTri* t)
    {
        the_tri = t;

        // ��ȡԭʼ�ĵ������
        for (uint k = 0; k < 3; k++)
            overts[k] = iprob->newOrigVert(the_tri->verts[k]);

        for (uint k = 0; k < 3; k++)
            oedges[k] = iprob->newOrigEdge(the_tri->edges[k], overts[(k + 1) % 3], overts[(k + 2) % 3]);

        // by Tora
        this->isctProbPtr = iprob;
    }


private:
 

    // �����²�Ķ���iv���ö�����ĳ�����̱��ϣ�tri_keyΪ�ô��̱����ڵ�����Ƭ��������Ҫ�¼ӵı��Ѵ��ڣ�������¼ӵı��Ѵ��������ɣ�
    inline void addEdge(IsctProblem* iprob, IsctVertType* iv, TopoTri* t1)
    {
        /*
            ����I:
                ��ǰtp��������Ƭt0�����̣�iv�Ǵ��¼ӵ�t0�ڲ��ĵ㣻t1�Ǵ��̱����ڵ�����Ƭ��

            ����2��
                ��ǰtp��������Ƭt0�������̱ߣ�iv�Ǵ�t0���̱�����Ҫ�¼ӵĵ㣻t1�Ǳ���������Ƭ��
        */

        IsctEdgeType* ie = find_edge(this->iedges, t1);      // ��ǰ����Ƭ���߶������в��Һ�����Ƭt1�Ľ��߶Σ��еĻ�������ָ�룻

        if (ie)
        {
            // I: ����tp��������t1����Ƭ�Ľ��߶�*ie�Ѿ����ɹ��ˣ����ý��߶�β�˵���Ϊiv
            ie->ends[1] = iv;
            iv->edges.push_back(ie);            // ��������ı�������ϱ����߶�*ie��
        }
        else
        {
            // I������tp������û�б���ý��߶�*ie������֮�����棻
            ie = iprob->newIsctEdge(iv, t1);
            iedges.push_back(ie);
        }
    }


    // addBoundaryHelper()�������ɵ�ǰtp�����ĳ�����ϵ��¶���
    void addBoundaryHelper(TopoEdge* edge, IsctVertType* iv)
    {
        // ��ǰ��tp�����Ӧ������Ƭ��shield����Ƭ��edge����һ���ߣ��ñߴ����˱������Ƭ��iv���ɴ��̵����ɵ���Ҫ�����Ķ��㣻
        iv->boundary = true;            // true��ʾ�¶����ڱ��ϣ�
        iverts.push_back(iv);

        // hook up point to boundary edge interior!
        for (uint k = 0; k < 3; k++)            // �����Ե�ǰtriangleProblem��������Ƭ�������� 
        {
            OrigEdgeType* oe = this->oedges[k];             // ԭʼ�ı����ݣ�
            if (oe->concrete == edge)
            {
                oe->interior.push_back(iv);         // ԭʼ�������м�¼���м�Ҫ����Ķ��㣻
                iv->edges.push_back(oe);
 
                break;
            }
        }
    }

public:

    // consolidate()�����������н��߶Σ�����Ƿ���β�˵�ȱʧ������������޸���
    void consolidate(IsctProblem* iprob)
    {
        // ���һ������β�˵�û��ȷ������������ֻ����������Ƭ�Խ������û��ʹ�ý����Χ�еĻ���
        for (IsctEdgeType* ie : this->iedges)
        {
             if (ie->ends[1] == nullptr)
            {
                // 1. ��ǰtp���������Ƭt0���ͽ��߶ι���������Ƭt1�Ƿ��й����㣻
                TopoVert* vert = commonVert(the_tri, ie->other_tri_key);        // �����ǰsheild����Ƭ��needle����Ƭ�й����㣬ʹ�øö��㣻

                // 2. ��û�й����㣬�����˳�����
                ENSURE(vert);       // bad if we can't find a common vertex

                // �ҵ������sick_edge��Ӧ��ԭʼ���ݣ�������
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


    // addInteriorEndpoint()��������ߺ�����Ƭ�Ľ��㣬�����ݴ˽������ɴ������¶���IsctVertType����Ȼ�󷵻أ�
    IsctVertType* addInteriorEndpoint(IsctProblem* iprob, TopoEdge* edge, GluePointMarker* glue)
    {
        // ��ǰtp���������Ƭ�Ǳ����̵�����Ƭ�����������ڸ�����Ƭ�ڲ���

        // 1. ����ߺ�����Ƭ�����λ�ã�������Ҫ�����ĵ����iv�������浽��ǰtriangleProblem�����У�
        IsctVertType*       iv = iprob->newIsctVert(edge, the_tri, glue);       // ���̱�����Ҫ�¼ӵĶ��㣻
        iv->boundary = false;                       // false��ʾ��������������Ƭ�ڲ���
        iverts.push_back(iv);                               // ����������Ƭ�ڲ�Ҳ��Ҫ������¶��㣻�����¼ӵĶ����غϣ�
        
        // 2. ����Ϊ�����Ķ���iv�����µı����ݡ�������IsctEdgeType���󣬲�һ�������ɣ�
        for (TopoTri* spearTriPtr : edge->tris)         // �������̱����ڵ�����Ƭ��
            addEdge(iprob, iv, spearTriPtr);                    // ����Ƭ���ڵ���������Ƭ������Ҫ�¼ӵıߣ�

        return iv;
    }


    // addBoundaryEndpoint()�����ڵ�ǰtp���������Ƭ�ı����������㡢�����ݣ�
    void addBoundaryEndpoint(IsctProblem* iprob, TopoTri* shieldTri, TopoEdge* spearEdge, IsctVertType* iv)
    {
        // thisָ����Ǵ��̱�spearEdge���ڵ�triangleProblem����shieldTri�Ǳ����̵�����Ƭ��spearEdge�Ǵ��̸�����Ƭ�ıߣ�iv�Ǳߺ�����Ƭ�Ľ��㣻

        // 1. ������
        iv = iprob->copyIsctVert(iv);
        addBoundaryHelper(spearEdge, iv);            // ���ɱ��ϵ��¶��㣻

        // 2. ������iv�ڵ�ǰ����Ƭ�ı��ϣ���Ҫ����͵�ǰ����Ƭ�����������������±ߣ�
        addEdge(iprob, iv, shieldTri);
    }


    // ����ttt������ʹ�õ����ɽ���ӿڣ�
    void addInteriorPoint(IsctProblem* iprob, TopoTri* t1, TopoTri* t2, GluePointMarker* glue)
    {
        // ��ǰtp�����Ӧ������Ƭ��t0��t0t1t2��������Ƭ��ttt���Σ��������ཻ��

        // note this generates wasted re-computation of coordinates 3X
        IsctVertType* iv = iprob->newIsctVert(the_tri, t1, t2, glue);               // iv������Ƭ�ڲ��ĵ㣻
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


    // subdivide()�����Ե�ǰtp���������Ƭִ�������ʷ֣�
    void subdivide(IsctProblem* iprob)
    {
        // 1. ���ϵ�ǰ����Ƭ���ж��㣬����ԭʼ���������㣬����֮ǰ���ɵĽ��㣻
        ShortVec<GenericVertType*, 7> points;
        for (uint k = 0; k < 3; k++)
            points.push_back(overts[k]);

        for (IsctVertType* iv : iverts)
            points.push_back(iv);

        for (uint i = 0; i < points.size(); i++)
            points[i]->idx = i;
 
        // split edges and marshall data, for safety, we zero out references to pre-subdivided edges, which may have been destroyed

        // 2. ���������ϱ���Ϣ��
        ShortVec<GenericEdgeType*, 8> edges;
        for (uint k = 0; k < 3; k++)
        {
            subdivideEdge(iprob, oedges[k], edges);
            oedges[k] = nullptr;
        }
 
        for (IsctEdgeType*& ie : iedges)
        {
            subdivideEdge(iprob, ie, edges);
            ie = nullptr;               // дΪ��ָ�룻
        }
        for (uint i = 0; i < edges.size(); i++)
            edges[i]->idx = i;

        // 3. ��������ƬͶӰ�Ķ�άƽ�棬�����䷨������
        Vec3d normal = cross(overts[1]->coord - overts[0]->coord,
            overts[2]->coord - overts[0]->coord);
        uint normdim = maxDim(abs(normal));
        uint dim0 = (normdim + 1) % 3;
        uint dim1 = (normdim + 2) % 3;
        double sign_flip = (normal.v[normdim] < 0.0) ? -1.0 : 1.0;

        // 4. ִ�������ʷ֣�
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


        // 5. ������
        if (out.numberofpoints != in.numberofpoints) 
        {
            {
                std::cout << "�����ʷ��쳣��" << std::endl;
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
                    std::cout << "��ǰ���޶˵���Ϣ��" << std::endl;
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
 

        // 6.���������ɵ�����Ƭ��
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


    // δʹ�ã�
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


// lsctProblem�ࡪ������Ƭ�ཻ������������
template<class VertData, class TriData>
class Mesh<VertData, TriData>::IsctProblem : public TopoCache
{
    // ��Ա����
public:
    IterPool<GluePointMarker>   glue_pts;
    IterPool<TriangleProblem>   tprobs;                     // �洢���ں�����������Ƭ�ͱ߽������Ϣ��
    IterPool<IsctVertType>      ivpool;
    IterPool<OrigVertType>      ovpool;
    IterPool<IsctEdgeType>      iepool;
    IterPool<OrigEdgeType>      oepool;
    IterPool<SplitEdgeType>     sepool;
    IterPool<GenericTriType>    gtpool;
    std::vector<Vec3d>          quantized_coords;           // ������������ԭ����Ķ������ݣ�ò���Ŷ���������Щ�����Ͻ��У����ǻ��дԭ�е����񶥵㣻

    AABVH<TopoEdge*> bvhObj;                   // by Tora�����������бߵĲ�ΰ�Χ�У�����bvh_edge_tri()ʱ���ɸö���
    std::vector< GeomBlob<TopoEdge*> > edge_geoms;       // by Tora�����������бߵ�GB���󣬰���ÿ���ߵİ�Χ�У�����bvh_edge_tri()ʱ���ɸö���

#ifdef TEST_CROSS_BOX_ACCE
    BBox3d boxIsct;                                                  // �������Χ�н����γɵİ�Χ�У�
    std::vector<uint> selectedTrisIdx;
    std::vector<unsigned> selectedTriFlags;
#endif

public:

    // step3.0 ����IsctProblem����
    IsctProblem(Mesh* owner) : TopoCache(owner)
    {
        // 1. ��ʼ��������������Ƭ���ݣ��������κ�һ��triangleProblem����
        TopoCache::tris.for_each([](TopoTri* t)
            {
                t->data = nullptr;
            });

        // 2. ������Callibrate the quantization unit...
        double maxMag = 0.0;
        for (VertData& v : TopoCache::mesh->verts)
            maxMag = std::max(maxMag, max(abs(v.pos)));
        Quantization::callibrate(maxMag);

        // �������ݴ��븨������quantized_coords;
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


    // vPos()����ȡ���˶�����������
    inline Vec3d vPos(TopoVert* v) const
    {
        return *(reinterpret_cast<Vec3d*>(v->data));
    }


    // �������������triangleProblem����
    TriangleProblem* getTprob(TopoTri* t)
    {
        TriangleProblem* prob = reinterpret_cast<TriangleProblem*>(t->data);
        if (!prob)
        {
            t->data = prob = tprobs.alloc();
            prob->init(this, t);            // �����������ʼ����
        }

        return prob;
    }


    GluePointMarker* newGluePt()
    {
        GluePointMarker* glue = glue_pts.alloc();
        glue->split_type = false;
        return glue;
    }


    // newIsctVert()���������e������Ƭt�Ľ��㣬�ɴ˽�������IsctVertType���󷵻أ�
    inline IsctVertType* newIsctVert(TopoEdge* e, TopoTri* t, GluePointMarker* glue)
    {
        IsctVertType*       iv = ivpool.alloc();
        iv->concrete = nullptr;
        iv->coord = computeCoords(e, t);
        iv->glue_marker = glue;
        glue->copies.push_back(iv);

        return      iv;
    }


    // ����ttt����ʱ��ʹ�õ����ɽ���Ľӿڣ�
    inline IsctVertType* newIsctVert(TopoTri* t0, TopoTri* t1, TopoTri* t2, GluePointMarker* glue)
    {
        // t0t1t2��������Ƭ��ttt���Σ��������ཻ��

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


    // ɾ��һ�����߶Σ�
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

    // In that case, we can perturb the positions of points������һ�Թ�������Ƭʩ��С���Ŷ���ʹ�䲻���棻
    void perturbPositions();

    // in order to give things another try, discard partial work
    void reset();


    // debug�ӿ�
public:
    void dumpIsctPoints(std::vector<Vec3d>* points);                                        // ����ཻ�ĵ�
    void dumpIsctEdges(std::vector< std::pair<Vec3d, Vec3d> >* edges);          // ����ཻ�ıߣ�

public:
    inline GeomBlob<TopoEdge*> edge_blob(TopoEdge* e);                  // ����ĳ���ߵ�gb����

    inline void for_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)>);
    inline void bvh_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)>);


    inline BBox3d bboxFromTptr(TopoTri* t);

    inline BBox3d buildBox(TopoEdge* e) const;
    inline BBox3d buildBox(TopoTri* t) const;

    inline void marshallArithmeticInput(Empty3d::TriIn& input, TopoTri* t) const;
    inline void marshallArithmeticInput(Empty3d::EdgeIn& input, TopoEdge* e) const;
    inline void marshallArithmeticInput(Empty3d::TriEdgeIn& input, TopoEdge* e, TopoTri* t) const;
    inline void marshallArithmeticInput(Empty3d::TriTriTriIn& input, TopoTri* t0, TopoTri* t1, TopoTri* t2) const;

    bool checkIsct(TopoEdge* e, TopoTri* t) const;                           // ����e������Ƭt�Ƿ����ཻ
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


// EdgeCache�ࡪ��IsctProblem��֧���࣬�����ں�����ı���Ϣ��
template<class VertData, class TriData>
class Mesh<VertData, TriData>::IsctProblem::EdgeCache
{
    // ֧������
public:
    struct EdgeEntry
    {
        EdgeEntry(uint id) : vid(id) {}
        EdgeEntry() {}
        uint vid;
        TopoEdge* e;
    };

    // ��Ա���ݣ�
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


// TriTripleTemp�ࡪ������ttt�ཻ���εĸ����ࣻ
struct TriTripleTemp
{
    TopoTri* t0;
    TopoTri* t1;
    TopoTri* t2;

    TriTripleTemp(TopoTri* tp0, TopoTri* tp1, TopoTri* tp2) :
        t0(tp0), t1(tp1), t2(tp2)
    {}
};



//////////////////////////////////////////////////////////////////////////////////����ʵ�֣�

// ���ҵ�ǰtp�����iedges���ݣ����������ɵ���Ҫ�����ı����ݣ����ĳ����
template<uint LEN> inline
IsctEdgeType* find_edge(ShortVec<IsctEdgeType*, LEN>& vec, TopoTri* key)
{
    // ��Ŀ�о���ʹ��ʱ��vecΪ��ǰtp�����iedges���飬���������Ҫ�����ı�
    for (IsctEdgeType* ie : vec) 
    {
        if (ie->other_tri_key == key)
            return ie;
    }
    return nullptr;
}


// commonVert()������t0, t1�й����㣬���ظö���ָ�룻
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


// ����shortVec������Ԫ����ɵ�Ԫ�ضԣ�
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


// edge_blob()��������ĳ���ߵ�GeomBlob����
template<class VertData, class TriData> inline
GeomBlob<TopoEdge*> Mesh<VertData, TriData>::IsctProblem::edge_blob(TopoEdge* e) 
{
    GeomBlob<TopoEdge*>  blob;
    blob.bbox = buildBox(e);
    blob.point = (blob.bbox.minp + blob.bbox.maxp) / 2.0;
    blob.id = e;             
    return blob;
}


// bvh_edge_tri()���������������бߵĲ�ΰ�Χ�У����뺯����func�����������ڲ�ΰ�Χ�����нڵ㣬��������func����false������ֹ������
template<class VertData, class TriData> inline
void Mesh<VertData, TriData>::IsctProblem::bvh_edge_tri(std::function<bool(TopoEdge* e, TopoTri* t)> func) 
{
    this->edge_geoms.clear();

#ifdef TEST_CROSS_BOX_ACCE            // ����Ƭ�Ƿ��ڽ����Χ���ڵı�ǣ�
    this->selectedTriFlags.resize(this->mesh->tris.size(), 0);
    for (const auto& index : this->selectedTrisIdx)
        this->selectedTriFlags[index] = 1;
#endif

    // 1. �������бߵ�GeomBlob��Ϣ��
    TopoCache::edges.for_each(\
        [&](TopoEdge* e) 
        {
            this->edge_geoms.push_back(edge_blob(e));
        });
 

    // 2. �������бߵ�AABVH���� 
    this->bvhObj.build(this->edge_geoms);                      // by Tora������ΰ�Χ�ж���洢����Ա�����У�����debug;
 

    // 3. ����������������Ƭ��
    bool aborted = false;
    TopoCache::tris.for_each([&](TopoTri* t)                // ����������Ƭ�ı���
        {
#ifdef TEST_CROSS_BOX_ACCE
            if (1 == this->selectedTriFlags[t->ref])
            {
#endif
                BBox3d bbox = buildBox(t);              // ��ǰ����Ƭ�İ�Χ�У�
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


// step3.1.2. ����������Ƿ�������Ƭ�ཻ�����Σ����ɱ����̵�����Ƭ��Ӧ��triangleProblem����return true; �����˻������򷵻�false;
template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::tryToFindIntersections()
{
    //  ע�⣺���������˻����Σ��򷵻�false
    Empty3d::degeneracy_count = 0;

    // 1. �����ں��������бߵĲ�ΰ�Χ��aabvh������ʹ��ÿһ������Ƭ����aabvh��Ѱ�ұߺ�����Ƭ�Ľ��㣻
    tiktok& tt = tiktok::getInstance();
    tt.takeArecord();
    bvh_edge_tri([&](TopoEdge* eisct, TopoTri* tisct)->bool 
        {
            // 1.1 ��⵱ǰ����Ƭ�Ƿ񱻴��̣�
            if (checkIsct(eisct, tisct)) 
            {
#ifdef TEST_CROSS_BOX_ACCE
                // for test��������ǰ���̱����ڵ�����Ƭ��ȫ���ڰ�Χ��֮�⣬��������
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

                // 1.1.1 ����GluePointMarker�����ڼ�������Ƭ�ͱ߽���ʱ���ã�
                GluePointMarker*      glue = newGluePt();
                glue->edge_tri_type = true;
                glue->e = eisct;
                glue->t[0] = tisct;

                // 1.1.2 ��ǰ�����̵�����Ƭ����triangleProblem����������Ҫ�¼ӵĵ㣻
                /*
                    ��ǰ����Ƭ������ �� ���ɸ�����Ƭ��Ӧ��tiangleProblem�������ڲ�����1����������iv��2��������ie0, ie1��
                    ivΪ������Ƭ�����̴��ĵ㣬������Ƭ�ڲ���
                    ��ʱ���������ߵ�״̬��ȫ��ͬ���˵㶼��δ��״̬������һ���˵�ȡΪiv����һ���˵��ָ����nullptr;
                */
                TriangleProblem* tpPtr = getTprob(tisct);
                IsctVertType* iv = tpPtr->addInteriorEndpoint(this, eisct, glue);           


                // 1.1.3 ���ɵ�ǰ����Ƭ��Ҫ�¼ӵı����ݣ�
                for (TopoTri* spearTri : eisct->tris)        // �����������̱ߵ���������Ƭ����Щ����ƬҲ��Ҫ�¼ӵ�ͱߣ�
                {
#ifdef TEST_CROSS_BOX_ACCE
                    // for test����tisctһ���ǽ����Χ���ڵ�����Ƭ��eisct��������������Ƭ�У����ܴ��ڽ����Χ���������Ƭ������������                   
                    if (0 == this->selectedTriFlags[spearTri->ref])
                        continue;
#endif
                    TriangleProblem* spearTPptr = getTprob(spearTri);
                    spearTPptr->addBoundaryEndpoint(this, tisct, eisct, iv);
                }
            }

            // 1.2 
            if (Empty3d::degeneracy_count > 0)
                return false;               // �Բ�ΰ�Χ�еı�����ֹ
            else
                return true;                // continue
        });
    tt.takeArecord();
    auto rIter = tt.records.rbegin();
    DWORD time2 = *rIter++;
    DWORD time1 = *rIter;
    std::cout << "����BVH��ʱ��" << time2 - time1 << std::endl;
 
    // 2. �������˻�����
    if (Empty3d::degeneracy_count > 0) 
        return false;                     // restart / abort

    // 3. ����tp����Ѱ��ttt�ཻ���Ρ�������������Ƭt0, t1, t2���������ཻ
    tt.takeArecord();
    std::vector<TriTripleTemp> triples;
    tprobs.for_each([&](TriangleProblem* tprob) 
        {
            TopoTri* t0 = tprob->the_tri;

            for_pairs<IsctEdgeType*, 2>(tprob->iedges, \
                [&](IsctEdgeType*& ie1, IsctEdgeType*& ie2)                 // �Ե�ǰtp�����߶εı������ǳɶԵر�����
                {
                    TopoTri* t1 = ie1->other_tri_key;               // ���߶ι�������һ������Ƭ��
                    TopoTri* t2 = ie2->other_tri_key;
 
                    if (t0 < t1 && t0 < t2)             //      �������о����ifû��Ҫ��
                    {
#ifdef TEST_CROSS_BOX_ACCE
                        // ���t1�ǽ�����������Ƭ��������
                        if (nullptr == t1->data)
                            return;
#endif

                        TriangleProblem* prob1 = reinterpret_cast<TriangleProblem*>(t1->data);              // t1��tp����

                        for (IsctEdgeType* ie : prob1->iedges)      // ������Ƭt1�Ľ��߶εı�����
                        {
                            if (ie->other_tri_key == t2)            // ��t1��t2�н��߶Σ���˵��t0, t1, t2�����ཻ��
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
    std::cout << "����ttt�ཻ��ʱ��" << time2 - time1 << std::endl;

    // 4. ��֤ttt�ཻ��������ʵ���ڣ������˻����Σ����˻����η���false����ֹ����
    for (TriTripleTemp t : triples) 
    {
        if (!checkIsct(t.t0, t.t1, t.t2))               // ���¼��һ����������Ƭ�Ľ������
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


// step3.1.1.  perturbPositions()������һ�Թ�������Ƭʩ��С���Ŷ���ʹ�䲻���棻
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::perturbPositions()
{
    const double EPSILON = 1.0e-5;                          // ��С���Ŷ�����
    for (Vec3d& coord : quantized_coords) 
    {
        Vec3d perturbation(\
            Quantization::quantize(drand(-EPSILON, EPSILON)),
            Quantization::quantize(drand(-EPSILON, EPSILON)),
            Quantization::quantize(drand(-EPSILON, EPSILON)));          // ��С���Ŷ�������

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


// step3.1. Ѱ�ұߺ�����Ƭ�Ľ���
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::findIntersections()
{
    // 3.1.1 ��ʼ�Ŷ�����
    int nTrys = 5;
    perturbPositions();                 // always perturb for safety...

    // 3.1.2 �����⣬���ɲ�ΰ�Χ�У����ɽ�������Ƭ��triangleProblem����
    while (nTrys > 0) 
    {
        if (!tryToFindIntersections())  // ������false��˵�����˻����Σ�ʩ���Ŷ������¼�⣻
        {
            reset();
            perturbPositions();       
            nTrys--;
        }
        else 
            break;
    }


    // 3.1.3 �������������Դ����þ�����Ȼ���˻����Σ��˳�����
    if (nTrys <= 0)
    {
        CORK_ERROR("Ran out of tries to perturb the mesh");
        exit(1);
    }

    // ���ҡ�ɾ����̬���߶Ρ���̬���㣻
#ifdef IGNORE_SELF_ISCT
    tiktok& tt = tiktok::getInstance();
    tt.takeArecord();

        // ���߶�ie�������this->iepool�У�tp�����д�����Щie��ָ�룻
    std::set<IsctVertType*> allSickVers;
    std::set<IsctEdgeType*> allSickEdges;
    std::set<IsctVertType*> sickVersPtrSet;
    std::set<IsctEdgeType*> sickEdgesPtrSet;

    // ���Ҳ�̬���߶Ρ�������ȱʧ�˵����ж�Ϊ��̬���߶Σ�
    for (auto& ie : this->iepool)
    {
        if (nullptr == ie.ends[1] || nullptr == ie.ends[0])
            sickEdgesPtrSet.insert(&ie);
    }

    // ��������Ϣ��
    for (auto& iePtr : sickEdgesPtrSet)
        allSickEdges.insert(iePtr);

    do
    {
        // ���Ҳ�̬���㡪�����ý�������ı�ȫ�����ǲ�̬�ıߣ���õ�Ϊ��̬���㣻
        sickVersPtrSet.clear();
        this->ivpool.for_each([&](IsctVertType* iv)                // �Խ���صı�����
            {
                auto relaEdges = iv->edges.toVec();
                for (auto& oePtr : relaEdges)       // �Ըý�������ıߵı�����
                {
                    IsctEdgeType* iePtr = reinterpret_cast<IsctEdgeType*>(oePtr);
                    if (isIn(allSickEdges, iePtr))       // ���ñ���sickedge���Ӹý����edges������ɾ���ñ� 
                        iv->edges.erase(oePtr);
                }
                if (0 == iv->edges.size())
                    sickVersPtrSet.insert(iv);                  // ���ý�������ı�ȫ�����ǲ�̬�ıߣ���õ�Ϊ��̬���㣻
            });

        // ��������Ϣ��
        for (auto& ivPtr : sickVersPtrSet)
            allSickVers.insert(ivPtr);

        // ɾ����һ�ֲ��ҳ��Ĳ�̬���߶Ρ����ֲ��ҳ��Ĳ�̬����
        for (auto& iePtr : sickEdgesPtrSet)
            this->killIsctEdge(iePtr);
        for (auto& ivPtr : sickVersPtrSet)
            this->killIsctVert(ivPtr);

        // ���Ҳ�̬���߶Σ�
        sickEdgesPtrSet.clear();
        this->iepool.for_each([&](IsctEdgeType* iePtr)
            {
                // ��ȱʧ�˵㣬�ж�Ϊ��̬���߶Σ�
                if (nullptr == iePtr->ends[0] || nullptr == iePtr->ends[1])
                    sickEdgesPtrSet.insert(iePtr);
            });

        // ��������Ϣ��
        for (auto& iePtr : sickEdgesPtrSet)
            allSickEdges.insert(iePtr);

    } while (!sickEdgesPtrSet.empty());

    unsigned ivCount = this->ivpool.size();
    unsigned ieCount = this->iepool.size();

    // ÿһ��tp������ɾ������̬�Ľ���ͽ��߶ε�ָ�룬�� 
    this->tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
        {
            auto ivertsVec = tpPtr->iverts.toVec();
            for (auto& ivPtr : ivertsVec)
            {
                if (isIn(allSickVers, ivPtr))
                    tpPtr->iverts.erase(ivPtr);
                tpPtr->iverts.erase(nullptr);
            }

            // ������shortVecʹ��erase()ʱ������ı䣬��forѭ����ʹ��erase()ò�����׳���
            auto copyVec = tpPtr->iedges.toVec();
            for (auto& iePtr : copyVec)
            {
                if (isIn(allSickEdges, iePtr))
                    tpPtr->iedges.erase(iePtr);
                tpPtr->iedges.erase(nullptr);
            }
        });

    // ��tp�������Ѿ�û�н��߶Σ�������ieָ�붼Ϊ�գ�ɾ����tp����
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
    std::cout << "���ҡ�ɾ����̬���߶Ρ����㡢tp�����ʱ��" << time2 - time1 << std::endl;

#else

    // 3.1.4  ����triangleProblem����������ĳ��ȱ�ٶ˵㣬���Ծ������ݣ�
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


// �ж������Ƿ����Խ���
template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::hasIntersections()
{
    bool foundIsct = false;             // �Ƿ������ཻ�ı�־��
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


// ����ĳ���߶�Ӧ��������Χ��BBox3d����
template<class VertData, class TriData> inline
BBox3d Mesh<VertData, TriData>::IsctProblem::buildBox(TopoEdge* e) const
{
    Vec3d p0 = vPos(e->verts[0]);
    Vec3d p1 = vPos(e->verts[1]);
    return BBox3d(min(p0, p1), max(p0, p1));
}


// ����ĳ������Ƭ��Ӧ��BBox3d����
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


// checkIsct()�������һ���ߺ�һ������Ƭ�Ƿ����ཻ��
template<class VertData, class TriData>
bool Mesh<VertData, TriData>::IsctProblem::checkIsct(TopoEdge* e, TopoTri* t) const
{
    // ���ɸ�����Ƭ�ͱߵ�AABB;
    BBox3d      ebox = buildBox(e);
    BBox3d      tbox = buildBox(t);

    // ���ж�û���ཻ��return false;
    if (!hasIsct(ebox, tbox))
        return      false;

    // ������Ƭ�ͱ��й������㣬����Ϊ�������β������ཻ��
    if (hasCommonVert(e, t))
        return      false;

    Empty3d::TriEdgeIn input;
    marshallArithmeticInput(input, e, t);
    bool empty = Empty3d::emptyExact(input);

    // by Tora�����������˻����Σ���ӡ����һ���˻�������ص�����Ƭ�Ͷ��㣺
    if (Empty3d::degeneracy_count > 0)
    {
        std::vector<CorkVertex> degEdgeVers(2);
        CorkMesh degTriMesh;
        selectedTris2Mesh(degTriMesh, *this->mesh, std::vector<uint>{t->ref});
        debugWriteMesh("�˻�����Ƭ", degTriMesh);
        const auto& degTri = this->mesh->tris[t->ref];      // debugTri.a == 3617��������Ϊ��(-16.672457, -0.54398, 13.377321)
        uint index0 = e->verts[0]->ref;                                  // index0 == 1030, ������Ϊ��(-16.672457, -0.54398, 13.377321);  
        uint index1 = e->verts[1]->ref;                                  // ����Ƭ�ͱ߽���������Ƭ��һ�����㣬�ñ�һ�˵�͸�����Ƭ����������ͬ��ȴ���������в�ͬ����ֵ���������㣻
        degEdgeVers[0] = this->mesh->verts[index0];
        degEdgeVers[1] = this->mesh->verts[index1];
        debugWriteVers("�˻��ߵĶ���", degEdgeVers);
        
        std::cout << "����Ƭ���������㣺" << degTri.a << ", " << degTri.b << ", " << degTri.c << std::endl;
        this->mesh->verts[degTri.a].disp();
        this->mesh->verts[degTri.b].disp();
        this->mesh->verts[degTri.c].disp();

        std::cout << "�ߵ������˵㣺" << index0 << ", " << index1 << std::endl;
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

    tris[t_piece_ref].data.subdivide(input);        // ִ�������ʷ֣�
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

    TopoVert*        v = TopoCache::newVert();          // �����¶��㣻
    VertData& data = TopoCache::mesh->verts[v->ref];
    data.pos = glue->copies[0]->coord;

    fillOutVertData(glue, data);
    for (IsctVertType* iv : glue->copies)
        iv->concrete = v;
}


// createRealTriangles()��������tp�����е������ʷ���Ϣ������������Ƭ��������
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::createRealTriangles(TriangleProblem* tprob, EdgeCache& ecache) 
{
    // for debug�������ڼ��ӵ����ã�
    const auto& meshTris = this->mesh->tris;

    for (GenericTriType* gt : tprob->gtris)             // ���������е�gt����
    {
        TopoTri*        t = TopoCache::newTri();                // ԭ�ں�����������������Ƭ��
        gt->concrete = t;           
        Tri& tri = TopoCache::mesh->tris[t->ref];           

        for (uint k = 0; k < 3; k++)            // ������������Ƭ����ı�����
        {
            TopoVert*    v = gt->verts[k]->concrete;
            t->verts[k] = v;
            v->tris.push_back(t);
            tri.v[k] = v->ref;

            TopoEdge*    e = ecache.getTriangleEdge(gt, k, tprob->the_tri);
            e->tris.push_back(t);
            t->edges[k] = e;
        }

        fillOutTriData(t, tprob->the_tri);                          // ԭ�ں�����������������Ƭ��
    }

    // Once all the pieces are hooked up, let's kill the old triangle!
    TopoCache::deleteTri(tprob->the_tri);
}


// step3.2����ִ�в�㡢�����ʷ֡�����������Ƭ�Ĳ�����
template<class VertData, class TriData>
void Mesh<VertData, TriData>::IsctProblem::resolveAllIntersections()
{
    // for debug����һЩ���ڼ��ӵ����ã�
    const auto& meshTris = this->mesh->tris;
    const auto& meshVers = this->mesh->verts;
    unsigned versCountOri = meshVers.size();

    // 3.2.1.ִ�������ʷ֣�������Ƭ��Ϣ�������TP�����У���ʱû�д���ԭ����
    this->tprobs.for_each([&](TriangleProblem* tprob) 
        {
            tprob->subdivide(this);
        });

    // 3.2.2 ��������glue���������µĶ�����뵽ԭ�ں������У�
    this->glue_pts.for_each([&](GluePointMarker* glue) 
        {
            createRealPtFromGluePt(glue);
        });

    // Now that we have concrete vertices plugged in, we can go through the diced triangle pieces and create concrete triangles  for each of those.

    // Along the way, let's go ahead and hook up edges as appropriate

    // 3.2.3 ����tp�����е������ʷ���Ϣ������������Ƭ��������
    EdgeCache ecache(this);
    this->tprobs.for_each([&](TriangleProblem* tprob) 
        {
            createRealTriangles(tprob, ecache);     // ����������ʷ֣�
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


// ������㣻
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


// ���������Ƭ�ཻ�ıߣ�
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


// ���Խӿڡ���
template<class VertData, class TriData>
void Mesh<VertData, TriData>::testingComputeStaticIsctPoints(std::vector<Vec3d>* points) 
{
    IsctProblem iproblem(this);

    iproblem.findIntersections();
    iproblem.dumpIsctPoints(points);
}


// ���Խӿڡ���
template<class VertData, class TriData>
void Mesh<VertData, TriData>::testingComputeStaticIsct(std::vector<Vec3d>* points,  std::vector< std::pair<Vec3d, Vec3d> >* edges) 
{
    IsctProblem iproblem(this);

    iproblem.findIntersections();
    iproblem.dumpIsctPoints(points);
    iproblem.dumpIsctEdges(edges);
}


// ����Ƭ�󽻣��ҳ�ÿ���ཻ����Ƭ�Ľ��߶Σ������¶��㣬ִ�������ʷ�
template<class VertData, class TriData>
void Mesh<VertData, TriData>::resolveIntersections()
{
    // �����ʷֹ����У��Ƚ������µ������Ƭɾ����Ȼ�������ʷֲ�����������Ƭβ�����룻

    IsctProblem iproblem(this);

#ifdef TEST_CROSS_BOX_ACCE
    iproblem.getSelectedTrisIdx(this->boxIsct);
#endif

    iproblem.findIntersections();                       // Ѱ������Ƭ���ߣ�

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

// by Tora����ȷ�������Χ���ڵ�����Ƭ
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

        // ����Ƭ����������ֻҪ��һ���ڴ˺��ھͱ�ѡ�У�
        bool flag0 = isIn(ver0.pos, boxIsct);
        bool flag1 = isIn(ver1.pos, boxIsct);
        bool flag2 = isIn(ver2.pos, boxIsct);

        if (flag0 | flag1 | flag2)
            this->selectedTrisIdx.push_back(i);
    }
}
#endif
