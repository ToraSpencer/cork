#pragma once

#define DEBUG_BOOL



// ����������ص�ʵ��

// BoolProblem�ࡪ�����񲼶����㹦����
template<class VertData, class TriData>
class Mesh<VertData, TriData>::BoolProblem
{
public:
    BoolProblem(Mesh* owner) : mesh(owner)
    {}

    virtual ~BoolProblem() {}

    void doSetup(Mesh& rhs);

    // ��������Ƭ��
    enum TriCode { KEEP_TRI, DELETE_TRI, FLIP_TRI };            // ����Ƭ�����ı�ǡ���������ɾ������ת��

    void doDeleteAndFlip(std::function<TriCode(byte bool_alg_data)> classify);      // ɾ����ת����Ƭ��


public:

    // ���������������еı����ݣ�
    struct BoolEdata
    {
        bool is_isct;                   // �������˱��Ƿ���ԭ��������Ƭ�ཻ�γɵıߣ�����
    };

    inline byte& boolData(uint tri_id)
    {
        return mesh->tris[tri_id].data.bool_alg_data;
    }


    // �������Ϣ��
    void populateECache()
    {
        ecache = mesh->createEGraphCache<BoolEdata>();

        // label some of the edges as intersection edges and others as not
        ecache.for_each([&](uint i, uint j, EGraphEntry<BoolEdata>& entry)
            {
                entry.data.is_isct = false;
                byte operand = boolData(entry.tids[0]);
                for (uint k = 1; k < entry.tids.size(); k++)
                {
                    if (boolData(entry.tids[k]) != operand)
                    {
                        entry.data.is_isct = true;
                        break;
                    }
                }
            });
    }


    // ������ʹ�ô���ĺ�����action����������this->ecache�е�ÿһ������Ϣ��
    inline void for_ecache(std::function<void(uint i, uint j, bool isisct, const ShortVec<uint, 2>& tids)> action)
    {
        ecache.for_each([&](uint i, uint j, EGraphEntry<BoolEdata>& entry)
            {
                if (entry.data.is_isct)                     // ���ñ�������Ƭ���ߣ�
                {
                    ShortVec<uint, 2> tid0s;
                    ShortVec<uint, 2> tid1s;
                    for (uint elem : entry.tids)
                    {
                        if (boolData(elem) & 1)
                            tid1s.push_back(elem);
                        else
                            tid0s.push_back(elem);
                    }
                    action(i, j, true, tid1s);
                    action(i, j, true, tid0s);
                }
                else                                              // ���ñ߲�������Ƭ���ߣ�
                    action(i, j, false, entry.tids);
            });
    }


    // �ж�����ΪtriIdx������Ƭ�Ƿ�����һ�������ڲ���labelΪtriIdx�������Ƭ��������ı�ǣ�
    bool isInside(uint triIdx, byte label)
    {
        Vec3d p(0, 0, 0);               // ����Ƭ����
        p += mesh->verts[mesh->tris[triIdx].a].pos;
        p += mesh->verts[mesh->tris[triIdx].b].pos;
        p += mesh->verts[mesh->tris[triIdx].c].pos;
        p /= 3.0;

        // 1. ��������
        Ray3d r;
        r.p = p;
        r.r = Vec3d(drand(0.5, 1.5), drand(0.5, 1.5), drand(0.5, 1.5));         // ���߷���������ģ��ڵ�һ�����ڣ�
        int winding = 0;                  // ������winding number

        // 2. ���߶����б�ǲ�Ϊlabel������Ƭ�󽻣�
        for (Tri& tri : mesh->tris)
        {
            if ((tri.data.bool_alg_data & 1) == label)
                continue;

            double flip = 1.0;
            uint   a = tri.a;
            uint   b = tri.b;
            uint   c = tri.c;
            Vec3d va = mesh->verts[a].pos;
            Vec3d vb = mesh->verts[b].pos;
            Vec3d vc = mesh->verts[c].pos;

            // �������е�ǰ����Ƭ����������������
            if (a > b)
            {
                std::swap(a, b);
                std::swap(va, vb);
                flip = -flip;
            }
            if (b > c)
            {
                std::swap(b, c);
                std::swap(vb, vc);
                flip = -flip;
            }
            if (a > b)
            {
                std::swap(a, b);
                std::swap(va, vb);
                flip = -flip;
            }

            double t;
            Vec3d bary;
            if (isct_ray_triangle(r, va, vb, vc, &t, &bary))
            {
                Vec3d normal = flip * cross(vb - va, vc - va);

                if (dot(normal, r.r) > 0.0)
                    winding++;          // UNSAFE
                else
                    winding--;
            }
        }

        // 3. ����������0�򷵻�true��
        return winding > 0;
    }

public:
    Mesh* mesh;
    EGraphCache<BoolEdata>      ecache;             // ����Ϣ��ͨ������ populateECache()�ӿ�������ecache���ݣ�
};


// ��������Ƭ���
static inline double triArea(Vec3d a, Vec3d b, Vec3d c)
{
    return len(cross(b - a, c - a));
}


// ��������Ļ�������
template<class VertData, class TriData>
void Mesh<VertData, TriData>::BoolProblem::doSetup(Mesh& rhs)
{
    // 1. ����Ƭ��ǡ���mesh1��bool_alg_data���Ϊ0��mesh2�ı��Ϊ1��
    mesh->for_tris([](TriData& tri, VertData&, VertData&, VertData&)
        {
            tri.bool_alg_data = 0;
        });

    rhs.for_tris([](TriData& tri, VertData&, VertData&, VertData&)
        {
            tri.bool_alg_data = 1;
        });

#ifdef TEST_CROSS_BOX_ACCE
    // ��������������԰�Χ�е��ཻ��Χ�У�
    BBox3d box0 = getAABB(*this->mesh);
    BBox3d box1 = getAABB(rhs);
    BBox3d boxIsct = isct(box0, box1);
    BBox3d boxIsct101 = multiply(boxIsct, 1.01);
    this->mesh->boxIsct = boxIsct101;

#ifdef MY_DEBUG
    CorkMesh boxMesh0, boxMesh1, boxIsctMesh;
    getAABBmesh(boxMesh0, box0);
    getAABBmesh(boxMesh1, box1);
    getAABBmesh(boxIsctMesh, boxIsct101);
    debugWriteMesh("boxMesh0", boxMesh0);
    debugWriteMesh("boxMesh1", boxMesh1);
    debugWriteMesh("boxIsctMesh", boxIsctMesh);
#endif

#endif

    // 2. ��������ĵ������ݺϲ��洢�ڵ�ǰmesh����ĳ�Ա�����У�
    mesh->disjointUnion(rhs);

    // 3. ����Ƭ�󽻣��ҳ�ÿ���ཻ����Ƭ�Ľ��߶Σ�
    mesh->resolveIntersections();

    // 4. �������Ϣ
    populateECache();
 
    // 5. form connected components;
    /*
         we get one component for each connected component in one of the two input meshes.
         These components are not necessarily uniformly inside or outside  of the other operand mesh.
    */
    UnionFind uf(mesh->tris.size());
    for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
        {
            uint tid0 = tids[0];
            for (uint k = 1; k < tids.size(); k++)
                uf.unionIds(tid0, tids[k]);
        });


    // 6. re-organize the results of the union find as follows:
    std::vector<uint> uq_ids(mesh->tris.size(), uint(-1));
    std::vector< std::vector<uint> > components;
    for (uint i = 0; i < mesh->tris.size(); i++)
    {
        uint ufid = uf.find(i);
        if (uq_ids[ufid] == uint(-1))
        {
            // unassigned
            uint N = components.size();
            components.push_back(std::vector<uint>());

            uq_ids[ufid] = uq_ids[i] = N;
            components[N].push_back(i);
        }
        else
        {
            // assigned already
            uq_ids[i] = uq_ids[ufid];               // propagate assignment
            components[uq_ids[i]].push_back(i);
        }
    }
 
    // 7. find the "best" triangle in each component, ʹ���������ж�����Ƭ���ڲ������ⲿ����������Ƭ���޸�����Ƭ��ǣ�
    std::vector<bool> visited(mesh->tris.size(), false);
    for(unsigned i = 0; i < components.size(); ++i)
    {
        auto& comp = components[i];

        // 7.1 �ҳ���ǰ����Ƭ���������������Ƭ��
        uint maxTriIdx = comp[0];
        double areaMax = 0.0;
        for (uint tid : comp)
        {
            Vec3d va = mesh->verts[mesh->tris[tid].a].pos;
            Vec3d vb = mesh->verts[mesh->tris[tid].b].pos;
            Vec3d vc = mesh->verts[mesh->tris[tid].c].pos;

            double area = triArea(va, vb, vc);
            if (area > areaMax)
            {
                areaMax = area;
                maxTriIdx = tid;
            }
        }

        byte operand = boolData(maxTriIdx);
        bool inside = isInside(maxTriIdx, operand);

        // begin by tagging the first triangle
        boolData(maxTriIdx) |= (inside) ? 2 : 0;

        // NOW PROPAGATE classification throughout the component. do a breadth first propagation
        std::queue<uint> work;
        visited[maxTriIdx] = true;
        work.push(maxTriIdx);

        while (!work.empty())
        {
            uint curr_tid = work.front();
            work.pop();

            for (uint k = 0; k < 3; k++)
            {
                uint a = mesh->tris[curr_tid].v[k];
                uint b = mesh->tris[curr_tid].v[(k + 1) % 3];
                auto& entry = ecache(a, b);

                byte inside_sig = boolData(curr_tid) & 2;
                if (entry.data.is_isct)  
                    inside_sig ^= 2;

                for (uint nbrTriIdx : entry.tids)
                {
                    if (visited[nbrTriIdx])
                        continue;

                    if ((boolData(nbrTriIdx) & 1) != operand)
                        continue;

                    boolData(nbrTriIdx) |= inside_sig;
                    visited[nbrTriIdx] = true;
                    work.push(nbrTriIdx);
                }
            }
        }
    }

 
}


// ���뺯����classify������ÿ������Ƭ�Ĳ����룬Ȼ��ִ�в�����classify����������Ƭ��ǣ����������ĺ����ӣ�
template<class VertData, class TriData>
void Mesh<VertData, TriData>::BoolProblem::doDeleteAndFlip(std::function<TriCode(byte bool_alg_data)> classify)
{
    // 1. ���ɺϲ������������Ϣ��
    TopoCache topocache(mesh);            

    // 2. �ҳ���Ҫɾ��������Ƭ��

    //       for debug
    std::vector<TriCode> triCodes;                  // ÿ������Ƭ�Ĳ����룻

    std::vector<TopoTri*> toDelete;         
    topocache.tris.for_each([&](TopoTri* tptr)
        {
            // ��������Ƭ�����룻
            TriCode code = classify(boolData(tptr->ref));
            
            //      for debug
            triCodes.push_back(code);

            switch (code)
            {
            case DELETE_TRI:
                toDelete.push_back(tptr);
                break;

            case FLIP_TRI:
                topocache.flipTri(tptr);
                break;

            case KEEP_TRI:

            default:
                break;
            }
        });

    for (TopoTri* tptr : toDelete)
        topocache.deleteTri(tptr);

    topocache.commit();
}


// ���������㣺
template<class VertData, class TriData>
void Mesh<VertData, TriData>::boolUnion(Mesh& rhs)
{

    BoolProblem bprob(this);

    bprob.doSetup(rhs);

    bprob.doDeleteAndFlip([](byte data) -> typename BoolProblem::TriCode
        {
            if ((data & 2) == 2)                                    // part of op 0/1 INSIDE op 1/0
                return BoolProblem::DELETE_TRI;
            else                                                          // part of op 0/1 OUTSIDE op 1/0
                return BoolProblem::KEEP_TRI;
        });
}


// �����������
template<class VertData, class TriData>
void Mesh<VertData, TriData>::boolDiff(Mesh& rhs)
{
    BoolProblem bprob(this);

    bprob.doSetup(rhs);

    bprob.doDeleteAndFlip([](byte data) -> typename BoolProblem::TriCode
        {
            if (data == 2 ||         // part of op 0 INSIDE op 1
                data == 1)           // part of op 1 OUTSIDE op 0
                return BoolProblem::DELETE_TRI;
            else if (data == 3)      // part of op 1 INSIDE op 1
                return BoolProblem::FLIP_TRI;
            else                    // part of op 0 OUTSIDE op 1
                return BoolProblem::KEEP_TRI;
        });
}


// ���������㣻
template<class VertData, class TriData>
void Mesh<VertData, TriData>::boolIsct(Mesh& rhs)
{
    BoolProblem bprob(this);

    bprob.doSetup(rhs);

    bprob.doDeleteAndFlip([](byte data) -> typename BoolProblem::TriCode {
        if ((data & 2) == 0)     // part of op 0/1 OUTSIDE op 1/0
            return BoolProblem::DELETE_TRI;
        else                    // part of op 0/1 INSIDE op 1/0
            return BoolProblem::KEEP_TRI;
        });
}


// ����������㣻
template<class VertData, class TriData>
void Mesh<VertData, TriData>::boolXor(Mesh& rhs)
{
    BoolProblem bprob(this);

    bprob.doSetup(rhs);

    bprob.doDeleteAndFlip([](byte data) -> typename BoolProblem::TriCode {
        if ((data & 2) == 0)     // part of op 0/1 OUTSIDE op 1/0
            return BoolProblem::KEEP_TRI;
        else                    // part of op 0/1 INSIDE op 1/0
            return BoolProblem::FLIP_TRI;
        });
}


 
/// ////////////////////////////////////////////////////////////////////////// һЩdebug�ӿڣ�
void copyEdgesInfo(const CorkMesh& mesh, const CorkMesh::EGraphCache<CorkMesh::BoolProblem::BoolEdata>& gCache, \
    std::vector<edgeInfo>& edgesInfo);
void copyEdgesInfo(const CorkMesh::BoolProblem& bprob, std::vector<edgeInfo>& edgesInfo);
void getLabelInfo(const CorkMesh::BoolProblem& bprob, std::vector<unsigned>& BADcopy, std::vector<CorkVertex>& debugVers);



/// ////////////////////////////////////////////////////////////////////////// debug�ӿ�ʵ�֣�

// ��EGraphCache����ȡ�����ߵ�������Ϣ����дΪ���򵥵�edgeInfo��ʽ���浽�����У�
void copyEdgesInfo(const CorkMesh& mesh, const CorkMesh::EGraphCache<CorkMesh::BoolProblem::BoolEdata>& gCache, \
    std::vector<edgeInfo>& edgesInfo)
{
    edgesInfo.clear();

    unsigned V = mesh.verts.size();
    unsigned F = mesh.tris.size();
    edgesInfo.reserve(V + F + 100);                     // ŷ����ʽ��E = V + F - 2*(1-g);
    const auto& skeleton = gCache.skeleton;
    for (unsigned i = 0; i < V; ++i)
    {
        for (unsigned j = 0; j < skeleton[i].size(); ++j)
        {
            const auto& temp = skeleton[i][j];
            std::vector<unsigned> tidsVec;
            temp.tids.toVec(tidsVec);
            std::pair<unsigned, unsigned> tids = std::make_pair(tidsVec[0], tidsVec[1]);
            edgeInfo currentEdgeInfo = std::make_tuple(i, temp.vid, tids, temp.data.is_isct);
            edgesInfo.push_back(currentEdgeInfo);
        }
    }
    edgesInfo.shrink_to_fit();
}

// ͬ�ϣ������Ǵ�boolProblem�е�EGraphCache����ȡ��
void copyEdgesInfo(const CorkMesh::BoolProblem& bprob, std::vector<edgeInfo>& edgesInfo)
{
    edgesInfo.clear();

    unsigned V = bprob.mesh->verts.size();
    unsigned F = bprob.mesh->tris.size();
    edgesInfo.reserve(V + F + 100);                     // ŷ����ʽ��E = V + F - 2*(1-g);
    const auto& skeleton = bprob.ecache.skeleton;
    for (unsigned i = 0; i < V; ++i)
    {
        for (unsigned j = 0; j < skeleton[i].size(); ++j)
        {
            const auto& temp = skeleton[i][j];
            std::vector<unsigned> tidsVec;
            temp.tids.toVec(tidsVec);
            std::pair<unsigned, unsigned> tids = std::make_pair(tidsVec[0], tidsVec[1]);
            edgeInfo currentEdgeInfo = std::make_tuple(i, temp.vid, tids, temp.data.is_isct);
            edgesInfo.push_back(currentEdgeInfo);
        }
    }
    edgesInfo.shrink_to_fit();
}



// �õ�����Ƭ��ǣ��ͱ��Ϊ0������Ƭ���㣻
void getLabelInfo(const CorkMesh::BoolProblem& bprob, std::vector<unsigned>& BADcopy, std::vector<CorkVertex>& debugVers)
{
    BADcopy.clear();
    debugVers.clear();

    BADcopy.resize(bprob.mesh->tris.size());
    for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
        BADcopy[i] = bprob.mesh->tris[i].data.bool_alg_data;
    debugVers.reserve(3 * bprob.mesh->tris.size());
    for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
    {
        if (0 == BADcopy[i])
        {
            const auto& tri = bprob.mesh->tris[i];
            debugVers.push_back(bprob.mesh->verts[tri.a]);
            debugVers.push_back(bprob.mesh->verts[tri.b]);
            debugVers.push_back(bprob.mesh->verts[tri.c]);
        }
    }
    debugVers.shrink_to_fit();
}


// ��ȡ�ཻ������
void getIsctEdges(const CorkMesh::BoolProblem& bprob, std::vector<CorkVertex>& vers, \
    std::vector<std::pair<unsigned, unsigned>>& edges)
{
    vers.clear();
    edges.clear();
    vers = bprob.mesh->verts;

    for (unsigned i = 0; i < bprob.ecache.skeleton.size(); ++i)
    {
        unsigned currentEdgesCount = bprob.ecache.skeleton[i].size();       // ����Ϊi�Ķ�������ı�����
        for (unsigned j = 0; j < currentEdgesCount; ++j)
        {
            if (bprob.ecache.skeleton[i][j].data.is_isct)
            {
                const auto& currentEdgeInfo = bprob.ecache.skeleton[i][j];
                edges.push_back(std::make_pair(i, currentEdgeInfo.vid));
            }
        }
    }
}