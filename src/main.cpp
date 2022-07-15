// �ʼ�
/*
    *   ���������ļ�ֻ����OFF�ļ���


    README�е�һЩ��Ϣ��

    ������Դ��һЩ��Ϣ��
    https://stephanfr.blog/2016/03/21/cork-a-high-performance-library-for-geometric-booleancsg-operations/
            Gilbert Bernstein is currently a Ph.D. student at Stanford and had published some remarkable papers on computational geometry. 
    I was first drawn to his work by his 2009 paper on Fast, Exact, Linear Booleans as my interest in 3D printing led me to create some tooling 
    of my own.  The various libraries I found online for performing Constructive Solid Geometry (CSG) operations were certainly good but overall, 
    very slow.  CGAL is one library I had worked with and I found that the time required for operations on even moderately complex meshes was 
    quite long.  CGAL��s numeric precision and stability is impeccable, the 3D CSG operations in CGAL are based on 3D Nef Polyhedra but I found 
    myself waiting quite a while for results.
            I exchanged a couple emails with Gilbert and he pointed me to a new library he had published, Cork.  One challenge with the models he 
    used in his Fast, Exact paper is that the internal representation of 3D meshes was not all that compatible with other toolsets.  Though the boolean 
    operations were fast, using those algorithms imposed a conversion overhead and eliminates the ability to take other algorithms developed on 
    standard 3D mesh representations and use them directly on the internal data structures.  Cork is fast but uses a ��standard�� internal representation 
    of 3D triangulated meshes, a win-win proposition.
            I��ve always been one to tinker with code, so I forked Gilbert��s code to play with it.  I spent a fair amount of time working with the code and 
     I don��t believe I found any defects but I did find a few ways to tune it and bump up the performance.  I also took a swag at parallelizing sections 
     of the code to further reduce wall clock time required for operation, though with limited success.  I believe the main problem I ran into is related to 
     cache invalidation within the x86 CPU.  I managed to split several of the most computationally intensive sections into multiple thread of execution �C but
     the performance almost always dropped as a result.  I am not completely finished working on threading the library, I may write a post later on 
     what I believe I have seen and how to approach parallelizing algorithms like Cork on current generation CPUs.
            ......
     *      cork�������<Fast, Exact, Linear Booleans>�����е��㷨��ʹ���˸���׼�����㷺���õ��������ݽṹ��

*/
 
#include "mesh.h"
#include <iostream>
#include <sstream>
#include <list>
#include <unordered_set>
#include <set>

///////////////////////////////////////////////////////////////////////////////////////////////////// ���Ժ���

// ���������ֱ�Ӳ���
namespace TEST_BOOLEAN
{
    // ���������ֱ�Ӳ��ԣ�
    void test0()
    {
        tiktok& tt = tiktok::getInstance();

        // 1. ��ȡ����
        CorkTriMesh ioMesh0ori, ioMesh1ori, ioMesh2ori;
        loadMesh(g_debugPath + std::string{ "s9block��������/s9block_ҧ����.obj" }, &ioMesh0ori);
        loadMesh(g_debugPath + std::string{ "s9block��������/fatTeeth1_ҧ����.obj" }, &ioMesh1ori);
        loadMesh(g_debugPath + std::string{ "s9block��������/fatTeeth2_ҧ����.obj" }, &ioMesh2ori);
 
        // 3. ������д�����أ�
        CorkTriMesh meshOut0, meshOut1;
        tt.start();
        computeDifference(ioMesh0ori, ioMesh1ori, &meshOut0);
        computeDifference(meshOut0, ioMesh2ori, &meshOut1);
        tt.endCout("�ܺ�ʱ��");
        debugWriteMesh("mesh_diff", meshOut1);

        std::cout << "finished." << std::endl;
        getchar();
    }


    // ��������ķֲ������
    void test1()
    {
        // ֪ʶ�㣺 1. ��ΰ�Χ�У�����Ƭ����ʹ�ã�2. ������&������������Ƭ������������ж���ʹ�ã�
        tiktok& tt = tiktok::getInstance();

        // 0. ��ȡ���񣬼�����������Ƿ�Ϸ������ɼ�������������ɲ������������
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "cmesh1.off" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "cylinder2.off" }, &ioMesh1);
        //loadMesh(g_debugPath + std::string{ "s9block��������/s9block_ҧ��ǳ.obj" }, &ioMesh0);
        //loadMesh(g_debugPath + std::string{ "s9block��������/fatTeeth1_ҧ��ǳ.obj" }, &ioMesh1);

        // isSolid()�ӿ�ò�������⣬��cylinder2.obj�����ж�Ϊ��solid�ģ�

        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        CorkMesh::BoolProblem bprob(&mesh0);

        //              for debug
        std::vector<unsigned> BADcopy;              // ����Ƭ��ǵĿ�����mesh->tris[i].data.bool_alg_data
        std::vector<CorkVertex> debugVers;          // ����debug�ĵ���
        std::vector<std::pair<unsigned, unsigned>> debugEdges;      // ����debug�ı����ݣ� 

        unsigned timeAll = 0;
        tt.start();
        // 1. ����Ƭ��ǡ���mesh0��bool_alg_data���Ϊ0��mesh1�ı��Ϊ1��
        for (auto& tri : bprob.mesh->tris)
            tri.data.bool_alg_data = static_cast<byte>(0);

        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);

#ifdef TEST_CROSS_BOX_ACCE
        // ��������������԰�Χ�е��ཻ��Χ�У�
        BBox3d box0 = getAABB(mesh0);
        BBox3d box1 = getAABB(mesh1);
        BBox3d boxIsct = isct(box0, box1);
        BBox3d boxIsct101 = multiply(boxIsct, 1.01);
#endif

        // 2. ֱ���ں���������
        bprob.mesh->disjointUnion(mesh1);                          // ��������ֱ���ں�
        tt.endCout("step1& step2��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        //              for debug;
        debugWriteMesh("step2ֱ���ںϺ������", *bprob.mesh);
        getLabelInfo(bprob, BADcopy, debugVers);
        debugWriteVers("step2���Ϊ0������Ƭ����", debugVers);

#ifdef TEST_CROSS_BOX_ACCE
        bprob.mesh->boxIsct = boxIsct101;

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


        // 3. ��������Ƭ���ߣ����������ʷ֣����в�㣻
        tt.start();
        bprob.mesh->resolveIntersections();
        tt.endCout("step3��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        //              for debug;
        debugWriteMesh("step3���㽻�ߡ������ʷ�֮��", *bprob.mesh);
        getLabelInfo(bprob, BADcopy, debugVers);
        debugWriteVers("step3���Ϊ0������Ƭ����", debugVers);


        // 4. ����ecache��Ա���ݡ���ȷ���ں���������бߣ����ұ�ǳ���Щ�� ������Ƭ�ཻ�ߣ�
        tt.start();
        bprob.populateECache();
        tt.endCout("step4��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        //          for debug
        debugWriteMesh("step4", *bprob.mesh);
        getLabelInfo(bprob, BADcopy, debugVers);

        // ��5��6��ò�Ʋ���Ҫ��
#if 1
    // 5.����������UnionFind����uf����֪���Ǹ�ɶ�ģ�
        tt.start();
        UnionFind uf(bprob.mesh->tris.size());
        bprob.for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
            {
                uint tid0 = tids[0];
                for (uint k = 1; k < tids.size(); k++)
                    uf.unionIds(tid0, tids[k]);
            });
        tt.endCout("step5��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        debugWriteMesh("step5", *bprob.mesh);


        // 6. ������
        tt.start();
        std::vector<uint> uq_ids(bprob.mesh->tris.size(), uint(-1));
        std::vector< std::vector<uint> > components;
        for (uint i = 0; i < bprob.mesh->tris.size(); i++)
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
        tt.endCout("step6��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        //      for debug 
        debugWriteMesh("step6", *bprob.mesh);
        debugVers.clear();
        for (const auto& index : components[0])
        {
            const auto& currentTri = bprob.mesh->tris[index];
            debugVers.push_back(bprob.mesh->verts[currentTri.a]);
            debugVers.push_back(bprob.mesh->verts[currentTri.b]);
            debugVers.push_back(bprob.mesh->verts[currentTri.c]);
        }
        debugWriteVers("step6����components[0]����������Ƭ�Ķ���", debugVers);
        getLabelInfo(bprob, BADcopy, debugVers);
        debugWriteVers("step6�������Ϊ0������Ƭ����", debugVers);
#else

    // �����Ϊ0��1������Ƭ�ֱ�ŵ����������У�
        std::vector<std::vector<uint>> components(2);
        components[0].reserve(bprob.mesh->tris.size());
        components[1].reserve(bprob.mesh->tris.size());
        for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
        {
            const auto& tri = bprob.mesh->tris[i];
            if (0 == tri.data.bool_alg_data)
                components[0].push_back(i);
            else
                components[1].push_back(i);
        }
        components[1].shrink_to_fit();
        components[0].shrink_to_fit();

        //              for debug
        getLabelInfo(bprob, BADcopy, debugVers);
#endif

        // 7. ʹ���������ж�����Ƭ���ڲ������ⲿ����������Ƭ���޸�����Ƭ��ǣ�
        tt.start();
        std::vector<bool> visited(bprob.mesh->tris.size(), false);              // ��¼����Ƭ�Ƿ��Ѿ���������
        for (unsigned i = 0; i < components.size(); ++i)
        {
            auto& comp = components[i];

            // 7.1 �ҳ���ǰ����Ƭ���������������Ƭ��
            uint maxTriIdx = comp[0];                       // �����������Ƭ������
            double areaMax = 0.0;
            for (uint tid : comp)
            {
                Vec3d va = bprob.mesh->verts[bprob.mesh->tris[tid].a].pos;
                Vec3d vb = bprob.mesh->verts[bprob.mesh->tris[tid].b].pos;
                Vec3d vc = bprob.mesh->verts[bprob.mesh->tris[tid].c].pos;

                double area = triArea(va, vb, vc);
                if (area > areaMax)
                {
                    areaMax = area;
                    maxTriIdx = tid;
                }
            }

            // 7.2 �жϵ�ǰ�������Ƭ�Ƿ����ں������ڲ����޸ĸ��������Ƭ�ı�ǣ�
            byte currentMeshLabel = bprob.boolData(maxTriIdx);
            bool inside = bprob.isInside(maxTriIdx, currentMeshLabel);
            byte& maxTriLabel = bprob.boolData(maxTriIdx);
            maxTriLabel |= (inside) ? 2 : 0;            // ��0��10B����λ�����㣬��ǰ�����������Ƭ�������ں������ڲ�������labelǰһλ��Ϊ1�� ������Ƭ��Ǵ�����ϣ�

            // 7.3 ���������Ƭ��ʼ������������Ƭ�����ķ�ʽ����������������Ƭ�����ݲ�ͬ�����޸�����Ƭ��ǣ�
            std::queue<uint> work;                    // ��������Ƭ�Ĺ������У�        
            visited[maxTriIdx] = true;
            work.push(maxTriIdx);

            while (!work.empty())
            {
                uint curr_tid = work.front();
                byte& currentTriLabel = bprob.boolData(curr_tid);           // ��������Ƭ�ı�ǣ�
                work.pop();

                for (uint k = 0; k < 3; k++)        // ������ǰ����Ƭ�������ߣ�����������ǰ����Ƭ��������������Ƭ��
                {
                    uint a = bprob.mesh->tris[curr_tid].v[k];
                    uint b = bprob.mesh->tris[curr_tid].v[(k + 1) % 3];
                    auto& entry = bprob.ecache(a, b);                      // ��a, b�������ߵ���Ϣ��

                    // ����Ƭ��Ǻ�10B��������
                    byte inside_sig = currentTriLabel & 2;          // ��λ������������ȡĳһλ����Ϣ�������10B����λ�����㣬��ȡ��currentTriLabel��ǰһλ����ʾ�Ƿ��������ڣ�
                    if (entry.data.is_isct)
                        inside_sig ^= 2;                     // ���ñ�������Ƭ���ߣ���currentTri��nbrTri������״̬һ��������ģ�
                                                                         // ����Ƭ�����Ĺ����У����û����������Ƭ���ߣ�������Ƭ������״̬��һֱ����ģ�
                                                                          // ��10B��������㣬��ʾ������״̬ȡ����

                    for (uint nbrTriIdx : entry.tids)   // ���������߹�������������Ƭ����������ǰ����Ƭ��������������Ƭ��
                    {
                        byte& nbrTriLabel = bprob.boolData(nbrTriIdx);
                        if (visited[nbrTriIdx])
                            continue;

                        if ((nbrTriLabel & 1) != currentMeshLabel)         // ����������Ƭ������һ����������������ѭ��ֻ����ǰ�����е�����Ƭ��
                            continue;

                        nbrTriLabel |= inside_sig;          // 
                        visited[nbrTriIdx] = true;
                        work.push(nbrTriIdx);
                    }
                }
            }

            currentMeshLabel++;
        }
        tt.endCout("step7��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        // ��������Ƭ��ǣ���һλ��ʾ������Ƭԭ�������ĸ�����ǰһλ��ʾ������Ƭ�Ƿ����ں�������ⲿ��
        /*
            0��00B������0���������Ƭ����ȥ1�����ں������ڲ�����Щ��
            1��01B������1���������Ƭ����ȥ0�����ں������ڲ�����Щ��
            2��10B������0��������Ƭ�У����ں������ڲ�����Щ����Ƭ��
            3��11B������1��������Ƭ�У����ں������ڲ�����Щ����Ƭ��
        */

        //              for debug;
        {
            debugWriteMesh("step7", *bprob.mesh);
            getLabelInfo(bprob, BADcopy, debugVers);
            CorkMesh selectedMesh0, selectedMesh1, selectedMesh2, selectedMesh3;
            std::vector<unsigned> triIdx0, triIdx1, triIdx2, triIdx3;
            for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
            {
                switch (bprob.mesh->tris[i].data.bool_alg_data)
                {
                case 0:
                    triIdx0.push_back(i);
                    break;
                case 1:
                    triIdx1.push_back(i);
                    break;
                case 2:
                    triIdx2.push_back(i);
                    break;
                case 3:
                    triIdx3.push_back(i);
                    break;
                default:
                    break;
                }
            }
            selectedTris2Mesh(selectedMesh0, *bprob.mesh, triIdx0);
            selectedTris2Mesh(selectedMesh1, *bprob.mesh, triIdx1);
            selectedTris2Mesh(selectedMesh2, *bprob.mesh, triIdx2);
            selectedTris2Mesh(selectedMesh3, *bprob.mesh, triIdx3);
            debugWriteMesh("step7����Ƭ0��Ӧ������", selectedMesh0);
            debugWriteMesh("step7����Ƭ1��Ӧ������", selectedMesh1);
            debugWriteMesh("step7����Ƭ2��Ӧ������", selectedMesh2);
            debugWriteMesh("step7����Ƭ3��Ӧ������", selectedMesh3);
        }

        // 8.��������������
        tt.start();
        bprob.doDeleteAndFlip([](byte data) -> typename CorkMesh::BoolProblem::TriCode
            {
                if (data == 2 || data == 1)           // part of op 1 OUTSIDE op 0
                    return CorkMesh::BoolProblem::DELETE_TRI;                           // ɾ�����Ϊ2, 1������Ƭ
                else if (data == 3)                   // part of op 1 INSIDE op 1
                    return CorkMesh::BoolProblem::FLIP_TRI;                                // ��ת���Ϊ3������Ƭ��
                else                                    // part of op 0 OUTSIDE op 1
                    return CorkMesh::BoolProblem::KEEP_TRI;                               // �������Ϊ0������Ƭ��
            });
        tt.endCout("step8��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        debugWriteMesh("meshDiff_δ��meshFix����Ľ��", *bprob.mesh);



        std::cout << "�ܹ���ʱ��" << timeAll << std::endl;           // ���������ʱ��ֱ�Ӳ��ԵĻ���ʱ6200+ms;
        std::cout << "finished." << std::endl;
        getchar();
    }


    // ����ISOLATE_TRIS_IN_BOX
    void test11()
    {
        // ֪ʶ�㣺 1. ��ΰ�Χ�У�����Ƭ����ʹ�ã�2. ������&������������Ƭ������������ж���ʹ�ã�
        tiktok& tt = tiktok::getInstance();

        // 0. ��ȡ���񣬼�����������Ƿ�Ϸ������ɼ�������������ɲ������������
        CorkTriMesh ioMesh1, ioMesh0;
        //loadMesh(g_debugPath + std::string{ "cmesh1.off" }, &ioMesh0);
        //loadMesh(g_debugPath + std::string{ "cylinder2.off" }, &ioMesh1);
        loadMesh(g_debugPath + std::string{ "s9block��������/s9block_preprocessed.obj" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "s9block��������/fatTeeth1_preprocessed.obj" }, &ioMesh1);

        // isSolid()�ӿ�ò�������⣬��cylinder2.obj�����ж�Ϊ��solid�ģ�

        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        CorkMesh::BoolProblem bprob(&mesh0);

        unsigned timeAll = 0;
        tt.start();
        // 1. ����Ƭ��ǡ���mesh0��bool_alg_data���Ϊ0��mesh1�ı��Ϊ1��
        for (auto& tri : bprob.mesh->tris)
            tri.data.bool_alg_data = static_cast<byte>(0);

        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);

        // 2. ֱ���ں���������
        bprob.mesh->disjointUnion(mesh1);                          // ��������ֱ���ں�
        tt.endCout("step1& step2��ʱ��");
        timeAll += tt.endTik - tt.startTik;
        debugWriteMesh("step2ֱ���ںϵ�����", *bprob.mesh);

        // ��������������԰�Χ�е��ཻ��Χ�У�
        BBox3d box0 = getAABB(mesh0);
        BBox3d box1 = getAABB(mesh1);
        BBox3d boxIsct = isct(box0, box1);
        BBox3d boxIsct101 = multiply(boxIsct, 1.01);

#ifdef MY_DEBUG
        CorkMesh boxMesh0, boxMesh1, boxIsctMesh;
        getAABBmesh(boxMesh0, box0);
        getAABBmesh(boxMesh1, box1);
        getAABBmesh(boxIsctMesh, boxIsct101);
        debugWriteMesh("boxMesh0", boxMesh0);
        debugWriteMesh("boxMesh1", boxMesh1);
        debugWriteMesh("boxIsctMesh", boxIsctMesh);
#endif


#ifdef ISOLATE_TRIS_IN_BOX
        tt.start();
        unsigned versCount = bprob.mesh->verts.size();
        unsigned trisCount = bprob.mesh->tris.size();

        // ������Ƭ��������һ�������ڰ�Χ���ڣ���ȡ������Ƭ��
        std::list<CorkMesh::Tri> selectedTris, otherTris;
        std::set<uint> selectedVerIdx;

        int accum = 0;
        std::vector<int> oldNewTriIdxTable(bprob.mesh->tris.size(), -1);
        for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
        {
            auto& tri = bprob.mesh->tris[i];
            CorkVertex va = bprob.mesh->verts[tri.a];
            CorkVertex vb = bprob.mesh->verts[tri.b];
            CorkVertex vc = bprob.mesh->verts[tri.c];
            if (isIn(va.pos, boxIsct101) || isIn(vb.pos, boxIsct101) || isIn(vc.pos, boxIsct101))
            {
                selectedTris.push_back(tri);
                selectedVerIdx.insert(tri.a);
                selectedVerIdx.insert(tri.b);
                selectedVerIdx.insert(tri.c);

                oldNewTriIdxTable[i] = accum++;
            }
            else
                otherTris.push_back(tri);
        }
        std::vector<uint> newOldTriIdxTable;
        newOldTriIdxTable.reserve(selectedTris.size());
        for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
        {
            if (oldNewTriIdxTable[i] >= 0)
                newOldTriIdxTable.push_back(i);
        }

        std::vector<int> oldNewIdxTable(versCount, -1);
        std::vector<unsigned> newOldIdxTable(selectedVerIdx.size());
        CorkMesh newMesh;
        newMesh.verts.resize(selectedVerIdx.size());
        accum = 0;
        for (auto iter = selectedVerIdx.begin(); iter != selectedVerIdx.end(); ++iter)
        {
            unsigned oldIdx = *iter;
            newMesh.verts[accum] = bprob.mesh->verts[oldIdx];
            newOldIdxTable[accum] = oldIdx;
            oldNewIdxTable[oldIdx] = accum++;
        }

        newMesh.tris.insert(newMesh.tris.end(), selectedTris.begin(), selectedTris.end());

        // �޸�����������Ƭ:
        unsigned newMeshVersCountOri = newMesh.verts.size();
        unsigned newMeshTrisCountOri = newMesh.tris.size();
        for (auto& tri : newMesh.tris)
        {
            tri.a = static_cast<uint>(oldNewIdxTable[tri.a]);
            tri.b = static_cast<uint>(oldNewIdxTable[tri.b]);
            tri.c = static_cast<uint>(oldNewIdxTable[tri.c]);
        }


        // ����������������ʷ֣�
        CorkMesh::BoolProblem bprobNew(&newMesh);
        CorkMesh::IsctProblem iproblem(&newMesh);
        iproblem.findIntersections();                       // Ѱ������Ƭ���ߣ�

        std::vector<uint> tagTrisIdx;                     // �������ʷֵ�����Ƭ��������
        iproblem.tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
            {
                if (tpPtr->iverts.size() > 0)
                    tagTrisIdx.push_back(tpPtr->the_tri->ref);
            });

 
#ifdef MY_DEBUG
        debugWriteMesh("step3��ȡ�ĺ�������", newMesh);
#endif

        iproblem.resolveAllIntersections();
        iproblem.commit();

#ifdef MY_DEBUG
        debugWriteMesh("step3�������ʷֵ�����Ƭ", combainer);
        debugWriteMesh("step3��������㽻�ߡ������ʷ�֮��", *bprobNew.mesh);

 
        CorkMesh tempMesh;
        tempMesh.verts = bprobNew.mesh->verts;
        tempMesh.tris.insert(tempMesh.tris.end(), bprobNew.mesh->tris.begin(), bprobNew.mesh->tris.begin() + (newMeshTrisCountOri - tagTrisIdx.size()) + 1);
        debugWriteMesh("step3ɾ�����ʷֵ�����Ƭ֮�������", tempMesh);
#endif

        unsigned tagTrisCount = tagTrisIdx.size();              // ���ʷֵ�����Ƭ������
        newMeshTrisCountOri -= tagTrisCount;
        unsigned insertVersCount = newMesh.verts.size() - newMeshVersCountOri;
        unsigned insertTrisCount = newMesh.tris.size() - newMeshTrisCountOri;
        std::vector<CorkVertex> insertVers;
        insertVers.insert(insertVers.end(), newMesh.verts.begin() + newMeshVersCountOri, newMesh.verts.end());
        std::vector<CorkMesh::Tri> insertTris;
        insertTris.insert(insertTris.end(), newMesh.tris.begin() + newMeshTrisCountOri, newMesh.tris.end());

#ifdef MY_DEBUG
        debugWriteVers("step3�����Ķ���", insertVers);
        tempMesh.clear();
        tempMesh.verts = newMesh.verts;
        tempMesh.tris.insert(tempMesh.tris.end(), newMesh.tris.begin() + newMeshTrisCountOri, newMesh.tris.end());
        debugWriteMesh("step3����������Ƭ", tempMesh);
#endif

        // �޸�����������Ƭ���ݣ�
        auto handleIndex = [&](const uint nmIdx)->uint
        {
            // ԭ�ȵĶ��㣬����ӳ���ϵ��newOldIdxTable; �����Ķ��㣬��������һ��ƫ������(versCount - newMeshVersCountOri)
            if (nmIdx < newMeshVersCountOri)
                return newOldIdxTable[nmIdx];
            else
                return (nmIdx + versCount - newMeshVersCountOri);
        };

        for (auto& tri : insertTris)
        {
            tri.a = handleIndex(tri.a);
            tri.b = handleIndex(tri.b);
            tri.c = handleIndex(tri.c);
        }

        // ԭ�ں���������µ� 
        bprob.mesh->verts.insert(bprob.mesh->verts.end(), insertVers.begin(), insertVers.end());

        // ԭ�ں�����ɾ����Ҫ�����ʷֵ�����Ƭ������������Ƭ��
        std::vector<CorkMesh::Tri> tempTris;
        std::set<uint> triIdx2DelSet;
        std::vector<uint> triIdx2Del;
        for (auto& index : tagTrisIdx)
            triIdx2DelSet.insert(newOldTriIdxTable[index]);
        triIdx2Del.reserve(triIdx2DelSet.size());
        for (const auto& index : triIdx2DelSet)
            triIdx2Del.push_back(index);

        tempTris.insert(tempTris.end(), bprob.mesh->tris.begin(), bprob.mesh->tris.begin() + triIdx2Del[0]);
        for (unsigned i = 0; i < triIdx2Del.size() - 1; ++i)
            tempTris.insert(tempTris.end(), bprob.mesh->tris.begin() + triIdx2Del[i] + 1, bprob.mesh->tris.begin() + triIdx2Del[i+1]);
        tempTris.insert(tempTris.end(), bprob.mesh->tris.begin() + *triIdx2Del.rbegin() + 1, bprob.mesh->tris.end());

        tempTris.insert(tempTris.end(), insertTris.begin(), insertTris.end());
        bprob.mesh->tris = std::move(tempTris);

        //              for debug;
#ifdef MY_DEBUG
        debugWriteMesh("step3ԭ�ں���������µ㡢������Ƭ֮��", *bprob.mesh);
        std::vector<unsigned> BADcopy;              // ����Ƭ��ǵĿ�����mesh->tris[i].data.bool_alg_data
        std::vector<CorkVertex> debugVers;
        getLabelInfo(bprob, BADcopy, debugVers);
#endif

        tt.endCout("ISOLATE_TRIS_IN_BOX�����ʷֺ�ʱ��");
#endif

        
#ifdef ISOLATE_TRIS_IN_BOX
        



#else
        // 3. ��������Ƭ���ߣ����������ʷ֣����в�㣻
        unsigned versCount = bprob.mesh->verts.size();
        unsigned trisCount = bprob.mesh->tris.size();
        tt.start();
        bprob.mesh->resolveIntersections();
        tt.endCout("step3��ʱ��");
        timeAll += tt.endTik - tt.startTik;
        //              for debug;
        debugWriteMesh("step3���㽻�ߡ������ʷ�֮��", *bprob.mesh);

        unsigned insertVersCount = bprob.mesh->verts.size() - versCount;
        unsigned insertTrisCount = bprob.mesh->tris.size() - trisCount;
        CorkMesh tempMesh;
        tempMesh.verts = bprob.mesh->verts;
        tempMesh.tris.insert(tempMesh.tris.end(), bprob.mesh->tris.begin() + trisCount, bprob.mesh->tris.end());
        debugWriteMesh("step3����������Ƭ", tempMesh);
        std::vector<CorkVertex> newVers;
        newVers.insert(newVers.end(), bprob.mesh->verts.begin() + versCount, bprob.mesh->verts.end());
        debugWriteVers("step3�����Ķ���", newVers);
#endif

        // 4. ����ecache��Ա���ݡ���ȷ���ں���������бߣ����ұ�ǳ���Щ�� ������Ƭ�ཻ�ߣ�
        tt.start();
        bprob.populateECache();
        tt.endCout("step4��ʱ��");
        timeAll += tt.endTik - tt.startTik;


        // ��5��6��ò�Ʋ���Ҫ��
#if 1
    // 5.����������UnionFind����uf����֪���Ǹ�ɶ�ģ�
        tt.start();
        UnionFind uf(bprob.mesh->tris.size());
        bprob.for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
            {
                uint tid0 = tids[0];
                for (uint k = 1; k < tids.size(); k++)
                    uf.unionIds(tid0, tids[k]);
            });
        tt.endCout("step5��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        debugWriteMesh("step5", *bprob.mesh);


        // 6. ������
        tt.start();
        std::vector<uint> uq_ids(bprob.mesh->tris.size(), uint(-1));
        std::vector< std::vector<uint> > components;
        for (uint i = 0; i < bprob.mesh->tris.size(); i++)
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
        tt.endCout("step6��ʱ��");
        timeAll += tt.endTik - tt.startTik;

 
#else

    // �����Ϊ0��1������Ƭ�ֱ�ŵ����������У�
        std::vector<std::vector<uint>> components(2);
        components[0].reserve(bprob.mesh->tris.size());
        components[1].reserve(bprob.mesh->tris.size());
        for (unsigned i = 0; i < bprob.mesh->tris.size(); ++i)
        {
            const auto& tri = bprob.mesh->tris[i];
            if (0 == tri.data.bool_alg_data)
                components[0].push_back(i);
            else
                components[1].push_back(i);
        }
        components[1].shrink_to_fit();
        components[0].shrink_to_fit();

        //              for debug
        getLabelInfo(bprob, BADcopy, debugVers);
#endif

        // 7. ʹ���������ж�����Ƭ���ڲ������ⲿ����������Ƭ���޸�����Ƭ��ǣ�
        tt.start();
        std::vector<bool> visited(bprob.mesh->tris.size(), false);              // ��¼����Ƭ�Ƿ��Ѿ���������
        for (unsigned i = 0; i < components.size(); ++i)
        {
            auto& comp = components[i];

            // 7.1 �ҳ���ǰ����Ƭ���������������Ƭ��
            uint maxTriIdx = comp[0];                       // �����������Ƭ������
            double areaMax = 0.0;
            for (uint tid : comp)
            {
                Vec3d va = bprob.mesh->verts[bprob.mesh->tris[tid].a].pos;
                Vec3d vb = bprob.mesh->verts[bprob.mesh->tris[tid].b].pos;
                Vec3d vc = bprob.mesh->verts[bprob.mesh->tris[tid].c].pos;

                double area = triArea(va, vb, vc);
                if (area > areaMax)
                {
                    areaMax = area;
                    maxTriIdx = tid;
                }
            }

            // 7.2 �жϵ�ǰ�������Ƭ�Ƿ����ں������ڲ����޸ĸ��������Ƭ�ı�ǣ�
            byte currentMeshLabel = bprob.boolData(maxTriIdx);
            bool inside = bprob.isInside(maxTriIdx, currentMeshLabel);
            byte& maxTriLabel = bprob.boolData(maxTriIdx);
            maxTriLabel |= (inside) ? 2 : 0;            // ��0��10B����λ�����㣬��ǰ�����������Ƭ�������ں������ڲ�������labelǰһλ��Ϊ1�� ������Ƭ��Ǵ�����ϣ�

            // 7.3 ���������Ƭ��ʼ������������Ƭ�����ķ�ʽ����������������Ƭ�����ݲ�ͬ�����޸�����Ƭ��ǣ�
            std::queue<uint> work;                    // ��������Ƭ�Ĺ������У�        
            visited[maxTriIdx] = true;
            work.push(maxTriIdx);

            while (!work.empty())
            {
                uint curr_tid = work.front();
                byte& currentTriLabel = bprob.boolData(curr_tid);           // ��������Ƭ�ı�ǣ�
                work.pop();

                for (uint k = 0; k < 3; k++)        // ������ǰ����Ƭ�������ߣ�����������ǰ����Ƭ��������������Ƭ��
                {
                    uint a = bprob.mesh->tris[curr_tid].v[k];
                    uint b = bprob.mesh->tris[curr_tid].v[(k + 1) % 3];
                    auto& entry = bprob.ecache(a, b);                      // ��a, b�������ߵ���Ϣ��

                    // ����Ƭ��Ǻ�10B��������
                    byte inside_sig = currentTriLabel & 2;          // ��λ������������ȡĳһλ����Ϣ�������10B����λ�����㣬��ȡ��currentTriLabel��ǰһλ����ʾ�Ƿ��������ڣ�
                    if (entry.data.is_isct)
                        inside_sig ^= 2;                     // ���ñ�������Ƭ���ߣ���currentTri��nbrTri������״̬һ��������ģ�
                                                                         // ����Ƭ�����Ĺ����У����û����������Ƭ���ߣ�������Ƭ������״̬��һֱ����ģ�
                                                                          // ��10B��������㣬��ʾ������״̬ȡ����

                    for (uint nbrTriIdx : entry.tids)   // ���������߹�������������Ƭ����������ǰ����Ƭ��������������Ƭ��
                    {
                        byte& nbrTriLabel = bprob.boolData(nbrTriIdx);
                        if (visited[nbrTriIdx])
                            continue;

                        if ((nbrTriLabel & 1) != currentMeshLabel)         // ����������Ƭ������һ����������������ѭ��ֻ����ǰ�����е�����Ƭ��
                            continue;

                        nbrTriLabel |= inside_sig;          // 
                        visited[nbrTriIdx] = true;
                        work.push(nbrTriIdx);
                    }
                }
            }

            currentMeshLabel++;
        }
        tt.endCout("step7��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        // ��������Ƭ��ǣ���һλ��ʾ������Ƭԭ�������ĸ�����ǰһλ��ʾ������Ƭ�Ƿ����ں�������ⲿ��
        /*
            0��00B������0���������Ƭ����ȥ1�����ں������ڲ�����Щ��
            1��01B������1���������Ƭ����ȥ0�����ں������ڲ�����Щ��
            2��10B������0��������Ƭ�У����ں������ڲ�����Щ����Ƭ��
            3��11B������1��������Ƭ�У����ں������ڲ�����Щ����Ƭ��
        */

        // 8.��������������
        tt.start();
        bprob.doDeleteAndFlip([](byte data) -> typename CorkMesh::BoolProblem::TriCode
            {
                if (data == 2 || data == 1)           // part of op 1 OUTSIDE op 0
                    return CorkMesh::BoolProblem::DELETE_TRI;                           // ɾ�����Ϊ2, 1������Ƭ
                else if (data == 3)                   // part of op 1 INSIDE op 1
                    return CorkMesh::BoolProblem::FLIP_TRI;                                // ��ת���Ϊ3������Ƭ��
                else                                    // part of op 0 OUTSIDE op 1
                    return CorkMesh::BoolProblem::KEEP_TRI;                               // �������Ϊ0������Ƭ��
            });
        tt.endCout("step8��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        debugWriteMesh("meshDiff_δ��meshFix����Ľ��", *bprob.mesh);

        std::cout << "�ܹ���ʱ��" << timeAll << std::endl;           // ���������ʱ��ֱ�Ӳ��ԵĻ���ʱ6200+ms;
        std::cout << "finished." << std::endl;
        getchar();
    }
}




// ���������󽻣�
namespace TEST_RAY
{
    void test0()
    {
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(std::string{ "./data/cube.off" }, &ioMesh0);
        loadMesh(std::string{ "./data/cylinder2.off" }, &ioMesh1);
        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);

        CorkMesh testTriangle;
        testTriangle.verts.resize(3);
        testTriangle.tris.resize(1);
        testTriangle.tris[0].a = 0;
        testTriangle.tris[0].b = 1;
        testTriangle.tris[0].c = 2;
        testTriangle.verts[0].pos = Vec3d{ 5, 5, 5 };
        testTriangle.verts[1].pos = Vec3d{ -5, 5, 5 };
        testTriangle.verts[2].pos = Vec3d{ 0, 0, 0 };
        debugWriteMesh("testTriangle1", testTriangle);
        CorkVertex& va = testTriangle.verts[0];
        CorkVertex& vb = testTriangle.verts[1];
        CorkVertex& vc = testTriangle.verts[2];

        if (myIsInside(va, vb, vc, mesh0))
            std::cout << "��������" << std::endl;
        else
            std::cout << "��������" << std::endl;

        vc.pos = Vec3d{ 10, 10, 10 };
        debugWriteMesh("testTriangle2", testTriangle);
        if (myIsInside(va, vb, vc, mesh0))
            std::cout << "��������" << std::endl;
        else
            std::cout << "��������" << std::endl;

        getchar();
    }
}



// ������Ŀ�Զ���Ļ������ͣ�
namespace TEST_BASIC
{
    // ���Ե����ڴ��iterPool
    void test0()
    {
        std::vector<int> intVec{ -1, -2, -3, -4, -5, -6, -7, -8 };
        IterPool<int> intPool1, intPool2;

        // alloc()�������з����һ����Ԫ�صĿռ䣻
        for (unsigned i = 1; i <= 9; ++i)
        {
            int* elemPtr = intPool1.alloc();
            *elemPtr = i;
        }

        // for_each()�����������е�Ԫ�أ�
        unsigned elemCount = intPool1.size();
        std::cout << "disp all elements in intPool1: " << std::endl;
        intPool1.for_each([&](int* intPtr)
            {
                std::cout << *intPtr << ", ";
            });
        std::cout << std::endl;

        std::cout << "finished." << std::endl;
    }

    // ����AABB��bbox3d
    void test1()
    {
        CorkMesh::Tri tri1(0, 1, 2);
        std::vector<CorkVertex> vers1{ CorkVertex(-3, 0, 8), CorkVertex(-3, 3, 4), CorkVertex(0, -3, 4) };
        CorkMesh triMesh1;
        triMesh1.verts = vers1;
        triMesh1.tris.push_back(tri1);
        debugWriteMesh("triMesh1", triMesh1);

        // ��ӡ��һ������Ƭ�İ�Χ�У�
        CorkMesh::TopoCache tCache(&triMesh1);
        TopoTri tt0 = *tCache.tris.begin();
        BBox3d tbox0 = getAABB(&tt0, triMesh1);

        CorkMesh boxMesh0;
        getAABBmesh(boxMesh0, tbox0);
        debugWriteMesh("boxMesh", boxMesh0);
    }


    // ���Կɵ����ڴ��iterPool
    void test2() 
    {
        IterPool<int> numPool;
        for (unsigned i = 1; i <= 99; ++i) 
        {
            int* elemPtr = numPool.alloc();
            *elemPtr = static_cast<int>(i);
        }

        std::vector<int*> numPtrs;
        numPtrs.reserve(numPool.size());
        numPool.for_each([&](int* ip) 
            {
                numPtrs.push_back(ip);
            });


        // �����ָ�������
        std::cout << "�����ָ�������" << std::endl;
        {
            for (unsigned i = 0; i < 10; ++i)
                std::cout << *numPtrs[i] << ", ";
            std::cout << std::endl;

            auto rIter = numPtrs.rbegin();
            for (unsigned i = 0; i < 10; ++i, ++rIter)
                std::cout << **rIter << ", ";
            std::cout << std::endl; 
        }

        // ����ɾ��������
        numPool.for_each([&](int* ip) 
            {
                if (0 != *ip % 2)
                    numPool.free(ip);
            });
          
        // ������ĵ�����
        std::cout << "���Լ��ĵ�����" << std::endl;
        auto iter = numPool.begin(); 
        for (unsigned i = 0; i < 10; ++i, ++iter)
            std::cout << *iter << ", ";        
        std::cout << std::endl;

        // �����ָ�������
        std::cout << "�����ָ�������" << std::endl;
        {
            for (unsigned i = 0; i < 10; ++i)
                std::cout << *numPtrs[i] << ", ";                               // ����Ԫ���ѱ��ͷţ����Ұָ�룬���ǿ�ָ�룻
            std::cout << std::endl;

            auto rIter = numPtrs.rbegin();
            for (unsigned i = 0; i < 10; ++i, ++rIter)
                std::cout << **rIter << ", ";
            std::cout << std::endl;
        }

        std::cout << "�����ָ�������" << std::endl;
        {
            for (unsigned i = 0; i < 10; ++i)
                std::cout << (nullptr == numPtrs[i] ? "nullPtr": "concrete") << ", ";      // ����Ԫ���ѱ��ͷţ����Ұָ�룻
            std::cout << std::endl;

            auto rIter = numPtrs.rbegin();
            for (unsigned i = 0; i < 10; ++i, ++rIter)
                std::cout << (nullptr == *rIter ? "nullPtr" : "concrete") << ", ";
            std::cout << std::endl;
        }
    }


    // �������ڴ�صĶ�̬����ShortVec
    void test3() 
    {
        ShortVec<int, 3> intSvec;
        intSvec.push_back(1);
        intSvec.push_back(1);
        intSvec.push_back(2);
        intSvec.push_back(3);
        intSvec.push_back(1);
        intSvec.push_back(3);
        intSvec.push_back(3);
        intSvec.push_back(2);
        intSvec.push_back(4);
        intSvec.push_back(1);
        intSvec.push_back(6);
        intSvec.push_back(0);
        intSvec.push_back(2);
        intSvec.disp();

        int* numPtr = nullptr;
        for (auto& num : intSvec)           
        {
            if (num == 2)
            {
                intSvec.erase(num);             // erase֮��numPtrò����Ұָ�롣
                numPtr = &num;
            }
        }

        int ret = std::distance(intSvec.begin(), numPtr);       // 12
        int temp = *numPtr;     // 2
        intSvec.disp();
        std::cout << ret << std::endl;
    }


    // ��������������Ϣ�ṹ��TopoCache
    void test4()
    {
        CorkTriMesh  ioMesh0;
        loadMesh(g_debugPath + std::string{ "cube.off" }, &ioMesh0);
        CorkMesh mesh0;
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        // 1. ��������������Ϣ����TopoCache��EGraphCache
        CorkMesh::TopoCache tCache(&mesh0);                             // ����mesh0��TopoCache��ʾ��������Ϣ�����캯�����ɣ�
 
        // 2. gCache�еı���Ϣ�����Զ���ĸ�����edgeInfo��ʾ��
        std::vector<edgeInfo> edgeInfoVecG;
        unsigned edgesCount1 = tCache.edges.size();

         // EGraphCache�м�¼�ı�һ���Ƕ���������С����ǰ����ں�
        std::vector<TopoEdge> tpEdges = tCache.edges.toVec();
        std::vector<TopoEdgeInfo> tpEdgesInfo;
        std::vector<std::pair<uint, uint>> edgeInfoVecT;
        tpEdgesInfo.reserve(tpEdges.size());
        edgeInfoVecT.reserve(tpEdges.size());
        for (const auto& tpEdge : tpEdges)
        {
            auto info = TopoEdgeInfo(tpEdge);
            tpEdgesInfo.push_back(info);
            edgeInfoVecT.push_back(std::make_pair(info.verts_rela[0].ref, info.verts_rela[1].ref));
        }

        // ��������Ƭ��
        std::vector<std::tuple<uint, uint, uint>> trisInfoVec;
        trisInfoVec.reserve(mesh0.tris.size());
        for (const auto& tpTri: tCache.tris) 
        {
            auto tuple = std::make_tuple(tpTri.verts[0]->ref, tpTri.verts[1]->ref, tpTri.verts[2]->ref);
            trisInfoVec.push_back(tuple);
        }

        std::cout << "finished." << std::endl;
    }
 

    // ����EGraphCache����Ϣ��
    void test5() 
    {
        CorkTriMesh  ioMesh0;
        loadMesh(g_debugPath + std::string{ "cube.off" }, &ioMesh0);
        CorkMesh mesh0;
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        //      EGraphCache()
        CorkMesh::EGraphCache<CorkMesh::BoolProblem::BoolEdata> gCache = \
            mesh0.createEGraphCache<CorkMesh::BoolProblem::BoolEdata>();    // ����mesh0��EGraphCache��ʾ��������Ϣ

        // 2. gCache�еı���Ϣ�����Զ���ĸ�����edgeInfo��ʾ��
        std::vector<edgeInfo> edgeInfoVecG;
        copyEdgesInfo(mesh0, gCache, edgeInfoVecG);
        unsigned edgesCount2 = edgeInfoVecG.size();              // EGraphCache�м�¼�ı�������TopoCache����������Ϊ���߰�(1,2)(2,1)��¼��һ���ߣ�

        std::cout << "finished." << std::endl;
    }


    // ����BVH
    void test6()
    {
        // ֪ʶ�㣺 1. ��ΰ�Χ�У�����Ƭ����ʹ�ã�2. ������&������������Ƭ������������ж���ʹ�ã�
        tiktok& tt = tiktok::getInstance();

        // 0. ��ȡ���񣬼�����������Ƿ�Ϸ������ɼ�������������ɲ������������
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "cmesh1.off" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "cylinder2.off" }, &ioMesh1);

        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        CorkMesh::BoolProblem bprob(&mesh0);
 
        unsigned timeAll = 0;
        tt.start();
        // 1. ����Ƭ��ǡ���mesh0��bool_alg_data���Ϊ0��mesh1�ı��Ϊ1��
        for (auto& tri : bprob.mesh->tris)
            tri.data.bool_alg_data = static_cast<byte>(0);

        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);

        // 2. ֱ���ں���������
        bprob.mesh->disjointUnion(mesh1);                          // ��������ֱ���ں�
        tt.endCout("step1& step2��ʱ��");
        timeAll += tt.endTik - tt.startTik;
        debugWriteMesh("step2ֱ���ںϺ������", *bprob.mesh);
        CorkMesh& mesh = *bprob.mesh;
        debugWriteTriMesh("isctTri", mesh, 7779);           // �����̵�����Ƭ��
        debugWriteEdge("isctEdge", CorkEdge(mesh.verts[633], mesh.verts[636]));     // ����7779����Ƭ�ıߣ�

        // 3. ��õ�ǰ�������бߵĲ�ΰ�Χ�У�
        Mesh<CorkVertex, CorkTriangle>::IsctProblem isctObj(&mesh);
        AABVH<TopoEdge*>& bvh = isctObj.bvhObj;
        isctObj.edge_geoms.clear();

        //      3.1. �������бߵ�GeomBlob��Ϣ��
       isctObj.edges.for_each(\
            [&](TopoEdge* ePtr)
            {
                isctObj.edge_geoms.push_back(isctObj.edge_blob(ePtr));
            });

        //      3.2. �������бߵ�AABVH���� 
        bvh.build(isctObj.edge_geoms);                     
 

        // 4. ��������isctEdge��bvh�ڵ㣺
        AABVHNode<TopoEdge*>* nodePtr = bvh.searchLeaf(\
            [&](AABVHNode<TopoEdge*>* nodePtr)->bool
            {
                // ������ǰҶ�ӽڵ��е����бߣ�
                for (uint gbIdx : nodePtr->blobids)
                {
                    const GeomBlob<TopoEdge*>& currentBlob = bvh.blobs[gbIdx];
                    TopoEdge& te = *currentBlob.id;
                    if (te.verts[0]->ref == 633 || te.verts[0]->ref == 636)
                    {
                        if (te.verts[1]->ref == 633 || te.verts[1]->ref == 636)
                            return true;
                    }
                }
                return false;
            });

        // 5. ��ӡ�ýڵ�İ�Χ������
        CorkMesh boxMesh;
        getAABBmesh(boxMesh, nodePtr->bbox);
        debugWriteMesh("boxMesh_node", boxMesh);

        // 6. ��ýڵ�����и��ڵ㣬��ǰ�д���
        std::vector<AABVHNode<TopoEdge*>*> parents;
        bvh.searchParents(parents, nodePtr);

        for (unsigned i = 0; i < parents.size(); ++i)
        {
            auto np = parents[i];
            char str[256];
            sprintf(str, "nodeBox%d", i);
            getAABBmesh(boxMesh, np->bbox);
            debugWriteMesh(str, boxMesh);
        }

        std::cout << "finished." << std::endl;
    }
}
 

// ���Բ�ΰ�Χ��
namespace TEST_BVH
{
    //  ����step3
    void test0()
    {
        tiktok& tt = tiktok::getInstance();
        unsigned timeAll = 0;

        // step3֮ǰ�Ĳ��裺
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "s9block.off" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "fatTeeth.off" }, &ioMesh1);
        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);

        for (auto& tri : mesh0.tris)
            tri.data.bool_alg_data = static_cast<byte>(0);
        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);
        mesh0.disjointUnion(mesh1);
        debugWriteMesh("step2����ֱ���ںϵ���������", mesh0);

        // 3. step3Ѱ������Ƭ���߶Σ�ִ�������ʷ֣�
        tt.start();
        Mesh<CorkVertex, CorkTriangle>::IsctProblem isctObj(&mesh0);

        //      һЩ����debug���ӵ����ã�
        std::vector< GeomBlob<TopoEdge*> >& edge_geoms = isctObj.edge_geoms;        // ��������AABVH�����б����ݣ�
        AABVH<TopoEdge*>& bvhObj = isctObj.bvhObj;                                                       // isctObj�в�ΰ�Χ�ж�������ã�
        IterPool<Mesh<CorkVertex, CorkTriangle>::TriangleProblem>& tprobs = isctObj.tprobs;     // �洢������Ƭ������Ϣ��
        const auto& meshVers = mesh0.verts;
        const auto& meshTris = mesh0.tris;

        //      3.1  Ѱ������Ƭ���ߣ�
        isctObj.findIntersections();
        tt.endCout("step3.1��ʱ��");                       // ��ȥ�����ಽ��Ļ���ʱ5078ms;
        timeAll += tt.endTik - tt.startTik;

        // for debug
        {
            // �ҳ���ǰ�����ཻ������Ƭ����ӡ��Ӧ������
            std::vector<unsigned> isctTrisIdx;
            isctTrisIdx.reserve(tprobs.size());
            CorkMeshCombainer isctTrisMesh;
            tprobs.for_each([&](Mesh<CorkVertex, CorkTriangle>::TriangleProblem* tpPtr)
                {
                    isctTrisIdx.push_back(tpPtr->the_tri->ref);
                });
            for (const auto& index : isctTrisIdx)
            {
                CorkMesh currentTriMesh;
                const auto& currentTri = mesh0.tris[index];
                currentTriMesh.verts.push_back(mesh0.verts[currentTri.a]);
                currentTriMesh.verts.push_back(mesh0.verts[currentTri.b]);
                currentTriMesh.verts.push_back(mesh0.verts[currentTri.c]);
                currentTriMesh.tris.resize(1);
                currentTriMesh.tris[0].a = 0;
                currentTriMesh.tris[0].b = 1;
                currentTriMesh.tris[0].c = 2;
                isctTrisMesh.add(currentTriMesh);
            }
            debugWriteMesh("�ཻ������Ƭ", isctTrisMesh);

            // ��ӡ��һ���ཻ����Ƭ��������ص���Ϣ��
            auto iter = tprobs.begin();
            Mesh<CorkVertex, CorkTriangle>::TriangleProblem& tprob0 = *iter;        // ��һ��triangleProblem����
            CorkMesh triMesh0;
            const auto& tri0 = mesh0.tris[tprob0.the_tri->ref];
            triMesh0.verts.push_back(mesh0.verts[tri0.a]);
            triMesh0.verts.push_back(mesh0.verts[tri0.b]);
            triMesh0.verts.push_back(mesh0.verts[tri0.c]);
            triMesh0.tris.resize(1);
            triMesh0.tris[0].a = 0;
            triMesh0.tris[0].b = 1;
            triMesh0.tris[0].c = 2;
            debugWriteMesh("��һ���ཻ����Ƭ", triMesh0);

            unsigned isctEdgeCount0 = tprob0.iedges.size();         // ��triMesh0�ཻ�ı�����
            IsctEdgeType isctEdge0 = *tprob0.iedges[0];
            GenericVertType gVer00, gVer01;
            gVer00 = *isctEdge0.ends[0];
            gVer01 = *isctEdge0.ends[1];
            CorkVertex ver00, ver01;
            ver00.pos = gVer00.coord;
            ver01.pos = gVer01.coord;
            std::pair<unsigned, unsigned> tmpPair = std::make_pair(0, 1);
            debugWriteEdges("�͵�һ���ཻ����Ƭ�ཻ�ı�", std::vector<CorkVertex>{ver00, ver01}, \
                std::vector<std::pair<unsigned, unsigned>>{tmpPair});
        }

        //      3.2 
        tt.start();
        isctObj.resolveAllIntersections();              // ��������Ƭ����������
        tt.endCout("step3.2��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        debugWriteMesh("step3.2����resolveAllIntersections", mesh0);

        //      3.3
        tt.start();
        isctObj.commit();       // ����Ƭ���м��١���������ȥ���ظ�����Ƭ��������
        tt.endCout("step3.3��ʱ��");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        std::vector<uint> ivertsSizeInfo, iedgesSizeInfo, gtrisSizeInfo;
        ivertsSizeInfo.reserve(tprobs.size());
        iedgesSizeInfo.reserve(tprobs.size());
        gtrisSizeInfo.reserve(tprobs.size());
        tprobs.for_each([&](Mesh<CorkVertex, CorkTriangle>::TriangleProblem* tpPtr)
            {
                ivertsSizeInfo.push_back(tpPtr->iverts.user_size);
                iedgesSizeInfo.push_back(tpPtr->iedges.user_size);
                gtrisSizeInfo.push_back(tpPtr->gtris.user_size);
            });

        debugWriteMesh("step3���������ʷֺ���ں�����", mesh0);

        std::cout << "step3�ܺ�ʱ��" << timeAll << std::endl;
        std::cout << "finished." << std::endl;
    }


    // �����õ�������Ƭ�����񽻲�
    void test1()
    {
        CorkTriMesh ioMesh0;
        loadMesh(g_debugPath + std::string{ "cube.off" }, &ioMesh0);
        CorkMesh mesh0;

        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        for (auto& tri : mesh0.tris)
            tri.data.bool_alg_data = static_cast<byte>(0);
        debugWriteMesh("mesh0", mesh0);

        // ��һ����mesh0�ཻ������Ƭtri1��
        CorkMesh::Tri tri1(0, 1, 2);
        std::vector<CorkVertex> vers1{ CorkVertex(-3, 0, 8), CorkVertex(0, 3, 0), CorkVertex(0, -3, 0) };
        CorkMesh triMesh1;
        triMesh1.verts = vers1;
        triMesh1.tris.push_back(tri1);
        debugWriteMesh("triMesh1", triMesh1);

        // ����Ƭ�����mesh0�ں�
        mesh0.disjointUnion(triMesh1);

        // �����ں�������ÿһ���ߵ�gb����
        CorkMesh::TopoCache tCache(&mesh0);
        std::vector<GeomBlob<TopoEdge*>> edgeBlobs;
        edgeBlobs.reserve(tCache.edges.size());
        tCache.edges.for_each([&](TopoEdge* ePtr)
            {
                GeomBlob<TopoEdge*> blob = getEdgeBlob(ePtr, mesh0);
                edgeBlobs.push_back(blob);
            });

        // �����ں��������б��������ɵ�AABVH��ΰ�Χ�ж���
        AABVH<TopoEdge*> aabvh(edgeBlobs);

        // �����ں�������������Ƭ��
        bool findIsct = false;
        std::vector<AABVHNode<TopoEdge*>*> isctNodes;        // �洢���ҵ��ĺ�ĳһ������Ƭ�ཻ������aabvhҶ�ӽڵ�ָ��
        TopoTri* isctTriPtr0 = nullptr;                            // ��һ�������̵�����Ƭ��
        tCache.tris.for_each([&](TopoTri* triPtr)
            {
                if (findIsct)
                    return;         // ���Ѿ��ҵ���һ�������̵�����Ƭ���򲻼������ң�

                BBox3d tbbox = getAABB(triPtr, mesh0);       // ��ǰ����Ƭ�İ�Χ�У�

                // ʹ�õ�ǰ����Ƭ�İ�Χ�У�����aabvh���ҳ��͵�ǰ��Χ���ཻ��Ҷ�ӽڵ㣻
                aabvh.for_each_in_box(tbbox, [&](AABVHNode<TopoEdge*>* nodePtr)           // ��aabvh�ı�����
                    {
                        isctNodes.push_back(nodePtr);
                        isctTriPtr0 = triPtr;
                    });

                if (isctNodes.size() > 0)
                    findIsct = true;
            });

        CorkMesh::Tri isctTri0 = mesh0.tris[isctTriPtr0->ref];
        CorkMesh isctTriMesh0;
        isctTriMesh0.verts = std::vector<CorkVertex>{ mesh0.verts[isctTri0.a], mesh0.verts[isctTri0.b], mesh0.verts[isctTri0.c] };
        isctTriMesh0.tris.push_back(CorkMesh::Tri(0, 1, 2));
        debugWriteMesh("��һ�������̵�����Ƭ", isctTriMesh0);

        BBox3d box0 = getAABB(isctTriPtr0, mesh0);
        CorkMesh boxMesh0;
        getAABBmesh(boxMesh0, box0);
        debugWriteMesh("��һ�������̵�����Ƭ�İ�Χ��", boxMesh0);

        std::vector<BBox3d> nodeBoxs;           // ��ѡ�еĽڵ�İ�Χ�У�
        nodeBoxs.reserve(isctNodes.size());
        std::vector<TopoEdge> isctEdges;        // ��ѡ�еĽڵ��е����бߣ�
        for (const auto& nodePtr : isctNodes)
        {
            std::vector<uint> blobIdxVec = nodePtr->blobids.toVec();
            nodeBoxs.push_back(nodePtr->bbox);

            unsigned edgeCount = blobIdxVec.size();
            for (const auto& index : blobIdxVec)
            {
                const GeomBlob<TopoEdge*>& currentBlob = aabvh.blobs[index];
                TopoEdge& currentEdge = *currentBlob.id;
                isctEdges.push_back(currentEdge);
            }
        }
        debugWriteEdges("��ѡ�еĽڵ��е����б�", isctEdges, mesh0);

        CorkMesh nodeBoxMesh;
        for (unsigned i = 0; i < isctNodes.size(); ++i)
        {
            getAABBmesh(nodeBoxMesh, nodeBoxs[i]);
            char str0[100];
            sprintf(str0, "��ѡ�нڵ�%d�İ�Χ��", i);
            debugWriteMesh(str0, nodeBoxMesh);
        }

        std::cout << "finished." << std::endl;
    }


    // �������ֶ�ʩ��΢�ŵ�����½��е�������
    void test3()
    {
        tiktok& tt = tiktok::getInstance();

        // step3֮ǰ�Ĳ��裺
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "s9block��������/input1.obj" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "s9block��������/input2.obj" }, &ioMesh1);
        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);

        tt.start();

        // ��������������԰�Χ�е��ཻ��Χ�У�
        BBox3d box0 = getAABB(mesh0);
        BBox3d box1 = getAABB(mesh1);
        BBox3d boxIsct = isct(box0, box1);
        BBox3d boxIsct101 = multiply(boxIsct, 1.01);
        CorkMesh boxMesh0, boxMesh1, boxMeshIsct, boxMeshIsct101;
        getAABBmesh(boxMesh0, box0);
        getAABBmesh(boxMesh1, box1);
        getAABBmesh(boxMeshIsct, boxIsct);
        getAABBmesh(boxMeshIsct101, boxIsct101);

        for (auto& tri : mesh0.tris)
            tri.data.bool_alg_data = static_cast<byte>(0);
        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);
        mesh0.disjointUnion(mesh1);

        // for debug:
        debugWriteMesh("�ں�����", mesh0);
        debugWriteTriMesh("degTri", mesh0, 229465);
        debugWriteEdge("degEdge", mesh0.verts[10599], mesh0.verts[43353]);

        // 3. step3Ѱ������Ƭ���߶Σ�ִ�������ʷ֣�
        CorkMesh::IsctProblem isctObj(&mesh0);

#ifdef TEST_CROSS_BOX_ACCE
        isctObj.getSelectedTrisIdx(boxIsct101);
#endif

        //      3.1.1 ʩ��΢�ţ������˲���
        int nTrys = 5;
        isctObj.perturbPositions();                 // always perturb for safety...

        //      3.1.2 tryToFindIntersections()Ѱ������Ƭ�ͱ��ཻ�����Σ�ԭ��Ŀ�г���5�Σ�����ֻ����һ��

        // 1. �����ں��������бߵĲ�ΰ�Χ��aabvh������ʹ��ÿһ������Ƭ����aabvh��Ѱ�ұߺ�����Ƭ�Ľ��㣻
        isctObj.bvh_edge_tri([&](TopoEdge* eisct, TopoTri* tisct)->bool
            {
                // 1.1 ��⵱ǰ����Ƭ�Ƿ񱻴��̣�
                if (isctObj.checkIsct(eisct, tisct))
                {

#ifdef TEST_CROSS_BOX_ACCE
                    // for test��������ǰ���̱����ڵ�����Ƭ��ȫ���ڰ�Χ��֮�⣬��������
                    bool allSpearTriOut = true;
                    for (unsigned i = 0; i < eisct->tris.user_size; ++i)
                    {
                        TopoTri* spearTriPtr = eisct->tris[i];
                        if (nullptr == spearTriPtr)
                            continue;
                        if(1 == isctObj.selectedTriFlags[spearTriPtr->ref])
                        {
                            allSpearTriOut = false;
                            break;
                        }
                    }
                    if (allSpearTriOut)
                        return true;
#endif

                    // 1.1.1 ����GluePointMarker�����ڼ�������Ƭ�ͱ߽���ʱ���ã�
                    GluePointMarker* glue = isctObj.newGluePt();
                    glue->edge_tri_type = true;
                    glue->e = eisct;
                    glue->t[0] = tisct;

                    // 1.1.2 ��ǰ�����̵�����Ƭ����triangleProblem����������Ҫ�¼ӵĵ㣻
                    /*
                        ��ǰ����Ƭ������ �� ���ɸ�����Ƭ��Ӧ��tiangleProblem�������ڲ�����1����������iv��2��������ie0, ie1��
                        ivΪ������Ƭ�����̴��ĵ㣬������Ƭ�ڲ���
                        ��ʱ���������ߵ�״̬��ȫ��ͬ���˵㶼��δ��״̬������һ���˵�ȡΪiv����һ���˵��ָ����nullptr;
                    */
                    CorkMesh::TriangleProblem* tpPtr = isctObj.getTprob(tisct);
                    IsctVertType* iv = tpPtr->addInteriorEndpoint(&isctObj, eisct, glue);


                    // 1.1.3 ���ɵ�ǰ����Ƭ��Ҫ�¼ӵı����ݣ�
                    for (TopoTri* spearTri : eisct->tris)        // �����������̱ߵ���������Ƭ����Щ����ƬҲ��Ҫ�¼ӵ�ͱߣ�
                    {
#ifdef TEST_CROSS_BOX_ACCE
                        // for test����tisctһ���ǽ����Χ���ڵ�����Ƭ��eisct��������������Ƭ�У����ܴ��ڽ����Χ���������Ƭ������������                   
                        if (0 == isctObj.selectedTriFlags[spearTri->ref])
                            continue;
#endif
                        CorkMesh::TriangleProblem* spearTPptr = isctObj.getTprob(spearTri);
                        spearTPptr->addBoundaryEndpoint(&isctObj, tisct, eisct, iv);
                    }
                }

                // 1.2 
                if (Empty3d::degeneracy_count > 0)
                {
                    // for debug:
                    TopoEdgeInfo eInfo(*eisct);
                    TopoTriInfo tInfo(*tisct);
                    return false;               // �Բ�ΰ�Χ�еı�����ֹ
                }
                else
                    return true;                // continue
            });

        // 2. �������˻�����
        if (Empty3d::degeneracy_count > 0)
        {
            std::cout << "Empty3d::degeneracy_count > 0" << std::endl;
            getchar();
            return;
        }

    // 3. ����tp����Ѱ��ttt�ཻ���Ρ�������������Ƭt0, t1, t2���������ཻ
        std::vector<TriTripleTemp> triples;
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tprob)
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

                            CorkMesh::TriangleProblem* prob1 = reinterpret_cast<CorkMesh::TriangleProblem*>(t1->data);              // t1��tp����

                            for (IsctEdgeType* ie : prob1->iedges)      // ������Ƭt1�Ľ��߶εı�����
                            {
                                if (ie->other_tri_key == t2)            // ��t1��t2�н��߶Σ���˵��t0, t1, t2�����ཻ��
                                    triples.push_back(TriTripleTemp(t0, t1, t2));
                            }
                        }
                    });
            });

        // 4. ��֤ttt�ཻ��������ʵ���ڣ������˻����Σ����˻����η���false����ֹ����
        for (TriTripleTemp t : triples)
        {
            if (!isctObj.checkIsct(t.t0, t.t1, t.t2))               // ���¼��һ����������Ƭ�Ľ������
                continue;

            // Abort if we encounter a degeneracy
            if (Empty3d::degeneracy_count > 0)
                break;

            GluePointMarker* glue = isctObj.newGluePt();
            glue->edge_tri_type = false;
            glue->t[0] = t.t0;
            glue->t[1] = t.t1;
            glue->t[2] = t.t2;
            isctObj.getTprob(t.t0)->addInteriorPoint(&isctObj, t.t1, t.t2, glue);
            isctObj.getTprob(t.t1)->addInteriorPoint(&isctObj, t.t0, t.t2, glue);
            isctObj.getTprob(t.t2)->addInteriorPoint(&isctObj, t.t0, t.t1, glue);
        }

        if (Empty3d::degeneracy_count > 0)
        {
            std::cout << "Empty3d::degeneracy_count > 0" << std::endl;
            getchar();
            return;
        }

        //      3.1.3 ������������
        if (nTrys <= 0)
        {
            CORK_ERROR("Ran out of tries to perturb the mesh");
            exit(1);
        }

        // �ϵĴ���ȱʧβ�˵�Ľ��߶εĳ���
#ifndef IGNORE_SELF_ISCT
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tprob)
            {
                tprob->consolidate(&isctObj);
            });
#endif

        // ���ҡ�ɾ����̬���߶Ρ���̬���㣻
#ifdef IGNORE_SELF_ISCT

        // ���߶�ie�������isctObj.iepool�У�tp�����д�����Щie��ָ�룻
        std::set<IsctVertType*> allSickVers;
        std::set<IsctEdgeType*> allSickEdges;
        std::set<IsctVertType*> sickVersPtrSet;
        std::set<IsctEdgeType*> sickEdgesPtrSet;

        // ���Ҳ�̬���߶Ρ�������ȱʧ�˵����ж�Ϊ��̬���߶Σ�
        for (auto& ie : isctObj.iepool)
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
            isctObj.ivpool.for_each([&](IsctVertType* iv)                // �Խ���صı�����
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
                isctObj.killIsctEdge(iePtr);
            for (auto& ivPtr : sickVersPtrSet)
                isctObj.killIsctVert(ivPtr);

            // ���Ҳ�̬���߶Σ�
            sickEdgesPtrSet.clear();
            isctObj.iepool.for_each([&](IsctEdgeType* iePtr)
                {
                    // ��ȱʧ�˵㣬�ж�Ϊ��̬���߶Σ�
                    if (nullptr == iePtr->ends[0] || nullptr == iePtr->ends[1])
                        sickEdgesPtrSet.insert(iePtr);
                });

            // ��������Ϣ��
            for (auto& iePtr : sickEdgesPtrSet)
                allSickEdges.insert(iePtr);

        } while (!sickEdgesPtrSet.empty());

        unsigned ivCount = isctObj.ivpool.size();
        unsigned ieCount = isctObj.iepool.size();

        // ÿһ��tp������ɾ������̬�Ľ���ͽ��߶ε�ָ�룬�� 
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
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
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
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
            isctObj.tprobs.free(tpPtr);

#endif

        // 3.2
        isctObj.resolveAllIntersections();

        // 3.3
        isctObj.commit();

        // ʣ�µĲ��裺
        CorkMesh::BoolProblem bprob(&mesh0);

        // 4. ����ecache��Ա���ݡ���ȷ���ں���������бߣ����ұ�ǳ���Щ�� ������Ƭ�ཻ�ߣ�
        bprob.populateECache();

        // 5.����������UnionFind����uf����֪���Ǹ�ɶ�ģ�
        UnionFind uf(bprob.mesh->tris.size());
        bprob.for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
            {
                uint tid0 = tids[0];
                for (uint k = 1; k < tids.size(); k++)
                    uf.unionIds(tid0, tids[k]);
            });

        // 6. ������
        std::vector<uint> uq_ids(bprob.mesh->tris.size(), uint(-1));
        std::vector< std::vector<uint> > components;
        for (uint i = 0; i < bprob.mesh->tris.size(); i++)
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

        // 7. ʹ���������ж�����Ƭ���ڲ������ⲿ����������Ƭ���޸�����Ƭ��ǣ�
        std::vector<bool> visited(bprob.mesh->tris.size(), false);              // ��¼����Ƭ�Ƿ��Ѿ���������

        for (unsigned i = 0; i < components.size(); ++i)
        {
            auto& comp = components[i];

            // 7.1 �ҳ���ǰ����Ƭ���������������Ƭ��
            uint maxTriIdx = comp[0];                       // �����������Ƭ������
            double areaMax = 0.0;
            for (uint tid : comp)
            {
                Vec3d va = bprob.mesh->verts[bprob.mesh->tris[tid].a].pos;
                Vec3d vb = bprob.mesh->verts[bprob.mesh->tris[tid].b].pos;
                Vec3d vc = bprob.mesh->verts[bprob.mesh->tris[tid].c].pos;

                double area = triArea(va, vb, vc);
                if (area > areaMax)
                {
                    areaMax = area;
                    maxTriIdx = tid;
                }
            }

            // 7.2 �жϵ�ǰ�������Ƭ�Ƿ����ں������ڲ����޸ĸ��������Ƭ�ı�ǣ�
            byte currentMeshLabel = bprob.boolData(maxTriIdx);
            bool inside = bprob.isInside(maxTriIdx, currentMeshLabel);
            byte& maxTriLabel = bprob.boolData(maxTriIdx);
            maxTriLabel |= (inside) ? 2 : 0;            // ��0��10B����λ�����㣬��ǰ�����������Ƭ�������ں������ڲ�������labelǰһλ��Ϊ1�� ������Ƭ��Ǵ�����ϣ�

            // 7.3 ���������Ƭ��ʼ������������Ƭ�����ķ�ʽ����������������Ƭ�����ݲ�ͬ�����޸�����Ƭ��ǣ�
            std::queue<uint> work;                    // ��������Ƭ�Ĺ������У�        
            visited[maxTriIdx] = true;
            work.push(maxTriIdx);

            while (!work.empty())
            {
                uint curr_tid = work.front();
                byte& currentTriLabel = bprob.boolData(curr_tid);           // ��������Ƭ�ı�ǣ�
                work.pop();

                for (uint k = 0; k < 3; k++)        // ������ǰ����Ƭ�������ߣ�����������ǰ����Ƭ��������������Ƭ��
                {
                    uint a = bprob.mesh->tris[curr_tid].v[k];
                    uint b = bprob.mesh->tris[curr_tid].v[(k + 1) % 3];
                    auto& entry = bprob.ecache(a, b);                      // ��a, b�������ߵ���Ϣ��

                    // ����Ƭ��Ǻ�10B��������
                    byte inside_sig = currentTriLabel & 2;          // ��λ������������ȡĳһλ����Ϣ�������10B����λ�����㣬��ȡ��currentTriLabel��ǰһλ����ʾ�Ƿ��������ڣ�
                    if (entry.data.is_isct)
                        inside_sig ^= 2;                     // ���ñ�������Ƭ���ߣ���currentTri��nbrTri������״̬һ��������ģ�
                                                                         // ����Ƭ�����Ĺ����У����û����������Ƭ���ߣ�������Ƭ������״̬��һֱ����ģ�
                                                                          // ��10B��������㣬��ʾ������״̬ȡ����

                    for (uint nbrTriIdx : entry.tids)   // ���������߹�������������Ƭ����������ǰ����Ƭ��������������Ƭ��
                    {
                        byte& nbrTriLabel = bprob.boolData(nbrTriIdx);
                        if (visited[nbrTriIdx])
                            continue;

                        if ((nbrTriLabel & 1) != currentMeshLabel)         // ����������Ƭ������һ����������������ѭ��ֻ����ǰ�����е�����Ƭ��
                            continue;

                        nbrTriLabel |= inside_sig;          // 
                        visited[nbrTriIdx] = true;
                        work.push(nbrTriIdx);
                    }
                }
            }

            currentMeshLabel++;
        }

        // 8. ��ͬ�Ĳ������㣬�Ա�ǵ�����Ƭ����ͬ�Ĳ�����
        bprob.doDeleteAndFlip([](byte data) -> typename CorkMesh::BoolProblem::TriCode
            {
                if (data == 2 || data == 1)           // part of op 1 OUTSIDE op 0
                    return CorkMesh::BoolProblem::DELETE_TRI;                           // ɾ�����Ϊ2, 1������Ƭ
                else if (data == 3)                   // part of op 1 INSIDE op 1
                    return CorkMesh::BoolProblem::FLIP_TRI;                                // ��ת���Ϊ3������Ƭ��
                else                                    // part of op 0 OUTSIDE op 1
                    return CorkMesh::BoolProblem::KEEP_TRI;                               // �������Ϊ0������Ƭ��
            });

        tt.endCout("�ܺ�ʱ��");

        debugWriteMesh("mesh_diff", *bprob.mesh);
        debugWriteMesh("box0", boxMesh0);
        debugWriteMesh("box1", boxMesh1);
        debugWriteMesh("boxIsct", boxMeshIsct);
        debugWriteMesh("boxIsct_�Ŵ�1.01��", boxMeshIsct101);
        std::cout << "successed!!!" << std::endl;
    }
}


int main(int argc, char* argv[])
{
    //// ԭʼ�Ŀ���̨����

    //return runCmd(argc, argv);

    // TEST_BOOLEAN::test0();

    // TEST_BOOLEAN::test1();

    // TEST_BOOLEAN::test2();

    // TEST_ISCT::test0();

    TEST_BASIC::test6();
    
    //TEST_BVH::test3();                              // ��ΰ�Χ�У�
 
    std::cout << "int main() finished." << std::endl;
    getchar();
    return 1;
}