// 笔记
/*
    *   输入网格文件只能是OFF文件；


    README中的一些信息：

    其他来源的一些信息：
    https://stephanfr.blog/2016/03/21/cork-a-high-performance-library-for-geometric-booleancsg-operations/
            Gilbert Bernstein is currently a Ph.D. student at Stanford and had published some remarkable papers on computational geometry. 
    I was first drawn to his work by his 2009 paper on Fast, Exact, Linear Booleans as my interest in 3D printing led me to create some tooling 
    of my own.  The various libraries I found online for performing Constructive Solid Geometry (CSG) operations were certainly good but overall, 
    very slow.  CGAL is one library I had worked with and I found that the time required for operations on even moderately complex meshes was 
    quite long.  CGAL’s numeric precision and stability is impeccable, the 3D CSG operations in CGAL are based on 3D Nef Polyhedra but I found 
    myself waiting quite a while for results.
            I exchanged a couple emails with Gilbert and he pointed me to a new library he had published, Cork.  One challenge with the models he 
    used in his Fast, Exact paper is that the internal representation of 3D meshes was not all that compatible with other toolsets.  Though the boolean 
    operations were fast, using those algorithms imposed a conversion overhead and eliminates the ability to take other algorithms developed on 
    standard 3D mesh representations and use them directly on the internal data structures.  Cork is fast but uses a ‘standard’ internal representation 
    of 3D triangulated meshes, a win-win proposition.
            I’ve always been one to tinker with code, so I forked Gilbert’s code to play with it.  I spent a fair amount of time working with the code and 
     I don’t believe I found any defects but I did find a few ways to tune it and bump up the performance.  I also took a swag at parallelizing sections 
     of the code to further reduce wall clock time required for operation, though with limited success.  I believe the main problem I ran into is related to 
     cache invalidation within the x86 CPU.  I managed to split several of the most computationally intensive sections into multiple thread of execution – but
     the performance almost always dropped as a result.  I am not completely finished working on threading the library, I may write a post later on 
     what I believe I have seen and how to approach parallelizing algorithms like Cork on current generation CPUs.
            ......
     *      cork库相比于<Fast, Exact, Linear Booleans>论文中的算法，使用了更标准化、广泛采用的网格数据结构；

*/
 
#include "mesh.h"
#include <iostream>
#include <sstream>
#include <list>
#include <unordered_set>
#include <set>

///////////////////////////////////////////////////////////////////////////////////////////////////// 测试函数

// 布尔运算的直接测试
namespace TEST_BOOLEAN
{
    // 布尔运算的直接测试；
    void test0()
    {
        tiktok& tt = tiktok::getInstance();

        // 1. 读取网格：
        CorkTriMesh ioMesh0ori, ioMesh1ori, ioMesh2ori;
        loadMesh(g_debugPath + std::string{ "s9block测试数据/s9block_咬得深.obj" }, &ioMesh0ori);
        loadMesh(g_debugPath + std::string{ "s9block测试数据/fatTeeth1_咬得深.obj" }, &ioMesh1ori);
        loadMesh(g_debugPath + std::string{ "s9block测试数据/fatTeeth2_咬得深.obj" }, &ioMesh2ori);
 
        // 3. 输出结果写到本地：
        CorkTriMesh meshOut0, meshOut1;
        tt.start();
        computeDifference(ioMesh0ori, ioMesh1ori, &meshOut0);
        computeDifference(meshOut0, ioMesh2ori, &meshOut1);
        tt.endCout("总耗时：");
        debugWriteMesh("mesh_diff", meshOut1);

        std::cout << "finished." << std::endl;
        getchar();
    }


    // 布尔运算的分步骤测试
    void test1()
    {
        // 知识点： 1. 层次包围盒，三角片求交中使用；2. 射线求交&缠绕数，三角片在网格内外的判断中使用；
        tiktok& tt = tiktok::getInstance();

        // 0. 读取网格，检测输入网格是否合法，生成计算网格对象，生成布尔操作类对象；
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "cmesh1.off" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "cylinder2.off" }, &ioMesh1);
        //loadMesh(g_debugPath + std::string{ "s9block测试数据/s9block_咬得浅.obj" }, &ioMesh0);
        //loadMesh(g_debugPath + std::string{ "s9block测试数据/fatTeeth1_咬得浅.obj" }, &ioMesh1);

        // isSolid()接口貌似有问题，把cylinder2.obj网格判定为非solid的；

        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        CorkMesh::BoolProblem bprob(&mesh0);

        //              for debug
        std::vector<unsigned> BADcopy;              // 三角片标记的拷贝；mesh->tris[i].data.bool_alg_data
        std::vector<CorkVertex> debugVers;          // 用于debug的点云
        std::vector<std::pair<unsigned, unsigned>> debugEdges;      // 用于debug的边数据； 

        unsigned timeAll = 0;
        tt.start();
        // 1. 三角片标记——mesh0的bool_alg_data标记为0，mesh1的标记为1；
        for (auto& tri : bprob.mesh->tris)
            tri.data.bool_alg_data = static_cast<byte>(0);

        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);

#ifdef TEST_CROSS_BOX_ACCE
        // 计算两个网格各自包围盒的相交包围盒：
        BBox3d box0 = getAABB(mesh0);
        BBox3d box1 = getAABB(mesh1);
        BBox3d boxIsct = isct(box0, box1);
        BBox3d boxIsct101 = multiply(boxIsct, 1.01);
#endif

        // 2. 直接融合两个网格
        bprob.mesh->disjointUnion(mesh1);                          // 两个网格直接融合
        tt.endCout("step1& step2耗时：");
        timeAll += tt.endTik - tt.startTik;

        //              for debug;
        debugWriteMesh("step2直接融合后的网格", *bprob.mesh);
        getLabelInfo(bprob, BADcopy, debugVers);
        debugWriteVers("step2标记为0的三角片顶点", debugVers);

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


        // 3. 计算三角片交线，进行三角剖分；会有插点；
        tt.start();
        bprob.mesh->resolveIntersections();
        tt.endCout("step3耗时：");
        timeAll += tt.endTik - tt.startTik;

        //              for debug;
        debugWriteMesh("step3计算交线、三角剖分之后", *bprob.mesh);
        getLabelInfo(bprob, BADcopy, debugVers);
        debugWriteVers("step3标记为0的三角片顶点", debugVers);


        // 4. 计算ecache成员数据——确定融合网格的所有边，并且标记出哪些边 是三角片相交边；
        tt.start();
        bprob.populateECache();
        tt.endCout("step4耗时：");
        timeAll += tt.endTik - tt.startTik;

        //          for debug
        debugWriteMesh("step4", *bprob.mesh);
        getLabelInfo(bprob, BADcopy, debugVers);

        // 第5，6步貌似不需要；
#if 1
    // 5.？？？生成UnionFind对象uf，不知道是干啥的；
        tt.start();
        UnionFind uf(bprob.mesh->tris.size());
        bprob.for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
            {
                uint tid0 = tids[0];
                for (uint k = 1; k < tids.size(); k++)
                    uf.unionIds(tid0, tids[k]);
            });
        tt.endCout("step5耗时：");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        debugWriteMesh("step5", *bprob.mesh);


        // 6. ？？？
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
        tt.endCout("step6耗时：");
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
        debugWriteVers("step6——components[0]中所有三角片的顶点", debugVers);
        getLabelInfo(bprob, BADcopy, debugVers);
        debugWriteVers("step6——标记为0的三角片顶点", debugVers);
#else

    // 被标记为0和1的三角片分别放到两个数组中：
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

        // 7. 使用射线来判断三角片在内部还是外部，处理三角片，修改三角片标记；
        tt.start();
        std::vector<bool> visited(bprob.mesh->tris.size(), false);              // 记录三角片是否已经经过处理；
        for (unsigned i = 0; i < components.size(); ++i)
        {
            auto& comp = components[i];

            // 7.1 找出当前三角片组中面积最大的三角片；
            uint maxTriIdx = comp[0];                       // 面积最大的三角片索引；
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

            // 7.2 判断当前最大三角片是否在融合网格内部，修改该最大三角片的标记；
            byte currentMeshLabel = bprob.boolData(maxTriIdx);
            bool inside = bprob.isInside(maxTriIdx, currentMeshLabel);
            byte& maxTriLabel = bprob.boolData(maxTriIdx);
            maxTriLabel |= (inside) ? 2 : 0;            // 与0或10B做按位或运算，当前网格最大三角片，若在融合网格内部，则将其label前一位改为1； 该三角片标记处理完毕；

            // 7.3 从最大三角片开始，用相邻三角片生长的方式遍历本组所有三角片，根据不同情形修改三角片标记；
            std::queue<uint> work;                    // 处理三角片的工作队列；        
            visited[maxTriIdx] = true;
            work.push(maxTriIdx);

            while (!work.empty())
            {
                uint curr_tid = work.front();
                byte& currentTriLabel = bprob.boolData(curr_tid);           // 队首三角片的标记；
                work.pop();

                for (uint k = 0; k < 3; k++)        // 遍历当前三角片的三条边，进而遍历当前三角片的所有相邻三角片；
                {
                    uint a = bprob.mesh->tris[curr_tid].v[k];
                    uint b = bprob.mesh->tris[curr_tid].v[(k + 1) % 3];
                    auto& entry = bprob.ecache(a, b);                      // （a, b）这条边的信息；

                    // 三角片标记和10B做与运算
                    byte inside_sig = currentTriLabel & 2;          // 按位与运算用于提取某一位的信息，这里和10B做按位与运算，提取了currentTriLabel的前一位，表示是否在网格内；
                    if (entry.data.is_isct)
                        inside_sig ^= 2;                     // 若该边是三角片交线，则currentTri和nbrTri的内外状态一定是相异的；
                                                                         // 三角片生长的过程中，如果没有碰到三角片交线，则三角片的内外状态是一直不变的；
                                                                          // 和10B做异或运算，表示将内外状态取反；

                    for (uint nbrTriIdx : entry.tids)   // 遍历该条边关联的两个三角片，即遍历当前三角片的所有相邻三角片；
                    {
                        byte& nbrTriLabel = bprob.boolData(nbrTriIdx);
                        if (visited[nbrTriIdx])
                            continue;

                        if ((nbrTriLabel & 1) != currentMeshLabel)         // 若相邻三角片属于另一个网格，则跳过，此循环只处理当前网格中的三角片；
                            continue;

                        nbrTriLabel |= inside_sig;          // 
                        visited[nbrTriIdx] = true;
                        work.push(nbrTriIdx);
                    }
                }
            }

            currentMeshLabel++;
        }
        tt.endCout("step7耗时：");
        timeAll += tt.endTik - tt.startTik;

        // 四种三角片标记：后一位表示该三角片原先属于哪个网格，前一位表示该三角片是否在融合网格的外部；
        /*
            0（00B）——0网格的三角片，除去1网格（融合网格）内部的那些；
            1（01B）——1网格的三角片，除去0网格（融合网格）内部的那些；
            2（10B）——0网格三角片中，在融合网格内部的那些三角片；
            3（11B）——1网格三角片中，在融合网格内部的那些三角片；
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
            debugWriteMesh("step7三角片0对应的网格", selectedMesh0);
            debugWriteMesh("step7三角片1对应的网格", selectedMesh1);
            debugWriteMesh("step7三角片2对应的网格", selectedMesh2);
            debugWriteMesh("step7三角片3对应的网格", selectedMesh3);
        }

        // 8.布尔减法操作：
        tt.start();
        bprob.doDeleteAndFlip([](byte data) -> typename CorkMesh::BoolProblem::TriCode
            {
                if (data == 2 || data == 1)           // part of op 1 OUTSIDE op 0
                    return CorkMesh::BoolProblem::DELETE_TRI;                           // 删除标记为2, 1的三角片
                else if (data == 3)                   // part of op 1 INSIDE op 1
                    return CorkMesh::BoolProblem::FLIP_TRI;                                // 翻转标记为3的三角片；
                else                                    // part of op 0 OUTSIDE op 1
                    return CorkMesh::BoolProblem::KEEP_TRI;                               // 保留标记为0的三角片；
            });
        tt.endCout("step8耗时：");
        timeAll += tt.endTik - tt.startTik;

        debugWriteMesh("meshDiff_未被meshFix处理的结果", *bprob.mesh);



        std::cout << "总共耗时：" << timeAll << std::endl;           // 第三步最耗时；直接测试的话耗时6200+ms;
        std::cout << "finished." << std::endl;
        getchar();
    }


    // 测试ISOLATE_TRIS_IN_BOX
    void test11()
    {
        // 知识点： 1. 层次包围盒，三角片求交中使用；2. 射线求交&缠绕数，三角片在网格内外的判断中使用；
        tiktok& tt = tiktok::getInstance();

        // 0. 读取网格，检测输入网格是否合法，生成计算网格对象，生成布尔操作类对象；
        CorkTriMesh ioMesh1, ioMesh0;
        //loadMesh(g_debugPath + std::string{ "cmesh1.off" }, &ioMesh0);
        //loadMesh(g_debugPath + std::string{ "cylinder2.off" }, &ioMesh1);
        loadMesh(g_debugPath + std::string{ "s9block测试数据/s9block_preprocessed.obj" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "s9block测试数据/fatTeeth1_preprocessed.obj" }, &ioMesh1);

        // isSolid()接口貌似有问题，把cylinder2.obj网格判定为非solid的；

        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        CorkMesh::BoolProblem bprob(&mesh0);

        unsigned timeAll = 0;
        tt.start();
        // 1. 三角片标记——mesh0的bool_alg_data标记为0，mesh1的标记为1；
        for (auto& tri : bprob.mesh->tris)
            tri.data.bool_alg_data = static_cast<byte>(0);

        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);

        // 2. 直接融合两个网格
        bprob.mesh->disjointUnion(mesh1);                          // 两个网格直接融合
        tt.endCout("step1& step2耗时：");
        timeAll += tt.endTik - tt.startTik;
        debugWriteMesh("step2直接融合的网格", *bprob.mesh);

        // 计算两个网格各自包围盒的相交包围盒：
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

        // 若三角片中至少有一个顶点在包围盒内，则取该三角片：
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

        // 修改新网格三角片:
        unsigned newMeshVersCountOri = newMesh.verts.size();
        unsigned newMeshTrisCountOri = newMesh.tris.size();
        for (auto& tri : newMesh.tris)
        {
            tri.a = static_cast<uint>(oldNewIdxTable[tri.a]);
            tri.b = static_cast<uint>(oldNewIdxTable[tri.b]);
            tri.c = static_cast<uint>(oldNewIdxTable[tri.c]);
        }


        // 对新网格进行三角剖分：
        CorkMesh::BoolProblem bprobNew(&newMesh);
        CorkMesh::IsctProblem iproblem(&newMesh);
        iproblem.findIntersections();                       // 寻找三角片交线；

        std::vector<uint> tagTrisIdx;                     // 待三角剖分的三角片的索引；
        iproblem.tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
            {
                if (tpPtr->iverts.size() > 0)
                    tagTrisIdx.push_back(tpPtr->the_tri->ref);
            });

 
#ifdef MY_DEBUG
        debugWriteMesh("step3提取的盒内网格", newMesh);
#endif

        iproblem.resolveAllIntersections();
        iproblem.commit();

#ifdef MY_DEBUG
        debugWriteMesh("step3待三角剖分的三角片", combainer);
        debugWriteMesh("step3新网格计算交线、三角剖分之后", *bprobNew.mesh);

 
        CorkMesh tempMesh;
        tempMesh.verts = bprobNew.mesh->verts;
        tempMesh.tris.insert(tempMesh.tris.end(), bprobNew.mesh->tris.begin(), bprobNew.mesh->tris.begin() + (newMeshTrisCountOri - tagTrisIdx.size()) + 1);
        debugWriteMesh("step3删除待剖分的三角片之后的网格", tempMesh);
#endif

        unsigned tagTrisCount = tagTrisIdx.size();              // 被剖分的三角片数量；
        newMeshTrisCountOri -= tagTrisCount;
        unsigned insertVersCount = newMesh.verts.size() - newMeshVersCountOri;
        unsigned insertTrisCount = newMesh.tris.size() - newMeshTrisCountOri;
        std::vector<CorkVertex> insertVers;
        insertVers.insert(insertVers.end(), newMesh.verts.begin() + newMeshVersCountOri, newMesh.verts.end());
        std::vector<CorkMesh::Tri> insertTris;
        insertTris.insert(insertTris.end(), newMesh.tris.begin() + newMeshTrisCountOri, newMesh.tris.end());

#ifdef MY_DEBUG
        debugWriteVers("step3新增的顶点", insertVers);
        tempMesh.clear();
        tempMesh.verts = newMesh.verts;
        tempMesh.tris.insert(tempMesh.tris.end(), newMesh.tris.begin() + newMeshTrisCountOri, newMesh.tris.end());
        debugWriteMesh("step3新增的三角片", tempMesh);
#endif

        // 修改新增的三角片数据：
        auto handleIndex = [&](const uint nmIdx)->uint
        {
            // 原先的顶点，索引映射关系查newOldIdxTable; 新增的顶点，索引增加一个偏移量：(versCount - newMeshVersCountOri)
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

        // 原融合网格插入新点 
        bprob.mesh->verts.insert(bprob.mesh->verts.end(), insertVers.begin(), insertVers.end());

        // 原融合网格删除需要三角剖分的三角片，插入新三角片：
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
        debugWriteMesh("step3原融合网格插入新点、新三角片之后", *bprob.mesh);
        std::vector<unsigned> BADcopy;              // 三角片标记的拷贝；mesh->tris[i].data.bool_alg_data
        std::vector<CorkVertex> debugVers;
        getLabelInfo(bprob, BADcopy, debugVers);
#endif

        tt.endCout("ISOLATE_TRIS_IN_BOX三角剖分耗时：");
#endif

        
#ifdef ISOLATE_TRIS_IN_BOX
        



#else
        // 3. 计算三角片交线，进行三角剖分；会有插点；
        unsigned versCount = bprob.mesh->verts.size();
        unsigned trisCount = bprob.mesh->tris.size();
        tt.start();
        bprob.mesh->resolveIntersections();
        tt.endCout("step3耗时：");
        timeAll += tt.endTik - tt.startTik;
        //              for debug;
        debugWriteMesh("step3计算交线、三角剖分之后", *bprob.mesh);

        unsigned insertVersCount = bprob.mesh->verts.size() - versCount;
        unsigned insertTrisCount = bprob.mesh->tris.size() - trisCount;
        CorkMesh tempMesh;
        tempMesh.verts = bprob.mesh->verts;
        tempMesh.tris.insert(tempMesh.tris.end(), bprob.mesh->tris.begin() + trisCount, bprob.mesh->tris.end());
        debugWriteMesh("step3新增的三角片", tempMesh);
        std::vector<CorkVertex> newVers;
        newVers.insert(newVers.end(), bprob.mesh->verts.begin() + versCount, bprob.mesh->verts.end());
        debugWriteVers("step3新增的顶点", newVers);
#endif

        // 4. 计算ecache成员数据——确定融合网格的所有边，并且标记出哪些边 是三角片相交边；
        tt.start();
        bprob.populateECache();
        tt.endCout("step4耗时：");
        timeAll += tt.endTik - tt.startTik;


        // 第5，6步貌似不需要；
#if 1
    // 5.？？？生成UnionFind对象uf，不知道是干啥的；
        tt.start();
        UnionFind uf(bprob.mesh->tris.size());
        bprob.for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
            {
                uint tid0 = tids[0];
                for (uint k = 1; k < tids.size(); k++)
                    uf.unionIds(tid0, tids[k]);
            });
        tt.endCout("step5耗时：");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        debugWriteMesh("step5", *bprob.mesh);


        // 6. ？？？
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
        tt.endCout("step6耗时：");
        timeAll += tt.endTik - tt.startTik;

 
#else

    // 被标记为0和1的三角片分别放到两个数组中：
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

        // 7. 使用射线来判断三角片在内部还是外部，处理三角片，修改三角片标记；
        tt.start();
        std::vector<bool> visited(bprob.mesh->tris.size(), false);              // 记录三角片是否已经经过处理；
        for (unsigned i = 0; i < components.size(); ++i)
        {
            auto& comp = components[i];

            // 7.1 找出当前三角片组中面积最大的三角片；
            uint maxTriIdx = comp[0];                       // 面积最大的三角片索引；
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

            // 7.2 判断当前最大三角片是否在融合网格内部，修改该最大三角片的标记；
            byte currentMeshLabel = bprob.boolData(maxTriIdx);
            bool inside = bprob.isInside(maxTriIdx, currentMeshLabel);
            byte& maxTriLabel = bprob.boolData(maxTriIdx);
            maxTriLabel |= (inside) ? 2 : 0;            // 与0或10B做按位或运算，当前网格最大三角片，若在融合网格内部，则将其label前一位改为1； 该三角片标记处理完毕；

            // 7.3 从最大三角片开始，用相邻三角片生长的方式遍历本组所有三角片，根据不同情形修改三角片标记；
            std::queue<uint> work;                    // 处理三角片的工作队列；        
            visited[maxTriIdx] = true;
            work.push(maxTriIdx);

            while (!work.empty())
            {
                uint curr_tid = work.front();
                byte& currentTriLabel = bprob.boolData(curr_tid);           // 队首三角片的标记；
                work.pop();

                for (uint k = 0; k < 3; k++)        // 遍历当前三角片的三条边，进而遍历当前三角片的所有相邻三角片；
                {
                    uint a = bprob.mesh->tris[curr_tid].v[k];
                    uint b = bprob.mesh->tris[curr_tid].v[(k + 1) % 3];
                    auto& entry = bprob.ecache(a, b);                      // （a, b）这条边的信息；

                    // 三角片标记和10B做与运算
                    byte inside_sig = currentTriLabel & 2;          // 按位与运算用于提取某一位的信息，这里和10B做按位与运算，提取了currentTriLabel的前一位，表示是否在网格内；
                    if (entry.data.is_isct)
                        inside_sig ^= 2;                     // 若该边是三角片交线，则currentTri和nbrTri的内外状态一定是相异的；
                                                                         // 三角片生长的过程中，如果没有碰到三角片交线，则三角片的内外状态是一直不变的；
                                                                          // 和10B做异或运算，表示将内外状态取反；

                    for (uint nbrTriIdx : entry.tids)   // 遍历该条边关联的两个三角片，即遍历当前三角片的所有相邻三角片；
                    {
                        byte& nbrTriLabel = bprob.boolData(nbrTriIdx);
                        if (visited[nbrTriIdx])
                            continue;

                        if ((nbrTriLabel & 1) != currentMeshLabel)         // 若相邻三角片属于另一个网格，则跳过，此循环只处理当前网格中的三角片；
                            continue;

                        nbrTriLabel |= inside_sig;          // 
                        visited[nbrTriIdx] = true;
                        work.push(nbrTriIdx);
                    }
                }
            }

            currentMeshLabel++;
        }
        tt.endCout("step7耗时：");
        timeAll += tt.endTik - tt.startTik;

        // 四种三角片标记：后一位表示该三角片原先属于哪个网格，前一位表示该三角片是否在融合网格的外部；
        /*
            0（00B）——0网格的三角片，除去1网格（融合网格）内部的那些；
            1（01B）——1网格的三角片，除去0网格（融合网格）内部的那些；
            2（10B）——0网格三角片中，在融合网格内部的那些三角片；
            3（11B）——1网格三角片中，在融合网格内部的那些三角片；
        */

        // 8.布尔减法操作：
        tt.start();
        bprob.doDeleteAndFlip([](byte data) -> typename CorkMesh::BoolProblem::TriCode
            {
                if (data == 2 || data == 1)           // part of op 1 OUTSIDE op 0
                    return CorkMesh::BoolProblem::DELETE_TRI;                           // 删除标记为2, 1的三角片
                else if (data == 3)                   // part of op 1 INSIDE op 1
                    return CorkMesh::BoolProblem::FLIP_TRI;                                // 翻转标记为3的三角片；
                else                                    // part of op 0 OUTSIDE op 1
                    return CorkMesh::BoolProblem::KEEP_TRI;                               // 保留标记为0的三角片；
            });
        tt.endCout("step8耗时：");
        timeAll += tt.endTik - tt.startTik;

        debugWriteMesh("meshDiff_未被meshFix处理的结果", *bprob.mesh);

        std::cout << "总共耗时：" << timeAll << std::endl;           // 第三步最耗时；直接测试的话耗时6200+ms;
        std::cout << "finished." << std::endl;
        getchar();
    }
}




// 测试射线求交；
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
            std::cout << "在网格内" << std::endl;
        else
            std::cout << "在网格外" << std::endl;

        vc.pos = Vec3d{ 10, 10, 10 };
        debugWriteMesh("testTriangle2", testTriangle);
        if (myIsInside(va, vb, vc, mesh0))
            std::cout << "在网格内" << std::endl;
        else
            std::cout << "在网格外" << std::endl;

        getchar();
    }
}



// 测试项目自定义的基础类型：
namespace TEST_BASIC
{
    // 测试迭代内存池iterPool
    void test0()
    {
        std::vector<int> intVec{ -1, -2, -3, -4, -5, -6, -7, -8 };
        IterPool<int> intPool1, intPool2;

        // alloc()——池中分配出一个新元素的空间；
        for (unsigned i = 1; i <= 9; ++i)
        {
            int* elemPtr = intPool1.alloc();
            *elemPtr = i;
        }

        // for_each()——遍历池中的元素：
        unsigned elemCount = intPool1.size();
        std::cout << "disp all elements in intPool1: " << std::endl;
        intPool1.for_each([&](int* intPtr)
            {
                std::cout << *intPtr << ", ";
            });
        std::cout << std::endl;

        std::cout << "finished." << std::endl;
    }

    // 测试AABB类bbox3d
    void test1()
    {
        CorkMesh::Tri tri1(0, 1, 2);
        std::vector<CorkVertex> vers1{ CorkVertex(-3, 0, 8), CorkVertex(-3, 3, 4), CorkVertex(0, -3, 4) };
        CorkMesh triMesh1;
        triMesh1.verts = vers1;
        triMesh1.tris.push_back(tri1);
        debugWriteMesh("triMesh1", triMesh1);

        // 打印第一个三角片的包围盒；
        CorkMesh::TopoCache tCache(&triMesh1);
        TopoTri tt0 = *tCache.tris.begin();
        BBox3d tbox0 = getAABB(&tt0, triMesh1);

        CorkMesh boxMesh0;
        getAABBmesh(boxMesh0, tbox0);
        debugWriteMesh("boxMesh", boxMesh0);
    }


    // 测试可迭代内存池iterPool
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


        // 保存的指针遍历：
        std::cout << "保存的指针遍历：" << std::endl;
        {
            for (unsigned i = 0; i < 10; ++i)
                std::cout << *numPtrs[i] << ", ";
            std::cout << std::endl;

            auto rIter = numPtrs.rbegin();
            for (unsigned i = 0; i < 10; ++i, ++rIter)
                std::cout << **rIter << ", ";
            std::cout << std::endl; 
        }

        // 池中删除奇数：
        numPool.for_each([&](int* ip) 
            {
                if (0 != *ip % 2)
                    numPool.free(ip);
            });
          
        // 池自身的迭代：
        std::cout << "池自己的迭代：" << std::endl;
        auto iter = numPool.begin(); 
        for (unsigned i = 0; i < 10; ++i, ++iter)
            std::cout << *iter << ", ";        
        std::cout << std::endl;

        // 保存的指针遍历：
        std::cout << "保存的指针遍历：" << std::endl;
        {
            for (unsigned i = 0; i < 10; ++i)
                std::cout << *numPtrs[i] << ", ";                               // 池中元素已被释放，变成野指针，不是空指针；
            std::cout << std::endl;

            auto rIter = numPtrs.rbegin();
            for (unsigned i = 0; i < 10; ++i, ++rIter)
                std::cout << **rIter << ", ";
            std::cout << std::endl;
        }

        std::cout << "保存的指针遍历：" << std::endl;
        {
            for (unsigned i = 0; i < 10; ++i)
                std::cout << (nullptr == numPtrs[i] ? "nullPtr": "concrete") << ", ";      // 池中元素已被释放，变成野指针；
            std::cout << std::endl;

            auto rIter = numPtrs.rbegin();
            for (unsigned i = 0; i < 10; ++i, ++rIter)
                std::cout << (nullptr == *rIter ? "nullPtr" : "concrete") << ", ";
            std::cout << std::endl;
        }
    }


    // 利用了内存池的动态数组ShortVec
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
                intSvec.erase(num);             // erase之后numPtr貌似是野指针。
                numPtr = &num;
            }
        }

        int ret = std::distance(intSvec.begin(), numPtr);       // 12
        int temp = *numPtr;     // 2
        intSvec.disp();
        std::cout << ret << std::endl;
    }


    // 测试网格拓扑信息结构体TopoCache
    void test4()
    {
        CorkTriMesh  ioMesh0;
        loadMesh(g_debugPath + std::string{ "cube.off" }, &ioMesh0);
        CorkMesh mesh0;
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        // 1. 生成网格拓扑信息对象：TopoCache，EGraphCache
        CorkMesh::TopoCache tCache(&mesh0);                             // 生成mesh0的TopoCache表示的拓扑信息，构造函数生成；
 
        // 2. gCache中的边信息改用自定义的更简洁的edgeInfo表示；
        std::vector<edgeInfo> edgeInfoVecG;
        unsigned edgesCount1 = tCache.edges.size();

         // EGraphCache中记录的边一律是顶点索引中小的在前大的在后；
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

        // 拓扑三角片：
        std::vector<std::tuple<uint, uint, uint>> trisInfoVec;
        trisInfoVec.reserve(mesh0.tris.size());
        for (const auto& tpTri: tCache.tris) 
        {
            auto tuple = std::make_tuple(tpTri.verts[0]->ref, tpTri.verts[1]->ref, tpTri.verts[2]->ref);
            trisInfoVec.push_back(tuple);
        }

        std::cout << "finished." << std::endl;
    }
 

    // 测试EGraphCache边信息；
    void test5() 
    {
        CorkTriMesh  ioMesh0;
        loadMesh(g_debugPath + std::string{ "cube.off" }, &ioMesh0);
        CorkMesh mesh0;
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        debugWriteMesh("mesh0", mesh0);

        //      EGraphCache()
        CorkMesh::EGraphCache<CorkMesh::BoolProblem::BoolEdata> gCache = \
            mesh0.createEGraphCache<CorkMesh::BoolProblem::BoolEdata>();    // 生成mesh0的EGraphCache表示的拓扑信息

        // 2. gCache中的边信息改用自定义的更简洁的edgeInfo表示；
        std::vector<edgeInfo> edgeInfoVecG;
        copyEdgesInfo(mesh0, gCache, edgeInfoVecG);
        unsigned edgesCount2 = edgeInfoVecG.size();              // EGraphCache中记录的边数量是TopoCache的两倍，因为后者把(1,2)(2,1)记录成一条边；

        std::cout << "finished." << std::endl;
    }


    // 测试BVH
    void test6()
    {
        // 知识点： 1. 层次包围盒，三角片求交中使用；2. 射线求交&缠绕数，三角片在网格内外的判断中使用；
        tiktok& tt = tiktok::getInstance();

        // 0. 读取网格，检测输入网格是否合法，生成计算网格对象，生成布尔操作类对象；
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "cmesh1.off" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "cylinder2.off" }, &ioMesh1);

        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        CorkMesh::BoolProblem bprob(&mesh0);
 
        unsigned timeAll = 0;
        tt.start();
        // 1. 三角片标记——mesh0的bool_alg_data标记为0，mesh1的标记为1；
        for (auto& tri : bprob.mesh->tris)
            tri.data.bool_alg_data = static_cast<byte>(0);

        for (auto& tri : mesh1.tris)
            tri.data.bool_alg_data = static_cast<byte>(1);

        // 2. 直接融合两个网格
        bprob.mesh->disjointUnion(mesh1);                          // 两个网格直接融合
        tt.endCout("step1& step2耗时：");
        timeAll += tt.endTik - tt.startTik;
        debugWriteMesh("step2直接融合后的网格", *bprob.mesh);
        CorkMesh& mesh = *bprob.mesh;
        debugWriteTriMesh("isctTri", mesh, 7779);           // 被穿刺的三角片；
        debugWriteEdge("isctEdge", CorkEdge(mesh.verts[633], mesh.verts[636]));     // 穿刺7779三角片的边；

        // 3. 获得当前网格所有边的层次包围盒：
        Mesh<CorkVertex, CorkTriangle>::IsctProblem isctObj(&mesh);
        AABVH<TopoEdge*>& bvh = isctObj.bvhObj;
        isctObj.edge_geoms.clear();

        //      3.1. 生成所有边的GeomBlob信息；
       isctObj.edges.for_each(\
            [&](TopoEdge* ePtr)
            {
                isctObj.edge_geoms.push_back(isctObj.edge_blob(ePtr));
            });

        //      3.2. 生成所有边的AABVH对象； 
        bvh.build(isctObj.edge_geoms);                     
 

        // 4. 搜索包含isctEdge的bvh节点：
        AABVHNode<TopoEdge*>* nodePtr = bvh.searchLeaf(\
            [&](AABVHNode<TopoEdge*>* nodePtr)->bool
            {
                // 遍历当前叶子节点中的所有边：
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

        // 5. 打印该节点的包围盒网格：
        CorkMesh boxMesh;
        getAABBmesh(boxMesh, nodePtr->bbox);
        debugWriteMesh("boxMesh_node", boxMesh);

        // 6. 求该节点的所有父节点，当前有错误；
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
 

// 测试层次包围盒
namespace TEST_BVH
{
    //  测试step3
    void test0()
    {
        tiktok& tt = tiktok::getInstance();
        unsigned timeAll = 0;

        // step3之前的步骤：
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
        debugWriteMesh("step2——直接融合的两个网格", mesh0);

        // 3. step3寻找三角片交线段，执行三角剖分；
        tt.start();
        Mesh<CorkVertex, CorkTriangle>::IsctProblem isctObj(&mesh0);

        //      一些用于debug监视的引用：
        std::vector< GeomBlob<TopoEdge*> >& edge_geoms = isctObj.edge_geoms;        // 用于生成AABVH的所有边数据；
        AABVH<TopoEdge*>& bvhObj = isctObj.bvhObj;                                                       // isctObj中层次包围盒对象的引用；
        IterPool<Mesh<CorkVertex, CorkTriangle>::TriangleProblem>& tprobs = isctObj.tprobs;     // 存储了三角片交点信息；
        const auto& meshVers = mesh0.verts;
        const auto& meshTris = mesh0.tris;

        //      3.1  寻找三角片交线；
        isctObj.findIntersections();
        tt.endCout("step3.1耗时：");                       // 不去除冗余步骤的话耗时5078ms;
        timeAll += tt.endTik - tt.startTik;

        // for debug
        {
            // 找出当前所有相交的三角片，打印相应的网格；
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
            debugWriteMesh("相交的三角片", isctTrisMesh);

            // 打印第一个相交三角片，及其相关的信息：
            auto iter = tprobs.begin();
            Mesh<CorkVertex, CorkTriangle>::TriangleProblem& tprob0 = *iter;        // 第一个triangleProblem对象；
            CorkMesh triMesh0;
            const auto& tri0 = mesh0.tris[tprob0.the_tri->ref];
            triMesh0.verts.push_back(mesh0.verts[tri0.a]);
            triMesh0.verts.push_back(mesh0.verts[tri0.b]);
            triMesh0.verts.push_back(mesh0.verts[tri0.c]);
            triMesh0.tris.resize(1);
            triMesh0.tris[0].a = 0;
            triMesh0.tris[0].b = 1;
            triMesh0.tris[0].c = 2;
            debugWriteMesh("第一个相交三角片", triMesh0);

            unsigned isctEdgeCount0 = tprob0.iedges.size();         // 和triMesh0相交的边数；
            IsctEdgeType isctEdge0 = *tprob0.iedges[0];
            GenericVertType gVer00, gVer01;
            gVer00 = *isctEdge0.ends[0];
            gVer01 = *isctEdge0.ends[1];
            CorkVertex ver00, ver01;
            ver00.pos = gVer00.coord;
            ver01.pos = gVer01.coord;
            std::pair<unsigned, unsigned> tmpPair = std::make_pair(0, 1);
            debugWriteEdges("和第一个相交三角片相交的边", std::vector<CorkVertex>{ver00, ver01}, \
                std::vector<std::pair<unsigned, unsigned>>{tmpPair});
        }

        //      3.2 
        tt.start();
        isctObj.resolveAllIntersections();              // 点数三角片数都有增加
        tt.endCout("step3.2耗时：");
        timeAll += tt.endTik - tt.startTik;

        // for debug;
        debugWriteMesh("step3.2——resolveAllIntersections", mesh0);

        //      3.3
        tt.start();
        isctObj.commit();       // 三角片数有减少——？？？去除重复三角片？？？；
        tt.endCout("step3.3耗时：");
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

        debugWriteMesh("step3——三角剖分后的融合网格", mesh0);

        std::cout << "step3总耗时：" << timeAll << std::endl;
        std::cout << "finished." << std::endl;
    }


    // 测试用单个三角片和网格交叉
    void test1()
    {
        CorkTriMesh ioMesh0;
        loadMesh(g_debugPath + std::string{ "cube.off" }, &ioMesh0);
        CorkMesh mesh0;

        corkTriMesh2CorkMesh(ioMesh0, &mesh0);
        for (auto& tri : mesh0.tris)
            tri.data.bool_alg_data = static_cast<byte>(0);
        debugWriteMesh("mesh0", mesh0);

        // 做一个与mesh0相交的三角片tri1：
        CorkMesh::Tri tri1(0, 1, 2);
        std::vector<CorkVertex> vers1{ CorkVertex(-3, 0, 8), CorkVertex(0, 3, 0), CorkVertex(0, -3, 0) };
        CorkMesh triMesh1;
        triMesh1.verts = vers1;
        triMesh1.tris.push_back(tri1);
        debugWriteMesh("triMesh1", triMesh1);

        // 三角片网格和mesh0融合
        mesh0.disjointUnion(triMesh1);

        // 生成融合网格中每一条边的gb对象；
        CorkMesh::TopoCache tCache(&mesh0);
        std::vector<GeomBlob<TopoEdge*>> edgeBlobs;
        edgeBlobs.reserve(tCache.edges.size());
        tCache.edges.for_each([&](TopoEdge* ePtr)
            {
                GeomBlob<TopoEdge*> blob = getEdgeBlob(ePtr, mesh0);
                edgeBlobs.push_back(blob);
            });

        // 生成融合网格所有边数据生成的AABVH层次包围盒对象：
        AABVH<TopoEdge*> aabvh(edgeBlobs);

        // 遍历融合网格所有三角片：
        bool findIsct = false;
        std::vector<AABVHNode<TopoEdge*>*> isctNodes;        // 存储查找到的和某一个三角片相交的所有aabvh叶子节点指针
        TopoTri* isctTriPtr0 = nullptr;                            // 第一个被穿刺的三角片；
        tCache.tris.for_each([&](TopoTri* triPtr)
            {
                if (findIsct)
                    return;         // 若已经找到了一个被穿刺的三角片，则不继续查找；

                BBox3d tbbox = getAABB(triPtr, mesh0);       // 当前三角片的包围盒；

                // 使用当前三角片的包围盒，遍历aabvh，找出和当前包围盒相交的叶子节点；
                aabvh.for_each_in_box(tbbox, [&](AABVHNode<TopoEdge*>* nodePtr)           // 对aabvh的遍历：
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
        debugWriteMesh("第一个被穿刺的三角片", isctTriMesh0);

        BBox3d box0 = getAABB(isctTriPtr0, mesh0);
        CorkMesh boxMesh0;
        getAABBmesh(boxMesh0, box0);
        debugWriteMesh("第一个被穿刺的三角片的包围盒", boxMesh0);

        std::vector<BBox3d> nodeBoxs;           // 被选中的节点的包围盒；
        nodeBoxs.reserve(isctNodes.size());
        std::vector<TopoEdge> isctEdges;        // 被选中的节点中的所有边；
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
        debugWriteEdges("被选中的节点中的所有边", isctEdges, mesh0);

        CorkMesh nodeBoxMesh;
        for (unsigned i = 0; i < isctNodes.size(); ++i)
        {
            getAABBmesh(nodeBoxMesh, nodeBoxs[i]);
            char str0[100];
            sprintf(str0, "被选中节点%d的包围盒", i);
            debugWriteMesh(str0, nodeBoxMesh);
        }

        std::cout << "finished." << std::endl;
    }


    // 测试在手动施加微扰的情况下进行第三步：
    void test3()
    {
        tiktok& tt = tiktok::getInstance();

        // step3之前的步骤：
        CorkTriMesh ioMesh1, ioMesh0;
        loadMesh(g_debugPath + std::string{ "s9block测试数据/input1.obj" }, &ioMesh0);
        loadMesh(g_debugPath + std::string{ "s9block测试数据/input2.obj" }, &ioMesh1);
        CorkMesh mesh1, mesh0;
        corkTriMesh2CorkMesh(ioMesh1, &mesh1);
        corkTriMesh2CorkMesh(ioMesh0, &mesh0);

        tt.start();

        // 计算两个网格各自包围盒的相交包围盒：
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
        debugWriteMesh("融合网格", mesh0);
        debugWriteTriMesh("degTri", mesh0, 229465);
        debugWriteEdge("degEdge", mesh0.verts[10599], mesh0.verts[43353]);

        // 3. step3寻找三角片交线段，执行三角剖分；
        CorkMesh::IsctProblem isctObj(&mesh0);

#ifdef TEST_CROSS_BOX_ACCE
        isctObj.getSelectedTrisIdx(boxIsct101);
#endif

        //      3.1.1 施加微扰，跳过此步；
        int nTrys = 5;
        isctObj.perturbPositions();                 // always perturb for safety...

        //      3.1.2 tryToFindIntersections()寻找三角片和边相交的情形；原项目中尝试5次，这里只尝试一次

        // 1. 生成融合网格所有边的层次包围盒aabvh，依次使用每一个三角片遍历aabvh，寻找边和三角片的交点；
        isctObj.bvh_edge_tri([&](TopoEdge* eisct, TopoTri* tisct)->bool
            {
                // 1.1 检测当前三角片是否被穿刺；
                if (isctObj.checkIsct(eisct, tisct))
                {

#ifdef TEST_CROSS_BOX_ACCE
                    // for test——若当前穿刺边所在的三角片，全都在包围盒之外，则跳过；
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

                    // 1.1.1 创建GluePointMarker对象，在计算三角片和边交点时有用；
                    GluePointMarker* glue = isctObj.newGluePt();
                    glue->edge_tri_type = true;
                    glue->e = eisct;
                    glue->t[0] = tisct;

                    // 1.1.2 当前被穿刺的三角片生成triangleProblem对象，生成需要新加的点；
                    /*
                        当前三角片被穿刺 → 生成该三角片对应的tiangleProblem对象，其内部生成1个新增顶点iv，2条新增边ie0, ie1；
                        iv为该三角片被穿刺处的点，在三角片内部；
                        此时两条新增边的状态完全相同，端点都是未定状态，其中一个端点取为iv，另一个端点的指针是nullptr;
                    */
                    CorkMesh::TriangleProblem* tpPtr = isctObj.getTprob(tisct);
                    IsctVertType* iv = tpPtr->addInteriorEndpoint(&isctObj, eisct, glue);


                    // 1.1.3 生成当前三角片需要新加的边数据；
                    for (TopoTri* spearTri : eisct->tris)        // 遍历包含穿刺边的所有三角片；这些三角片也需要新加点和边；
                    {
#ifdef TEST_CROSS_BOX_ACCE
                        // for test——tisct一定是交叉包围盒内的三角片，eisct关联的所有三角片中，可能存在交叉包围盒外的三角片，尝试跳过；                   
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
                    return false;               // 对层次包围盒的遍历终止
                }
                else
                    return true;                // continue
            });

        // 2. ？？？退化处理？
        if (Empty3d::degeneracy_count > 0)
        {
            std::cout << "Empty3d::degeneracy_count > 0" << std::endl;
            getchar();
            return;
        }

    // 3. 遍历tp对象，寻找ttt相交情形——即三个三角片t0, t1, t2；两两都相交
        std::vector<TriTripleTemp> triples;
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tprob)
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

                            CorkMesh::TriangleProblem* prob1 = reinterpret_cast<CorkMesh::TriangleProblem*>(t1->data);              // t1的tp对象；

                            for (IsctEdgeType* ie : prob1->iedges)      // 对三角片t1的交线段的遍历；
                            {
                                if (ie->other_tri_key == t2)            // 若t1和t2有交线段，则说明t0, t1, t2两两相交；
                                    triples.push_back(TriTripleTemp(t0, t1, t2));
                            }
                        }
                    });
            });

        // 4. 验证ttt相交情形是真实存在，还是退化情形；有退化情形返回false，终止程序；
        for (TriTripleTemp t : triples)
        {
            if (!isctObj.checkIsct(t.t0, t.t1, t.t2))               // 重新检查一遍三个三角片的交叉情况
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

        //      3.1.3 错误处理，跳过；
        if (nTrys <= 0)
        {
            CORK_ERROR("Ran out of tries to perturb the mesh");
            exit(1);
        }

        // 老的处理缺失尾端点的交线段的程序；
#ifndef IGNORE_SELF_ISCT
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tprob)
            {
                tprob->consolidate(&isctObj);
            });
#endif

        // 查找、删除病态交线段、病态交点；
#ifdef IGNORE_SELF_ISCT

        // 交线段ie对象存在isctObj.iepool中，tp对象中存有这些ie的指针；
        std::set<IsctVertType*> allSickVers;
        std::set<IsctEdgeType*> allSickEdges;
        std::set<IsctVertType*> sickVersPtrSet;
        std::set<IsctEdgeType*> sickEdgesPtrSet;

        // 查找病态交线段——存在缺失端点则判定为病态交线段；
        for (auto& ie : isctObj.iepool)
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
            isctObj.ivpool.for_each([&](IsctVertType* iv)                // 对交点池的遍历：
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
                isctObj.killIsctEdge(iePtr);
            for (auto& ivPtr : sickVersPtrSet)
                isctObj.killIsctVert(ivPtr);

            // 查找病态交线段：
            sickEdgesPtrSet.clear();
            isctObj.iepool.for_each([&](IsctEdgeType* iePtr)
                {
                    // 若缺失端点，判定为病态交线段；
                    if (nullptr == iePtr->ends[0] || nullptr == iePtr->ends[1])
                        sickEdgesPtrSet.insert(iePtr);
                });

            // 汇入总信息；
            for (auto& iePtr : sickEdgesPtrSet)
                allSickEdges.insert(iePtr);

        } while (!sickEdgesPtrSet.empty());

        unsigned ivCount = isctObj.ivpool.size();
        unsigned ieCount = isctObj.iepool.size();

        // 每一个tp对象中删除掉病态的交点和交线段的指针，： 
        isctObj.tprobs.for_each([&](CorkMesh::TriangleProblem* tpPtr)
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

        // 剩下的步骤：
        CorkMesh::BoolProblem bprob(&mesh0);

        // 4. 计算ecache成员数据——确定融合网格的所有边，并且标记出哪些边 是三角片相交边；
        bprob.populateECache();

        // 5.？？？生成UnionFind对象uf，不知道是干啥的；
        UnionFind uf(bprob.mesh->tris.size());
        bprob.for_ecache([&](uint, uint, bool, const ShortVec<uint, 2>& tids)
            {
                uint tid0 = tids[0];
                for (uint k = 1; k < tids.size(); k++)
                    uf.unionIds(tid0, tids[k]);
            });

        // 6. ？？？
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

        // 7. 使用射线来判断三角片在内部还是外部，处理三角片，修改三角片标记；
        std::vector<bool> visited(bprob.mesh->tris.size(), false);              // 记录三角片是否已经经过处理；

        for (unsigned i = 0; i < components.size(); ++i)
        {
            auto& comp = components[i];

            // 7.1 找出当前三角片组中面积最大的三角片；
            uint maxTriIdx = comp[0];                       // 面积最大的三角片索引；
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

            // 7.2 判断当前最大三角片是否在融合网格内部，修改该最大三角片的标记；
            byte currentMeshLabel = bprob.boolData(maxTriIdx);
            bool inside = bprob.isInside(maxTriIdx, currentMeshLabel);
            byte& maxTriLabel = bprob.boolData(maxTriIdx);
            maxTriLabel |= (inside) ? 2 : 0;            // 与0或10B做按位或运算，当前网格最大三角片，若在融合网格内部，则将其label前一位改为1； 该三角片标记处理完毕；

            // 7.3 从最大三角片开始，用相邻三角片生长的方式遍历本组所有三角片，根据不同情形修改三角片标记；
            std::queue<uint> work;                    // 处理三角片的工作队列；        
            visited[maxTriIdx] = true;
            work.push(maxTriIdx);

            while (!work.empty())
            {
                uint curr_tid = work.front();
                byte& currentTriLabel = bprob.boolData(curr_tid);           // 队首三角片的标记；
                work.pop();

                for (uint k = 0; k < 3; k++)        // 遍历当前三角片的三条边，进而遍历当前三角片的所有相邻三角片；
                {
                    uint a = bprob.mesh->tris[curr_tid].v[k];
                    uint b = bprob.mesh->tris[curr_tid].v[(k + 1) % 3];
                    auto& entry = bprob.ecache(a, b);                      // （a, b）这条边的信息；

                    // 三角片标记和10B做与运算
                    byte inside_sig = currentTriLabel & 2;          // 按位与运算用于提取某一位的信息，这里和10B做按位与运算，提取了currentTriLabel的前一位，表示是否在网格内；
                    if (entry.data.is_isct)
                        inside_sig ^= 2;                     // 若该边是三角片交线，则currentTri和nbrTri的内外状态一定是相异的；
                                                                         // 三角片生长的过程中，如果没有碰到三角片交线，则三角片的内外状态是一直不变的；
                                                                          // 和10B做异或运算，表示将内外状态取反；

                    for (uint nbrTriIdx : entry.tids)   // 遍历该条边关联的两个三角片，即遍历当前三角片的所有相邻三角片；
                    {
                        byte& nbrTriLabel = bprob.boolData(nbrTriIdx);
                        if (visited[nbrTriIdx])
                            continue;

                        if ((nbrTriLabel & 1) != currentMeshLabel)         // 若相邻三角片属于另一个网格，则跳过，此循环只处理当前网格中的三角片；
                            continue;

                        nbrTriLabel |= inside_sig;          // 
                        visited[nbrTriIdx] = true;
                        work.push(nbrTriIdx);
                    }
                }
            }

            currentMeshLabel++;
        }

        // 8. 不同的布尔运算，对标记的三角片做不同的操作；
        bprob.doDeleteAndFlip([](byte data) -> typename CorkMesh::BoolProblem::TriCode
            {
                if (data == 2 || data == 1)           // part of op 1 OUTSIDE op 0
                    return CorkMesh::BoolProblem::DELETE_TRI;                           // 删除标记为2, 1的三角片
                else if (data == 3)                   // part of op 1 INSIDE op 1
                    return CorkMesh::BoolProblem::FLIP_TRI;                                // 翻转标记为3的三角片；
                else                                    // part of op 0 OUTSIDE op 1
                    return CorkMesh::BoolProblem::KEEP_TRI;                               // 保留标记为0的三角片；
            });

        tt.endCout("总耗时：");

        debugWriteMesh("mesh_diff", *bprob.mesh);
        debugWriteMesh("box0", boxMesh0);
        debugWriteMesh("box1", boxMesh1);
        debugWriteMesh("boxIsct", boxMeshIsct);
        debugWriteMesh("boxIsct_放大1.01倍", boxMeshIsct101);
        std::cout << "successed!!!" << std::endl;
    }
}


int main(int argc, char* argv[])
{
    //// 原始的控制台程序：

    //return runCmd(argc, argv);

    // TEST_BOOLEAN::test0();

    // TEST_BOOLEAN::test1();

    // TEST_BOOLEAN::test2();

    // TEST_ISCT::test0();

    TEST_BASIC::test6();
    
    //TEST_BVH::test3();                              // 层次包围盒；
 
    std::cout << "int main() finished." << std::endl;
    getchar();
    return 1;
}