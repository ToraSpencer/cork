#pragma once


// AABVH类——轴对齐层次包围盒类(axis-aligned bounding volume hierarchy)，是一个树形结构；
#include "bbox.h"
#include <stack>


//#define DEBUG_AABVH

static const uint LEAF_SIZE = 8;        // ？？？最大的leaf_size，表示叶子结点中最多可以有的三角片的数量？
// static const uint LEAF_SIZE = 1;

// 前置申明
struct TopoEdge;

// GeomBlob类——实例化时用于存储某条边的AABB及其相关信息；
template<class GeomIdx>         // 实例化时模板参数GeomIdx的类型是拓扑边类TopoEdge*
struct GeomBlob                 // 项目中生成边的gb对象的接口——IsctProblem::edge_blob()
{
    BBox3d  bbox;                   // 该边的轴对齐包围盒AABB对象；
    Vec3d   point;                    // 该边的中点坐标；也是AABB的中心点；
    GeomIdx id;                 // 实例化时该类型是TopoEdge*;
};


// AABNHNode类——层次包围盒节点；
template<class GeomIdx>
struct AABVHNode
{
    BBox3d                          bbox;
    AABVHNode                       *left;
    AABVHNode                       *right;
    ShortVec<uint, LEAF_SIZE>       blobids;            // 包含了此包围盒包围的所有边对应的gb对象在AABVH::blobs向量中的索引；？？？只有叶子节点中此项不为空，
    
    // 判断是否是叶子节点；
    inline bool isLeaf() const 
    {
        return left == nullptr; 
    }
};


// AABVH类——层次包围盒类；项目中实例化时生成的是网格所有边的层次包围盒；
template<class GeomIdx>             // 实例化时模板参数GeomIdx的类型是TopoEdge*
class AABVH
{
// 成员数据：
public:
    AABVHNode<GeomIdx>* root;                              // 根节点；
    IterPool< AABVHNode<GeomIdx> >      node_pool;      // ？？？具体的数据都存在内存池里？
    std::vector< GeomBlob<GeomIdx> >    blobs;
    std::vector<uint>                   tmpids;                         // used during construction

#ifdef DEBUG_AABVH
    std::vector<GeomBlob<GeomIdx>> expBlobs;            // ——by Tora, 保存一些用于输出到外部的gb对象；
    std::vector<TopoEdge> isctEdges;
#endif

public:
    // 构造函数——输入网格所有边的GeomBlob信息，生成层次包围盒；
    AABVH(const std::vector< GeomBlob<GeomIdx> > &geoms) :
        root(nullptr), blobs(geoms), tmpids(geoms.size())
    {
        ENSURE(blobs.size() > 0);
        
        for(uint k=0; k<tmpids.size(); k++)
            tmpids[k] = k;
        
        root = constructTree(0, tmpids.size(), 2);
    }

    AABVH() : root(nullptr)
    {}

    ~AABVH() {}
    
    // for_each_in_box()——遍历AABVH中的所有包围盒，找出和传入包围盒bbox相交的叶子节点，对其中的TopoEdge调用函数子action;
    inline void for_each_in_box(const BBox3d& bbox,  std::function<void(GeomIdx idx)> action)
    {
#ifdef DEBUG_AABVH
        // ——每次遍历AABVH之前都清空expBlobs；一次遍历只生成一组输出数据；
        this->expBlobs.clear();
        this->expBlobs.reserve(this->blobs.size());

        this->isctEdges.clear();
        this->isctEdges.reserve(this->blobs.size() * LEAF_SIZE);
#endif

        std::stack< AABVHNode<GeomIdx>* >  nodes;           // 用于遍历AABVH节点的栈；
        nodes.push(root);
        
        // 递归循环；
        while(!nodes.empty())
        {
            AABVHNode<GeomIdx> *node = nodes.top();
            nodes.pop();
            
            // a. 若传入的包围盒和当前包围盒有相交，进入下一层循环；
            if(!hasIsct(node->bbox, bbox))  
                continue;
            
            // b. 若传入的包围盒和当前包围盒没有相交；
            if(node->isLeaf())
            { 
                // ba. 若当前包围盒是叶子节点，且与传入包围盒有交叉，则
                for(uint gbIdx : node->blobids) 
                {
                    const GeomBlob<GeomIdx>& currentBlob = this->blobs[gbIdx];
                    if (hasIsct(bbox, currentBlob.bbox))          // 若两包围盒交叉；
                    {
                        TopoEdge& currentEdge = *currentBlob.id;
                        action(&currentEdge);

#ifdef DEBUG_AABVH
                        this->expBlobs.push_back(currentBlob);         // 保存和bbox有交叉的包围盒；
                        this->isctEdges.push_back(currentEdge);
#endif
                    }
                }
            }
            else
            {
                // bb. 若当前包围盒不是叶子节点，递归循环进入其子节点；
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }

#ifdef DEBUG_AABVH
        this->expBlobs.shrink_to_fit();
        this->isctEdges.shrink_to_fit();
#endif
    }
    

    // by Tora——同上，只不过函数子作用的是aabvh的节点对象的指针；
    inline void for_each_in_box(const BBox3d& bbox, std::function<void(AABVHNode<GeomIdx>* nodePtr)> action)
    {
#ifdef DEBUG_AABVH
        // ——每次遍历AABVH之前都清空expBlobs；一次遍历只生成一组输出数据；
        this->expBlobs.clear();
        this->expBlobs.reserve(this->blobs.size());

        this->isctEdges.clear();
        this->isctEdges.reserve(this->blobs.size() * LEAF_SIZE);
#endif

        std::stack< AABVHNode<GeomIdx>* >  nodes;           // 用于遍历AABVH节点指针的栈；
        nodes.push(root);

        // 递归循环；
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* nodePtr = nodes.top();
            nodes.pop();

            // a. 若传入的包围盒和当前包围盒有相交，进入下一层循环；
            if (!hasIsct(nodePtr->bbox, bbox))
                continue;

            // b. 若传入的包围盒和当前包围盒没有相交；
            if (nodePtr->isLeaf())
            {
                // ba. 若当前包围盒是叶子节点，且与传入包围盒有交叉，则
                for (uint gbIdx : nodePtr->blobids)
                {
                    const GeomBlob<TopoEdge*>& currentBlob = this->blobs[gbIdx];
                    if (hasIsct(bbox, currentBlob.bbox))          // 若两包围盒交叉；
                        action(nodePtr);
                }
            }
            else
            {
                // bb. 若当前包围盒不是叶子节点，递归循环进入其子节点；
                nodes.push(nodePtr->left);
                nodes.push(nodePtr->right);
            }
        }

#ifdef DEBUG_AABVH
        this->expBlobs.shrink_to_fit();
        this->isctEdges.shrink_to_fit();
#endif
    }


    // by Tora——搜索满足条件的叶子节点：
    inline AABVHNode<GeomIdx>* searchLeaf(std::function<bool(AABVHNode<GeomIdx>* nodePtr)> action)
    {
        std::stack< AABVHNode<GeomIdx>* >  nodes;           // 用于遍历AABVH节点指针的栈；
        nodes.push(root);

        // 递归循环；
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* nodePtr = nodes.top();
            nodes.pop();

            // 若当前节点是叶子节点：
            if (nodePtr->isLeaf())
            {
                if (action(nodePtr))
                    return nodePtr;
            }
            else
            {
                // bb. 若当前包围盒不是叶子节点，递归循环进入其子节点；
                nodes.push(nodePtr->left);
                nodes.push(nodePtr->right);
            }
        }

        return nullptr;         // 表示没有搜索到符合条件的节点；
    }


    // by Tora——搜索某叶子节点的所有父节点，按亲缘从近到远排列；
    inline void searchParents(std::vector<AABVHNode<GeomIdx>*>& parentsVec, AABVHNode<GeomIdx>* nodePtr0)
    {
        std::stack< AABVHNode<GeomIdx>* >  nodes;           // 用于遍历AABVH节点指针的栈；
        nodes.push(root);

        // 递归循环；
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* nodePtr = nodes.top();
            nodes.pop();

            // 若当前节点是叶子节点：
            if (nodePtr->isLeaf())
            {
                if (nodePtr0 == nodePtr)
                {
                    parentsVec.reserve(nodes.size());
                    while (!nodes.empty())
                    {
                        parentsVec.push_back(nodes.top());
                        nodes.pop();
                    }
                    break;
                }
            }
            else
            {
                // bb. 若当前包围盒不是叶子节点，递归循环进入其子节点；
                nodes.push(nodePtr->left);
                nodes.push(nodePtr->right);
            }
        }

    }


public:
    // build()——已有的aabvh对象清空自身数据，然后读取参数geoms调用构造函数构造； ——by Tora
    void build(const std::vector< GeomBlob<GeomIdx> >& geoms)
    {
        ENSURE(geoms.size() > 0);

        // 清空已有数据：
        this->root = nullptr;
        this->node_pool.clear();

        // 构造：
        this->blobs = geoms;
        this->tmpids.resize(geoms.size());
        for (uint k = 0; k < tmpids.size(); k++)
            tmpids[k] = k;

        this->root = constructTree(0, tmpids.size(), 2);
    }

    // 构造函数中调用，生成BVH树；
    AABVHNode<GeomIdx>* constructTree(uint begin, uint end, uint last_dim)
    {
        /*
             AABVHNode<GeomIdx>* constructTree(
                    uint begin,             起始索引
                    uint end,               终点索引
                    uint last_dim       交替地取0, 1, 2，  last_dim provides a hint by saying which dimension a split was last made along
                    )
        */

        ENSURE(end - begin > 0); // don't tell me to build a tree from nothing

        // base case
        if(end-begin <= LEAF_SIZE) 
        {
            AABVHNode<GeomIdx> *node = node_pool.alloc();
            node->left = nullptr;
            node->blobids.resize(end-begin);
            for(uint k=0; k<end-begin; k++) 
            {
                uint blobid = node->blobids[k] = tmpids[begin + k];
                node->bbox = convex(node->bbox, blobs[blobid].bbox);            // convex()——返回包含两个包围盒的最小包围盒；
            }
            return node;
        }

        // otherwise, let's try to split this geometry up
        uint dim = (last_dim+1)%3;
        uint mid = (begin + end) / 2;
        quickSelect(mid, begin, end, dim);
        
        // now recurse
        AABVHNode<GeomIdx> *node = node_pool.alloc();
        node->left = constructTree(begin, mid, dim);
        node->right = constructTree(mid, end, dim);
        node->bbox = convex(node->left->bbox, node->right->bbox);
        return node;
    }
    

    // 生成AVH树中的选取一定空间范围内的边，貌似是在XYZ方向上交替地平均二分：
    void quickSelect(uint select, uint begin, uint end, uint dim)
    {
        // NOTE: values equal to the pivot, may appear on either side of the split
        if(end-1 == select)     
            return;
        
        // p(ivot)i(ndex) and p(ivot)v(alue)
        uint pi = randMod(end-begin) + begin;
        double pv = this->blobs[tmpids[pi]].point[dim];               // 给定索引范围内随机取一条边，然后取其中点；
        
        int front = begin;
        int back  = end-1;
        while(front < back) 
        {
            if(this->blobs[tmpids[front]].point[dim] < pv)
                front++;
            else if(this->blobs[tmpids[back]].point[dim] > pv)
                back--;
            else 
            {
                std::swap(tmpids[front], tmpids[back]);
                front++;
                back--;
            }
        }

        if(front == back && blobs[tmpids[front]].point[dim] <= pv) 
            front++;

        if(select < uint(front)) 
            quickSelect(select, begin, front, dim);
         else 
            quickSelect(select, front, end, dim);
        
    }


    // 得到当前BVH所有叶子节点的轴向包围盒
    void getAllleafAABB(std::vector<BBox3d>& boxVec) 
    {
        std::stack< AABVHNode<GeomIdx>* >  nodes;           // 用于遍历AABVH节点的栈；
        nodes.push(root);

        // 递归循环；
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* node = nodes.top();
            nodes.pop();
 
            // 若当前节点是叶子节点：
            if (node->isLeaf())
                boxVec.push_back(node->bbox);
            else
            {
                // bb. 若当前包围盒不是叶子节点，递归循环进入其子节点；
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }
    }
};
