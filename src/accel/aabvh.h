#pragma once


// AABVH�ࡪ��������ΰ�Χ����(axis-aligned bounding volume hierarchy)����һ�����νṹ��
#include "bbox.h"
#include <stack>


//#define DEBUG_AABVH

static const uint LEAF_SIZE = 8;        // ����������leaf_size����ʾҶ�ӽ�����������е�����Ƭ��������
// static const uint LEAF_SIZE = 1;

// ǰ������
struct TopoEdge;

// GeomBlob�ࡪ��ʵ����ʱ���ڴ洢ĳ���ߵ�AABB���������Ϣ��
template<class GeomIdx>         // ʵ����ʱģ�����GeomIdx�����������˱���TopoEdge*
struct GeomBlob                 // ��Ŀ�����ɱߵ�gb����Ľӿڡ���IsctProblem::edge_blob()
{
    BBox3d  bbox;                   // �ñߵ�������Χ��AABB����
    Vec3d   point;                    // �ñߵ��е����ꣻҲ��AABB�����ĵ㣻
    GeomIdx id;                 // ʵ����ʱ��������TopoEdge*;
};


// AABNHNode�ࡪ����ΰ�Χ�нڵ㣻
template<class GeomIdx>
struct AABVHNode
{
    BBox3d                          bbox;
    AABVHNode                       *left;
    AABVHNode                       *right;
    ShortVec<uint, LEAF_SIZE>       blobids;            // �����˴˰�Χ�а�Χ�����б߶�Ӧ��gb������AABVH::blobs�����е�������������ֻ��Ҷ�ӽڵ��д��Ϊ�գ�
    
    // �ж��Ƿ���Ҷ�ӽڵ㣻
    inline bool isLeaf() const 
    {
        return left == nullptr; 
    }
};


// AABVH�ࡪ����ΰ�Χ���ࣻ��Ŀ��ʵ����ʱ���ɵ����������бߵĲ�ΰ�Χ�У�
template<class GeomIdx>             // ʵ����ʱģ�����GeomIdx��������TopoEdge*
class AABVH
{
// ��Ա���ݣ�
public:
    AABVHNode<GeomIdx>* root;                              // ���ڵ㣻
    IterPool< AABVHNode<GeomIdx> >      node_pool;      // ��������������ݶ������ڴ���
    std::vector< GeomBlob<GeomIdx> >    blobs;
    std::vector<uint>                   tmpids;                         // used during construction

#ifdef DEBUG_AABVH
    std::vector<GeomBlob<GeomIdx>> expBlobs;            // ����by Tora, ����һЩ����������ⲿ��gb����
    std::vector<TopoEdge> isctEdges;
#endif

public:
    // ���캯�����������������бߵ�GeomBlob��Ϣ�����ɲ�ΰ�Χ�У�
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
    
    // for_each_in_box()��������AABVH�е����а�Χ�У��ҳ��ʹ����Χ��bbox�ཻ��Ҷ�ӽڵ㣬�����е�TopoEdge���ú�����action;
    inline void for_each_in_box(const BBox3d& bbox,  std::function<void(GeomIdx idx)> action)
    {
#ifdef DEBUG_AABVH
        // ����ÿ�α���AABVH֮ǰ�����expBlobs��һ�α���ֻ����һ��������ݣ�
        this->expBlobs.clear();
        this->expBlobs.reserve(this->blobs.size());

        this->isctEdges.clear();
        this->isctEdges.reserve(this->blobs.size() * LEAF_SIZE);
#endif

        std::stack< AABVHNode<GeomIdx>* >  nodes;           // ���ڱ���AABVH�ڵ��ջ��
        nodes.push(root);
        
        // �ݹ�ѭ����
        while(!nodes.empty())
        {
            AABVHNode<GeomIdx> *node = nodes.top();
            nodes.pop();
            
            // a. ������İ�Χ�к͵�ǰ��Χ�����ཻ��������һ��ѭ����
            if(!hasIsct(node->bbox, bbox))  
                continue;
            
            // b. ������İ�Χ�к͵�ǰ��Χ��û���ཻ��
            if(node->isLeaf())
            { 
                // ba. ����ǰ��Χ����Ҷ�ӽڵ㣬���봫���Χ���н��棬��
                for(uint gbIdx : node->blobids) 
                {
                    const GeomBlob<GeomIdx>& currentBlob = this->blobs[gbIdx];
                    if (hasIsct(bbox, currentBlob.bbox))          // ������Χ�н��棻
                    {
                        TopoEdge& currentEdge = *currentBlob.id;
                        action(&currentEdge);

#ifdef DEBUG_AABVH
                        this->expBlobs.push_back(currentBlob);         // �����bbox�н���İ�Χ�У�
                        this->isctEdges.push_back(currentEdge);
#endif
                    }
                }
            }
            else
            {
                // bb. ����ǰ��Χ�в���Ҷ�ӽڵ㣬�ݹ�ѭ���������ӽڵ㣻
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }

#ifdef DEBUG_AABVH
        this->expBlobs.shrink_to_fit();
        this->isctEdges.shrink_to_fit();
#endif
    }
    

    // by Tora����ͬ�ϣ�ֻ�������������õ���aabvh�Ľڵ�����ָ�룻
    inline void for_each_in_box(const BBox3d& bbox, std::function<void(AABVHNode<GeomIdx>* nodePtr)> action)
    {
#ifdef DEBUG_AABVH
        // ����ÿ�α���AABVH֮ǰ�����expBlobs��һ�α���ֻ����һ��������ݣ�
        this->expBlobs.clear();
        this->expBlobs.reserve(this->blobs.size());

        this->isctEdges.clear();
        this->isctEdges.reserve(this->blobs.size() * LEAF_SIZE);
#endif

        std::stack< AABVHNode<GeomIdx>* >  nodes;           // ���ڱ���AABVH�ڵ�ָ���ջ��
        nodes.push(root);

        // �ݹ�ѭ����
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* nodePtr = nodes.top();
            nodes.pop();

            // a. ������İ�Χ�к͵�ǰ��Χ�����ཻ��������һ��ѭ����
            if (!hasIsct(nodePtr->bbox, bbox))
                continue;

            // b. ������İ�Χ�к͵�ǰ��Χ��û���ཻ��
            if (nodePtr->isLeaf())
            {
                // ba. ����ǰ��Χ����Ҷ�ӽڵ㣬���봫���Χ���н��棬��
                for (uint gbIdx : nodePtr->blobids)
                {
                    const GeomBlob<TopoEdge*>& currentBlob = this->blobs[gbIdx];
                    if (hasIsct(bbox, currentBlob.bbox))          // ������Χ�н��棻
                        action(nodePtr);
                }
            }
            else
            {
                // bb. ����ǰ��Χ�в���Ҷ�ӽڵ㣬�ݹ�ѭ���������ӽڵ㣻
                nodes.push(nodePtr->left);
                nodes.push(nodePtr->right);
            }
        }

#ifdef DEBUG_AABVH
        this->expBlobs.shrink_to_fit();
        this->isctEdges.shrink_to_fit();
#endif
    }


    // by Tora������������������Ҷ�ӽڵ㣺
    inline AABVHNode<GeomIdx>* searchLeaf(std::function<bool(AABVHNode<GeomIdx>* nodePtr)> action)
    {
        std::stack< AABVHNode<GeomIdx>* >  nodes;           // ���ڱ���AABVH�ڵ�ָ���ջ��
        nodes.push(root);

        // �ݹ�ѭ����
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* nodePtr = nodes.top();
            nodes.pop();

            // ����ǰ�ڵ���Ҷ�ӽڵ㣺
            if (nodePtr->isLeaf())
            {
                if (action(nodePtr))
                    return nodePtr;
            }
            else
            {
                // bb. ����ǰ��Χ�в���Ҷ�ӽڵ㣬�ݹ�ѭ���������ӽڵ㣻
                nodes.push(nodePtr->left);
                nodes.push(nodePtr->right);
            }
        }

        return nullptr;         // ��ʾû�����������������Ľڵ㣻
    }


    // by Tora��������ĳҶ�ӽڵ�����и��ڵ㣬����Ե�ӽ���Զ���У�
    inline void searchParents(std::vector<AABVHNode<GeomIdx>*>& parentsVec, AABVHNode<GeomIdx>* nodePtr0)
    {
        std::stack< AABVHNode<GeomIdx>* >  nodes;           // ���ڱ���AABVH�ڵ�ָ���ջ��
        nodes.push(root);

        // �ݹ�ѭ����
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* nodePtr = nodes.top();
            nodes.pop();

            // ����ǰ�ڵ���Ҷ�ӽڵ㣺
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
                // bb. ����ǰ��Χ�в���Ҷ�ӽڵ㣬�ݹ�ѭ���������ӽڵ㣻
                nodes.push(nodePtr->left);
                nodes.push(nodePtr->right);
            }
        }

    }


public:
    // build()�������е�aabvh��������������ݣ�Ȼ���ȡ����geoms���ù��캯�����죻 ����by Tora
    void build(const std::vector< GeomBlob<GeomIdx> >& geoms)
    {
        ENSURE(geoms.size() > 0);

        // ����������ݣ�
        this->root = nullptr;
        this->node_pool.clear();

        // ���죺
        this->blobs = geoms;
        this->tmpids.resize(geoms.size());
        for (uint k = 0; k < tmpids.size(); k++)
            tmpids[k] = k;

        this->root = constructTree(0, tmpids.size(), 2);
    }

    // ���캯���е��ã�����BVH����
    AABVHNode<GeomIdx>* constructTree(uint begin, uint end, uint last_dim)
    {
        /*
             AABVHNode<GeomIdx>* constructTree(
                    uint begin,             ��ʼ����
                    uint end,               �յ�����
                    uint last_dim       �����ȡ0, 1, 2��  last_dim provides a hint by saying which dimension a split was last made along
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
                node->bbox = convex(node->bbox, blobs[blobid].bbox);            // convex()�������ذ���������Χ�е���С��Χ�У�
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
    

    // ����AVH���е�ѡȡһ���ռ䷶Χ�ڵıߣ�ò������XYZ�����Ͻ����ƽ�����֣�
    void quickSelect(uint select, uint begin, uint end, uint dim)
    {
        // NOTE: values equal to the pivot, may appear on either side of the split
        if(end-1 == select)     
            return;
        
        // p(ivot)i(ndex) and p(ivot)v(alue)
        uint pi = randMod(end-begin) + begin;
        double pv = this->blobs[tmpids[pi]].point[dim];               // ����������Χ�����ȡһ���ߣ�Ȼ��ȡ���е㣻
        
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


    // �õ���ǰBVH����Ҷ�ӽڵ�������Χ��
    void getAllleafAABB(std::vector<BBox3d>& boxVec) 
    {
        std::stack< AABVHNode<GeomIdx>* >  nodes;           // ���ڱ���AABVH�ڵ��ջ��
        nodes.push(root);

        // �ݹ�ѭ����
        while (!nodes.empty())
        {
            AABVHNode<GeomIdx>* node = nodes.top();
            nodes.pop();
 
            // ����ǰ�ڵ���Ҷ�ӽڵ㣺
            if (node->isLeaf())
                boxVec.push_back(node->bbox);
            else
            {
                // bb. ����ǰ��Χ�в���Ҷ�ӽڵ㣬�ݹ�ѭ���������ӽڵ㣻
                nodes.push(node->left);
                nodes.push(node->right);
            }
        }
    }
};
