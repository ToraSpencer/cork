#pragma once

#include "prelude.h"


// IterPool类——带迭代功能的内存池，是先进后出，底层是个stack？？？；！！！注意使用迭代器的时候begin()对应的数据是最后一个填充的数据；
/*
     | An IterPool is
     |    *)  a memory pool -- new items can be requested, old items released
     |    *)  iterable -- all of the allocated items can be iterated over

     | It was built primarily to support dynamic connectivity structures
     | for triangle meshes, such as those used during re-meshing.
     |
     | These processes often require the frequent allocation and
     | deallocation of mesh elements and become easier to write when
     | elements do not move around in memory, since valid pointers can be
     | maintained.  std::vector does not provide guaranteed static locations.
     |
     | Using a memory pool--threaded with a linked list--provides fast
     | allocation/deallocation and relatively speedy enumeration of all
     | currently allocated elements.  While not as memory access friendly
     | as an array or std::vector, the cost of linked list enumeration
     | comes out in the wash compared to the pointer chasing performed
     | for each element.
*/


#include "memPool.h"
#include <utility>

// 注意！！！ 和池中元素填充的顺序是相反的；IterPool支持范围for循环，遍历的顺序也是和元素填充的顺序相反的；

template<class T>
class IterPool
{
// 支持类型
public:
    // Block类——双向链表节点；
    struct Block
    {
        T       datum;
        Block* next;
        Block* prev;
    };

    // iterator类——迭代器类；
    class iterator
    {
    public:
        iterator() : ptr(NULL) {}
        iterator(Block* init) : ptr(init) {}
        iterator(const iterator& cp) : ptr(cp.ptr) {}

        iterator& operator++()
        { // prefix version
            ptr = ptr->next;
            return *this;
        }

        iterator operator++(int)
        {
            iterator it(ptr);
            ptr = ptr->next;
            return it;
        } // postfix version

        T& operator*()
        {
            return ptr->datum;
        }

        T* operator->() 
        {
            return (T*)(ptr);
        }

        bool operator==(const iterator& rhs) 
        {
            return ptr == rhs.ptr;
        }

        bool operator!=(const iterator& rhs) 
        {
            return ptr != rhs.ptr;
        }

    private:
        Block* ptr;
    };


// 成员数据：
public:
    Block* block_list;               // 数据首指针？？？
    uint numAlloced;                // 存储元素数；
    MemPool<Block> pool;    // 内存池对象；

public:
    IterPool(int minInitBlocks =10) :
        numAlloced(0),
        block_list(nullptr),
        pool(minInitBlocks)
    {}

    // 移动构造；
    IterPool(IterPool &&src)
        : numAlloced(src.numAlloced),
          block_list(src.block_list),
          pool(std::move(src.pool))
    {
        src.block_list = nullptr;
    }

    ~IterPool() 
    {
        // run through and destruct all remaining elements
        for_each([](T* obj) 
            {
            obj->~T();
            });
    }
    
    // clear()——清空内存池中的数据
    void clear() 
    {
        for_each([](T* obj) 
            {
                obj->~T();
            });

        numAlloced = 0;
        block_list = nullptr;
        pool.clear();
    }
    
    // 移动赋值；
    void operator=(IterPool &&src)
    {
        for_each([](T* obj) 
            {
            obj->~T();
            });
        block_list = src.block_list;
        src.block_list = nullptr;
        pool = std::move(src.pool);
    }
    
    // by Tora——迭代内存池中的数据拷贝到std::vector中：！！！注意vector中元素按索引增大的顺序，和池中元素填充的顺序是相反的；
    void toVec(std::vector<T>& vec) 
    {
        vec.resize(numAlloced);
        Block* blockPtr = block_list;
        for (unsigned i = 0; i < numAlloced; ++i) 
        {
            vec[i] = blockPtr->datum;
            blockPtr = blockPtr->next;
        }
    }

    // by Tora——同上：
    std::vector<T> toVec()
    {
        std::vector<T> vec;
        vec.resize(numAlloced);
        Block* blockPtr = block_list;
        for (unsigned i = 0; i < numAlloced; ++i)
        {
            vec[i] = blockPtr->datum;
            blockPtr = blockPtr->next;
        }

        return vec;
    }


public:        

    // alloc()——开辟一个新元素的空间，返回该元素的指针；
    T* alloc() 
    {
        Block *new_block = pool.alloc();
        if(block_list) 
            block_list->prev = new_block;
        new_block->next = block_list;
        new_block->prev = NULL;
        block_list = new_block;
        
        T* obj = (T*)new_block;
        new (obj) T();              // invoke default constructor when allocating
        
        numAlloced++;
        return obj;
    }

    // 删除元素，元素指针变成野指针，不是空指针
    void free(T* item) 
    {
        if(item == NULL)   return;
        item->~T(); // invoke destructor before releasing
        
        numAlloced--;
        
        Block *ptr = (Block*)(item);
        if(ptr->next)   ptr->next->prev = ptr->prev;
        if(ptr->prev)   ptr->prev->next = ptr->next;
        if(ptr == block_list)   block_list = ptr->next;
        pool.free(ptr);
    }
    

public:

    // for_each()——使用函数子func遍历作用于池中的所有元素；
    inline void for_each(std::function<void(T*)> func) const 
    {
        for(Block *block = block_list;  block != NULL;  block = block->next) 
            func((T*)(block));
    }


    // contains()——检查池中是否有某元素；
    inline bool contains(T* tptr) const 
    {
        for(Block *block = block_list;
          block != NULL;
          block = block->next) 
        {
            if(tptr == (T*)(block))
                return true;
        }
        return false;
    }

    inline uint size() const 
    {
        return numAlloced;
    }


    // 获得迭代器
public:            
    iterator begin() 
    {
        return iterator(block_list);
    }

    iterator end() 
    {
        return iterator(NULL);
    }
};



