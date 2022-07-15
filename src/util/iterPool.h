#pragma once

#include "prelude.h"


// IterPool�ࡪ�����������ܵ��ڴ�أ����Ƚ�������ײ��Ǹ�stack��������������ע��ʹ�õ�������ʱ��begin()��Ӧ�����������һ���������ݣ�
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

// ע�⣡���� �ͳ���Ԫ������˳�����෴�ģ�IterPool֧�ַ�Χforѭ����������˳��Ҳ�Ǻ�Ԫ������˳���෴�ģ�

template<class T>
class IterPool
{
// ֧������
public:
    // Block�ࡪ��˫������ڵ㣻
    struct Block
    {
        T       datum;
        Block* next;
        Block* prev;
    };

    // iterator�ࡪ���������ࣻ
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


// ��Ա���ݣ�
public:
    Block* block_list;               // ������ָ�룿����
    uint numAlloced;                // �洢Ԫ������
    MemPool<Block> pool;    // �ڴ�ض���

public:
    IterPool(int minInitBlocks =10) :
        numAlloced(0),
        block_list(nullptr),
        pool(minInitBlocks)
    {}

    // �ƶ����죻
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
    
    // clear()��������ڴ���е�����
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
    
    // �ƶ���ֵ��
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
    
    // by Tora���������ڴ���е����ݿ�����std::vector�У�������ע��vector��Ԫ�ذ����������˳�򣬺ͳ���Ԫ������˳�����෴�ģ�
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

    // by Tora����ͬ�ϣ�
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

    // alloc()��������һ����Ԫ�صĿռ䣬���ظ�Ԫ�ص�ָ�룻
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

    // ɾ��Ԫ�أ�Ԫ��ָ����Ұָ�룬���ǿ�ָ��
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

    // for_each()����ʹ�ú�����func���������ڳ��е�����Ԫ�أ�
    inline void for_each(std::function<void(T*)> func) const 
    {
        for(Block *block = block_list;  block != NULL;  block = block->next) 
            func((T*)(block));
    }


    // contains()�����������Ƿ���ĳԪ�أ�
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


    // ��õ�����
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



