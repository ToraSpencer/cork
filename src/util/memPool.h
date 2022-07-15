#pragma once

// MemPool�ࡪ�����ṹ���ڴ��
#include <new>
#include <algorithm>


/*
 *  RESPONSIBILITIES OF THE USER / CLIENT CODE
 *
 *      MemPool will not invoke constructors or destructors on memory that it allocates or frees. 
        There is a simple  reason for this:  One advantage of MemPool is that a large number
 of allocated entries can be disposed of very quickly by simply destroying the pool. 
        However, if MemPool were to call constructors  (and thus be obligated to call destructors), then it would have
 to run through and destruct each element individually when it, itself is destroyed.

 *      TAKE-AWAY: Make sure to construct/destruct objects explicity
 *  when they are allocated with a raw MemPool.
 *
 */


template<class T>
class MemPool
{
    // ֧�����ͣ� 
public:
    union Block
    {
        byte   datum[sizeof(T)];         // enough space for a T
        Block* next;
    };

    struct Chunk
    {
        Block* data;           // array of size nBlocks
        int     nBlocks;
        Chunk* next;
    };


// ��Ա���ݣ�
public:
    Chunk* chunk_list;
    Block* free_list;


public:      
    MemPool(int minInitBlocks=10);
    MemPool(MemPool &&src);
    ~MemPool();
    
    void clear();
    
    void operator=(MemPool &&);

private:            // �ⲿ���ÿ�������
    MemPool(const MemPool &) {}
    MemPool& operator=(const MemPool &) { return *this; }
    
public:                
    T* alloc();                 // ����һ��Ԫ�صĿռ䣬������ָ�룻
    void free(T*);

private:                     
    void addChunk();     // this should be called only when the free list is empty
};


template<class T>
MemPool<T>::MemPool(int minInitBlocks)
{
    // decide on the size of the first chunk
    const int MIN_BLOCKS = 2;           // will not create a pool with fewer

    // than 2 blocks
    int nBlocks = std::max(minInitBlocks, MIN_BLOCKS);
    
    // allocate the first chunk
    chunk_list          = (Chunk*)(new byte[sizeof(Chunk)]);
    chunk_list->next    = NULL;
    chunk_list->nBlocks = nBlocks;
    chunk_list->data    = (Block*)(new byte[sizeof(Block)*nBlocks]);
    
    // setup the free list
    free_list           = chunk_list->data;
    Block* last_block   = chunk_list->data + nBlocks - 1;
    for(Block *it = free_list; it != last_block; it++)
        it->next = it+1;
    last_block->next    = NULL;
}


template<class T>
MemPool<T>::MemPool(MemPool &&src)
    : chunk_list(src.chunk_list), free_list(src.free_list)
{
    src.chunk_list = nullptr;
    src.free_list = nullptr;
}


template<class T>
void MemPool<T>::operator=(MemPool &&src)
{
    chunk_list = src.chunk_list;
    free_list = src.free_list;
    src.chunk_list = nullptr;
    src.free_list = nullptr;
}


template<class T>
MemPool<T>::~MemPool()
{
    // We can ignore the free list here and just go through
    // destroying all of the chunks
    while(chunk_list != NULL) {
        delete[] (byte*)(chunk_list->data);
        Chunk *next = chunk_list->next;
        delete[] (byte*)(chunk_list);
        chunk_list = next;
    }
}


template<class T>
void MemPool<T>::clear()
{
    // reset the free list
    free_list = nullptr;
    for(Chunk *chunk = chunk_list; chunk != nullptr; chunk = chunk->next) {
        // thread the blocks in this chunk
        Block *last_block = chunk->data + chunk->nBlocks - 1;
        for(Block *it = chunk->data; it != last_block; it++)
            it->next = it+1;
        // then push them onto the front of the free list
        last_block->next    = free_list;
        free_list = chunk->data;
    }
}


template<class T> inline
T* MemPool<T>::alloc()
{
    // if we're out of blocks, add a new chunk
    if(free_list == NULL)
        addChunk();
    
    // pop a block off the free_list
    T* ret = reinterpret_cast<T*>(free_list);
    free_list = free_list->next;
    
    // invoke constructor on allocated block
    //new (ret) T();  // SEE INSTRUCTIONS AT TOP
    
    return ret;
}


template<class T> inline
void MemPool<T>::free(T* datum)
{
    if(datum == NULL)   return; // silent fail for freeing NULLed pointers
    // invoke destructor
    //datum->~T();  // SEE INSTRUCTIONS AT TOP
    
    Block *block = reinterpret_cast<Block*>(datum);
    
    // push this block onto the front of the free_list
    // so it can be re-allocated in the future
    block->next = free_list;
    free_list = block;
}


template<class T>
void MemPool<T>::addChunk()
{
    Chunk *new_chunk    = (Chunk*)(new byte[sizeof(Chunk)]);
    new_chunk->next     = chunk_list;
    // we always double the size of each new chunk.  This ensures that
    // the total # of times that we have to add chunks is logarithmic.
    // Regardless, the amortized cost of chunk allocation is constant.
    new_chunk->nBlocks  = chunk_list->nBlocks * 2;
    new_chunk->data     = (Block*)(new byte[sizeof(Block) * 
                                            (new_chunk->nBlocks)]);
    chunk_list          = new_chunk;
    
    // now we need to lace the blocks of the fresh chunk up into the free_list
    Block *last_block   = new_chunk->data + new_chunk->nBlocks - 1;
    last_block->next    = free_list;
    free_list           = new_chunk->data;
    for(Block *it = new_chunk->data; it != last_block; it++)
        it->next = it+1;
}





