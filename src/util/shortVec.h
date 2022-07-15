 #pragma once

// ShortVec�ࡪ���Զ���������ࣻ���õ����ڴ��MemPool����������Ч�ʣ�����

#include "prelude.h"
#include "memPool.h"

#include <algorithm>



/*
     Don't know why, but can't get this to compile��when I make the datablock def. a member of ShortVec<T,LEN>
     We allocate these blocks instead of typed arrays in order to ensure that we get control of allocation/deallocation
     rather than the allocator attempting to do so.
*/
template<class T, uint LEN>
struct ShortVecBlock_Private 
{
    byte data[sizeof(T)*LEN];
};




 // ���û������Ԫ����������LEN�����ڴ��pool�������ڴ�ռ䣻����������ڴ棻������ò��Ԫ����С��LENʱЧ�ʻ�ߣ�
template<class T, uint LEN>
class ShortVec
{
// ��Ա����
public: 
    static MemPool< ShortVecBlock_Private<T, LEN> > pool;       // shared data structures and data.

    // instance data
    uint user_size;         // Ԫ������
    uint internal_size;     // ʵ�ʿ��ٵ��ڴ��ֽ���; if allocated from the memory pool, this is 0
    T* data;                    // ��Ԫ�ص�ָ�룻

public:  
    ShortVec(uint size = 0);
    ShortVec(uint size, const T &fill_val);
    ShortVec(const ShortVec<T,LEN> &cp);
    ~ShortVec();
    
    ShortVec<T,LEN>& operator=(const ShortVec<T,LEN> &vec);

    // by Tora����shortVec�е�Ԫ�ؿ�����std::vector��ȥ��
    inline void toVec(std::vector<T>& vec)  const
    {
        vec.clear();
        if (0 == this->user_size)
            return;
        vec.resize(this->user_size);
        std::memcpy(&vec[0], this->data, sizeof(T) * this->user_size);
    }

    // by Tora����ͬ�ϣ�
    inline std::vector<T> toVec() const
    {
        std::vector<T> vec;
        if (0 == this->user_size)
            return vec;

        vec.resize(this->user_size);
        std::memcpy(&vec[0], this->data, sizeof(T) * this->user_size);
        return vec;
    }

    // by Tora������ӡԪ�أ�
    void disp() const
    {
        auto iter = this->begin();
        for (; iter != this->end(); ++iter) 
            std::cout << *iter << ", ";
        std::cout << std::endl;
    }

public:
    // []���������������ȡԪ�أ�
    inline       T& operator[](uint i)      
    {
        return data[i];
    }

    inline const T& operator[](uint i) const 
    {
        return data[i];
    }
    

// iterators
public: 
    typedef       T* iterator;
    typedef const T* const_iterator;
    iterator        begin()       { return data; }
    const_iterator  begin() const { return data; }
    iterator        end()         { return data + user_size; }
    const_iterator  end()   const { return data + user_size; }
 

// inspectors
public: 
    inline uint size() const { return user_size; }
    

// modifiers
public: 
    void resize(uint newsize);
    void push_back(const T &datum);
    void erase(const T &val);           // erase if it can be found
    
    
// helper functions
private: 
    T*   allocData(uint space, uint &allocated);
    void deallocData(T* data_ptr, uint allocated);
    
    void constructRange(T* array, int begin, int end);
    void copyConstructRange(T* src, T* dest, int begin, int end);
    void destructRange(T* array, int begin, int end);
    void resizeHelper(uint newsize);          // resize, manage allocation/deallocation, but not construction/destruction
   
};



template<class T, uint LEN>
MemPool< ShortVecBlock_Private<T,LEN> > ShortVec<T,LEN>::pool;

template<class T, uint LEN> inline
T* ShortVec<T,LEN>::allocData(uint space, uint &allocated)
{
    // �û�Ҫ�󿪱�space��Ԫ�ص��ڴ�ռ䣬����ʵ�ʿ�����allocated��Ԫ�ص��ڴ�ռ䣻

    T* result;
    if(space <= LEN)            // ���û������Ԫ����������LEN�����ڴ��pool�������ڴ�ռ䣻
    {
        allocated = LEN;            
        result =  reinterpret_cast<T*>(pool.alloc());
    } 
    else 
    {
        allocated = space;
        result = reinterpret_cast<T*>(new byte[sizeof(T)*space]);
    }
 
    return result;
}


template<class T, uint LEN> inline
void ShortVec<T,LEN>::deallocData(T* data_ptr, uint allocated)
{
    //if(LEN == 2) std::cout << "        Deallocing: " << data_ptr << std::endl;
    if(allocated <= LEN)
        pool.free(reinterpret_cast< ShortVecBlock_Private<T,LEN>* >(data_ptr));
    else
        delete[] reinterpret_cast<byte*>(data_ptr);
}


template<class T, uint LEN> inline
void ShortVec<T,LEN>::constructRange(T* array, int begin, int end)
{
    for(int i=begin; i<end; i++)
        new (&(array[i])) T();
}


template<class T, uint LEN> inline
void ShortVec<T,LEN>::copyConstructRange(T* src, T* dest, int begin, int end)
{
    // copy actual data over
    for(int i=begin; i<end; i++)
        new (&(dest[i])) T(src[i]);
}


template<class T, uint LEN> inline
void ShortVec<T,LEN>::destructRange(T* array, int begin, int end)
{
    for(int i=begin; i<end; i++)
        (&(array[i]))->~T();
}

// we use a strictly increasing allocation size policy
// with array length doubling to ensure that the cost of
// copying array entries on a reallocation has
// constant amortized cost.  This is important when the
// vector is used to accumulate a list of values.
template<class T, uint LEN> inline
void ShortVec<T,LEN>::resizeHelper(uint newsize) {
    if(newsize > internal_size) { // we need more space!
        // setup the new data block with at least twice as much space
        uint new_space;
        T *new_data = allocData(std::max(newsize, internal_size*2), new_space);
        // copy data and destroy old copies
        copyConstructRange(data, new_data, 0, user_size);
        destructRange(data, 0, user_size);
        // free old data
        deallocData(data, internal_size);
        data = new_data;
        internal_size = new_space;
    }
    user_size = newsize;
}


template<class T, uint LEN> inline
ShortVec<T,LEN>::ShortVec(uint size) : user_size(size)
{
    data = allocData(user_size, internal_size);
    constructRange(data, 0, user_size);
}


template<class T, uint LEN> inline
ShortVec<T,LEN>::ShortVec(uint size, const T &fill_val) : user_size(size)
{
    // 1. �����ڴ棻
    data = allocData(user_size, internal_size);

    // 2. Ԫ�ظ�ֵ��
    for(uint i=0; i<user_size; i++)
        new (&data[i]) T(fill_val);
}


template<class T, uint LEN> inline
ShortVec<T,LEN>::ShortVec(const ShortVec<T,LEN> &cp) : user_size(cp.user_size)
{
    data = allocData(user_size, internal_size);
    // copy actual data over
    copyConstructRange(cp.data, data, 0, user_size);
}


/*template<class T, uint LEN> inline
ShortVec<T,LEN>::ShortVec(ShortVec<T,LEN> &&cp)
{
    user_size = cp.user_size;
    internal_size = cp.internal_size;
    data = cp.data;
    cp.user_size = 0; // ensure that no destructors are called
    cp.internal_size = 0; // ensure that pool free is called
    cp.data = NULL; // on a null pointer, which will do nothing
}*/
template<class T, uint LEN> inline
ShortVec<T,LEN>::~ShortVec()
{
    destructRange(data, 0, user_size);
    deallocData(data, internal_size);
}


template<class T, uint LEN> inline
ShortVec<T,LEN>& ShortVec<T,LEN>::operator=(const ShortVec<T,LEN> &vec)
{
    uint old_size = user_size;
    
    // ensure there is enough space allocated at the destination
    resizeHelper(vec.user_size);
    
    // copy assignment for all data in range overlap
    for(uint i=0; i<std::min(vec.user_size, old_size); i++)
        data[i] = vec.data[i];
    
    // if the new range is larger, copy construct the portion
    // outside of the old range
    if(vec.user_size > old_size) {
        copyConstructRange(vec.data, data, old_size, vec.user_size);
    }
    
    // if the new range is smaller, destruct old unused entries
    if(vec.user_size < old_size)
        destructRange(data, vec.user_size, old_size);
    
    return *this;
}


template<class T, uint LEN> inline
void ShortVec<T,LEN>::resize(uint newsize) 
{
    uint oldsize = user_size;
    
    resizeHelper(newsize);
    
    if(oldsize < newsize)
        constructRange(data, oldsize, newsize);
    
    if(newsize < oldsize)
        destructRange(data, newsize, oldsize);
}


template<class T, uint LEN> inline
void ShortVec<T,LEN>::push_back(const T &datum)
{
    uint i = user_size;
    resizeHelper(user_size+1); // make room
    new (&(data[i])) T(datum);
}


//  �������в���ֵΪval��Ԫ�أ��ҵ��Ļ���������ɾ��������ЩԪ��    ������shortVecʹ��erase()ʱ������ı䣬��forѭ����ʹ��erase()���׳���
template<class T, uint LEN> inline
void ShortVec<T,LEN>::erase(const T &val)
{
    for(uint i=0; i<user_size; i++) 
    {
        if(data[i] == val) 
        {
            std::swap(data[i], data[user_size-1]);
            resize(user_size-1);
            break;
        }
    }
}



