#pragma once

typedef int Rank;
#define DEFAULT_CAPACITY 3                                  // 默认初始容量(实际应用可设置更大)

template<typename T>
class Vector{
protected:
    Rank _size;                                             // 规模
    int _capacity;                                          // 容量
    T* _elem;                                               // 数据区
    void copyFrom(T const* A, Rank lo, Rank hi);            // 复制数据区间A[lo, hi)
    void expand();                                          // 空间不足时进行扩容
    void shrink();                                          // 装填因子过小压缩
    bool bubble(Rank lo, Rank hi);                          // 扫描交换
    void bubbleSort(Rank lo, Rank hi);                      // 气泡排序算法
    Rank max(Rank lo, Rank hi);                             // 选取最大元素
    void merge(Rank lo, Rank mi, Rank hi);                  // 归并算法
    void mergeSort(Rank lo, Rank hi);                           // 归并排序算法
    void selectionSort(Rank lo, Rank hi);                   // 选择排序算法


public:
    // 构造函数
    Vector(int C = DEFAULT_CAPACITY, int s = 0, T v = 0) {   // 容量为c、规模为s、所有元素初始为v
        _elem = new T[_capacity = C];
        for(_size = 0; _size < s; _elem[_size++] = v);
    }
    Vector(T const* A, Rank n){ copyFrom(A, 0, n); }
    Vector(T const* A, Rank lo, Rank hi){ copyFrom(A, lo, hi); }
    Vector(Vector<T> const& V){ copyFrom(V._elem, 0, V._size); }
    Vector(Vector<T> const& V, Rank lo, Rank hi){ copyFrom(V._elem, lo, hi); }

    // 析构函数
    ~Vector(){ delete _elem; }  // 释放内部空间
    // 只读访问接口
    Rank size() const{ return _size; }
    Rank empty() const { return !_size; }
    int disordered() const;     // 判断向量是否排序完成
    Rank find(T const& e) const{ return find(e, 0, _size); }    //无序向量的整体查找
    Rank find(T const& e, Rank lo, Rank hi) const;              // 无序向量的区间查找
    Rank search(T const& e) const{ return(_size <= 0) ? -1 : search(e, 0, _size); } // 有序向量查找
    Rank search(T const& e, Rank lo, Rank hi) const;            // 有序向量区间查找

    // 可写访问接口
    T& operator[](Rank r) const;                                // 重载下标操作符,可类似于数组形式引用各元素
    Vector<T> &operator=(Vector<T> const&);                     // 重载复制操作，可直接克隆向量
    T remove(Rank r);                                           // 删除秩为r的元素
    int remove(Rank lo, Rank hi);                               // 删除秩为[lo, hi)之内的元素
    Rank insert(Rank r, T const& e);                            // 插入元素
    Rank insert(T const& e){ return insert(_size, e); }         // 默认作为末元素插入
    void sort(Rank lo, Rank hi);                                // 对[lo, hi)排序
    void sort(){ sort(0, _size); }                              // 整体排序
    void unsort(Rank lo, Rank hi);                              // 对[lo, hi)置乱
    void unsort(){ unsort(0, _size);}                           // 整体置乱
    int deduplicate();                                          // 无序去重
    int uniquify();                                             // 有序去重

    // 遍历
    void traverse( void(*) (T&));                               // 遍历(使用函数指针, 只读或局部性修改）
    template<typename VST> void traverse(VST&);                 // 遍历(使用函数对象，可局部修改)

};

#include "vector_implementation.h"