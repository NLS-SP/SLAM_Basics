//
// Created by Robotics_qi on 2019-11-20.
//

#ifndef BASIC_ALGORITHM_LIST_H
#define BASIC_ALGORITHM_LIST_H

#include "listNode.h"

template<typename T>
class List{                                                                 // 列表模板类
private:
    int _size;                                                              // 数据规模
    listnodePosi(T) header;                                                 // 头哨兵节点
    listnodePosi(T) tail;                                                // 尾哨兵节点

protected:
    void init();                                                            // 列表创建时候初始化
    int clear();                                                            // 清楚所有节点
    void copyNodes( listnodePosi(T), int);                                  // 复制列表中自位置p起的n项
    void selectionSort(listnodePosi(T), int);
    void insertionSort(listnodePosi(T), int);

public:
    // 构造函数
    List(){ init(); }                                                       // 默认
    List( const List<T> &L);                                                // 整体默认复制列表L
    List(listnodePosi(T) p, int n);
    List( const List<T> &L, Rank r, int n);                                 // 复制列表自r项起的n项
    // 析构函数
    ~List();                                                                // 释放所有节点(包括头哨兵和尾哨兵节点)
    // 只读访问接口
    Rank size() const { return _size; }                                     // 规模
    bool empty() const { return _size <= 0; }                               // 判断是否为空
    T& operator[](Rank r) const;                                            // 重载，支持循秩访问
    listnodePosi(T) first() const { return header->succ; }                  // 首节点位置
    listnodePosi(T) last() const { return trailer->pred; }                  // 末节点位置
    bool valid(listnodePosi(T) p){                                          // 判断当前位置对外是否合法
        return p &&( trailer != p) && (header != p);                        // 头尾节点等同于NULL
    }
    int disordered() const;                                                 // 判断列表是否已排序
    listnodePosi(T) find(const T &e) const {
        return find(e, _size, trailer);                                     // 无序列表查找
    }
    listnodePosi(T) find(const T &e, int n, listnodePosi(T) p);             // 无序区间查找
    listnodePosi(T) search(const T &e) {
        return search(e, _size, trailer);                                   // 有序列表查找
    }
    listnodePosi(T) search(const T &e, int n, listnodePosi(T) p) const;     // 有序区间查找
    listnodePosi(T) selectMax(listnodePosi(T), int n);                      // 在p及其n-1个后继中选出最大者
    listnodePosi(T) selectMax(){ return selectMax(header->succ, _size); }   // 整体最大者

    // 可写访问接口
    listnodePosi(T) insertAsFirst(const T &e);                              // 将e当做首节点插入
    listnodePosi(T) insertAsLast(const T &e);                               // 将e当做末节点插入
    listnodePosi(T) insertA(listnodePosi(T) p, const T &e);                 // 将e当做p的后继插入
    listnodePosi(T) insertB(listnodePosi(T) p, const T &e);                 // 将e当做p的前驱插入

    T remove(listnodePosi(T) p);                                            // 删除合法位置p处的节点,返回被删除的节点

    int deduplicate();                                                      // 无序去重
    int uniquify();                                                         // 有序去重
    void reverse();                                                         // 前后倒置
    //遍历
    void traverse(void(*) (T&));                                            // 遍历，依次实施visit操作（函数指针,只读或局部性更改）
    template<typename VST>
    void traverse(VST&);                                                    // 遍历，依次实施visit操作（函数对象，可全局修改)
};

#include "list_implementation.h"

#endif //BASIC_ALGORITHM_LIST_H
