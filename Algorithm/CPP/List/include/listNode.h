//
// Created by Robotics_qi on 2019-11-20.
//

#ifndef BASIC_ALGORITHM_LISTNODE_H
#define BASIC_ALGORITHM_LISTNODE_H

typedef int Rank;           //秩
#define listnodePosi(T) listNode<T>*

template <typename T>
struct listNode{
    T data;                                         // 数据
    listnodePosi(T) pred;                           // 前驱
    listnodePosi(T) succ;                           // 后继

    // 构造函数
    listNode(){}
    listNode(T e, listnodePosi(T) P = NULL, listnodePosi(T) s = NULL):
            data(e), pred(p), succ(s){}  // 默认构造器

    // 操作接口
    listnodePosi(T) insertAsPred(const T &e);               // 紧靠当前节点之前插入
    listnodePosi(T) insertAsSucc(const T &e);               // 紧靠当前节点之后插入
};

#include "listnode_implementation.h"

#endif //BASIC_ALGORITHM_LISTNODE_H
