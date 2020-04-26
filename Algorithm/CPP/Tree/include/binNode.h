//
// Created by Robotics_qi on 2020/4/23.
//

#ifndef BASIC_ALGORITHM_BINNODE_H
#define BASIC_ALGORITHM_BINNODE_H

#include <cstdlib>

#define BinNodePosi(T) BinNode<T>* // 节点位置
#define stature(p)( (p) ? (p)->height : -1)     // 节点高度
typedef enum {RB_RED, RB_BLACK} RBColor;    // 节点颜色

template<typename T> struct BinNode{    //二叉树节点模板类
    // 成员
    T data;// 数值
    BinNodePosi(T) parent; BinNodePosi(T) lc; BinNodePosi(T) rc;    // 父节点及左右孩子
    int height;
    int npl;    // Null Path Length(左式堆，可以直接用height来代替)
    RBColor color;  //颜色（红黑树）
    // 构造函数
    BinNode():parent(NULL), lc(NULL), rc(NULL),height(0), npl(1), color(RB_RED){}
    BinNode(T e, BinNodePosi(T) p = NULL, BinNodePosi(T) lc = NULL, BinNodePosi(T) rc = NULL, int h = 0, int l = 1,
            RBColor c = RB_RED): data(e), parent(p), lc(lc), rc(rc), height(h), npl(l), color(c){}

    // 操作接口
    int size();     // 统计当前节点后代综述，亦即以其为根的子树规模
    BinNodePosi(T) insertAsLC(T const&);
    BinNodePosi(T) insertAsRC(T const&);
    BinNodePosi(T) succ();  //取当前节点的直接后继
    template<typename VST> void travLevel(VST&);
    template<typename VST> void travPre(VST&); // 子树先序遍历
    template<typename VST> void travIn(VST&);   // 子树中序遍历
    template<typename VST> void travPost(VST&); //子树后序遍历

    // 比较器
    bool operator<(BinNode const& bn){ return data < bn.data; }
    bool operator==(BinNode const& bn) {return data == bn.data; }

    BinNodePosi(T) zig();   //顺时针旋转
    BinNodePosi(T) zag();   //逆时针旋转
};

#endif //BASIC_ALGORITHM_BINNODE_H
