//
// Created by Robotics_qi on 2019-11-20.
//

#ifndef BASIC_ALGORITHM_LISTNODE_INSERTASSUCC_H
#define BASIC_ALGORITHM_LISTNODE_INSERTASSUCC_H

template<typename T>
listnodePosi(T) listNode<T>::insertAsSucc(const T &e) {
    listnodePosi(T) x = new listNode(e, this, succ);
    succ->pred = x; succ = x;
    return x;
}


#endif //BASIC_ALGORITHM_LISTNODE_INSERTASSUCC_H
