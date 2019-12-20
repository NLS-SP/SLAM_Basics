//
// Created by Robotics_qi on 2019-11-20.
//

#ifndef BASIC_ALGORITHM_LISTNODE_INSERTASPRED_H
#define BASIC_ALGORITHM_LISTNODE_INSERTASPRED_H

template<typename T>
listnodePosi(T) listNode<T>::insertAsPred(const T &e) {
    listnodePosi(T) x = new listNode(e, pred, this);
    pred->succ = x; pred = x;
    return x;
}


#endif //BASIC_ALGORITHM_LISTNODE_INSERTASPRED_H
