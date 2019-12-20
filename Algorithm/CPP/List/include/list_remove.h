//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_REMOVE_H
#define BASIC_ALGORITHM_LIST_REMOVE_H

template<typename T>
T List<T>::remove(listnodePosi(T) p){
    T e = p->data;
    p->pred->succ = p->succ;
    p->succ->pred = p->pred;
    delete p; _size--;
    return e;
}

#endif //BASIC_ALGORITHM_LIST_REMOVE_H
