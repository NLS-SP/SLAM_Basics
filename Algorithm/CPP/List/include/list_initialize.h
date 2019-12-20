//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_INITIALIZE_H
#define BASIC_ALGORITHM_LIST_INITIALIZE_H

template<typename T>
void List<T>::init(){
    header = new listNode<T>;
    trailer = new listNode<T>;
    header->succ = trailer; header->pred = NULL;
    trailer->pred = header; trailer->succ = NULL;
    _size = 0;
}

#endif //BASIC_ALGORITHM_LIST_INITIALIZE_H
