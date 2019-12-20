//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_COPYNODES_H
#define BASIC_ALGORITHM_LIST_COPYNODES_H

template<typename T>
void List<T>::copyNodes(listnodePosi(T) p, int n){
    init();
    while(n--){
        insertAsLast(p->data);
        p = p->succ;
    }
}

#endif //BASIC_ALGORITHM_LIST_COPYNODES_H
