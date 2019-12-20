//
// Created by Robotics_qi on 2019-11-23.
//

#ifndef BASIC_ALGORITHM_LIST_TRAVERSE_H
#define BASIC_ALGORITHM_LIST_TRAVERSE_H

template<typename T>
void List<T>::traverse(void(*visit)(T&)){
    for(listnodePosi(T) p = header->succ; p != trailer; p = p->succ) visit(p->data);
}

template<typename T>
template<typename VST>
void List<T>::traverse(VST& visit){
    for(listnodePosi(T) p = header->succ; p != trailer; p = p->succ) visit(p->data);
}

#endif //BASIC_ALGORITHM_LIST_TRAVERSE_H
