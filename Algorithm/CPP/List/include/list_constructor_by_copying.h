//
// Created by Robotics_qi on 2019-11-21.
//

#ifndef BASIC_ALGORITHM_LIST_CONSTRUCTOR_BY_COPYING_H
#define BASIC_ALGORITHM_LIST_CONSTRUCTOR_BY_COPYING_H

template<typename T>
List<T>::List(listnodePosi(T) p, int n){
    copyNodes(p, n);
}

template<typename T>
List<T>::List(List<T> const &L){
    copyNodes(L.first(), L._size);
}

template<typename T>
List<T>::List(List<T> const& L, int r, int n){
    copyNodes(L[r], n);
}

#endif //BASIC_ALGORITHM_LIST_CONSTRUCTOR_BY_COPYING_H
