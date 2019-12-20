//
// Created by Robotics_qi on 2019-11-23.
//

#ifndef BASIC_ALGORITHM_LIST_INSERT_H
#define BASIC_ALGORITHM_LIST_INSERT_H

template<typename T>
listnodePosi(T) List<T>::insertAsFirst(const T &e){
    _size++;
    return header->insertAsSucc(e);
}

template<typename T>
listnodePosi(T) List<T>::insertAsLast(const T &e){
    _size++;
    return tail->insertAsPred(e);
}

template<typename T>
listnodePosi(T) List<T>::insertA(listnodePosi(T) p, const T &e){
    _size++;
    return p->insertAsSucc(e);
}

template<typename T>
listnodePosi(T) List<T>::insertB(listnodePosi(T) p, const T &e){
    _size++;
    return p->insertAsPred(e);
}

#endif //BASIC_ALGORITHM_LIST_INSERT_H
