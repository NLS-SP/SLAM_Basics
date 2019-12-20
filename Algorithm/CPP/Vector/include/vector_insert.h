//
// Created by Robotics_qi on 2019-11-15.
//

#ifndef BASIC_ALGORITHM_VECTOR_INSERT_H
#define BASIC_ALGORITHM_VECTOR_INSERT_H

template<typename T>
Rank Vector<T>::insert(Rank r, T const& e){
    expand();
    for(int i = _size; i > r; i--) _elem[i] = _elem[i-1];
    _elem[r] = e; _size++;
    return r;
}

#endif //BASIC_ALGORITHM_VECTOR_INSERT_H
