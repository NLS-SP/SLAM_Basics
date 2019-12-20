//
// Created by Robotics_qi on 2019-11-15.
//

#ifndef BASIC_ALGORITHM_VECTOR_CONSTRUCTOR_BY_COPYING_H
#define BASIC_ALGORITHM_VECTOR_CONSTRUCTOR_BY_COPYING_H

template<typename T>
void Vector<T>::copyFrom(T const *A, Rank lo, Rank hi) {
    _elem = new T[_capacity = 2 * (hi - lo)];   _size = 0;
    while(lo < hi)
        _elem[_size++] = A[lo++];
}

#endif //BASIC_ALGORITHM_VECTOR_CONSTRUCTOR_BY_COPYING_H
