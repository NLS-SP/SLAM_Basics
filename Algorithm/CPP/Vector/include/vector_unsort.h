//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_UNSORT_H
#define BASIC_ALGORITHM_VECTOR_UNSORT_H

template<typename T>
void Vector<T>::unsort(Rank lo, Rank hi){
    T* V = _elem + lo;
    for(Rank i = hi - lo; i > 0; i--)
        swap(V[i-1], V[rand() % i]);
}

#endif //BASIC_ALGORITHM_VECTOR_UNSORT_H
