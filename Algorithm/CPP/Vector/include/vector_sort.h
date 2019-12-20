//
// Created by Robotics_qi on 2019-11-17.
//

#ifndef BASIC_ALGORITHM_VECTOR_SORT_H
#define BASIC_ALGORITHM_VECTOR_SORT_H

template<typename T>
void Vector<T>::sort(Rank lo, Rank hi){
    mergeSort(lo, hi);
}
#endif //BASIC_ALGORITHM_VECTOR_SORT_H
