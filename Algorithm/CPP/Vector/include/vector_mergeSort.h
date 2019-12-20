//
// Created by Robotics_qi on 2019-11-17.
//

#ifndef BASIC_ALGORITHM_VECTOR_MERGESORT_H
#define BASIC_ALGORITHM_VECTOR_MERGESORT_H

template <typename T>
void Vector<T>::mergeSort(Rank lo, Rank hi) {
    if(hi - lo < 2) return;
    int mi = (lo + hi) >> 1;
    mergeSort(lo, mi);
    mergeSort(mi, hi);
    merge(lo, mi, hi);
}

#endif //BASIC_ALGORITHM_VECTOR_MERGESORT_H
