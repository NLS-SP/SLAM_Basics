//
// Created by Robotics_qi on 2019-11-17.
//

#ifndef BASIC_ALGORITHM_VECTOR_MERGE_H
#define BASIC_ALGORITHM_VECTOR_MERGE_H

template<typename T>
void Vector<T>::merge(Rank lo, Rank mi, Rank hi){
    T *A = _elem+lo;
    int lb = mi - lo; T *B = new T[lb];
    for(Rank i = 0; i < lb; i++) B[i] = A[i];
    int lc = hi - mi; T *C = _elem + mi;
    for(Rank i =  0, j = 0, k = 0; (j < lb) || (k < lc);){
        if((j < lb) && ( lc <= k || (B[j] <= C[k]))) A[i++] = B[j++];
        if((k < lc) && ( lb <= j || (C[k] <  B[j]))) A[i++] = C[k++];
    }
    delete[] B;
}

#endif //BASIC_ALGORITHM_VECTOR_MERGE_H
