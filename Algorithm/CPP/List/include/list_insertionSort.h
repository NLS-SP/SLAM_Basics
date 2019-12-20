//
// Created by Robotics_qi on 2019-11-25.
//

#ifndef BASIC_ALGORITHM_LIST_INSERTIONSORT_H
#define BASIC_ALGORITHM_LIST_INSERTIONSORT_H

template<typename T>
void List<T>::insertionSort(listnodePosi(T) p, int n){
    for(int r = 0; r < n; r++){
        insertA(search(p->data, r, p));
        p = p->succ; remove(p->pred);
    }
}

#endif //BASIC_ALGORITHM_LIST_INSERTIONSORT_H
