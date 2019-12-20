//
// Created by Robotics_qi on 2019-11-16.
//

#ifndef BASIC_ALGORITHM_VECTOR_SEARCH_H
#define BASIC_ALGORITHM_VECTOR_SEARCH_H

template<typename T>
Rank Vector<T>::search(T const& e, Rank lo, Rank hi) const{
    return  binSearch(_elem, e, lo, hi);
}

#endif //BASIC_ALGORITHM_VECTOR_SEARCH_H
