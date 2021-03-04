//
// Created by Robotics_qi on 2021/2/19.
//

#ifndef VECTOR_STUDY_SORT_H
#define VECTOR_STUDY_SORT_H
#include <type_traits>
#include <random.h>
/// 实现常用排序算法，用迭代器做接口
/// iterator_traits解析
using std::iterator_traits;

template<typename BidirectionIt>
void selectionSort(BidirectionIt first, BidirectionIt end){
    typedef typename std::iterator_traits<BidirectionIt>::value_type value_type;
    if(first == end) return;
    for(auto i = first; i != end; ++i){
        auto min = i;
        for(auto j = i+1; j != end; ++j)
            if(*j < *min) min = j;
            std::swap(*i, *min);
    }
}

/// 插入排序
//!@brief 简化版，未做移动，效率较差
template<typename BidirectionalIT>
void insertionSort(BidirectionalIT first, BidirectionalIT last){
    typedef typename std::iterator_traits<BidirectionalIT>::value_type value_type;
    if(first == last) return;
    for(auto i = first + 1; i != last; ++i)
        for(auto j = i; j > first && *j < *(j-1); --j)
            std::swap(*j, *j-1);
}

template<typename BidirectionalIt>
void insertionSortX(BidirectionalIt first, BidirectionalIt last){
    typedef typename iterator_traits<BidirectionalIt>::value_type value_type;
    if(first == last) return;
    for(auto i = (first + 1) + 1; i != last; ++i){
        auto t = *i;
        auto j = i;
        for(; t < *(j - 1); --j)
            *j = std::move(*(j-1));
        *j = std::move(t);
    }
}

#endif //VECTOR_STUDY_SORT_H
