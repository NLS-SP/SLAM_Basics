//
// Created by Robotics_qi on 2019-11-15.
//

#ifndef BASIC_ALGORITHM_VECTOR_ASSIGNMENT_H
#define BASIC_ALGORITHM_VECTOR_ASSIGNMENT_H

template<typename T>
Vector<T>& Vector<T>::operator=(Vector<T> const& V) {
    if( _elem ) delete[] _elem;                         // 释放原有内容
    copyFrom(V._elem, 0, V.size());                  // 整体复制
    return *this;
}

#endif //BASIC_ALGORITHM_VECTOR_ASSIGNMENT_H
