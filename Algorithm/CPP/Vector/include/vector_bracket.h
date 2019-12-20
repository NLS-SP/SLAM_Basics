//
// Created by Robotics_qi on 2019-11-15.
//

#ifndef BASIC_ALGORITHM_VECTOR_BRACKET_H
#define BASIC_ALGORITHM_VECTOR_BRACKET_H

template<typename T>
T& Vector<T>::operator[](Rank r) const{
    return _elem[r];
}

#endif //BASIC_ALGORITHM_VECTOR_BRACKET_H
