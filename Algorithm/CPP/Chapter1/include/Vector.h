//
// Created by Robotics_qi on 2021/2/10.
//

#ifndef VECTOR_STUDY_VECTOR_H
#define VECTOR_STUDY_VECTOR_H

#include <memory>

template<typename Item>
class Vector{
public:
    //!@brief 拷贝构造函数
    Vector() {}

    //!@brief 拷贝控制
    Vector(const Vector &s){
        auto newdata = alloc_n_copy(s.begin(), s.end());
        elements = newdata.first;
        first_free = cap = newdata.second;
    }

    Vector &operator=(const Vector& rhs){
        auto newdata = alloc_n_copy(rhs.begin(), rhs.end());
        free();
        elements = newdata.first;
        first_free = cap = newdata.second;
        return *this;
    }

    Vector(Vector &&s) noexcept
        : elements(s.elements), first_free(s.first_free), cap(s.cap){
        s.elements = s.first_free = s.cap = nullptr;
    }

    Vector &operator=(Vector &&rhs) noexcept{
        if(this != rhs){
            // 先释放本身内存
            free();

            // 将rhs中的元素赋值到当前集合中
            elements = rhs.elements;
            first_free = rhs.first_free;
            cap = rhs.cap;

            // 将rhs中的元素释放
            rhs.elements = rhs.first_free = rhs.cap = nullptr;
        }
        return *this;
    }

    //!@brief 释放本身元素
    ~Vector() noexcept{
        free();
    }

    //!@brief 列表赋值
    Vector &operator=(std::initializer_list<Item> il){
        auto newdata = alloc_n_copy(il.begin(), il.end());
        free();
        elements = newdata.first;
        first_free = cap = newdata.second;
    }

    //!@brief 添加元素
    void push_back(const Item& s){
        chk_n_alloc();
        alloc.construct(first_free++, s);
    }

    void push_back(Item &&s){
        chk_n_alloc();
        alloc.construct(first_free++, std::move(s));
    }

    //TODO 缺省参数的用法
    //TODO Emplace_back和push_back的区别支持
    template<class ...Args>
    void emplace_back(Args&&... args){
        chk_n_alloc();
        alloc.construct(first_free++, std::forward<Args>(args)...);
    }

    /************************************************
     * !@brief 内存和容量
     ***********************************************/
     size_t size() const{ return first_free - elements; }
     size_t capacity() const { return cap - elements; }

    //!@brief 元素访问
    Item &operator[](size_t n) { return elements[n]; }
    const Item &operator[](size_t n) const { return elements[n];}

    //!@brief 迭代器接口
    Item* begin() const{ return elements; }
    Item* end()   const{ return first_free; }
private:
    // 需要弄清楚std::allocator的用法
    static std::allocator<Item> alloc;

    //!@brief 检查内存中是否还有空间，如果没有的话，进行重新分配
    void chk_n_alloc(){
        if(first_free == cap)
            reallocate();
    }

    std::pair<Item*, Item*> alloc_n_copy(const Item *b, const Item *e){
        auto data = alloc.allocate(e - b);
        return {data, std::uninitialized_copy(b, e, data)};
    }

    //!@brief 销毁元素，释放内存
    void free(){
        // 用逆序的方式先删除最老的元素
        for(auto p = first_free; p != elements; )
            alloc.destroy(--p);

        // 重新分配内存
        if(elements)
            alloc.deallocate(elements, cap-elements);
    }

    void reallocate(){
        auto newcapacity = size() ? 2 * size() : 2;

        // 分配新的空间
        auto first = alloc.allocate(newcapacity);
        auto dest = first;
        auto elem = elements;

        // 移动元素
        for(size_t i = 0; i!= size(); ++i)
            alloc.construct(dest++, std::move(*elem++));
        free();

        elements = first;
        first_free = dest;
        cap = elements + newcapacity;
    }

    Item *elements = nullptr;                //!@brief 指向数组首元素地址
    Item *first_free = nullptr;              //!@brief 指向数组中第一个空闲元素
    Item *cap = nullptr;                    //!@brief 指向数组尾后位置指针
};
//!@brief 静态成员变量定义
template<typename Item>
std::allocator<Item> Vector<Item>::alloc;
#endif //VECTOR_STUDY_VECTOR_H
