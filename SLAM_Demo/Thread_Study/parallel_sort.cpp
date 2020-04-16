//
// Created by Robotics_qi on 2020/3/28.
//

#include <stdio.h>
#include <list>
#include <future>
#include <vector>

template<typename T>
struct sorter{
    struct chunk_to_sort{
        std::list<T> data;
        std::promise<std::list<T> > promise;
    };

    std::vector<chunk_to_sort> chunks;
    std::vector<std::thread> threads;
    unsigned const max_thread_count;
    std::atomic<bool> end_of_data;

    sorter():max_thread_count(std::thread::hardware_concurrency() - 1), end_of_data(false){}

    ~sorter(){
        end_of_data = true;
        for(unsigned i = 0; i < threads.size(); ++i)
            threads[i].join();
    }

    void try_sort_chunk(){
        boost::shared_ptr<chunk_to_sort> chunk = chunks.pop();
        if(chunk){
            sort_chunk(chunk);
        }
    }

    std::list<T> do_sort()
};