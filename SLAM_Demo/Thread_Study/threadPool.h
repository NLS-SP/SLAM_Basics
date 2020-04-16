//
// Created by Robotics_qi on 2020/3/20.
//

#ifndef THREAD_STUDY_THREADPOOL_H
#define THREAD_STUDY_THREADPOOL_H

#include <functional>
#include <thread>
#include <atomic>
#include <vector>
#include <memory>
#include <exception>
#include <future>
#include <mutex>
#include <boost/lockfree/queue.hpp>

#define  thread_pool_length 100

namespace threadPool{
    class thread_Pool{
    public:
        thread_Pool() : q(thread_pool_length) { this->init(); }
        thread_Pool(int nThreads, int queue_size = thread_pool_length) : q(queue_size){
            this->init();
            this->resize(nThreads);
        }

        ~thread_Pool(){ this->stop(true); }

        // get the number of running threads in the pool.
        int size() { return static_cast<int>(this->threads.size());}

        // number of waiting threds.
        int n_idle() { return this->nWaiting; }
        std::thread& get_thread(int i) { return *this->threads[i]; }

        // change the number of threads in the pool
        // should be called from one thread, otherwise be careful to not interleave, also with this->stop()
        // nThreads must be >= 0
        void resize(int nThreads){
            if(!this->isStop && !this->isDone){
                int oldNThreads = static_cast<int>(this->threads.size());
                if(oldNThreads <= nThreads){        // if the number of threads is increased.
                    this->threads.resize(nThreads);
                    this->flags.resize(nThreads);
                    fo(int i = oldNThreads; i < nThreads; ++i){
                        this->flags[i] = std::make_shared<std::atomic<bool> >(false);
                        this->set_thread(i);
                    }
                }else{ // the number of threads is decreased.
                    for(int i = oldNThreads - 1; i >= nThreads; --i){
                        *this->flags[i] = true;     // this thread will finish.
                        this->threads[i]->detach();
                    }
                    {
                        // stop the detached threads that were waiting.
                        std::unique_lock<std::mutex> lock<this->mutex>;
                        this->cv.notify_all();
                    }
                    this->threads.resize(nThreads);
                    this->flags.resize(nThreads);
                }
            }
        }

        // empty the queue.
        void clear_queue(){
            std::function<void(int id)>* _f;
            while(this->q.pop(_f))
                delete _f;
        }

        void stop(bool is_wait = false){

        }

        void set_thread(int i){

        }


    private:

        // deleted
        thread_Pool(const thread_Pool &);
        thread_Pool(thread_Pool &&);
        thread_Pool& operator=(const thread_Pool&);
        thread_Pool& operator=(thread_Pool&&);

        void init(){
            this->nWaiting = 0;
            this->isStop = false;
            this->isDone = false;
        }

        std::vector<std::unique_ptr<std::thread> > threads;
        std::vector<std::shared_ptr<std::atomic<bool>>> flags;
        mutable boost::lockfree::queue<std::function<void(int id)> *> q;
        std::atomic<bool> isDone;
        std::atomic<bool> isStop;
        std::atomic<bool> nWaiting;

        std::mutex threads_mutex;
        std::condition_variable cv;
    };

}
#endif //THREAD_STUDY_THREADPOOL_H
