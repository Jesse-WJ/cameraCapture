#ifndef THREADPOOL_H
#define THREADPOOL_H

#include <list>
#include <cstdio>
#include <exception>
#include <pthread.h>
#include "locker.h"


template <typename T>
class threadpool
{
public:
    /*thread_number是线程池中线程的数量，max_requests是请求队列中最多允许的、等待处理的请求的数量*/
    threadpool(int thread_number = 8, int max_request = 10000);
    ~threadpool();
    bool append_p(T &request);
    bool append_f(T &request);

private:
    /*工作线程运行的函数，它不断从工作队列中取出任务并执行之*/
    static void *worker(void *arg);
    static void *worker2(void *arg);
    void run();
    void show();

private:
    int m_thread_number;        //线程池中的线程数
    int m_max_requests;         //请求队列中允许的最大请求数
    pthread_t *m_threads;       //描述线程池的数组，其大小为m_thread_number
    std::list<T> m_workqueue;   //请求队列
    std::list<T> m_framequeue;   //请求队列
    locker m_queuelocker;       //保护请求队列的互斥锁
    locker m_framelocker;       //保护请求队列的互斥锁
    sem m_queuestat;            //是否有任务需要处理
    sem m_framequeuestat;            //是否有任务需要处理
    bool EnableRun;
};

template <typename T>
threadpool<T>::threadpool( int thread_number, int max_requests) : m_thread_number(thread_number), m_max_requests(max_requests), m_threads(NULL)
{
    if (thread_number <= 0 || max_requests <= 0)
        throw std::exception();
    m_threads = new pthread_t[m_thread_number];
    if (!m_threads)
        throw std::exception();
    EnableRun = true;
    for (int i = 0; i < thread_number-1; ++i)
    {
        if (pthread_create(m_threads + i, NULL, worker, this) != 0)
        {
            delete[] m_threads;
            throw std::exception();
        }
        if (pthread_detach(m_threads[i]))
        {
            delete[] m_threads;
            throw std::exception();
        }
    }
    if (pthread_create(m_threads + thread_number-1, NULL, worker2, this) != 0)
    {
        delete[] m_threads;
        throw std::exception();
    }
    if (pthread_detach(m_threads[thread_number-1]))
    {
        delete[] m_threads;
        throw std::exception();
    }

    printf("%d threads created!\n",thread_number);
}

template <typename T>
threadpool<T>::~threadpool()
{
    EnableRun = false;
    for(int i=0;i<m_thread_number-1;++i)
        m_queuestat.post();
    m_framequeuestat.post();
    delete[] m_threads;
    printf("delete threadpool<T>::m_threads.\n");
}

template <typename T>
bool threadpool<T>::append_p(T &request)
{
    m_queuelocker.lock();
    if (m_workqueue.size() >= m_max_requests)
    {
        m_queuelocker.unlock();
        return false;
    }
    m_workqueue.push_back(request);
    m_queuelocker.unlock();
    m_queuestat.post();
    // printf("m_workqueue:%d\n",m_workqueue.size());
    return true;
}

template <typename T>
bool threadpool<T>::append_f(T &request)
{
    m_framelocker.lock();
    if (m_framequeue.size() >= m_max_requests)
    {
        m_framelocker.unlock();
        return false;
    }
    m_framequeue.push_back(request);
    m_framelocker.unlock();
    m_framequeuestat.post();
    // printf("m_framequeue%d\n",m_framequeue.size());
    return true;
}

template <typename T>
void *threadpool<T>::worker(void *arg)
{
    threadpool *pool = (threadpool *)arg;
    pool->run();
    printf("thread[%d] closed.\n",pthread_self());
    return pool;
}

template <typename T>
void *threadpool<T>::worker2(void *arg)
{
    threadpool *pool = (threadpool *)arg;
    pool->show();
    printf("thread[%d] closed.\n",pthread_self());
    return pool;
}

template <typename T>
void threadpool<T>::run()
{
    while (EnableRun)
    {
        m_queuestat.wait();
        if(!EnableRun)
            break;
        m_queuelocker.lock();
        if (m_workqueue.empty())
        {
            m_queuelocker.unlock();
            continue;
        }
        T request = m_workqueue.front();
        m_workqueue.pop_front();
        m_queuelocker.unlock();
        if (!request.empty())
            request.process();
    }
}

template <typename T>
void threadpool<T>::show()
{
    while (EnableRun)
    {
        m_framequeuestat.wait();
        if(!EnableRun)
            break;
        m_framelocker.lock();
        if (m_framequeue.empty())
        {
            m_framelocker.unlock();
            continue;
        }
        T request = m_framequeue.front();
        m_framequeue.pop_front();
        m_framelocker.unlock();
        if (!request.empty())
            request.show();
    }
}

#endif
