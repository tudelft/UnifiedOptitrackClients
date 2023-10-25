#ifndef H_THREAD_SAFE_FIXED_SIZE_DEQUE
#define H_THREAD_SAFE_FIXED_SIZE_DEQUE
#include <deque>
#include <vector>
#include <mutex>
template<class T>
class ThreadSafeFixedSizeDeque {
public:

    ThreadSafeFixedSizeDeque(unsigned int max_size=180);

    /* Pushes back a new object in a thread safe manner */
    void push_back(const T &t);
    
    /* Clears all values inside the deque */
    void clear();

    /* Get object at index i in a threadsafe manner */
    T get(unsigned int i);

    /* Get last element of the deque in a threadsafe manner */
    T back();

    /* Get a copy of the full data in a threadsafe manner */
    std::vector<T> get_data();
private:
    std::deque<T>               _deque;
    std::mutex                  _mutex;

    const unsigned int _MAX_SIZE;
};  

#endif //H_THREAD_SAFE_FIXED_SIZE_DEQUE