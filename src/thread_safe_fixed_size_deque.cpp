#include "thread_safe_fixed_size_deque.hpp"

template<class T>
ThreadSafeFixedSizeDeque<T>::ThreadSafeFixedSizeDeque(unsigned int max_size) :
    _MAX_SIZE(max_size)
{
}

template<class T>
void ThreadSafeFixedSizeDeque<T>::push_back(const T &t) 
{    
    /* Lock for thread safety*/
    std::lock_guard<std::mutex> lock{ this->_mutex }; 

    /* If we reached the size limitation we first need to remove an object */
    if(this->_deque.size() >= this->_MAX_SIZE)
        this->_deque.pop_front();
    /* Add the new element*/
    _deque.push_back(t); 
}

template<class T>
void ThreadSafeFixedSizeDeque<T>::clear()
{
    std::lock_guard<std::mutex> lock{ this->_mutex }; 
    this->_deque.clear();
}

template<class T>
T ThreadSafeFixedSizeDeque<T>::get(unsigned int i)
{
    /* Lock for thread safety*/
    std::lock_guard<std::mutex> lock{ this->_mutex };     

    return this->_deque.at(i);
}

template<class T>
T ThreadSafeFixedSizeDeque<T>::back()
{
    /* Lock for thread safety*/
    std::lock_guard<std::mutex> lock{ this->_mutex };     

    return this->_deque.back();
}

template<class T>
std::vector<T> ThreadSafeFixedSizeDeque<T>::get_data()
{
    /* Lock for thread safety*/
    std::lock_guard<std::mutex> lock{ this->_mutex }; 

    /* Return the data as vector */
    return std::vector<T>(this->_deque.begin(), this->_deque.end());
}



