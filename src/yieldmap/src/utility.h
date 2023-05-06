#pragma once

#include <atomic>
#include <chrono>
#include <memory>
#include <thread>


namespace utility
{

template <typename T>
class SyncedDataExchange
{

private:
    std::atomic<T *> a_ptr;

    bool is_empty() {return (a_ptr.load() == nullptr);}
    
public:
    void send(T const &_obj)
    {
        T *new_ptr = new T(std::move(_obj));

        while (!is_empty())
        {
            std::this_thread::sleep_for(std::chrono::milliseconds(3));
        }
        // 确保旧指针在函数退出时自动释放，从而避免内存泄漏
        std::unique_ptr<T> old_ptr(a_ptr.exchange(new_ptr));
    }

    void receive(T &obj)
    {
        std::unique_ptr<T> ptr;
        do
        {
            while (is_empty())
            {
                std::this_thread::sleep_for(std::chrono::milliseconds(3));
            }
            // 这个函数将原子地交换 a_ptr 中存储的指针值为 nullptr，并返回以前存储的指针值
            // 然后将 ptr 指向一个新的对象
            ptr.reset(a_ptr.exchange(nullptr));
        } while (!ptr);
        obj = std::move(*ptr);  // 这里的 std::move() 用于将左值转换为右值引用
    }

    SyncedDataExchange() : a_ptr(nullptr) {}
};

} // namespace utility