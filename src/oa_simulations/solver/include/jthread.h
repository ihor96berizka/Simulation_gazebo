#pragma once

#include <thread>
#include <functional>
#include <atomic>


namespace tools{
class jthread
{
public:
    jthread(std::function<void()> callback);
    void request_stop();
    ~jthread();

private:
    std::unique_ptr<std::thread> thread_;
    std::atomic<bool> stop_token_{false};
    std::function<void()> callback_;
};
}