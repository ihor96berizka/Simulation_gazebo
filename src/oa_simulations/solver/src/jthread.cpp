#include <jthread.h>

namespace tools
{
jthread::jthread(std::function<void()> callback)
{
    callback_ = std::move(callback);
    thread_ = std::make_unique<std::thread>([this]()
    {
        while(!stop_token_)
        {
            callback_();
        }
        
    });
}

jthread::~jthread()
{
    stop_token_.store(true);
    if (thread_->joinable())
    {
        thread_->join();
    }
}
void jthread::request_stop()
{
    stop_token_.store(true);
}

}