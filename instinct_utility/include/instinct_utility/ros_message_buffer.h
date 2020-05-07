#pragma once
#include <boost/circular_buffer.hpp>
#include <ros/ros.h>
#include <mutex>

/**
 * Automatic message buffer for ROS topics that uses a fixed size circular buffer to store incoming messages.
 * Handles subscription and processing of topics using callbacks.
 *
 * @tparam T ROS topic message type
 */
template <typename T>
class RosMessageBuffer
{
  public:
    /**
     * Create a new message buffer with a internal buffer of bufferSize elements
     * @param bufferSize Number of elements to store
     */
    RosMessageBuffer(int bufferSize)
    {
        messageBuffer_.set_capacity(bufferSize);
    }

    virtual ~RosMessageBuffer()
    {
        shutdown();
    }

    /**
     * Subscribe the buffer to a new ROS topics.
     *
     * NOTE: Shutdown must be called before calling this again.
     *
     * @param topic Topic ID
     * @param handle ROS handle to use when subscribing
     */
    void subscribe(const std::string& topic, ros::NodeHandle& handle)
    {
        this->topicsSub_ = handle.subscribe<T>(topic, 5, &RosMessageBuffer::newMessage, this);
        handle_ = handle;
    }

    /**
     * Shutdown the active topic subscription
     */
    void shutdown()
    {
        topicsSub_.shutdown();
    }

    /**
     * @return The internal buffer holding the message references
     */
    boost::circular_buffer<boost::shared_ptr<T const>> getBufferedMessages()
    {
        return messageBuffer_;
    }

    /**
     * Get the most recent message from the buffer if one exists.
     * If no message exists wait until one is published for the given topic.
     *
     * @return Most recent topic message
     */
    const boost::shared_ptr<T const> getRecentOrWait()
    {
        if (messageBuffer_.empty())
        {
            return ros::topic::waitForMessage<T>(topicsSub_.getTopic(), handle_);
        }
        else
        {
            std::lock_guard<std::mutex> guard(mx_);
            return messageBuffer_.back();
        }
    }

    /**
     * @return Most recent topic message in the buffer
     */
    const boost::shared_ptr<T const> getMostRecent()
    {
        std::lock_guard<std::mutex> guard(mx_);
        return messageBuffer_.back();
    }

    /**
     * @return Current buffer size
     */
    const int size()
    {
        return messageBuffer_.size();
    }

  private:
    std::mutex mx_;
    boost::circular_buffer<boost::shared_ptr<T const>> messageBuffer_;
    ros::Subscriber topicsSub_;
    ros::NodeHandle handle_;

    /**
     * Callback to push new messages into the internal buffer
     */
    void newMessage(const boost::shared_ptr<T const>& msg)
    {
        // Mutex lock releases on deconstuction at the end of scope
        // Ensures release if an error occurs in the function
        std::lock_guard<std::mutex> guard(mx_);
        messageBuffer_.push_back(msg);
    }
};