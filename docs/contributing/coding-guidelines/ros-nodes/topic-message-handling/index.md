# Topic message handling guideline

## Introduction

Here is coding guideline for a topic message handling method which includes more enhanced method than usually used one, which is roughly explained in [Discussions page](https://github.com/orgs/autowarefoundation/discussions/4612). Refer to the page to understand the basic concept of the enhanced method.
You can find sample source codes in [ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples) referred from this document.

## Typical message handling method usually used

At first, let us see a typical message handling method usually used.
[ROS 2 Tutorials](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node) is one of the most cited references for ROS 2 applications, including Autoware.
It implicitly recommends that messages received by subscriptions should be referred and processed by a dedicated callback function. Autoware follows that manner thoroughly.

```c++
  steer_sub_ = create_subscription<SteeringReport>(
    "input/steering", 1,
    [this](SteeringReport::SharedPtr msg) { current_steer_ = msg->steering_tire_angle; });
```

In code above, when a topic message whose name is `input/steering` is received, an anonymous function whose description is `{current_steer_ = msg->steering_tier_angle;}` is executed as a callback in a thread. The callback function is always executed when the message is received, which leads to waste computing resource if the message is not always necessary. Besides, it costs thread-wakeup overhead just to take a message.

## Enhanced method

There is an enhanced method to take a message using `Subscription->take()` method only when it is needed.
The sample code given below shows that `Subscription->take()` method is called during execution of any callback function. In most cases, `Subscription->take()` method is called before a received message is consumed by a main logic.
In this case, a topic message is not obtained by a subscription callback. To be exact, you need to program your code so that a callback function is not called automatically.

```c++
  SteeringReport::SharedPtr msg;
  rclcpp::MessageInfo msg_info;
  if (sub_->take(msg, msg_info)) {
    // processing and publishing after this
```

By using this manner, following advantages are achieved.

- It can reduce invocations of subscription callback functions
- There is no need to take a topic message, which a main logic does not consume, from a subscription
- There is no mandatory thread waking for the callback function, which leads to multi-threaded programming, data races and exclusive locking

## Methods to handle topic message data

This section introduces four manners including the recommended ones.

### 1. obtain data by calling `Subscription->take()`

To use the enhanced method using `Subscription->take()`, you need to do two things below basically.

1. Prevent calling a callback function when a topic message is received
2. Call `take()` method of Subscription object when a topic message is needed

You can see an example of the typical usage of `take()` method in [ros2_subscription_examples/simple_examples/src
/timer_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp).

#### prevent calling a callback function

It is needed to create a Subscription object with a callback group which makes registered callback functions not called automatically.
Note that it is needed to register a callback function to Subscription object even if the function will not be called. If you register `nullptr` instead of a callback function, it can not be built.
Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp).

```c++
    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    if (use_transient_local) {
      qos = qos.transient_local();
    }

    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, not_executed_callback, subscription_options);
```

In code above, `cb_group_noexec` is created by calling `create_callback_group` with its second argument `false`. A callback function which belongs to the callback group will not be called by Executor.
If `callback_group` member of `subscription_options` is set to `cb_group_noexec`, `not_executed_callback` will not be called when a corresponded topic message `chatter` is received.
The second argument of `create_callback_group` is defined as below.

```c++
rclcpp::CallbackGroup::SharedPtr create_callback_group(rclcpp::CallbackGroupType group_type, \
                                  bool automatically_add_to_executor_with_node = true)
```

If `automatically_add_to_executor_with_node` is set to `true`, callback functions included in a node which is added to Executor will be called automatically by Executor.

#### call `take()` method of Subscription object

Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp) in which `take()` method is used.

```c++
  std_msgs::msg::String msg;
  rclcpp::MessageInfo msg_info;
  if (sub_->take(msg, msg_info)) {
    RCLCPP_INFO(this->get_logger(), "Catch message");
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In code above, `take(msg, msg_info)` is called in a timer driven callback function by `sub_` instance which is created from Subscription class. `msg` and `msg_info` indicate a message body and meta data of it respectively. When `take(msg, msg_info)` is called, if there is a message in Subscription Queue, then the message is copied to `msg`.
`take(msg, msg_info)` returns `true` if a message is received from Subscription successfully. In this case in code above, a content of the message is outputted by `RCLCPP_INFO`.
`take(msg, msg_info)` returns `false` if a message is not received from Subscription.
When `take(msg, msg_info)` is called, if the size of Subscription Queue is larger than one and there are two or more messages in the queue, then the oldest message is copied to `msg`. If the size of Queue is one, the latest message is always obtained.

Note 1. `take()` method is irreversible in terms that a obtained message by `take()` method can not be returned to Subscription Queue. **You can determine whether a message is in Subscription Queue or not by the return value of `take()` method, but it changes Subscription queue state.** If you want to know whether there is a message in Subscription Queue or not, you can use `rclcpp::WaitSet`.
Refer to [[supplement] Use rclcpp::WaitSet](./02-supp-waitset.md) for more detail.

Note 2. You can use `take()` method to obtain a message which is passed through DDS as an inter-process communication. You can not use it for an intra-process communication. Refer to [[supplement] Obtain a received message through intra-process communication](./01-supp-intra-process-comm.md) in case of intra-process communication.

#### 1.1 obtain Serialized Message from Subscription

ROS 2 provides Serialized Message function which supports communication with arbitrary message types as explained in [Class SerializedMessage](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SerializedMessage.html). It is used by `topic_state_monitor` in Autoware.
You need to use `take_serialized()` method instead of `take()` method to obtain a message of Serialized Message type from Subscription object.

Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener_serialized_message.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener_serialized_message.cpp).

```c++
      // receive the serialized message.
      rclcpp::MessageInfo msg_info;
      auto msg = sub_->create_serialized_message();

      if (sub_->take_serialized(*msg, msg_info) == false) {
        return;
      }
```

In code above, `msg` is created by `create_serialized_message()` to store a received message, whose type is `std::shared_ptr<rclcpp::SerializedMessage>`. You can obtain a message of Serialized Message type by `take_serialized()` method. Note that `take_serialized()` method needs reference type data as its first argument therefore you need to convert `msg` type using `*`.

### 2. obtain multiple data stored in Subscription Queue

Subscription can hold multiple messages in its queue. Such messages can be obtained by consecutive calls of `take()` method from a callback function at a time. Note that in a normal manner in which a subscription callback function is used to take a topic message, if there are one or more messages in Subscription Queue, the oldest one is taken and a thread is assigned to execute a callback function, which continues until the queue becomes empty. If you use `take()` method, you can obtain multiple messages at a time.

Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_batch_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_batch_listener.cpp) in which `take()` method is called consecutively.

```c++
      std_msgs::msg::String msg;
      rclcpp::MessageInfo msg_info;
      while (sub_->take(msg, msg_info))
      {
        RCLCPP_INFO(this->get_logger(), "Catch message");
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In code above, `while(sub->take(msg, msg_info))` continues to take messages from Subscription Queue until the queue becomes empty. While a message is taken, the message is processed each by each.
Note that you need to determine Subscription Queue size considering frequency of an invocation of a callback function and that of a reception of message. For example, if frequency of an invocation of a callback function is 10Hz and that of a reception of message is 50Hz, Subscription Queue size needs to be at least 5 for messages not to be lost.

To assign a thread and let it execute a callback function each time a message is received leads to increase overhead in terms of performance. You can use the method introduced in this section to avoid the overhead.
It is effective to use the method if a reception of message occurs very frequently but it doesn't need to be processed with so high frequency. For example, if a message is received with more than 100 Hz such as CAN message, it is desirable to obtain multiple messages in a callback function at a time with one thread invocation.

### 3. obtain data by calling `Subscription->take` and then call a callback function

If you want to use the enhanced method introduced in this guideline, you may feel it is needed to implement a new function in which a message is taken by `take()` method and then processed. But there is a way to make a callback function be called indirectly which is registered to Subscription object.
Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener_using_callback.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener_using_callback.cpp).

```c++
      auto msg = sub_->create_message();
      rclcpp::MessageInfo msg_info;
      if (sub_->take_type_erased(msg.get(), msg_info)) {
        sub_->handle_message(msg, msg_info);

```

In code above, a message is taken by `take_type_erased()` method before calling a registered callback function. Note that you need to use `take_type_erased()` instead of `take()` and that `take_type_erased()` needs `void` type data as its first argument. You need to use `get()` method to convert `msg` whose type is `shared_ptr<void>` to `void` type. Then `handle_message()` method is called with the obtained message. A registered callback function is called inside of `handle_message()`.
You don't need to take special care of message type which is passed to `take_type_erased()`, the same as `take_type_erased()` and `handle_message()` are not based on any specific types. You can define message variable as `auto msg = sub_->create_message();`.
You can also refer to [API document](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SubscriptionBase.html#_CPPv4N6rclcpp16SubscriptionBase16take_type_erasedEPvRN6rclcpp11MessageInfoE) as for `create_message()`, `take_type_erased()` and `handle_message()`.

### 4. obtain data by a callback function

A typical method usually used is also usable, by which a message is obtained in subscription callback function . If you don't use a callback group with `automatically_add_to_executor_with_node = false`, a registered callback function is called automatically by Executor when a topic message is received.
One of advantages of this method is you don't need to take care whether a topic message is passed through inter-process or intra-process. Remind that in the enhanced method, `take()` can only be used in case of inter-process communication while `take_type_erased()` needs to be used in case of intra-process communication.

## Appendix

A callback function is used to obtain a topic message in many of ROS 2 applications as if it is a rule or a custom. As we have seen, you can use `Subscription->take()` method to obtain a topic message without calling a subscription callback function.
This method is also documented in [Template Class Subscription — rclcpp 16.0.8 documentation](https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Subscription.html#_CPPv4N6rclcpp12Subscription4takeER14ROSMessageTypeRN6rclcpp11MessageInfoE).
Many of ROS 2 users may feel anxious to use `take()` method because they may not be so familiar with it, but it is widely used in Executor implementation as in [rclcpp/executor.cpp](https://github.com/ros2/rclcpp/blob/47c977d1bc82fc76dd21f870bcd3ea473eca2f59/rclcpp/src/rclcpp/executor.cpp#L643-L648) shown below. Therefore it turns out that you use `take()` method indirectly whether you know it or not.

```c++
    std::shared_ptr<void> message = subscription->create_message();
    take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() {subscription->handle_message(message, message_info);});
```

Note: Strictly speaking, `take_type_erased()` method is called in Executor implementation instead of `take()`.
But `take_type_erased()` is called inside of `take()`.

If Executor is programmed to call a callback function, Executor itself determines when to do it. Because Executor calls a callback function with best-effort basis basically, it can occur that a message is not referred to or processed when it is needed in a node. Therefore it is desirable to call `take()` method directly **to ensue that a message is referred to or processed at the intended time.**
