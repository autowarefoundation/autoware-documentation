# Enhanced topic message handling guidelines

## Introduction

Here is coding guideline for an enhanced topic message handling method which is roughly explained in [Discussions page](https://github.com/orgs/autowarefoundation/discussions/4612). Refer to the page to understand the basic concept of the method.
There is sample source code in [ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples) referred from this document.

## What is Subscription->take()

### subscription callback function
As you know, ROS 2 is used as a basis of Autoware to communicate between nodes by publishing and subscription. A topic message which is received by Subscription is referred to and processed by a dedicated callback function in typical implementation using ROS 2.

```c++
  steer_sub_ = create_subscription<SteeringReport>(
    "input/steering", 1,
    [this](SteeringReport::SharedPtr msg) { current_steer_ = msg->steering_tire_angle; });
```

In code above, when a topic message whose name is `input/steering` is received, an anonymous function whose description is `{current_steer_ = msg->steering_tier_angle;}` is executed as a callback in a thread. The callback function is always executed when the message is received, which leads to waste computing resource if the message data is not always necessary.

### to avoid the waste
There is an effective way to take a message using `Subscription->take()` method when it is needed.
A callback function is used to receive topic message in many of ROS 2 applications as if it is a rule or a habit. You can use `Subscription->take()` method to get a topic message without executing a callback function.
As described in [Template Class Subscription — rclcpp 16.0.8 documentation](https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Subscription.html#_CPPv4N6rclcpp12Subscription4takeER14ROSMessageTypeRN6rclcpp11MessageInfoE), you can use `take()` method to obtain a received message through DDS as an inter-process communication. It is no need to call a callback function when using `take() method`.
Many of ROS 2 users may feel anxious to use `take()` method because it is not used so often, but it is widely used in Executor implementation as in [rclcpp/executor.cpp](https://github.com/ros2/rclcpp/blob/47c977d1bc82fc76dd21f870bcd3ea473eca2f59/rclcpp/src/rclcpp/executor.cpp#L643-L648). Therefore you use `take()` method indirectly whether you know it or not.

```c++
    std::shared_ptr<void> message = subscription->create_message();
    take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() {subscription->handle_message(message, message_info);});
```

Note: Strictly speaking, `take_type_erased()` method is called in Executor implementation instead of `take()`. `take_type_erased()` is called inside of `take()`.
The body of obtaining a message from Subscription in Executor is `take_type_erased()` strictly, but we use `take()` instead for easy exaplanation.

When Executor calls `take()` method and then a callback function, Executor itself determines when to do it. Because Executor calls a callback function with best-effort basis basically, it can occur that a message is not referred to or processed when it is needed in a node. Therefore it is desirable to call `take()` method directly **to ensue that a message is referred to or processed at the intended time.**

Note: you can use `take()` method to obtain a message through DDS as an inter-process communication. As for an intra-process communication, you can not use it. Refer to xxx in case of intra-process communication.

## typical scenario to use `Subscription->take()` method

1. obtain data by calling `Subscription->take()`
2. obtain multiple data stored in Subscription Queue by multiple calls of `Subscription->take()`
3. obtain data by calling `Subscription->take` and then call a callback function which is registered at creation of subscripition
4. obtain Serialized Message from Subscription

We will explain four scenarios above in order.

### obtain data by calling `Subscription->take()`

To obtain a received message of Subscription at the intended time using `take()` method of Subscription object, you need two changes below basically.

1. prevent calling of a callback function which is registered to Subscription object when a topic message is received
2. call `take()` method of Subscription object when a topic message is needed

You can see an example of the typical usage of `take()` method in [ros2_subscription_examples/simple_examples/src
/timer_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp).

#### prevent calling a callback function

It is needed to register a callback function to Subscription object when it is created using `create_subscritpion` even if the function will not be called. If you register `nullptr` instead of a callback function, it can not be built.
You need to use a callback group to prevent calling a callback function.
Here is a sample code excerpted from  [ros2_subscription_examples/simple_examples/src/timer_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp).

```c++
    rclcpp::CallbackGroup::SharedPtr cb_group_noexec = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false); // create callback group for not executing callback automatically
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_noexec;
    rclcpp::QoS qos(rclcpp::KeepLast(10));
    if (use_transient_local) { 
      qos = qos.transient_local();
    }
    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, not_executed_callback, subscription_options);
```

In code above, `cb_group_noexec` is created by calling `create_callback_group` with its second argument `false`. A callback function which belongs to the callback group will not be called by Executor.
If `callback_group` of `subscription_options` is set to `cb_group_noexec`, `not_executed_callback` which is registered to Subscription object will not be called when a corresponded topic message `chatter` is received.
The second argument of `create_callback_group` is defined as below.

```c++
rclcpp::CallbackGroup::SharedPtr create_callback_group(rclcpp::CallbackGroupType group_type, \
                                  bool automatically_add_to_executor_with_node = true)
```

If `automatically_add_to_executor_with_node` is set to `true`, callback functions included in a node which is added to Executor will be called automatically by Executor.

#### call `take()` method of Subscription object

Here is a sample code  excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp) in which `take()` method is used.

```c++
  std_msgs::msg::String msg;
  rclcpp::MessageInfo msg_info;
  if (sub_->take(msg, msg_info)) {
    RCLCPP_INFO(this->get_logger(), "Catch message");
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In code above, `take(msg, msg_info)` is called  in a timer driven callback function by `sub_` instance which is created from Subscription class. `msg` and `msg_info` indicate a message body and meta data of it respectively. When `take(msg, msg_info)` is called, if there is a message in Subscription Queue, then the oldest message in the queue is copied to `msg`.
`take(msg, msg_info)` returns `true` if a message is received from Subscription successfully. If this is the case, a content of the message is outputted by `RCLCPP_INFO`.
`take(msg, msg_info)` returns `false` if a mesage is not received from Subscription.
When `take(msg, msg_info)` is called, if size of Subscription Queue is larger than one and there are two or more message in the queue, then the oldest message is copied to `msg`. If size of Queue is one, the latest message is always obtained.

Note: `take()` method is irreversible in terms that a obtained message by `take()` method can not be returned to Subscription Queue. **You can determine whether a message is in Subscription Queue or not by the return alue of `take()` method, but it changes Subscription queue state.** If you want to know whether there is a message in Subscription Queue or not, you can use `rclcpp::WaitSet`.
As for `WaitSet`, refer to xxx for more detail.

### obtain multiple data stored in Subscription Queue

Subscription can hold multiple messages in its queue. Such messages can be obtained by consecutive calls of `take()` method from a callback function at a time. Note that in a normal manner, if there are one or more messages in Subscription Queue, the oldest one is taken and a thread is assigned to execute a callback function. If you use `take()` method, you can obtain multiple messages at a time.

Here is a sample code  excerpted from [ros2_subscription_examples/simple_examples/src/timer_batch_listener.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_batch_listener.cpp) in which `take()` method is called consecutively.

```c++
      std_msgs::msg::String msg;
      rclcpp::MessageInfo msg_info;
      while (sub_->take(msg, msg_info))
      {
        RCLCPP_INFO(this->get_logger(), "Catch message");
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In code above, `while(sub->take(msg, msg_info))` continues to take messages from Subscription Queue until the queue becomes empty. While a message is taken, the message is processed each by each.
Note that you need to determine Subscription Queue size considering frequency of an invocation of a callback function and that of a reception of messages. For example, if frequency of an invocation of a callback function is 10Hz and that of a reception of messages is 50Hz, Subscription Queue size needs to be at least 5 for messages not to be lost.

To assign a thread and let it execute a callback function each time a message is received leads to increase overhead in terms of performance. You can use the manner introduced in this section to avoid the overhead.
It is effective to use the manner if a reception of messages occurs very frequently but they need not to be processed with so high frequency. For example, if a message is received with more than 100 Hz such as CAN message, it is desirable to obtain multiple messages in a callback function at a time with one thread invocation.

### obtain data by calling `Subscription->take` and then call a callback function

If you want to use the manner introduced in this guideline, you may feel it is needed to implement a new funcation in which a message is taken by `take()` method and processed. But there is a way to make a callback function be called indirectly which is registered to Subscription object.
Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener_using_callback.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener_using_callback.cpp).

```c++
      auto msg = sub_->create_message(); // auto means shared_ptr<void>
      rclcpp::MessageInfo msg_info;
      if (sub_->take_type_erased(msg.get(), msg_info)) {
        sub_->handle_message(msg, msg_info);

```

In code above, a message is taken by `take_type_erased()` method before calling registered callback function. Note that you need to use `take_type_erased()` instead of `take()` and `take_type_erased()` needs `void` type data as its first argument. You need to use `get()` method to convert `msg` whose type is `shared_ptr<void>` to `void` type. Then `handle_message()` method is called with the obtained message given. Registered callback function is called inside of `handle_message()`.
You need not take special care of message type which is passed to `take_type_erased()`, the same as `take_type_erased()` and `handle_message()` are not based on any specific types. You can define message variable as `auto msg = sub_->create_message();`.
You can also refer to [API document](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SubscriptionBase.html#_CPPv4N6rclcpp16SubscriptionBase16take_type_erasedEPvRN6rclcpp11MessageInfoE) as for `create_message()`, `take_type_erased()` and `handle_message()`.

### obtain Serialized Message from Subscription

ROS 2 provides Serialized Message functon which supports communication with arbitrary message types as explained in [Class SerializedMessage](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SerializedMessage.html). It is used from `topic_state_monitor` in Autoware.
You need to use `take_serialized()` method instead of `take()` method to obtain a message of Serialized Message type from Subscription object.

Here is a sample code excerpted from [ros2_subscription_examples/simple_examples/src/timer_listener_serialized_message.cpp](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener_serialized_message.cpp).

```c++
      // receive the serialized message.
      rclcpp::MessageInfo msg_info;
      auto msg = sub_->create_serialized_message(); // std::shared_ptr<rclcpp::SerializedMessage> 
      if (sub_->take_serialized(*msg, msg_info) == false) {
        return;
      }
```

In code above, `msg` is created by `create_serialized_message()` to store a received message, whose type is `std::shared_ptr<rclcpp::SerializedMessage>`. You can obtain a message by `take_serialized()` method. Note that `take_serialized()` method needs reference type data as its first argument therefore you need to convert `msg` type using `*`.
