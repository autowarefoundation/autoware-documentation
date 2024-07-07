# Topic message handling guideline

## Introduction

Here is coding guideline for topic message handling in Autoware. It includes the recommended manner than conventional one, which is roughly explained in [_Discussions page_](https://github.com/orgs/autowarefoundation/discussions/4612). Refer to the page to understand the basic concept of the recommended manner.
You can find sample source code in [_ros2_subscription_examples_](https://github.com/takam5f2/ros2_subscription_examples) referred from this document.

## Conventional message handling manner

At first, let us see a conventional manner of handling messages that is commonly used.
[_ROS 2 Tutorials_](https://docs.ros.org/en/rolling/Tutorials/Beginner-Client-Libraries/Writing-A-Simple-Cpp-Publisher-And-Subscriber.html#write-the-subscriber-node) is one of the most cited references for ROS 2 applications, including Autoware.
It implicitly recommends that each of messages received by subscriptions should be referred to and processed by a dedicated callback function. Autoware follows that manner thoroughly.

```c++
  steer_sub_ = create_subscription<SteeringReport>(
    "input/steering", 1,
    [this](SteeringReport::SharedPtr msg) { current_steer_ = msg->steering_tire_angle; });
```

In the code above, when a topic message whose name is `input/steering` is received, an anonymous function whose description is `{current_steer_ = msg->steering_tier_angle;}` is executed as a callback in a thread. The callback function is always executed when the message is received, which leads to waste computing resource if the message is not always necessary. Besides, waking up a thread costs computational overhead.

## Recommended manner

This section introduces a recommended manner to take a message using `Subscription->take()` method only when the message is needed.
The sample code given below shows that `Subscription->take()` method is called during execution of any callback function. In most cases, `Subscription->take()` method is called before a received message is consumed by a main logic.
In this case, a topic message is retrieved from the subscription queue, the queue embedded in the subscription object, instead of using a callback function. To be precise, you have to program your code so that a callback function is not automatically called.

```c++
  SteeringReport msg;
  rclcpp::MessageInfo msg_info;
  if (sub_->take(msg, msg_info)) {
    // processing and publishing after this
```

Using this manner has the following benefits.

- It can reduce the number of calls to subscription callback functions
- There is no need to take a topic message from a subscription that a main logic does not consume
- There is no mandatory thread waking for the callback function, which leads to multi-threaded programming, data races and exclusive locking

## Manners to handle topic message data

This section introduces four manners, including the recommended ones.

### 1. Obtain data by calling `Subscription->take()`

To use the recommended manner using `Subscription->take()`, you basically need to do two things below.

1. Prevent a callback function from being called when a topic message is received
2. Call `take()` method of a subscription object when a topic message is needed

You can see an example of the typical use of `take()` method in [_ros2_subscription_examples/simple_examples/src/timer_listener.cpp_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp).

#### Prevent calling a callback function

To prevent a callback function from being called automatically, the callback function has to belong a callback group whose callback functions are not added to any executor.
According to the [_API specification of `create_subscription`_](http://docs.ros.org/en/iron/p/rclcpp/generated/classrclcpp_1_1Node.html), registering a callback function to a `rclcpp::Subscription` based object is mandatory even if the callback function has no operation.
Here is a sample code snippet from [_ros2_subscription_examples/simple_examples/src/timer_listener.cpp_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp).

```c++
    rclcpp::CallbackGroup::SharedPtr cb_group_not_executed = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive, false);
    auto subscription_options = rclcpp::SubscriptionOptions();
    subscription_options.callback_group = cb_group_not_executed;

    rclcpp::QoS qos(rclcpp::KeepLast(10));
    if (use_transient_local) {
      qos = qos.transient_local();
    }

    sub_ = create_subscription<std_msgs::msg::String>("chatter", qos, not_executed_callback, subscription_options);
```

In the code above, `cb_group_not_executed` is created by calling `create_callback_group` with the second argument `false`. Any callback function which belongs to the callback group will not be called by an executor.
If the `callback_group` member of `subscription_options` is set to `cb_group_not_executed`, then `not_executed_callback` will not be called when a corresponding topic message `chatter` is received.
The second argument to `create_callback_group` is defined as follows.

```c++
rclcpp::CallbackGroup::SharedPtr create_callback_group(rclcpp::CallbackGroupType group_type, \
                                  bool automatically_add_to_executor_with_node = true)
```

When `automatically_add_to_executor_with_node` is set to `true`, callback functions included in a node that is added to an executor will be automatically called by the executor.

#### Call `take()` method of Subscription object

To take a topic message from the `Subscription` based object, the `take()` method is called at the expected time.
Here is a sample code snippet from [_ros2_subscription_examples/simple_examples/src/timer_listener.cpp_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener.cpp) using `take()` method.

```c++
  std_msgs::msg::String msg;
  rclcpp::MessageInfo msg_info;
  if (sub_->take(msg, msg_info)) {
    RCLCPP_INFO(this->get_logger(), "Catch message");
    RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In the code above, `take(msg, msg_info)` is called by `sub_` object instantiated from the `rclcpp::Subscription` class. It is called in a timer driven callback function. `msg` and `msg_info` indicate a message body and its metadata respectively. If there is a message in the subscription queue when `take(msg, msg_info)` is called, then the message is copied to `msg`.
`take(msg, msg_info)` returns `true` if a message is successfully taken from the subscription. In this case, the above code prints out a string data of the message from `RCLCPP_INFO`.
`take(msg, msg_info)` returns `false` if a message is not taken from the subscription.
When `take(msg, msg_info)` is called, if the size of the subscription queue is greater than one and there are two or more messages in the queue, then the oldest message is copied to `msg`. If the size of the queue is one, the latest message is always obtained.

!!! note

    You can check the presence of incoming message with the returned value of `take()` method. However, you have to take care of the destructive nature of the take() method. The `take()` method modifies the subscription queue. Also, the `take()` method is irreversible and there is no undo operation against the `take()` method. Checking the incoming message with only the `take()` method always changes the subscription queue. If you want to check without changing the subscription queue, rclcpp::WaitSet is recommended. Refer to [_[supplement] Use rclcpp::WaitSet_](./supp-wait_set.md) for more detail.

!!! note

    The `take()` method is supported to only obtain a message which is passed through DDS as an inter-process communication. You must not use it for an intra-process communication because intra-process communication is based on another software stack of `rclcpp`. Refer to [_[supplement] Obtain a received message through intra-process communication_](./supp-intra-process-comm.md) in case of intra-process communication.

#### 1.1 Obtain Serialized Message from Subscription

ROS 2 provides Serialized Message function which supports communication with arbitrary message types as described in [_Class SerializedMessage_](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SerializedMessage.html). It is used by `topic_state_monitor` in Autoware.
You have to use the `take_serialized()` method instead of the `take()` method to obtain a `rclcpp::SerializedMessage` based message from a subscription.

Here is a sample code snippet from [_ros2_subscription_examples/simple_examples/src/timer_listener_serialized_message.cpp_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener_serialized_message.cpp).

```c++
      // receive the serialized message.
      rclcpp::MessageInfo msg_info;
      auto msg = sub_->create_serialized_message();

      if (sub_->take_serialized(*msg, msg_info) == false) {
        return;
      }
```

In the code above, `msg` is created by `create_serialized_message()` to store a received message, whose type is `std::shared_ptr<rclcpp::SerializedMessage>`. You can obtain a message of type `rclcpp::SerializedMessage` using the `take_serialized()` method. Note that the `take_serialized()` method needs reference type data as its first argument. Since `msg` is a pointer, `*msg` should be passed as the first argument to the `take_serialized().

!!! note

    ROS 2's `rclcpp` supports both `rclcpp::LoanedMessage` and `rclcpp::SerializedMessage`. If [_zero copy communication via loaned messages_](https://design.ros2.org/articles/zero_copy.html) is introduced to Autoware, `take_loaned()` method should be used for communication via loaned messages instead. In this document, the explanation of the `take_loaned()` method is omitted because it is not used for Autoware in this time (May. 2024).

### 2. Obtain multiple data stored in Subscription Queue

A subscription object can hold multiple messages in its queue if multiple queue size is configured with the QoS setting. The conventional manner using callback function forces a callback function to be executed per message. In other words, there is a constraint; a single cycle of callback function processes a single message . Note that with the conventional manner, if there are one or more messages in the subscription queue, the oldest one is taken and a thread is assigned to execute a callback function, which continues until the queue is empty.
The `take()` method would alleviate this limitation. The `take()` method can be called in multiple iterations, so that a single cycle of the callback function processes multiple messages taken by `take()` methods.

Here is a sample code, taken from [_ros2_subscription_examples/simple_examples/src/timer_batch_listener.cpp_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_batch_listener.cpp) which calls the `take()` method in a single cycle of a callback function.

```c++
      std_msgs::msg::String msg;
      rclcpp::MessageInfo msg_info;
      while (sub_->take(msg, msg_info))
      {
        RCLCPP_INFO(this->get_logger(), "Catch message");
        RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In the code above, `while(sub->take(msg, msg_info))` continues to take messages from the subscription queue until the queue is empty. Each message taken is processed per iteration.
Note that you must determine size of a subscription queue by considering both frequency of a callback function and frequency of a message reception. For example, if a callback function is invoked at 10Hz and topic messages are received at 50Hz, the size of the subscription queue must be at least 5 to avoid losing received messages.

Assigning a thread to execute a callback function per message will cause performance overhead. You can use the manner introduced in this section to avoid the unexpected overhead.
The manner will be effective when there is a large difference between reception frequency and consumption frequency. For example, even if a message, such as a CAN message, is received at higher than 100 Hz, a user logic consumes messages at slower frequency such as 10 Hz. In such a case, the user logic should retrieve the required number of messages with the `take()` method to avoid the unexpected overhead.

### 3. Obtain data by calling `Subscription->take` and then call a callback function

You can combine the `take()` (strictly `take_type_erased()`) method and the callback function to process received messages in a consistent way. Using this combination does not require waking up a thread.
Here is a sample code snippet from [_ros2_subscription_examples/simple_examples/src/timer_listener_using_callback.cpp_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/simple_examples/src/timer_listener_using_callback.cpp).

```c++
      auto msg = sub_->create_message();
      rclcpp::MessageInfo msg_info;
      if (sub_->take_type_erased(msg.get(), msg_info)) {
        sub_->handle_message(msg, msg_info);

```

In the code above, a message is taken by the `take_type_erased()` method before a registered callback function is called via the `handle_message()` method. Note that you must use `take_type_erased()` instead of `take()`. `take_type_erased()` needs `void` type data as its first argument. You must use the `get()` method to convert `msg` whose type is `shared_ptr<void>` to `void` type. Then the `handle_message()` method is called with the obtained message. A registered callback function is called within `handle_message()`.
You don't need to take care of message type which is passed to `take_type_erased()` and `handle_message()`. You can define the message variable as `auto msg = sub_->create_message();`.
You can also refer to [_the API document_](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1SubscriptionBase.html#_CPPv4N6rclcpp16SubscriptionBase16take_type_erasedEPvRN6rclcpp11MessageInfoE) as for `create_message()`, `take_type_erased()` and `handle_message()`.

### 4. Obtain data by a callback function

A conventional manner, typically used in ROS 2 application, is a message reference using a callback function, is available. If you don't use a callback group with `automatically_add_to_executor_with_node = false`, a registered callback function will be called automatically by an executor when a topic message is received.
One of the advantages of this manner is that you don't have to take care whether a topic message is passed through inter-process or intra-process. Remember that `take()` can only be used for inter-process communication via DDS, while another manner provided by `rclcpp` can be used for intra-process communication via `rclcpp`.

## Appendix

A callback function is used to obtain a topic message in many of ROS 2 applications. It is as like a rule or a custom. As this document page explains, you can use the `Subscription->take()` method to obtain a topic message without calling a subscription callback function.
This manner is also documented in [_Template Class Subscription — rclcpp 16.0.8 documentation_](https://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1Subscription.html#_CPPv4N6rclcpp12Subscription4takeER14ROSMessageTypeRN6rclcpp11MessageInfoE).

Many of ROS 2 users may be afraid to use the `take()` method because they may not be so familiar with it and there is a lack of documentation about `take()`, but it is widely used in the `rclcpp::Executor` implementation as shown in [_rclcpp/executor.cpp_](https://github.com/ros2/rclcpp/blob/47c977d1bc82fc76dd21f870bcd3ea473eca2f59/rclcpp/src/rclcpp/executor.cpp#L643-L648) shown below. So it turns out that you are indirectly using the `take()` method, whether you know it or not.

```c++
    std::shared_ptr<void> message = subscription->create_message();
    take_and_do_error_handling(
      "taking a message from topic",
      subscription->get_topic_name(),
      [&]() {return subscription->take_type_erased(message.get(), message_info);},
      [&]() {subscription->handle_message(message, message_info);});
```

!!!note

    Strictly speaking, the `take_type_erased()` method is called in the executor, but not the `take()` method.

    But `take_type_erased()` is the embodiment of `take()`, while `take()` internally calls `take_type_erased()`.

If `rclcpp::Executor` based object, an executor, is programmed to call a callback function, the executor itself determines when to do it. Because the executor is essentially calling a best-effort callback function, the message is not guaranteed to be necessarily referenced or processed even though it is received. Therefore it is desirable to call the `take()` method directly to ensure that a message is referenced or processed at the intended time.

---

As of May 2024, the recommended manners are beginning to be used in Autoware Universe.
See the following PR if you want an example in Autoware Universe.

[_feat(tier4_autoware_utils, obstacle_cruise): change to read topic by polling #6702_](https://github.com/autowarefoundation/autoware.universe/pull/6702)
