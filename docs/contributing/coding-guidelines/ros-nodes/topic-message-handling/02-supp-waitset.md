# [supplement] Use rclcpp::WaitSet

## What is `rclcpp::WaitSet`

As explained in [call take() method of Subscription object](./index#call-take-method-of-subscription-object), `take()` method is irreversible. Once `take()` method is executed, Subscription object state changes and undo can not be applied, therefore Subscription object can not be restored to previous state. You can use `rclcpp::WaitSet` in advance to call `take()` so that a message is received after verifying it is in Subscription Queue.
Here is a sample code in which `wait_set_.wait()` tells you that a message has already been received and can be obtained by `take()`.

```c++
      auto wait_result = wait_set_.wait(std::chrono::milliseconds(0));
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready &&
          wait_result.get_wait_set().get_rcl_wait_set().subscriptions[0]) {
            sub_->take(msg, msg_info);
            RCLCPP_INFO(this->get_logger(), "Catch message");
            RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

Also you can verify that there are messages in multiple Subscription Queues in advance.
If your code needs to proceed after several types of messages got together, using `rclcpp::WaitSet` is prefarable.

```c++
      auto wait_result = wait_set_.wait(std::chrono::milliseconds(0));
      bool received_all_messages = false;
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        for (auto wait_set_subs : wait_result.get_wait_set().get_rcl_wait_set().subscriptions) {
          if (!wait_set_subs) {
            RCLCPP_INFO_THROTTLE(get_logger(), clock, 5000, "Waiting for data...");
            return {};
          }
        }
        received_all_mesages = true;
      }
```

In code above, unless `rclcpp::WaitSet` is used, it is difficult to verify that all needed messages get together.

## Coding method

We will explain coding method of `rclcpp::WaitSet` using a sample code below.

- [ros2_subscription_examples/waitset_examples/src/talker_triple.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/talker_triple.cpp)
  - it publishes `/chatter` per one second, `/slower_chatter` per two seconds, and `/slowest_chatter` per three seconds periodically
- [ros2_subscription_examples/waitset_examples/src/timer_listener_triple_async.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/timer_listener_triple_async.cpp)
  - it queries `WaitSet` per one second and if there is a message available, it obtains the message by `take()`
  - it has each subscription for `/chatter` `/slower_chatter`, and `/slower_chatter`

Following three steps are needed to use `WaitSet`.

### 1. declare and initialize `WaitSet`

You need to declare WaitSet variable first to use.
Below is excerption from [ros2_subscription_examples/waitset_examples/src/timer_listener_triple_async.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/timer_listener_triple_async.cpp).

```c++
rclcpp::WaitSet wait_set_;
```

Note that there are three types of `WaitSet` as below.

- [Typedef rclcpp::WaitSet](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/typedef_namespacerclcpp_1ad6fb19c154de27e92430309d2da25ac3.html)
  - Subscription, Timer, and so on can be registered to WaitSet at any time
- [Typedef rclcpp::ThreadSafeWaitSet](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/typedef_namespacerclcpp_1acaec573e71549fd3078644e18e7f7127.html)
  - Subscription, Timer, and so on can be registered to WaitSet only in thread-safe state
  - sample code is here: [examples/rclcpp/wait_set/src/thread_safe_wait_set.cpp at rolling · ros2/examples](https://github.com/ros2/examples/blob/rolling/rclcpp/wait_set/src/thread_safe_wait_set.cpp)
- [Typedef rclcpp::StaticWaitSet](https://docs.ros.org/en/ros2_packages/humble/api/rclcpp/generated/typedef_namespacerclcpp_1adb06acf4a5723b1445fa6ed4e8f73374.html)
  - Subscription, Timer, and so on can be registered to WaitSet only at initialization
  - here are sample codes:
    - [ros2_subscription_examples/waitset_examples/src/timer_listener_twin_static.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/timer_listener_twin_static.cpp)
    - [examples/rclcpp/wait_set/src/static_wait_set.cpp at rolling · ros2/examples](https://github.com/ros2/examples/blob/rolling/rclcpp/wait_set/src/static_wait_set.cpp)

### 2. register trigger (Subscription, Timer, and so on) to `WaitSet`

You need to register a trigger to `WaitSet`.
Below is excerption from [ros2_subscription_examples/waitset_examples/src/timer_listener_triple_async.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/timer_listener_triple_async.cpp)

```c++
    subscriptions_array_[0] = create_subscription<std_msgs::msg::String>("chatter", qos, not_executed_callback, subscription_options);
    subscriptions_array_[1] = create_subscription<std_msgs::msg::String>("slower_chatter", qos, not_executed_callback, subscription_options);
    subscriptions_array_[2] = create_subscription<std_msgs::msg::String>("slowest_chatter", qos, not_executed_callback, subscription_options);

    // Add subscription to waitset
    for (auto & subscription : subscriptions_array_) {
      wait_set_.add_subscription(subscription);
    }
```

In code above, created subscriptions are registered to `WaitSet` by `add_subscription()`.
You can also register another type of trigger to WaitSet by which a callback function registered to Subscription is invoked, such as Timer, Service, or Action.
A sample code to register Timer trigger is here.

```c++
wait_set_.add_timer(much_slower_timer_);
```

A trigger can be registered at declaration and initialization as [https://github.com/ros2/examples/blob/rolling/rclcpp/wait_set/src/wait_set_topics_and_timer.cpp#L66](https://github.com/ros2/examples/blob/rolling/rclcpp/wait_set/src/wait_set_topics_and_timer.cpp#L66).

### 3. verify WaitSet result

You can see WaitSet result by doing 1 below first and then 2 below.

1. verifying if any trigger has been invoked
2. verifying if specified trigger has been invoked

As for 1, here is a sample code excerpted from [ros2_subscription_examples/waitset_examples/src/timer_listener_triple_async.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/timer_listener_triple_async.cpp).

```c++
      auto wait_result = wait_set_.wait(std::chrono::milliseconds(0));
      if (wait_result.kind() == rclcpp::WaitResultKind::Ready) {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that some subscription is ready");
      } else {
        RCLCPP_INFO(this->get_logger(), "wait_set tells that any subscription is not ready and return");
        return;
      }
```

In code above, it is verified whether any trigger has been invoked which is registered to `wait_set_` by `auto wait_result = wait_set_.wait(std::chrono::milliseconds(0))`.
if `wait_result.kind() == rclcpp::WaitResultKind::Ready` is `true`, it indicates any trigger has been invoked.

As for 2, here is a sample code excerpted from [ros2_subscription_examples/waitset_examples/src/timer_listener_triple_async.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/waitset_examples/src/timer_listener_triple_async.cpp).

```c++
      for (size_t i = 0; i < subscriptions_num; i++) {
        if (wait_result.get_wait_set().get_rcl_wait_set().subscriptions[i]) {
          std_msgs::msg::String msg;
          rclcpp::MessageInfo msg_info;
          if (subscriptions_array_[i]->take(msg, msg_info)) {
            RCLCPP_INFO(this->get_logger(), "Catch message via subscription[%ld]", i);
            RCLCPP_INFO(this->get_logger(), "I heard: [%s]", msg.data.c_str());
```

In code above, `wait_result.get_wait_set().get_rcl_wait_set().subscriptions[i]` indicates whether each individual trigger has been invoked or not. The result is stored to `subscriptions` array. The order in `subscriptions` array is the same as the order in which triggers are registered.
