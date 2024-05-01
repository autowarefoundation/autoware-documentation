# [supplement] Obtain a received message through intra-process communication

## Topic message handling in intra-process communication

rlcpp supports intra-process communication. As explained in [Topic message handling guideline](index.md), `take()` method of Subscription can not be used in case of intra-process communication. `take()` can not obtain a topic message which is received through inter-process communication. Besides, even though a method is provided to obtain a received topic message through intra-process communication, a method is not provided to refer to the message directly.
But a method are provided for intra-process communication similar to a method for inter-process communication described in [obtain data by calling Subscription->take and then call a callback function](./index.md#3-obtain-data-by-calling-subscription-take-and-then-call-a-callback-function).
`take_data()` method is provided to obtain a received data in case of intra-process communication and the received data must be processed through `execute()` method.
Refer to [Template Class SubscriptionIntraProcess — rclcpp 16.0.8 documentation](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html#_CPPv4N6rclcpp12experimental24SubscriptionIntraProcess9take_dataEv) for `take_data()` and `execute()` for more detail.

## coding method

Call `take_data()` method and then `execute()` method as below.

```c++
// Execute any entities of the Waitable that may be ready
std::shared_ptr<void> data = waitable.take_data();
waitable.execute(data);
```

Here is a sample program in [ros2_subscription_examples/intra_process_talker_listener/src/timer_listener_intra_process.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/intra_process_talker_listener/src/timer_listener_intra_process.cpp).
You can run the program as below. If you set `true` to `use_intra_process_comms`, intra-process communication is performed, while if you set `false`, inter-process communication is performed.

```console
ros2 intra_process_talker_listener talker_listener_intra_process.launch.py use_intra_process_comms:=true
```

Here is a excerption from [ros2_subscription_examples/intra_process_talker_listener/src/timer_listener_intra_process.cpp at main · takam5f2/ros2_subscription_examples](https://github.com/takam5f2/ros2_subscription_examples/blob/main/intra_process_talker_listener/src/timer_listener_intra_process.cpp).

```c++
      // check if intra-process communication is enabled.
      if (this->get_node_options().use_intra_process_comms()){

        // get the intra-process subscription's waitable.
        auto intra_process_sub = sub_->get_intra_process_waitable();

        // check if the waitable has data.
        if (intra_process_sub->is_ready(nullptr) == true) {

          // take the data and execute the callback.
          std::shared_ptr<void> data = intra_process_sub->take_data();

          RCLCPP_INFO(this->get_logger(), " Intra-process communication is performed.");

          // execute the callback.
          intra_process_sub->execute(data);
```

Below is explanation of above code one line by one.

- `if (this->get_node_options().use_intra_process_comms()){`

  - verify intra-process communication is enabled or not by using NodeOptions

- `auto intra_process_sub = sub_->get_intra_process_waitable();`

  - get an object which is used by Subscription in case of intra-process communication

- `if (intra_process_sub->is_ready(nullptr) == true) {`

  - check if a message has already been received through intra-process communication
  - the argument of `is_ready()` is of rcl_wait_set_t type, but because the argument is not used inside `is_ready()`, `nullptr` is used tentatively
    - using `nullptr` is a workaround at this point of time, because it is not preferable

- `std::shared_ptr<void> data = intra_process_sub->take_data();`

  - obtain a topic message
  - `intra_process_sub->take_data()` does not return Boolean value which indicates a message is received successfully or not, therefore it is needed to verify it by calling `is_ready()` in advance

- `intra_process_sub->execute(data);`
  - a callback function corresponded to the received message is called inside `execute()`
  - the callback function is executed by the thread which executes `execute()` without context switch
