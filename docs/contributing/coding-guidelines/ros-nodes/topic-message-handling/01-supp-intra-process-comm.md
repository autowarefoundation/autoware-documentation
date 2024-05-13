# [supplement] Obtain a received message through intra-process communication

## Topic message handling in intra-process communication

`rclcpp` supports intra-process communication. As explained in [Topic message handling guideline](index.md), `take()` method can not be used in the case of intra-process communication. `take()` can not return a topic message which is received through inter-process communication.  
However, methods for intra-process communication are provided, similar to the methods for inter-process communication described in [_obtain data by calling Subscription->take and then call a callback function_](./index.md#3-obtain-data-by-calling-subscription-take-and-then-call-a-callback-function).
`take_data()` method is provided to obtain a received data in the case of intra-process communication and the received data must be processed through `execute()` method. The return value of `take_data()` is based on the complicated data structure, `execute()` method should be used along with `take_data()` method.
Refer to [_Template Class SubscriptionIntraProcess — rclcpp 16.0.8 documentation_](http://docs.ros.org/en/humble/p/rclcpp/generated/classrclcpp_1_1experimental_1_1SubscriptionIntraProcess.html#_CPPv4N6rclcpp12experimental24SubscriptionIntraProcess9take_dataEv) for `take_data()` and `execute()` for more detail.

## Coding manner

To handle messages via intra-process communication, call `take_data()` method and then `execute()` method as below.

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

Here is a snippet of [_ros2_subscription_examples/intra_process_talker_listener/src/timer_listener_intra_process.cpp at main · takam5f2/ros2_subscription_examples_](https://github.com/takam5f2/ros2_subscription_examples/blob/main/intra_process_talker_listener/src/timer_listener_intra_process.cpp).

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

Below is a line-by-line explanation of the above code.

- `if (this->get_node_options().use_intra_process_comms()){`

  - The statement checks whether or not intra-process communication is enabled or not by using `NodeOptions`

- `auto intra_process_sub = sub_->get_intra_process_waitable();`

  - The statement means to get an embodied object which performs intra-process communication

- `if (intra_process_sub->is_ready(nullptr) == true) {`

  - The statement checks if a message has already been received through intra-process communication
  - The argument of `is_ready()` is of type `rcl_wait_set_t` type, but because the argument is not used within `is_ready()`, `nullptr` is used for the moment.
    - Using `nullptr` is currently a workaround, as it has no intent.

- `std::shared_ptr<void> data = intra_process_sub->take_data();`

  - This statement means to obtain a topic message from subscriptions for intra-process communication.
  - `intra_process_sub->take_data()` does not return Boolean value which indicates a message is received successfully or not, therefore it is needed to verify it by calling `is_ready()` in advance

- `intra_process_sub->execute(data);`
  - a callback function corresponded to the received message is called inside `execute()`
  - the callback function is executed by the thread which executes `execute()` without context switch
