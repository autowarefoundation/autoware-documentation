---
last_updated: YYYY-MM-DD
interface_type: topic
interface_name: /autoware/example/topic
data_type_name: std_msgs/msg/Bool
data_type_link: https://github.com/ros2/common_interfaces/blob/humble/std_msgs/msg/Bool.msg
rate: ---
qos_reliability: reliable or best_effort
qos_durability: volatile or transient_local
qos_depth: 1
---

# {{ interface_name }}

## Specifications

{% include 'design/autoware-architecture-v1/interfaces/templates/topic.jinja2' %}

## Description

Describe the feature of this interface here. If the feature is complex, you can create a separate page and link to it.
Also, please clarify any restrictions, such as relationships with other interfaces or timing conditions.
If necessary, adding state transition, sequence, architecture diagrams, etc. will make it easier to understand.

Requirements such as topic rate and response timeout may vary depending on the target ODD.
In such cases, cover all the necessary conditions for each ODD.
If only a specific ODD is being considered, state that it cannot be used under other ODDs.

## Message

Describe the message or service type specifications.
If there is information in the message definition file or message package README, you can link to that.
The message definition should clarify the following:

- Whether the field is required or optional, and how it will be handled if omitted.
- Time stamp and frame ID definitions and related fields.
- How positive and negative values are assigned (e.g. left and right).
- Range and units, invalid values.
- Behavior when invalid or out-of-range values ​​are input.
- Behavior when interface feature is not supported.

## Errors

List all possible errors and diagnostics that may occur when using this interface and explain their causes.
Additionally, state what is and is not guaranteed when an error occurs.
Generally, error handling depends on the system design, but if there is any recommended procedures, provide them as examples.

## Support

Describe the possible interface support status.
This means that all features always be supported, or there are cases where they are partially supported.
For example, interfaces that specify modes should state which modes are required and which are optional.
Also, describe how to handle cases where the interface features cannot be supported, such as sending NaN or an empty array.

## Limitations

Note any limitations regarding this interface here.

## Use Cases

This section is for interface designers. Describe the use case this interface targets.
This will help derive the requirements that should be maintained in future interface redesigns.

## Requirement

This section is for interface implementers. List the requirements that the interface must meet.
Also, please clearly state any parts that are left to the implementation.

## Design

This section explains why the current design was chosen, taking into account the above use cases and requirements.
Describe other designs considered and their advantages and disadvantages to help guide future design.

## History

| Date       | Description  |
| ---------- | ------------ |
| YYYY-MM-DD | 3rd release. |
| YYYY-MM-DD | 2nd release. |
| YYYY-MM-DD | 1st release. |
