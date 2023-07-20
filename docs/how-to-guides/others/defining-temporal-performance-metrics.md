# Defining temporal performance metrics on components

## Motivation to defining temporal performance metrics

### Objective of the page

This page introduces policies to define metrics to evaluate temporal performance on components of Autoware. The term "temporal performance" is often used throughout the page in order to distinguish between functional performance, which referred to as accuracy as well, and time-related performance.

It is expected that most algorithms employed for Autoware are executed with as high frequency and short response time as possible. In order to achieve safe autonomous driving, one of the desired outcomes is no time gap between perceived and actual situation. The time gap is commonly referred to as delay. If the delay is significant, the system may determine trajectory and maneuver based on outdated situation. Consequently, if the actual situation differs from the perceived one due to the delay, the system may make unexpected decisions.

As mentioned above, this page presents the policies to define metrics. Besides, the page contains lists of sample metrics that are crucial for the main functionalities of Autoware: Localization, Perception, Planning, and Control.

!!! note

    Other functionalities, such as system components for diagnosis, are excluded currently. However they will be taken into account in the near future.

### Contribution of the temporal performance metrics

Temporal performance metrics are important for evaluating Autoware. These metrics are particularly useful for assessing delays caused by new algorithms and logic. They can be employed when comparing the temporal performance of software on a desktop computer with that on a vehicle during the vehicle integration phase.

In addition, these metrics are useful for designers and evaluators of middleware, operating systems, and computers. They are selected based on user and product requirements. One of these requirements is to provide sufficient temporal performance for executing Autoware. "Sufficient temporal performance" is defined as a temporal performance requirement, but it can be challenging to define the requirement because it varies depending on the product type, Operational Design Domain (ODD), and other factors. Then, this page specifically focuses on temporal performance metrics rather than requirements.

Temporal performance metrics are important for evaluating the reliability of Autoware. However, ensuring the reliability of Autoware requires consideration of not only temporal performance metrics but also other metrics.

### Tools for evaluating the metrics

There are several tools available for evaluating Autoware according to the metrics listed in the page. For example, both [CARET](https://github.com/tier4/caret) and [ros2_tracing](https://github.com/ros2/ros2_tracing) are recommended options when evaluating Autoware on Linux and ROS 2. If you want to measure the metrics with either of these tools, refer to the corresponding user guide for instructions. It's important to note that if you import Autoware to a platform other than Linux and ROS 2, you will need to choose a supported tool for evaluation.

!!! note

    TIER IV plans to measure Autoware, which is running according to [the tutorial](../../tutorials/), and provide a performance evaluation report periodically. An example of such a report can be found [here](https://tier4.github.io/CARET_report/), although it may not include all of the metrics listed.

The page does not aim to provide instructions on how to use these tools or measure the metrics. Its primary focus is on the metrics themselves, as they are more important than the specific tools used. These metrics retain their relevance regardless of the employed platform.

## Policies to define temporal performance metrics

As mentioned above, the configuration of Autoware varies by the product type, ODD, and other factors. The variety of configurations makes it difficult to define the uniform metrics for evaluating Autoware.  
However, the policies used to define them are basically reused even when the configuration changes. Each of temporal performance metrics is categorized into two types: **execution frequency** and **response time**. Although there are many types of metrics, such as communication latency, the only two types are considered for simplicity.  
Execution frequency is observed using rate of Inter-Process Communication (IPC) messages. You will find an enormous number of messages in Autoware, but you don't have to take care of all. Some messages might be critical to functionality and they should be chosen for evaluation.  
Response time is duration elapsed through a series of processing. A series of processing is referred to as a path. Response time is calculated from timestamps of start and end of a path. Although many paths can be defined in Autoware, you have to choose significant paths.

As a hint, here are some characteristics of message and path in order to choose metrics.

1. Messages and paths on boundaries where observed values from sensors are consumed
2. Messages and paths on boundaries of functions, e.g., a boundary of perception and planning
3. Messages and paths on boundaries where timer-based frequency is switched
4. Messages and paths on boundaries where two different messages are synchronized and merged
5. Messages that must be transmitted at expected frequency, e.g., vehicle command messages

Those hints would be helpful for most configurations but there may be exclusions. Defining metrics precisely requires an understanding of configuration.

In addition, it is recommended that metrics be determined incrementally from the architectural level to the detailed design and implementation level. Mixing metrics at different levels of granularity can be confusing.

## List of sample metrics

This section demonstrates how to define metrics according to the policies explained and has lists of the metrics for Autoware launched according to [the tutorial](../../tutorials/). The section is divided into multiple subsections, each containing a model diagram and an accompanying list that explains the important temporal performance metrics. Each model is equipped with checkpoints that serve as indicators for these metrics.

The first subsection presents the top-level temporal performance metrics, which are depicted in the abstract structure of Autoware as a whole. The detailed metrics are not included in the model as they would add complexity to it. Instead, the subsequent section introduces the detailed metrics. The detailed metrics are subject to more frequent updates compared to the top-level ones, which is another reason for categorizing them separately.

Each list includes a column for the reference value. The reference value represents the observed value of each metric when Autoware is running according to [the tutorial](../../tutorials/). It is important to note that the reference value is not a required value, meaning that Autoware does not necessarily fail in [the tutorial](../../tutorials/) execution if certain metrics do not fulfill the reference value.

### Top-level temporal performance metrics for Autoware

The diagram below introduces the model for top-level temporal performance metrics.

![Model for top-level temporal performance metrics](./images/important-temporal-performance-metrics/model-for-top-level-metrics.svg)

The following three policies assist in selecting the top-level performance metrics:

- Splitting Autoware based on components that consume observed values, such as sensor data, and considering the processing frequency and response time around these components
- Dividing Autoware based on the entry point of Planning and Control and considering the processing frequency and response time around these components
- Showing the minimum metrics for the Vehicle Interface, as they may vary depending on the target vehicle

Additionally, it is assumed that algorithms are implemented as multiple nodes and function as a pipeline processing system.
<!-- cspell: ignore AWOV OSEG -->
| ID       | Representation in the model                          | Metric meaning                                                                                                   | Related functionality | Reference value | Reason to choose it as a metric                                                                               | Note                                                               |
| -------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------- | --------------------- | --------------- | ------------------------------------------------------------------------------------------------------------- | ------------------------------------------------------------------ |
| AWOV-001 | **Message rate** from CPA #9 to CPA #18              | Update rate of result from Prediction to Planning.                                                               | Perception            | 10 Hz           | Planning relies on fresh and up-to-date perceived data from Perception for creating accurate trajectory.      |                                                                    |
| AWOV-002 | **Response time** from CPA #0 to CPA #20 via CPA #18 | Response time in main body of Perception.                                                                        | Perception            | N/A             | Planning relies on fresh and up-to-date perceived data from Perception for creating accurate trajectory.      | The metric is used if delay compensation is disabled in Tracking.  |
| AWOV-003 | **Response time** from CPA #7 to CPA #20             | Response time from Tracking output of Tracking to its data consumption in Planning.                              | Perception            | N/A             | Planning relies on fresh and up-to-date perceived data from Perception for creating accurate trajectory.      | The metric is used if delay compensation is enabled in Tracking.   |
| AWOV-004 | **Response time** from CPA #0 to CPA #6              | Duration to process pointcloud data in Sensing and Detection.                                                    | Perception            | N/A             | Tracking relies on detection to provide real-time and up-to-date sensed data for accurate tracking.           | The metric is used if delay compensation is enabled in Tracking.   |
| AWOV-005 | **Message rate** from CPA #4 to CPA #5               | Update rate of Detection result received by Tracking.                                                            | Perception            | 10 Hz           | Tracking relies on detection to provide real-time and up-to-date sensed data for accurate tracking.           |                                                                    |
| AWOV-006 | **Response time** from CPA #0 to CPA #14             | Response time from output of observed data from LiDARs to its consumption in EKF Localizer via NDT Scan Matcher. | Localization          | N/A             | EKF Localizer relies on fresh and up-to-date observed data from sensors for accurate estimation of self pose. |                                                                    |
| AWOV-007 | **Message rate** from CPA #11 to CPA #13             | Update rate of pose estimated by NDT Scan Matcher.                                                               | Localization          | 10 Hz           | EKF Localizer relies on fresh and up-to-date observed data from sensors for accurate estimation of self pose. |                                                                    |
| AWOV-008 | **Message rate** from CPA #15 to CPA #12             | Update rate of feed backed pose estimated by EKF Localizer.                                                      | Localization          | 50 Hz           | NDT Scan Matcher relies on receiving estimated pose from EKF Localizer smoothly for linear interpolation.     |                                                                    |
| AWOV-009 | **Message rate** from CPA #17 to CPA #19             | Update rate of Localization result received by Planning.                                                         | Localization          | 50 Hz           | Planning relies on Localization to update the estimated pose frequently.                                      |                                                                    |
| AWOV-010 | **Response time** from CPA #20 to CPA #23            | Processing time from beginning of Planning to consumption of Trajectory message in Control.                      | Planning              | N/A             | A vehicle relies on Planning to update trajectory within a short time frame to achieve safe driving behavior. |                                                                    |
| AWOV-011 | **Message rate** from CPA #21 to CPA #22             | Update rate of Trajectory message from Planning.                                                                 | Planning              | 10 Hz           | A vehicle relies on Planning to update trajectory frequently to achieve safe driving behavior.                |                                                                    |
| AWOV-012 | **Message rate** from CPA #24 to CPA #25             | Update rate of Control command.                                                                                  | Control               | 33 Hz           | Control stability and comfort relies on sampling frequency of Control.                                        |                                                                    |
| AWOV-013 | **Message rate** between CPA #26 and Vehicle         | Communication rate between Autoware and Vehicle.                                                                 | Vehicle Interface     | N/A             | A vehicle requires Autoware to communicate with each other at predetermined frequency.                        | Temporal performance requirement varies depending on vehicle type. |

!!! note

    There is an assumption that each of sensors, such as LiDARs and cameras, outputs a set of pointcloud with a timestamp. CPA #0 is observed with the timestamp. If the sensors are not configured to output the timestamp, the time when Autoware receives the pointcloud is used instead. That is represented by CPA #1 in the model. The detailed metrics employs the idea as well.

### Detailed temporal performance metrics for Perception

The diagram below introduces the model for temporal performance metrics for Perception.

![Model for Perception temporal performance metrics](./images/important-temporal-performance-metrics/model-for-perception-metrics.svg)

The following two policies assist in selecting the performance metrics:

- Regarding the frequency and response time at which Recognition results from Object Recognition and Traffic Light Recognition are consumed in Planning
- Splitting Perception component on merging points of data from multiple processing paths and considering the frequency and response time around that point

The following list shows the temporal performance metrics for Perception.

| ID       | Representation in the model                          | Metric meaning                                                                                       | Related functionality     | Reference value | Reason to choose it as a metric                                                                                     | Note                                                                                   |
| -------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------------------- | ------------------------- | --------------- | ------------------------------------------------------------------------------------------------------------------- | -------------------------------------------------------------------------------------- |
| APER-001 | **Message rate** from CPP #2 to CPP #26              | Update rate of Traffic Light Recognition.                                                            | Traffic Light Recognition | 10 Hz           | Planning relies on fresh and up-to-date perceived data from Traffic Light Recognition for making precise decisions. |                                                                                        |
| APER-002 | **Response time** from CPP #0 to CPP #30             | Response time from camera input to consumption of the result in Planning.                            | Traffic Light Recognition | N/A             | Planning relies on fresh and up-to-date perceived data from Traffic Light Recognition for making precise decisions. |                                                                                        |
| APER-003 | **Message rate** from CPP #25 to CPP #28             | Update rate of result from Prediction (Object Recognition) to Planning.                              | Object Recognition        | 10 Hz           | Planning relies on fresh and up-to-date perceived data from Perception for creating accurate trajectory.            | The metric is same as AWOV-001.                                                        |
| APER-004 | **Response time** from CPP #6 to CPP #30             | Response time from Tracking output of Tracking to its data consumption in Planning.                  | Object Recognition        | N/A             | Planning relies on fresh and up-to-date perceived data from Perception for creating accurate trajectory.            | The metric is same as AWOV-002 and used if delay compensation is disabled in Tracking. |
| APER-005 | **Response time** from CPP #23 to CPP #30            | Response time from Tracking output of Tracking to its data consumption in Planning.                  | Object Recognition        | N/A             | Planning relies on fresh and up-to-date perceived data from Perception for creating accurate trajectory.            | The metric is same as AWOV-003 and used if delay compensation is enabled in Tracking.  |
| APER-006 | **Response time** from CPP #6 to CPP #21             | Duration to process pointcloud data in Sensing and Detection.                                        | Object Recognition        | N/A             | Tracking relies on Detection to provide real-time and up-to-date perceived data.                                    | The metrics is same as AWOV-004 and used if delay compensation is enabled in Tracking. |
| APER-007 | **Message rate** from CPP #20 to CPP #21             | Update rate of Detection result received by Tracking.                                                | Object Recognition        | 10 Hz           | Tracking relies on detection to provide real-time and up-to-date sensed data for accurate tracking.                 | The metric is same as AWOV-005                                                         |
| APER-008 | **Message rate** from CPP #14 to CPP #19             | Update rate of data sent from Sensor Fusion.                                                         | Object Recognition        | 10 Hz           | Association Merger relies on the data to be updated at expected frequency for data synchronization.                 |                                                                                        |
| APER-009 | **Message rate** from CPP #16 to CPP #19             | Update rate of data sent from Detection by Tracker.                                                  | Object Recognition        | 10 Hz           | Association Merger relies on the data to be updated at expected frequency for data synchronization.                 |                                                                                        |
| APER-010 | **Message rate** from CPP #18 to CPP #19             | Update rate of data sent from Validation                                                             | Object Recognition.       | 10 Hz           | Association Merger relies on the data to be updated at expected frequency for data synchronization.                 |
| APER-011 | **Response time** from CPP #6 to CPP #19 via CPP #14 | Response time to consume data sent from Sensor Fusion after LiDARs output pointcloud.                | Object Recognition        | N/A             | Association Merger relies on fresh and up-to-date data for data synchronization.                                    |                                                                                        |
| APER-012 | **Response time** from CPP #6 to CPP #19 via CPP #16 | Response time to consume data sent from Detection by Tracker after LiDARs output pointcloud.         | Object Recognition        | N/A             | Association Merger relies on fresh and up-to-date data for data synchronization.                                    |                                                                                        |
| APER-013 | **Response time** from CPP #6 to CPP #19 via CPP #18 | Response time to consume data sent from Validator after LiDARs output pointcloud.                    | Object Recognition        | N/A             | Association Merger relies on fresh and up-to-date data for data synchronization.                                    |                                                                                        |
| APER-014 | **Message rate** from CPP #10 to CPP #13             | Update rate of data sent from Clustering.                                                            | Object Recognition        | 10 Hz           | Sensor Fusion relies on the data to be updated at expected frequency for data synchronization.                      |                                                                                        |
| APER-015 | **Message rate** from CPP #5 to CPP #13              | Update rate of data sent from Camera-based Object detection.                                         | Object Recognition        | 10 Hz           | Sensor Fusion relies on the data to be updated at expected frequency for data synchronization.                      |                                                                                        |
| APER-016 | **Response time** from CPP #6 to CPP #13             | Response time to consume data sent from Clustering after LiDARs output pointcloud.                   | Object Recognition        | N/A             | Sensor Fusion relies on fresh and up-to-date data for data synchronization.                                         |                                                                                        |
| APER-017 | **Response time** from CPP #3 to CPP #13             | Response time to consume data sent from Camera-based Object detection after Cameras output images.   | Object Recognition        | N/A             | Sensor Fusion relies on fresh and up-to-date data for data synchronization.                                         |                                                                                        |
| APER-018 | **Message rate** from CPP #10 to CPP #17             | Update rate of data sent from Clustering.                                                            | Object Recognition        | 10 Hz           | Validator relies on the data to be updated at expected frequency for data synchronization.                          | It seems similar to APER-014, but the topic message is different.                      |
| APER-019 | **Message rate** from CPP #12 to CPP #17             | Update rate of data sent from DNN-based Object Recognition.                                          | Object Recognition        | 10 Hz           | Validator relies on the data to be updated at expected frequency for data synchronization.                          |
| APER-020 | **Response time** from CPP #6 to CPP #17 via CPP #10 | Response time to consume data sent from Clustering after LiDARs output pointcloud.                   | Object Recognition        | N/A             | Validator relies on fresh and update-date data for data synchronization.                                            | It seems similar to APER-015, but the topic message is different.                      |
| APER-021 | **Response time** from CPP #6 to CPP #17 via CPP #12 | Response time to consume data sent from DNN-based Object Recognition after LiDARs output pointcloud. | Object Recognition        | N/A             | Validator relies on fresh and update-date data for data synchronization.                                            |                                                                                        |

### Detailed temporal performance metrics for Paths between Obstacle segmentation and Planning

Obstacle segmentation, which is a crucial part of Perception, transmits data to Planning. The figure below illustrates the model that takes into account performance metrics related to Obstacle segmentation and Planning.

![Model for Obstacle segmentation temporal performance metrics](./images/important-temporal-performance-metrics/model-for-obstacle-segmentation-metrics.svg)

!!! note

    Both the Obstacle grid map and Obstacle segmentation transmit data to multiple sub-components of Planning. However, not all of these sub-components are described in the model. This is because our primary focus is on the paths from LiDAR to Planning via Obstacle segmentation.

The following list shows the temporal performance metrics around Obstacle segmentation and Planning.

| ID       | Representation in the model                          | Metric meaning                                                                           | Related functionality | Reference value | Reason to choose it as a metric                                                                                      | Note |
| -------- | ---------------------------------------------------- | ---------------------------------------------------------------------------------------- | --------------------- | --------------- | -------------------------------------------------------------------------------------------------------------------- | ---- |
| OSEG-001 | **Message rate** from CPS #4 to CPS #7               | Update rate of Occupancy grid map received by Planning (`behavior_path_planner`)         | Obstacle segmentation | 10 Hz           | Planning relies on Occupancy grid map to be updated frequently and smoothly for creating accurate trajectory.        |      |
| OSEG-002 | **Response time** from CPS #0 to CPS #9 via CPS #7   | Response time to consume Occupancy grid map after LiDARs output sensing data.            | Obstacle segmentation | N/A             | Planning relies on fresh and up-to-date perceived data from Occupancy grid map for creating accurate trajectory..    |      |
| OSEG-003 | **Message rate** from CPS #6 to CPS #11              | Update rate of obstacle segmentation received by Planning (`behavior_velocity_planner`). | Obstacle segmentation | 10 Hz           | Planning relies on Obstacle segmentation to be updated frequently and smoothly for creating accurate trajectory.     |      |
| OSEG-004 | **Response time** from CPS #0 to CPS #13 via CPS #11 | Response time to consume Obstacle segmentation after LiDARs output sensing data.         | Obstacle segmentation | N/A             | Planning relies on fresh and up-to-date perceived data from Obstacle segmentation for creating accurate trajectory.. |      |
