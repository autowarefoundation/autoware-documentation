# Performance Analysis


## Introduction

Autoware is a real-time system, and it is important to have a small response time. If Autoware appears to be slow, it is imperative to conduct performance measurements and implement improvements based on the analysis. However, Autoware is a complex software system comprising numerous ROS2 nodes, potentially complicating the process of identifying bottlenecks. To address this challenge, we will discuss methods for conducting detailed performance measurements for Autoware and provide case studies. It is worth noting that multiple factors can contribute to poor performance, such as scheduling and memory allocation in the OS layer, but our focus in this page will be on user code bottlenecks. The outline of this section is as follows:

 - Performance Measurement
   - Single Node Execution
   - Prepare separated cores
   - Run Single Node Separatedly
   - Measure turn-around time and performance counter
   - Visualization
 - Case Studies
   - Sensing Component
   - Planning Component


## Performance Measurement

To be filled by @sykwer



## Case Studies

In this section, we will present several case studies that demonstrate the performance improvements. These examples not only showcase our commitment to enhancing the system's efficiency but also serve as a valuable resource for developers who may face similar challenges in their own projects. The performance improvements discussed here span various components of the Autoware system, including sensing modules and planning modules. There are tendencies for each component regarding which points are becoming bottlenecks. By examining the methods, techniques, and tools employed in these case studies, readers can gain a better understanding of the practical aspects of optimizing complex software systems like Autoware.


### Sensing Component

To be filled by @sykwer



### Planning Component

In planning module, we take into consideration thousands to tens of thousands of point clouds, thousands of points in a path representing our own route, and polygons representing obstacles and detection areas in the surroundings, and we repeatedly create paths based on them. Therefore, we access the contents of the point clouds and paths multiple times using for-loops. In most cases, the bottleneck lies in these naive for-loops. Here, understanding Big O notation and reducing the order of computational complexity directly leads to performance improvements.

For example, in `detection_area` module in `behavior_velocity_planner` node, there is a program that checks whether each point cloud is contained in each detection area by. Below is the pseudocode.

```
for ( area : detection_areas )
  for ( point : point_clouds )
    if ( boost::geometry::within(point, area) )
      // do something with O(1)
```

Let `N` be the size of `point_clouds` and `M` be the size of `detection_areas`, then the computational complexity of this program is O(N^2 * M), since the complexity of `within` is O(N). Here, given that most of the point clouds are located far away from a certain detection area, a certain optimization can be achieved. First, calculate the minimum enclosing circle that completely covers the detection area, and then check whether the points are contained in that circle. Most of the point clouds can be quickly ruled out by this method, we don’t have to call the `within` function in most cases. Below is the pseudocode after optimization.

```
for ( area : detection_areas )
  calc_minimum_enclosing_circle(area)
  for ( point : point_clouds )
    if ( point is in the circle )
      if ( boost::geometry::within(point, area) )
        // do something with O(1)
```

By using O(N) algorithm for minimum enclosing circle, the computational complexity of this program is reduced to almost O(N * (N + M)) (note that the exact computational complexity does not really change).
Here is the [Pull Request](https://github.com/autowarefoundation/autoware.universe/pull/2846).

Another example is in `map_based_prediction` node. There is a program which calculates signed arc length from a initial point to each point in a path. Below is the pseudocode:

```
for ( i = 0 .. path.size() )
  tmp = motion_utils::calcSignedArcLength(path, 0, i)
```

In `calcSignedArcLength` function, it simply adds each distance between adjacent points. Let `N` be the size of path, then the program’s computational complexity is O(N^2). If we think a little, we can see that there is a lot of unnecessary calculation being done. Since the distance from the initial point is being calculated every time, we can use cumulative sums to improve efficiency. Below is the pseudocode after optimization.

```
for ( i = 1 .. path.size() )
  tmp = sum(i - 1) + motion_utils::calcSignedArcLength(base_path, i - 1, i);
  sum(i) = tmp
```

The computational complexity becomes O(N) in this program.
Here is the [Pull Request](https://github.com/autowarefoundation/autoware.universe/pull/2883).