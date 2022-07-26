# Parameters

Autoware has following parameters

- `p1`: reference parameter for the node package
  - e.g., [the parameter for the `behavior_path_planner` package](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner/config)
- `p2`: reference parameter for the module launch
  - e.g., [the parameter for the `tier4_planning_launch` package](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/config/)
- `p3`: [reference parameter for the `autoware_launch` package](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config)

`p3` contains the same parameters as `p2`.
`p2` contains all the parameters of `p1`, and also [parameters involving multiple nodes]().

## Parameters to be used

### When launching autoware via the `autoware_launch` package

In [`autoware.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml) of the `autoware_launch` package, parameters in the `autoware_launch` package (`p3`) are used.
For example, the `tier4_planning_launch` package receives `$(find-pkg-share autoware_launch)/config/tier4_planning_launch` as the `tier4_planning_launch_param_path` argument as follows.

```xml
<!-- Planning -->
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <arg name="tier4_planning_launch_param_path" value="$(find-pkg-share autoware_launch)/config/tier4_planning_launch"/>
</include>
```

### When launching autoware via the module launch

In [`planning.launch.xml`](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_planning_launch/launch/planning.launch.xml) of the `tier4_planning_launch` package, the default variable for the `tier4_planning_launch_param_path` argument is `$(find-pkg-share tier4_planning_launch)/config`.

```xml
<arg name="tier4_planning_launch_param_path" default="$(find-pkg-share tier4_planning_launch)/config" description="tier4_planning_launch parameter path"/>
```

## Automatic Synchronization of parameters

There is a PR generated automatically to synchronize `p3` with `p2`.

## How to manage parameters

We explain how to manage parameters in various use cases

### Create a new package

- Locates the parameters of `p1` and `p2` for the new package in `autowarefoundation/autoware.universe`.
- Create a PR to `autowarefoundation/autoware.universe`.
- Sync PR to `autowarefoundation/autoware_launch` is created automatically.
  - Or you can create this PR manually.
- Merge both PRs.

### Modify general parameters

- Modify the parameters of `p1` and `p2` in `autowarefoundation/autoware.universe`.
- Create a PR to `autowarefoundation/autoware.universe`.
- Sync PR to `autowarefoundation/autoware_launch` is created automatically.
  - Or you can create this PR manually.
- Merge both PRs.

### Modify integrated parameters

Assuming that there is your forked repositories of `autowarefoundation/autoware` and `autowarefoundation/autoware_launch`.

This change does not affect the Autoware Foundation's repository.
Therefore, just modifying your forked repository is fine.

- Modify the parameters of `p3` in your forked `autoware_launch` repository.
- Create a PR and merge.
