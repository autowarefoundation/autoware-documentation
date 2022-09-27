# Parameters

Under Construction

Autoware has the following types of parameters:

- `Node Parameter`: the reference parameter for the node package
  - e.g., [the parameter for the `behavior_path_planner` package](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner/config)
- `Component Parameter`: the reference parameter for the component launch
  - e.g., [the parameter for the `tier4_planning_launch` package](https://github.com/autowarefoundation/autoware.universe/tree/main/launch/tier4_planning_launch/config/)
- `Launch Parameter`: [the reference parameter for the `autoware_launch` package](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config)

`Launch Parameter` contains the same parameters as `Component Parameter`.
`Component Parameter` contains all the parameters of `Launch Parameter`, and also parameters of multiple node instances from a single node package with different parameters.

![parameter-architecture](images/parameter-architecture.svg){: style="width:900px"}

## Parameters to be used

### When launching autoware via the `autoware_launch` package

In [`autoware.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml) of the `autoware_launch` package, parameters in the `autoware_launch` package (`Launch Parameter`) are used.
For example, the `tier4_planning_launch` package receives `$(find-pkg-share autoware_launch)/config/tier4_planning_launch` as the `tier4_planning_launch_param_path` argument as follows.

```xml
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <arg name="tier4_planning_launch_param_path" value="$(find-pkg-share autoware_launch)/config/tier4_planning_launch"/>
</include>
```

### When launching an individual component via the component launch

In [`planning.launch.xml`](https://github.com/autowarefoundation/autoware.universe/blob/main/launch/tier4_planning_launch/launch/planning.launch.xml) of the `tier4_planning_launch` package, the default variable for the `tier4_planning_launch_param_path` argument is `$(find-pkg-share tier4_planning_launch)/config`.
Therefore, parameters in the `tier4_planning_launch` package (`Component Parameter`) are used.

```xml
<arg name="tier4_planning_launch_param_path" default="$(find-pkg-share tier4_planning_launch)/config" description="tier4_planning_launch parameter path"/>
```

![parameters-to-be-used](images/parameters-to-be-used.svg){: style="width:1200px"}

## Automatic synchronization of parameters

We provide a GitHub Actions workflow called `sync-param-files` that automatically creates a PR to synchronize `Launch Parameter` with `Component Parameter`.

You can see the synchronization setting file [here](https://github.com/autowarefoundation/autoware_launch/blob/main/.github/sync-param-files.yaml).
Note that all parameter files are listed one by one.

![parameter-sync](images/parameter-sync.svg){: style="width:900px"}

## How to manage parameters

We explain how to manage parameters in some use cases.

### When creating a new package

Step 1. Update `autowarefoundation/autoware.universe`.

- Create new parameter files (both `Node Parameter` and `Component Parameter`) for the new package in `autowarefoundation/autoware.universe`
- Create a PR and merge.

Step 2. Update `autowarefoundation/autoware_launch`.

- Update [a synchronization setting file](https://github.com/autowarefoundation/autoware_launch/blob/main/.github/sync-param-files.yaml).
- Merge a sync PR from `autowarefoundation/autoware.universe` to `autowarefoundation/autoware_launch`.
  - Note: This PR will be created automatically with GitHub Actions, but you can also create the PR manually.

### When modifying general parameters

Step 1. Update `autowarefoundation/autoware.universe`.

- Modify the parameter files (both `Node Parameter` and `Component Parameter`) in `autowarefoundation/autoware.universe`.
- Create a PR and merge.

Step 2. Update `autowarefoundation/autoware_launch`

- If you add a new config file, update [a synchronization setting file](https://github.com/autowarefoundation/autoware_launch/blob/main/.github/sync-param-files.yaml).
- Merge a sync PR from `autowarefoundation/autoware.universe` to `autowarefoundation/autoware_launch`.
  - Note: This PR will be created automatically with GitHub Actions, but you can also create the PR manually.

### How to maintain your custom parameters

For example if you want to integrate Autoware with your own vehicles, you may need to tune some parameters in Autoware.
In this case, you may need to maintain your custom parameters somehow.
A recommended way is to use `Launch Parameter` in `autowarefoundation/autoware_launch` as your custom parameters.
Please fork the repository (e.g. as `YOUR_ACCOUNT/autoware_launch.YOURS`) and modify the parameters as you want.

One advantage of doing so is that the `sync-param-files` GitHub Actions helps you to keep `YOUR_ACCOUNT/autoware_launch.YOURS` up-to-date with `autowarefoundation/autoware.universe`.
The GitHub Actions workflow will automatically create sync PR on a regular basis, so you can incorporate the latest changes in `autowarefoundation/autoware.universe` when you want.
