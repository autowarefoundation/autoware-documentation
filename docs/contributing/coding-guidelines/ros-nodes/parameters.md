# Parameters

Autoware has the following two types of parameters:

- `Node Parameter`: the reference parameter for the node package
  - e.g., [the parameter for the `behavior_path_planner` package](https://github.com/autowarefoundation/autoware.universe/tree/main/planning/behavior_path_planner/config)
- `Launch Parameter`: [the reference parameter for the `autoware_launch` package](https://github.com/autowarefoundation/autoware_launch/tree/main/autoware_launch/config)

## How is `Launch Parameter` loaded in Autoware?

Here we use the `planning` module as an example component, but this applies to all of the other components (`sensing`, `localization`, `system`, etc) as well.

`Launch Parameter` is used when launching [`autoware.launch.xml`](https://github.com/autowarefoundation/autoware_launch/blob/main/autoware_launch/launch/autoware.launch.xml) in the `autoware_launch` package.
`autoware.launch.xml` may include `tier4_planning_component.launch.xml` in which you call `tier4_planning_launch` with the arguments for `Launch Parameter` provided. Currently, the launch file is stored as `autoware_launch/launch/components/tier4_planning_components.launch.xml`.

```xml
<!-- autoware_launch: autoware_launch/launch/autoware.launch.xml -->
...
<include file="$(find-pkg-share autoware_launch)/launch/tier4_planning_component.launch.xml">
  ...
</include>
...
```
```xml
<!-- autoware_launch: autoware_launch/launch/components/tier4_planning_components.launch.xml -->
...
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <arg name="package_A_param_path" value="$(find-pkg-share autoware_launch)/config/planning/package_A.param.yaml"/>
</include>
...
```

## Adding a new `Launch Parameter`

If you want to customize the parameter for a package, a recommended way is to create a new `Launch Parameter` file.

- Add the parameter file in `autoware_launch/config`
- Write a path to the parameter file in the component launch file in `autoware_launch` (e.g. `autoware_launch/launch/components/tier4_planning_components.launch.xml`)
```xml
<!-- autoware_launch: autoware_launch/launch/components/tier4_planning_components.launch.xml -->
...
<include file="$(find-pkg-share tier4_planning_launch)/launch/planning.launch.xml">
  <arg name="package_A_param_path" value="$(find-pkg-share autoware_launch)/config/planning/package_A_customized.param.yaml"/>
</include>
...
```

- Load the parameter using the above argument, e.g. as follows.
```xml
<!-- autoware.universe: launch/tier4_planning_launch/launch/.../package_A.launch.xml -->
...
<include file="$(find-pkg-share package_A)/launch/package_A.launch.xml">
  <arg name="param_path" value="$(var package_A_param_path)"/>
</include>
...
```

## Maintaining your custom parameters

For example, if you want to integrate Autoware with your vehicles, you may need to tune some parameters in Autoware. In this case, you may need to maintain your custom parameters somehow. A recommended way is to use the `Launch Parameter` in `autowarefoundation/autoware_launch` as your custom parameters. Please fork the repository (e.g. `YOUR_ACCOUNT/autoware_launch.YOURS`) and modify the parameters as you want.
