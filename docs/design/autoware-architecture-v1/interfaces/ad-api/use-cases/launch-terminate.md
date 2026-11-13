# Launch and terminate

AD API does not have an API to launch or terminate Autoware. The application starts Autoware as a process, for example with `ros2 launch autoware_launch autoware.launch.xml`, and stops that process to terminate it.

The AD API nodes start together with Autoware. The `launch_api` argument of `autoware.launch.xml` is `true` by default. It includes `autoware_api.launch.xml` of `tier4_autoware_api_launch`, which starts the default AD API implementation (`default_adapi.launch.py` of `autoware_default_adapi_universe`).

## Related API

- {{ link_ad_api('/api/interface/version') }}

## Sequence

{{ diagram("sequence/launch-terminate.svg", "The application starts Autoware, checks the interface version, and stops Autoware before the shutdown.") }}

<div class="aw-steps" markdown>

1. **Start Autoware.** The application starts Autoware, for example as a system service or with `ros2 launch`. The AD API nodes start with it.
2. **Check the interface version.** Call {{ link_ad_api('/api/interface/version') }} until it responds. The response shows that AD API is ready. The version follows [Semantic Versioning](https://semver.org/), so a different major version means that the application is not compatible.
3. **Use the vehicle.** Continue with the other use cases, for example [initialize the pose](initialize-pose.md).
4. **Terminate Autoware.** Stop the process that runs Autoware, then shut down the computer.

</div>
