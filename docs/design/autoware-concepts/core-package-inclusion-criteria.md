# What belongs in Autoware Core

[Autoware Core](https://github.com/autowarefoundation/autoware_core) is the stable, quality-assured base of the Autoware stack.
This page explains what a package needs to live in Core, and how it gets there from Universe.

## What Core stands for

Core packages are the stable base of Autoware: they build, are tested, expose stable interfaces, and are maintained by the Autoware Foundation.
Universe is the opposite: a place for experimentation, vendor-specific code, and fast-moving research.

Adding a package to Core is a long-term commitment.
It must stay tested, reviewed, and released for everyone who builds on Core.
A package earns that place only by clearing the bar below.

!!! note "The litmus test"

    A package belongs in Core if you could hand it to a new user with no GPU, no vendor-specific stack, and no research models, and it would build, pass its tests, and speak only standardized interfaces.
    And if enough of the ecosystem depends on it that the Foundation should guarantee its stability.
    The one exception is shared GPU infrastructure, which lives in the optional CUDA layer described below.

## Requirements

A Core package must:

- **Depend only downward.** No dependency on any Universe package, only on other Core packages, ROS via rosdep, and well-established third-party libraries. (Universe depends on Core, never the reverse; see the [repository structure](../repository-structure.md) diagram.)
- **Use vendor-neutral interfaces.** No proprietary or company-namespaced packages (for example `tier4_*_msgs`). Communicate through the standard messages and `autoware_component_interface_specs`.
- **Keep the default build CPU-only.** Standard Core packages need no GPU, CUDA, or TensorRT. GPU code belongs only in the optional CUDA layer described below, never mixed into the default packages.
- **Be permissively licensed.** Apache 2.0 or compatible, with no restrictively licensed runtime dependency.
- **Be tested and green.** Unit tests, the coverage gate, and the full Core CI passing on every supported distribution (currently Humble and Jazzy). Core enforces a stricter clang-tidy profile than Universe, so passing there is not enough.
- **Be maintained.** Real maintainers, listed in `CODEOWNERS`, semantically versioned and released through Core.

It should also be mature: a stable, documented interface, validated beyond unit tests (ideally on a vehicle), and broadly useful rather than tied to one vendor or one research project.

## Getting into Core

New work starts in Universe. A package is promoted once it clearly meets the bar above.

Promotion is a deliberate port, not a copy: clean up dependencies, swap proprietary messages for standard ones, add tests.
Move foundations first (messages, utilities, base layers) so packages built on them can follow without breaking.
Package names do not change on promotion, so downstream code keeps working.

## The CUDA layer

Core ships in two forms: the default CPU-only packages, and an optional GPU layer, the `autoware_core_cuda` repository, built into separate CUDA images.
A package that needs CUDA or TensorRT can still be Core, but only in this layer, so the default build never requires a GPU.

The shared CUDA/TensorRT base (`autoware_tensorrt_common`, `autoware_tensorrt_plugins`, `autoware_cuda_utils`, `autoware_cuda_dependency_meta`) lives here.
It is Core-quality: it depends only on these packages, Core tooling, ROS, and one external CMake module, with nothing from Universe, and many downstream GPU packages build on it.
