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
    The one exception is shared GPU infrastructure, which goes into the optional CUDA layer described below.

## Requirements

A Core package must meet all of these requirements:

<div class="aw-features" markdown>

- :fa-cl-s fa-arrow-down: **Depend only downward**

  No dependency on any Universe package, only on other Core packages, ROS via rosdep, and well-established third-party libraries. Universe depends on Core, never the reverse. See the [repository structure](../repository-structure.md) diagram.

- :fa-cl-s fa-plug-circle-check: **Use vendor-neutral interfaces**

  No proprietary or company-namespaced packages (for example `tier4_*_msgs`). Communicate through the standard messages and `autoware_component_interface_specs`.

- :fa-cl-s fa-microchip: **Keep the default build CPU-only**

  Standard Core packages need no GPU, CUDA, or TensorRT. GPU code belongs only in the optional CUDA layer described below, never mixed into the default packages.

- :fa-cl-s fa-scale-balanced: **Be permissively licensed**

  Apache 2.0 or compatible, with no restrictively licensed runtime dependency.

- :fa-cl-s fa-circle-check: **Be tested and green**

  Unit tests, the coverage gate, and the full Core CI passing on every supported distribution (currently Humble and Jazzy). Core enforces a stricter clang-tidy profile than Universe, so passing there is not enough.

- :fa-cl-s fa-user-gear: **Be maintained**

  Real maintainers, listed in `CODEOWNERS`, semantically versioned and released through Core.

</div>

It should also be mature: a stable, documented interface, validated beyond unit tests (ideally on a vehicle), and broadly useful rather than tied to one vendor or one research project.

## Getting into Core

New work starts in Universe. A package is promoted once it clearly meets the bar above.
Promotion is a deliberate port, not a copy:

<div class="aw-steps" markdown>

1. **Clean up the package.** Clean up dependencies, swap proprietary messages for standard ones, and add tests.
2. **Move foundations first.** Move messages, utilities, and base layers first, so that packages built on them can follow without breaking.
3. **Keep the name.** Package names do not change on promotion, so downstream code keeps working.

</div>

## The CUDA layer

Core is planned to ship in two forms: the default CPU-only packages, and an optional GPU layer in the [`autoware_cuda`](https://github.com/autowarefoundation/autoware_cuda) repository, built into separate CUDA images.
A package that needs CUDA or TensorRT can still be Core, but only in this layer, so the default build never requires a GPU.

The shared CUDA/TensorRT base (`autoware_tensorrt_common`, `autoware_tensorrt_plugins`, `autoware_cuda_utils`, `autoware_cuda_dependency_meta`) is the first candidate for this layer.
It is Core-quality: it depends only on these packages, Core tooling, ROS, and one external CMake module, with nothing from Universe, and many downstream GPU packages build on it.

!!! info "Current state"

    The `autoware_cuda` repository does not contain packages yet.
    The shared CUDA/TensorRT base packages still live in [Autoware Universe](https://github.com/autowarefoundation/autoware_universe).
