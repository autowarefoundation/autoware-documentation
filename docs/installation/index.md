# Installation

## Target platforms

Autoware targets the platforms listed below. It may change in future versions of Autoware.

The Autoware Foundation provides no support on other platforms than those listed below.

### Architecture

- amd64
- arm64

### ROS version

- ROS 2 Galactic (**active development**)
- ROS 2 Humble (**will be supported in 2022**)

Refer to [REP-2000](https://www.ros.org/reps/rep-2000.html) for the system dependencies.

## Autoware

There are two ways to set up Autoware. Choose one according to your preference.

If any issues occur during installation, refer to the [Support page](https://autowarefoundation.github.io/autoware-documentation/main/support).

### 1. Docker installation

Docker can ensure that all developers in a project have a common, consistent development environment.
It is recommended for beginners, light users, people who do not use Ubuntu.

Refer to the [Docker installation guide](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/installation/autoware/docker-installation.md) for detailed steps.  
Refer to [autowarefoundation/autoware/docker/README.md](https://github.com/autowarefoundation/autoware/blob/main/docker/README.md) as well.

### 2. Source installation

Source installation is for the cases where more granular control of the installation environment is needed.
It is recommended for experienced users or people who want to customize their environment.  
Note that some problems may occur depending on your local environment.

Refer to the [source installation guide](https://github.com/autowarefoundation/autoware-documentation/blob/main/docs/installation/autoware/source-installation.md) for detailed steps.

## Related tools

Some other tools are required depending on the evaluation you want to do.
For example, to run an end-to-end simulation you need to install an appropriate simulator.

See [here](related-tools) for how to install the tools.

## Additional settings for developers

There are also tools and settings for developers, such as Shells or IDEs.

See [here](additional-settings-for-developers) for how to install the tools.
