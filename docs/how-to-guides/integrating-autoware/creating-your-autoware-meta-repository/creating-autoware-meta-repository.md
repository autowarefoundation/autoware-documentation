# Creating Autoware meta-repository

## What is Meta-repository?

A meta-repository is a repository that manages multiple repositories, and Autoware is one of them.
It serves as a centralized control point for referencing, configuring, and versioning other repositories.

By using Ansible and vcs, you can automatically set up your Autoware.
`autoware.repos` file manages the configuration of multiple repositories.

Note: vcs stands for Version Control System, such as Git or Subversion.

## How to create and customize your autoware meta-repository

### 1. Fork autoware repository

If you want to integrate Autoware into your vehicle, the first step is to create an Autoware meta-repository.

One easy way is to fork [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) and clone it.
For how to fork a repository, refer to [GitHub Docs](https://docs.github.com/en/get-started/quickstart/fork-a-repo).

```bash
git clone https://github.com/YOUR_NAME/autoware.git
```

If you set up multiple types of vehicles, adding a suffix like `autoware.vehicle_A` or `autoware.vehicle_B` is recommended

### 2. Customize your autoware.repos for your environment

You need to customize `autoware.repos` for your own vehicle's Autoware.

For example, if you want to customize the parameters in your `individual_params` or `autoware_launch` package to fit your vehicle, you can modify the configuration of each package and use them accordingly.

Please edit the parameters in Autoware's `autoware_individual_params` and `autoware_launch` packages to match your vehicle's specific requirements, as these packages provide sample parameters and may not be tailored to your vehicle by default.

If you want to fork `autoware_individual_params` and make modifications, it would be as follows:

Example: If you fork `individual_params` and rename `autoware_individual_params.vehicle_A`:

```diff
- param/autoware_individual_params:
-   type: git
-   url: https://github.com/autowarefoundation/autoware_individual_params
-   version: main
+ param/autoware_individual_params.vehicle_A:
+   type: git
+   url: https://github.com/YOUR_NAME/autoware_individual_params.vehicle_A
+   version: main
```

Please refer to the following documentation link for instructions on how to create and customize each `vehicle_interface`:

- [creating-vehicle-and-sensor-description](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-description/creating-vehicle-and-sensor-description)
- [creating-vehicle-interface-package](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/creating-vehicle-interface-for-ackerman-kinematic-model/)
- [customizing-for-differential-drive-model](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/customizing-for-differential-drive-model/)

Please remember to add all your custom packages, such as interfaces and descriptions, to your `autoware.repos` to ensure that your packages are properly included and managed within the Autoware repository.
