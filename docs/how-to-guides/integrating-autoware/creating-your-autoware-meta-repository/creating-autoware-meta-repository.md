# Creating an Autoware meta-repository

## What is Meta-repository?

A meta-repository is a repository that contains references or information about other repositories. It serves as a central point of reference or coordination for multiple repositories.

[Autoware](https://github.com/autowarefoundation/autoware) is a meta-repository. It contains references to several other repositories that provide source code used to realize autonomous driving.

## How to create and customize your autoware meta-repository

### 1. Create autoware repository

Most of the packages within the Autoware meta-repository are designed to be vehicle-agnostic, meaning they are not tied to any specific vehicle type. However, when integrating Autoware with your vehicle, it becomes necessary to create vehicle-specific packages tailored to your specific vehicle's implementation. Once you've developed these vehicle-specific packages, you can incorporate references to them within your Autoware meta-repository. This allows you to combine and utilize Autoware's vehicle-agnostic packages with your custom, vehicle-specific packages.

To create your own Autoware meta-repository, you can start by forking the [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) repository. For guidance on forking a repository, you can refer to [GitHub Docs](https://docs.github.com/en/get-started/quickstart/fork-a-repo).


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
- [creating-vehicle-interface-package](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/creating-a-vehicle-interface-for-an-ackermann-kinematic-model/)
- [customizing-for-differential-drive-model](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/customizing-for-differential-drive-model/)

Please remember to add all your custom packages, such as interfaces and descriptions, to your `autoware.repos` to ensure that your packages are properly included and managed within the Autoware repository.

By using Ansible and VCS, you can automatically set up your Autoware.
`autoware.repos` file manages the configuration of multiple repositories.

Note: VCS stands for Version Control System, such as Git or Subversion.
