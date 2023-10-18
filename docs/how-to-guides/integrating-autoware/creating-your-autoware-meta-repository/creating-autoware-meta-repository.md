# Creating an Autoware meta-repository

## What is Meta-repository?

A meta-repository is a repository that contains references or information about other repositories. It serves as a central point of reference or coordination for multiple repositories.

[Autoware](https://github.com/autowarefoundation/autoware) is a meta-repository. It contains references to several other repositories that provide source code used to realize autonomous driving.

## How to create and customize your autoware meta-repository

### 1. Create your autoware repository

Most of the packages within the Autoware meta-repository are designed to be vehicle-agnostic, meaning they are not tied to any specific vehicle type. However, when integrating Autoware with your vehicle, it becomes necessary to create vehicle-specific packages tailored to your specific vehicle's implementation. Once you've developed these vehicle-specific packages, you can incorporate references to them within your Autoware meta-repository. This allows you to combine and utilize Autoware's vehicle-agnostic packages with your custom, vehicle-specific packages.

To create your own Autoware meta-repository, you can start by forking the [autowarefoundation/autoware](https://github.com/autowarefoundation/autoware) repository. For guidance on forking a repository, you can refer to [GitHub Docs](https://docs.github.com/en/get-started/quickstart/fork-a-repo).


```bash
git clone https://github.com/YOUR_NAME/autoware.git
```
Note: If you set up multiple types of vehicles, adding a suffix like `autoware.vehicle_A` or `autoware.vehicle_B` is recommended.

Then, you can create forks of different Autoware repositories that you might need to customize. Some of the packages you might need to customize include the "vehicle_description", the "sensor_kit_description", `individual_params` and the "vehicle-interface" packages. Said packages contain information and parameters about your vehicle dimensions and sensor configuration.

Note that [Autoware](https://github.com/autowarefoundation/autoware) contains sample packages that you may use as templates (sample_vehicle_description, sample_sensort_kit, and individual_parameters packages, for example). But you may also need to create a package from scratch to define as autoware-vehicle interface. For more information, please take a look on the following guides:

- [creating-vehicle-and-sensor-description packages](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-description/creating-vehicle-and-sensor-description)
- [creating-vehicle-interface-package](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/creating-a-vehicle-interface-for-an-ackermann-kinematic-model/)
- [customizing-for-differential-drive-model](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/customizing-for-differential-drive-model/)


### 2. Referencing a custom repository on your Autoware meta-repository

Once you create a custom package that needs to be referenced by your autoware meta-repository, it should be referenced in your `autoware.repos` file.

As an Example: If you fork `individual_params` and rename it to `autoware_individual_params.vehicle_A` you would make the following changes to your `autoware.repos` file on your autoware meta-repository fork:

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

### 3. Setting up your workspace

By using Ansible and VCS, you can automatically set up your Autoware.
`autoware.repos` file manages the configuration of multiple repositories.

Note: VCS stands for Version Control System, such as Git or Subversion.

Once you have forked or created your custom Autoware repositories and added their references to `autoware.repos`, you can import them to your autoware project src folder in a similar manner to that of the [Autoware source installation guide](https://autowarefoundation.github.io/autoware-documentation/main/installation/autoware/source-installation/#how-to-set-up-a-workspace) by:

   ```bash
   cd autoware.your_vehicle_name
   mkdir src
   vcs import src < autoware.repos
   ```
then you can build and test your packages.

