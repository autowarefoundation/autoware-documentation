
# Creating autoware meta-repository

## What is Meta-repository?

A meta-repository is a repository that is used to manage multiple repositories. It serves as a centralized control point for referencing, configuring, and versioning other repositories.

autoware is manage multiple repositories.

We are performing automated environment setup of autoware using Ansible and repository management using vcs.

Note: vcs stands for Version Control System, such as Git or Subversion.

## How to create your autoware meta-repository
### 1. fork autoware repository
If you want to integrate Autoware into your vehicle, the first step is to create an Autoware meta-repository.

One easy way is to fork autowarefoundation/autoware and clone it. For how to fork a repository, refer to GitHub Docs.
```bash
git clone https://github.com/YOUR_NAME/autoware.git
```
If you set up multiple types of vehicles, adding a suffix like "autoware.vehicle_A" or "autoware.vehicle_B" is recommended

### 2. custamize your autoware.repos for your enviroment

You need to manage the repository for your own vehicle's Autoware.

For example, if you want to customize the parameters in your individual_params package or autoware_launch package to fit your vehicle, you can modify the configuration of each package and use them accordingly.

Please edit the parameters in Autoware's autoware_individual_params and autoware_launch packages to match your vehicle's specific requirements, as these packages provide sample parameters and may not be tailored to your vehicle by default.

If you want to fork autoware_individual_params and autoware_launch and make modifications, it would be as follows:

ex.1 If you fork individual_params and rename autoware_individual_params.vehicle_A
```bash
###before:
param/autoware_individual_params:
    type: git
    url: https://github.com/autowarefoundation/autoware_individual_params 
    version: main
###after:  
param/autoware_individual_params.vehicle_A:
    type: git
    url: https://github.com/YOUR_NAME/autoware_individual_params.vehicle_A
    version: main
```
ex.2 If you fork autoware_launch and rename autoware_launch.vehicle_A
```bash
###before:
launcher/autoware_launch:
    type: git
    url: ​https://github.com/autowarefoundation/autoware_launch 
    version: main
###after:
  launcher/autoware_launch:
    type: git
    url: ​https://github.com/YOUR_NAME/autoware_launch 
    version: main
```
Please refer to the following documentation link for instructions on how to create and customize each interface:
- [creating-vehicle-and-sensor-description](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-and-sensor-description/creating-vehicle-and-sensor-description)
- [creating-vehicle-interface-package](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/creating-vehicle-interface-for-ackerman-kinematic-model/)
- [customizing-for-differential-drive-model](https://autowarefoundation.github.io/autoware-documentation/main/how-to-guides/integrating-autoware/creating-vehicle-interface-package/customizing-for-differential-drive-model/)

Please remember to add any custom packages specific to your vehicle, such as interfaces or descriptions, to the `autoware.repos`. 
This ensures that your custom packages are properly included and managed within the Autoware repository.

## Fixing dependent package versions
Autoware manages dependent package versions in autoware.repos.
For example, let's say you make a branch in autoware.universe and add new features.
Suppose you update other dependencies with `vcs pull` after cutting a branch from autoware.universe. Then the version of autoware.universe you are developing and other dependencies will become inconsistent, and the entire Autoware build will fail.
We recommend saving the dependent package versions by executing the following command when starting the development.

```bash
vcs export src --exact > my_autoware.repos
```