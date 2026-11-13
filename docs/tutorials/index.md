# Tutorials

The tutorials show how to do real tasks with Autoware, step by step. They assume that you installed Autoware and ran it in a [simulation](../simulation/index.md). Use the [installation](../installation/index.md) pages first if you have not done that.

The tutorials have three parts:

- Integrating Autoware: put Autoware on your own vehicle.
- Training machine learning models: train the models of Autoware with your own data.
- Others: short guides for development, debugging, and evaluation.

## Integrating Autoware

These tutorials take you from a new vehicle to a vehicle that drives with Autoware. Start with the [overview](integrating-autoware/overview.md), and then follow the steps in order:

1. [Create your Autoware repositories](integrating-autoware/creating-your-autoware-repositories/creating-autoware-repositories.md): make your own copy of the `autoware` meta-repository and the repositories for your vehicle and sensors.
2. [Create the vehicle and sensor models](integrating-autoware/creating-vehicle-and-sensor-model/index.md): describe the dimensions of your vehicle, the positions of your sensors, and calibrate the sensors.
3. [Create the vehicle interface](integrating-autoware/creating-vehicle-interface-package/vehicle-interface.md): connect Autoware to the drive-by-wire system of your vehicle.
4. [Create maps](integrating-autoware/creating-maps/index.md): make the pointcloud map and the vector map of your area.
5. [Launch Autoware](integrating-autoware/launch-autoware/index.md): configure and start each component on your vehicle.
6. [Tune parameters and performance](integrating-autoware/tuning-parameters-and-performance/tuning-parameters/index.md): adjust Autoware to your vehicle and your environment.
7. [Integrate your vehicle in AWSIM](integrating-autoware/awsim-integration/index.md): test your vehicle in a simulator.
8. [Integrate sensors](integrating-autoware/integrating-sensors/integrating-cameras.md): add sensors, for example cameras.

## Training machine learning models

[Training and deploying models](training-machine-learning-models/training-models.md) explains how to train the traffic light classifier, the CenterPoint 3D object detector, and the YOLOX traffic light detector with your own data. It also explains how to convert the trained models to ONNX for Autoware.

## Others

These guides each cover one topic.

Development:

- [Add a custom ROS message](others/add-a-custom-ros-message.md): define a new message package.
- [Advanced usage of colcon](others/advanced-usage-of-colcon.md): select packages, change the build type, and speed up builds with Ccache.
- [Applying Clang-Tidy to ROS packages](others/applying-clang-tidy-to-ros-packages.md): find and fix problems in C++ code.
- [Fixing dependent package versions](others/fixing-dependent-package-versions.md): save and restore the versions of all repositories in your workspace.
- [Determining component dependencies](others/determining-component-dependencies.md): list the packages, system libraries, and topics of a component.

Running and debugging:

- [Debug Autoware](others/debug-autoware.md): use logging, the ROS 2 command line tools, RViz, rqt, and a debugger.
- [Running Autoware without CUDA](others/running-autoware-without-cuda.md): select the object detector and the traffic light recognition that run on the CPU.
- [Using divided pointcloud map](others/using-divided-map.md): divide a large pointcloud map and load only the part near the vehicle.
- [Reducing start delays on real vehicles](others/reducing-start-delays.md): find and reduce the delays between a start command and the vehicle motion.

Evaluation and research:

- [An example procedure for adding and evaluating a new node](others/an-example-procedure-for-adding-and-evaluating-a-new-node.md): record data, develop a node, and evaluate it in simulation and on a vehicle.
- [Defining temporal performance metrics on components](others/defining-temporal-performance-metrics.md): choose the message rates and response times to measure.
- [Planning evaluation using scenarios](others/planning-evaluation-using-scenarios.md): create scenarios, run them in Scenario Simulator, and read the results of the Autoware Evaluator.
- [Lane detection methods](others/lane-detection-methods.md): compare lane detection and multitask detection methods.

For the evaluation of Autoware components in simulation, also see [Simulation and evaluation](../simulation/evaluation/index.md).
