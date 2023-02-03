# How to Train Different Neural Network Models from Pytorch Hub inside Traffic Light Classifier Node

Currently in Autoware , the traffic light classifier node uses  pre-trained MobileNet V2 neural network model from Pytorch Hub. Also you can train a new neural network model using the Pytorch Hub model as a base model. This guide explains how to train a new neural network model and use it in the traffic light classifier node.

After creating custom dataset, you can download pretrained models or you can train a model from scratch. If you want to train from scratch, you can use `pretrained=False`  option when use `torchvision`models.

All models in referenced link trained and converted the `onnx` format. After that trained models tested on `traffic_light_classifier`.<https://github.com/qfgaohao/pytorch-ssd>.