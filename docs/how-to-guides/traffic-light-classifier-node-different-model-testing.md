# How to Deploy Different Neural Network Models from Pytorch Hub inside Traffic Light Classifier Node
Currently in Autoware ,we can re-train compatible models as base  from Pytorch hub and use it in traffic_light_classifier node. Here is a tutorial link for training <https://pytorch.org/tutorials/advanced/super_resolution_with_onnxruntime.html>. 
This guide explains which neural network models are trained and tested in the traffic light classifier node and for future developments it can help about replace current model to another model without changing TensorRT layers.

After creating custom dataset, you can download default pretrained models. If you want to train without pretrained weights, you can use `pretrained=False`  option when use `torchvision`models.

All models in referenced link are  trained and converted the `onnx` format. Here is a link for this onnx conversion <https://deci.ai/blog/how-to-convert-a-pytorch-model-to-onnx/>.
After that trained models tested on the `traffic_light_classifier` node and here is link for which models are used <https://pytorch.org/vision/0.8/models.html>.

Requirements for the tested models are below:

- `torch==1.12.1+cu116`
- `torchvision==0.13.1+cu116`
- `onnx==1.13.0`

Currently, the traffic light classifier node supports the following models with requirements above:
- `MobileNet v2`
- `ResNet`
- `DenseNet`
- `SqueezeNet`
- `GoogLeNet`
- `ShuffleNet v2`
- `Wide ResNet`


These models are supported by TensorRT and can be converted  to `onnx` and `trt engine` files. Then they can be used directly in the `traffic_light_classifier` node.