# How to Train SSD Mobilenetv2 Lite Inside traffic_light_ssd_fine_detector Node


SSD MobilenetV2 Lite  is a fine detector for traffic light recognition. This node is a part of the traffic light recognition and classification pipeline. You can download original repo and pretrained weights from here https://github.com/qfgaohao/pytorch-ssd. Output of this node is an array of traffic light bounding boxes and their labels and `traffic_light_classifier` node uses this output to classify the traffic light.

Neural Network supports two different annotation format as Open-Images and PASCAL-VOC datasets. You can train your dataset after editing it according to these formats.

Requirements are:
- `Python 3.6+`
- `Caffe2`
- `torchvision==0.13.0+cu116`
- `torch==1.12.0+cu116`
- `onnx==1.8.0`
- `onnxruntime==1.12.1`

## How to Train Model
If we are going to train in open images data format first we divide dataset as train test validation subsets and put them in a folder named as `open_images` . Also there should be train.csv test.cvs and validation.csv inside open-images folder.
After this step we should change label names `BACKGROUND` `traffic_light` to  in models/open-images-model-labels.txt file. Then we can start training by running following command:
```bash
python train_ssd.py --dataset_type open_images --datasets ~/data/open_images --net mb2-ssd-lite --pretrained_ssd models/mb2-ssd-lite-mp-0_686.pth --scheduler cosine --lr 0.01 --t_max 100 --validation_epochs 5 --num_epochs 200 --base_net_lr 0.001  --batch_size 8

```
To see tensorboard logs run following command:
```bash
tensorboard --logdir=runs
```

## How to Evaluate Model
To evaluate the model run following command:
```bash
python eval_ssd.py --net mb2-ssd-lite ~/open-images/test --trained_model models/mb2-ssd-lite-mp-0_686.pth --label_file models/open-images-model-labels.txt
```

## How to Convert .pth to ONNX
To convert the model to ONNX run following command:
```bash
 python convert_to_caffe2_models.py mb2-ssd-lite models/mb2-ssd-lite-mp-0_686.pth models/open-images-model-labels.txt
```

## How to Run Model Inference on Image
To run the example run following command:
```bash
python run_ssd_example.py mb2-ssd-lite  models/mb2-ssd-lite-mp-0_686.pth models/open-images-model-labels.txt /home/user/folder-of-test-iamges


