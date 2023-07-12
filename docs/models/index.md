# Machine learning models

The Autoware perception stack uses models for inference. These models are automatically downloaded if using `ansible`, but they can also be downloaded manually.

## ONNX model files

### Download instructions

The ONNX model files are stored in a common location, hosted by Web.Auto

Any tool that can download files from the web (e.g. `wget` or `curl`) is the only requirement for downloading these files:

```console
$ wget https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx \
       https://awf.ml.dev.web.auto/perception/models/coco.names \
       https://awf.ml.dev.web.auto/perception/models/hdl-64.caffemodel \
       https://awf.ml.dev.web.auto/perception/models/label.txt \
       https://awf.ml.dev.web.auto/perception/models/lamp_labels.txt \
       https://awf.ml.dev.web.auto/perception/models/mb2-ssd-lite-tlr.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_mobilenetv2.onnx \
       https://awf.ml.dev.web.auto/perception/models/vlp-16.caffemodel \
       https://awf.ml.dev.web.auto/perception/models/vls-128.caffemodel \
       https://awf.ml.dev.web.auto/perception/models/voc_labels_tl.txt \
       https://awf.ml.dev.web.auto/perception/models/yolov3.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov4.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5l.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5m.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5s.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolov5x.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-tiny.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.onnx \
       https://awf.ml.dev.web.auto/perception/models/yolox-sPlus-opt.EntropyV2-calibration.table \
       https://awf.ml.dev.web.auto/perception/models/label.txt \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_mobilenetv2_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_mobilenetv2_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_mobilenetv2_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_efficientNet_b1_batch_1.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_efficientNet_b1_batch_4.onnx \
       https://awf.ml.dev.web.auto/perception/models/traffic_light_classifier_efficientNet_b1_batch_6.onnx \
       https://awf.ml.dev.web.auto/perception/models/lamp_labels.txt
```
