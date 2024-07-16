# Machine learning models

The Autoware perception stack uses models for inference. These models are automatically downloaded as part of the `setup-dev-env.sh` script.

The models are hosted by Web.Auto.

Default models directory (`data_dir`) is `~/autoware_data`.

## Download instructions

Please follow the download instruction in [autoware download instructions](https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/README.md#L15) for updated models downloading.

The models can be also downloaded manually using download tools such as `wget` or `curl`. The latest urls of weight files and param files for each model can be found at [autoware main.yaml file](https://github.com/autowarefoundation/autoware/blob/main/ansible/roles/artifacts/tasks/main.yaml)

The example of downloading `lidar_centerpoint` model:

$ mkdir -p ~/autoware_data/lidar_centerpoint/
$ wget -P ~/autoware_data/lidar_centerpoint/ \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint.onnx> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint.onnx> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_voxel_encoder_centerpoint_tiny.onnx> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/pts_backbone_neck_head_centerpoint_tiny.onnx> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/centerpoint_ml_package.param.yaml> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/centerpoint_tiny_ml_package.param.yaml> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/centerpoint_sigma_ml_package.param.yaml> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/detection_class_remapper.param.yaml> \
 <https://awf.ml.dev.web.auto/perception/models/centerpoint/v2/deploy_metadata.yaml>
