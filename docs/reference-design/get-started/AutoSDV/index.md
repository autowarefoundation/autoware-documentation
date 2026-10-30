# AutoSDV

## Overview

<p align="center">
  <img src="images/logo_brand_black.png" alt="AutoSDV Logo" width="300"/>
</p>

The AutoSDV project (Autoware Software-Defined Vehicle) is an affordable 1/10 scale autonomous driving platform developed by NEWSLab at National Taiwan University. It provides a complete stack from hardware specifications to software implementation for educational and research institutes.

## Platform models

| Feature          | Base        | 360° LiDAR            | Solid-State        | Connected          |
| ---------------- | ----------- | --------------------- | ------------------ | ------------------ |
| **LiDAR**        | None        | Velodyne VLP-32C      | Seyond Robin-W     | Seyond Robin-W     |
| **LiDAR FOV**    | -           | 360° × 40°            | 120° × 25°         | 120° × 25°         |
| **LiDAR Range**  | -           | 100m                  | 150m               | 150m               |
| **Localization** | Vision only | NDT (full autonomous) | Development needed | Development needed |
| **Connectivity** | Standard    | Standard              | Standard           | 5G (Ataya/MOXA)    |
| **Battery Life** | 40 min      | 40 min                | 40 min             | 40 min             |

<table align="center" border="0" style="margin: 2em 0;">
  <tr>
    <td align="center" valign="bottom">
      <img src="https://github.com/NEWSLabNTU/AutoSDV/blob/6a5099b8f098cb0b0eac59fe72e6b6a7af006898/figures/model_robin-w.webp?raw=true" alt="Robin-W Solid-State LiDAR Kit" width="80%"/>
    </td>
    <td align="center" valign="bottom">
      <img src="https://github.com/NEWSLabNTU/AutoSDV/blob/6a5099b8f098cb0b0eac59fe72e6b6a7af006898/figures/model_velodyne_32c.webp?raw=true" alt="Velodyne 32C LiDAR Kit" width="80%"/>
    </td>
    <td align="center" valign="bottom">
      <img src="https://github.com/NEWSLabNTU/AutoSDV/blob/6a5099b8f098cb0b0eac59fe72e6b6a7af006898/figures/model_cube1_moxa-5g.webp?raw=true" alt="MOXA 5G Kit" width="80%"/>
    </td>
  </tr>
  <tr>
    <td align="center"><b>Solid-State LiDAR</b></td>
    <td align="center"><b>360° LiDAR</b></td>
    <td align="center"><b>5G Connected</b></td>
  </tr>
</table>

## Core components

| Component    | Specification                |
| ------------ | ---------------------------- |
| **Chassis**  | Tekno TKR9500 (16×11×5 inch) |
| **Computer** | NVIDIA Jetson AGX Orin 64GB  |
| **Storage**  | 1TB NVMe SSD                 |
| **Camera**   | ZED X Mini Stereo            |
| **IMU**      | MPU9250 9-axis               |
| **Motor**    | Brushless 4274/1500kv        |

## Software

AutoSDV runs on Autoware with ROS 2, providing autonomous navigation capabilities for outdoor environments. The AutoSDV repository defines all version dependencies in its [`versions.yaml`](https://github.com/NEWSLabNTU/AutoSDV/blob/main/versions.yaml) file.

| Component       | Details                                         |
| --------------- | ----------------------------------------------- |
| **Autoware**    | Autoware 2025.02 (AutoSDV v0.1.0)               |
| **JetPack**     | 6.0                                             |
| **ROS**         | ROS 2 Humble                                    |
| **Source Code** | [GitHub](https://github.com/NEWSLabNTU/AutoSDV) |

AutoSDV v0.1.0 is based on Autoware 2025.02, a release from before Autoware 2.0.0. The AutoSDV [`develop` branch](https://github.com/NEWSLabNTU/AutoSDV/blob/develop/versions.yaml) is AutoSDV 0.2.0, which is still in development. It uses Autoware 1.5.0 and JetPack 6.2.2 or newer.

## Resources

- **Documentation**: [AutoSDV Book](https://newslabntu.github.io/autosdv-book/)
- **Source Code**: [GitHub Repository](https://github.com/NEWSLabNTU/AutoSDV)
- **Issues**: [GitHub Issues](https://github.com/NEWSLabNTU/AutoSDV-book/issues)

## Contributors

- NEWSLab, National Taiwan University (Hsiang-Jui Lin, Chi-Sheng Shih)
