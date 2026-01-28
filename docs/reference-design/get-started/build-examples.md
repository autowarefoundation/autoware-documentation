# Design Choices by Example

This page provides detailed component specifications for each reference configuration. Use these as starting points for your own build.

## Configuration Overview

| Configuration | ECU | Primary LiDAR | Localization |
|---------------|-----|---------------|--------------|
| [Campus](#campus-configuration) | AGX Orin 32/64GB | Ouster OS1-64 | RTK GNSS |
| [Indoor](#indoor-configuration) | AGX Orin | Ouster OSDome | LiDAR SLAM + markers |
| [High-Performance](#high-performance-configuration) | x86 + RTX 4090 | OS2 + OSDome | GNSS + SLAM fusion |
| [Budget](#budget-configuration) | Orin NX 16GB | Ouster OS0-32 | Single GNSS |
| [Compact](#compact-configuration) | Orin Nano/NX | Ouster OSDome | Camera SLAM |

---

## Campus Configuration

Balanced configuration for outdoor paved environments with pedestrian interaction.

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **ECU** | NVIDIA AGX Orin 32/64GB | 275 TOPS, 15-60W, proven Autoware support |
| **LiDAR** | Ouster OS1-64 | 120m range, 64 channels, reliable ROS 2 driver |
| **Cameras** | 2-4 wide-angle | Object classification, sign detection |
| **GNSS** | Dual-antenna RTK | Cm-level accuracy, heading estimation |
| **IMU** | Integrated or standalone | Motion estimation, sensor fusion |

**Performance Targets:**

| Metric | Value |
|--------|-------|
| LiDAR detection | 10 Hz |
| Camera detection | 15 Hz |
| Planning | 10 Hz |
| Control | 50 Hz |
| End-to-end latency | <200 ms |
| Power (ECU) | <50W |

**ODD Coverage:** LSA-CAM-0001 to 0040 (normal driving through intersections)

**Real World Examples:** [TalTech iseAuto](find-your-reference-design.md#taltech-iseauto), [NC A&T Aggie Auto](find-your-reference-design.md#nc-at-aggie-auto)

For detailed ECU specifications, see [ARM-based ECUs](../hardware-configuration/ECUs/ARM/index.md).

---

## Indoor Configuration

Optimized for GPS-denied warehouse and indoor structured environments.

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **ECU** | AGX Orin | Same compute as Campus |
| **LiDAR** | Ouster OSDome | 180 deg FOV for narrow aisles, 45m range |
| **Cameras** | Wide-angle + depth | SLAM features, close-range detection |
| **GNSS** | None | GPS-denied environment |
| **Localization** | LiDAR SLAM + markers | AprilTags or reflectors for accuracy |

**Key Differences from Campus:**

- No GNSS dependency
- Wide-FOV sensors for tight maneuvering
- Marker infrastructure for localization accuracy
- Lower speed limits (5-10 kph typical)

For indoor scenarios, see [Indoor ODD](../odd-definition/indoor/index.md).

---

## High-Performance Configuration

Maximum capability for research and complex scenarios.

| Component | Choice | Benefit |
|-----------|--------|---------|
| **ECU** | x86 Xeon/Core + RTX 4090 | Maximum compute, 24GB VRAM |
| **LiDAR** | Ouster OS2 (200m) + OSDome | Long range + wide FOV coverage |
| **Cameras** | 6-8 surround cameras | 360 deg visual coverage |
| **GNSS** | Multi-constellation RTK | Maximum accuracy |
| **Localization** | GNSS + LiDAR SLAM fusion | Maximum robustness |

**ODD Coverage:** Full LSA-CAM-0001 to 0082

**Trade-offs:** Higher cost, power consumption (150-450W), complexity

---

## Budget Configuration

Cost-optimized with reduced capabilities.

| Component | Choice | Trade-off vs Campus |
|-----------|--------|---------------------|
| **ECU** | Jetson Orin NX 16GB | Lower compute headroom |
| **LiDAR** | Ouster OS0-32 or OS1-32 | Shorter range (35-90m), fewer channels |
| **Cameras** | 1-2 cameras | Reduced coverage |
| **GNSS** | Single-antenna | Lower heading accuracy |

**ODD Limitations:** LSA-CAM-0001 to 0020 (basic scenarios only)

**Best For:** Simple fixed routes, low-traffic environments

---

## Compact Configuration

For small vehicles with space and weight constraints.

| Component | Choice | Rationale |
|-----------|--------|-----------|
| **ECU** | Orin Nano or NX | Small form factor, 7-25W |
| **LiDAR** | Ouster OSDome | Compact, wide FOV |
| **Carrier** | Integrated board | Minimizes size and cabling |
| **Cameras** | 1-2 compact modules | Essential coverage |
| **Localization** | Camera SLAM | No external infrastructure |

**Best For:** Sidewalk delivery robots, small indoor AGVs

---

## Component Selection Reference

### ECU Selection by Criteria

| Criteria | Recommendation |
|----------|----------------|
| Balanced cost/performance | AGX Orin 32GB |
| Minimum cost | Orin NX 16GB |
| Maximum compute | x86 + RTX 4090 |
| Minimum size/power | Orin Nano |
| Industrial environment | Neousys 9160-GC |
| Wide temperature range | Crystal Rugged AVC series |

For full ECU specifications:

- [ARM-based ECUs](../hardware-configuration/ECUs/ARM/index.md)
- [x86-based ECUs](../hardware-configuration/ECUs/x86_64/index.md)

### LiDAR Selection by Application

| Application | Recommendation | Range | Channels |
|-------------|----------------|-------|----------|
| Outdoor campus | Ouster OS1-64 | 90m | 64 |
| Long-range outdoor | Ouster OS2-128 | 200m | 128 |
| Indoor/warehouse | Ouster OSDome | 20m | 128 (180 deg FOV) |
| Budget outdoor | Ouster OS0-32 | 35m | 32 |
| Compact vehicles | Ouster OSDome | 20m | 128 |

For sensor options, see [Sensors and Actuators](../hardware-configuration/Sensors-and-Actuators/index.md).

### Localization Method by Environment

| Environment | Primary Method | Backup/Enhancement |
|-------------|----------------|-------------------|
| Outdoor, open sky | RTK GNSS | NDT matching |
| Outdoor, urban canyon | GNSS + LiDAR SLAM | Visual odometry |
| Indoor, structured | LiDAR SLAM | Marker-based (AprilTags) |
| Indoor, featureless | Marker-based | Wheel odometry |

### Middleware Selection

| Scenario | Recommendation | Rationale |
|----------|----------------|-----------|
| Wireless operation | Zenoh | Better packet handling |
| Wired, lowest latency | Cyclone DDS | Minimal overhead |
| Enterprise integration | RTI Connext | Commercial support |

For middleware setup, see [Middleware Configuration](../software-configuration/middleware-configuration/index.md).

---

## Next Steps

- Use [Find Your Reference Design](find-your-reference-design.md) to see real-world examples for each configuration
- Review [Hardware Configuration](../hardware-configuration/index.md) for component details
- Follow [Software Configuration](../software-configuration/index.md) for setup guides
