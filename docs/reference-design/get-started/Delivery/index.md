<!-- cspell: ignore AVAS DCDC GOLDBOX HVAC -->

# Autoware delivery vehicle

This page describes the Autoware AW1, a concept for an autonomous delivery vehicle based on Autoware. The Reference Design Working Group discussed the concept in 2024. The full concept is in the [Autoware Delivery Vehicle Prototype (PDF)](Autoware_Delivery_Vehicle_Prototype.pdf).

!!! note

    The AW1 is a prototype concept, not a finished product. The concept defines the vehicle, its electronics, and example sensors. It does not yet select the drive-by-wire solution, the cameras, or the GNSS/IMU unit.

## Vehicle definition

The AW1 is an autonomous delivery vehicle that drives only on a private campus. It has these features:

- Symmetric vehicle body
- 4-wheel steering and crab steering (all wheels turn in the same direction, so the vehicle can move sideways)
- HVAC and an agile battery swap
- One side cargo door that opens automatically
- Variable height, to make unloading easier
- Light sensor, temperature sensor, and door sensor
- Headlights and taillights with turn signals, and license plate lights
- Horn, interior light, and an Acoustic Vehicle Alerting System (AVAS)
- 5G, BLE, and Wi-Fi connectivity
- Exterior screen
- Sensors: 1 LiDAR, 3 cameras, 1 short-range radar, and 1 GPS/IMU unit

## Dimensions and specifications

| Dimension          | Value (mm)  |
| ------------------ | ----------- |
| Length             | 3500        |
| Width              | 1900        |
| Height             | 2350 - 2660 |
| Internal clearance | 2100        |
| Ground clearance   | 40 - 200    |
| Variable height    | 160         |
| Wheelbase          | 2800        |

| Specification       | Value   |
| ------------------- | ------- |
| Tires               | 15"     |
| High voltage        | 96 V    |
| Battery             | 23 kWh  |
| Maximum speed       | 20 km/h |
| In-wheel motors (4) | 22 kW   |
| Wheel torque        | 840 Nm  |
| Payload             | 1000 kg |

The concept plans the first prototype in 8 months. The next 4 pre-series units follow in 4 to 5 months, and the next 5 units in 4 months. The concept says that this timeline can change when the details are refined.

## Electronics and electrical architecture

The figure shows where the electronic control units sit in the vehicle.

{{ diagram("images/delivery-vehicle-electronics.svg", "The AW1 has a motor controller at each wheel, a gateway, two control modules, an infotainment unit, and an autonomy computer.") }}

| Unit             | Proposed hardware                                                                                                                                                                       |
| ---------------- | --------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------- |
| Gateway/controls | A gateway and control unit based on the NXP S32G3 processor, for example the NXP GOLDBOX 3 vehicle networking development platform                                                      |
| Autonomy         | [Advantech MIC-733-AO](https://www.advantech.com/en-us/products/edge-ai-jetson-system/sub_9140b94e-bcfa-4aa4-8df2-1145026ad613), an industrial computer based on NVIDIA Jetson AGX Orin |
| Infotainment     | [ADLINK SP2-IMX8](https://www.adlinktech.com/Products/Panel_PCs_Monitors/Panel_PCs_Monitors/SP2-IMX8_Series), an Arm-based panel PC (i.MX8M)                                            |
| Control modules  | Steering, braking, battery management system (BMS), DC-DC converter (DCDC), on-board charger (OBC), access, and lighting                                                                |
| Motor control    | One controller for each in-wheel motor                                                                                                                                                  |

Most control modules come from the existing vehicle platform. The gateway module can control the sensors and actuators that are specific to the vehicle. The concept does not yet confirm that the GOLDBOX 3 is suitable for industrial use.

## Sensor suite

The concept gives these example sensors:

- LiDAR: Velodyne VLP-16 or Ouster OS0 series
- Radar: Continental ARS series

The cameras and the GNSS/IMU unit are open. For supported options, see [Cameras](../../reference-hw/cameras.md) and [IMU, AHRS & GNSS/INS](../../reference-hw/imu_ahrs_gnss_ins.md) in the reference hardware list.

The concept also lists options for the exterior lighting (turn signal, brake, and tail lights, and LED work lights) and the interior lighting (a flat LED downlight). To keep the design simple, the vehicle can use existing off-road lighting.

## Related pages

- [Find your reference design](../find-your-reference-design.md)
- [Low Speed Autonomy (LSA) guideline](https://autowarefoundation.github.io/LSA-reference-design-docs/main/), which covers low-speed vehicles on campus routes
- [Reference hardware](../../reference-hw/index.md)
