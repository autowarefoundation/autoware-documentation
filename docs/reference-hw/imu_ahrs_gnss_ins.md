# IMU, AHRS & GNSS/INS

## **NovAtel GNSS/INS Sensors**

NovAtel GNSS/INS sensors which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | INS Rate | Roll, Pitch, Yaw Acc.                  | GNSS                                    | ROS 2 Driver  | Autoware Tested (Y/N) |
| ----------------------- | -------- | -------------------------------------- | --------------------------------------- | ------------- | --------------------- |
| PwrPak7D-E2             | 200 Hz   | R (0.013°)<br>P (0.013°)<br>Y (0.070°) | 20 Hz<br>L1 / L2 / L5<br> 555 Channels  | Y             | -                     |
| Span CPT7               | 200 Hz   | R (0.01°) <br>P (0.01°) <br>Y (0.03°)  | 20 Hz <br>L1 / L2 / L5 <br>555 Channels | Y             | -                     |

Link to ROS 2 driver:  
[https://github.com/swri-robotics/novatel_gps_driver/tree/dashing-devel](https://github.com/swri-robotics/novatel_gps_driver/tree/dashing-devel)

Link to company website:  
[https://hexagonpositioning.com/](https://hexagonpositioning.com/)

## **XSens GNSS/INS & IMU Sensors**

XSens GNSS/INS sensors which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | INS/IMU Rate | Roll, Pitch, Yaw Acc.            | GNSS                             | ROS 2 Driver  | Autoware Tested (Y/N) |
| ----------------------- | ------------ | -------------------------------- | -------------------------------- | ------------- | --------------------- |
| MTi-680G                | 2 kHz        | R (0.2°)<br>P (0.2°)<br>Y (0.5°) | 5 Hz<br>L1 / L2 <br>184 Channels | Y             | -                     |
| MTi-300 AHRS            | 2 kHz        | R (0.2°)<br>P (0.2°)<br>Y (1°)   | Not Applicable                   | Y             | -                     |

Link to ROS 2 driver:  
[http://wiki.ros.org/xsens_mti_driver](http://wiki.ros.org/xsens_mti_driver)

Link to company website:  
[https://www.xsens.com/](https://www.xsens.com/)

## **SBG GNSS/INS & IMU Sensors**

SBG GNSS/INS sensors which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | INS/IMU Rate        | Roll, Pitch, Yaw Acc.             | GNSS                            | ROS 2 Driver  | Autoware Tested (Y/N) |
| ----------------------- | ------------------- | --------------------------------- | ------------------------------- | ------------- | --------------------- |
| Ellipse-D               | 200 Hz, 1 kHz (IMU) | R (0.1°)<br>P (0.1°)<br>Y (0.05°) | 5 Hz<br>L1 / L2<br>184 Channels | Y             | Y                     |
| Ellipse-A (AHRS)        | 200 Hz, 1 kHz (IMU) | R (0.1°)<br>P (0.1°)<br>Y (0.8°)  | Not Applicable                  | Y             | -                     |

Link to ROS 2 driver:  
[https://github.com/SBG-Systems/sbg_ros2](https://github.com/SBG-Systems/sbg_ros2)

Link to company website:  
[https://www.sbg-systems.com/products/ellipse-series/](https://www.sbg-systems.com/products/ellipse-series/)

## **Applanix GNSS/INS Sensors**

  <!-- cspell: ignore  POSLV  POLYNAV -->

SBG GNSS/INS sensors which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | INS/IMU Rate | Roll, Pitch, Yaw Acc.               | GNSS                         | ROS 2 Driver  | Autoware Tested (Y/N) |
| ----------------------- | ------------ | ----------------------------------- | ---------------------------- | ------------- | --------------------- |
| POSLVX                  | 200 Hz       | R (0.03°)<br>P (0.03°)<br>Y (0.09°) | L1 / L2 / L5<br>336 Channels | Y             | Y                     |
| POSLV220                | 200 Hz       | R (0.02°)<br>P (0.02°)<br>Y (0.05°) | L1 / L2 / L5<br>336 Channels | Y             | Y                     |

Link to ROS 2 driver:  
[http://wiki.ros.org/applanix_driver](http://wiki.ros.org/applanix_driver)

Link to company website:  
[https://www.applanix.com/products/poslv.htm](https://www.applanix.com/products/poslv.htm)

## **PolyExplore GNSS/INS Sensors**

PolyExplore GNSS/INS sensors which has ROS 2 driver and tested by one or more community members are listed below:

| Supported Products List | INS/IMU Rate | Roll, Pitch, Yaw Acc.                 | GNSS                    | ROS 2 Driver  | Autoware Tested (Y/N) |
| ----------------------- | ------------ | ------------------------------------- | ----------------------- | ------------- | --------------------- |
| POLYNAV 2000P           | 100 Hz       | R (0.01°)<br>P (0.01°)<br>Y (0.1°)    | L1 / L2<br>240 Channels | Y             | -                     |
| POLYNAV 2000S           | 100 Hz       | R (0.015°)<br>P (0.015°)<br>Y (0.08°) | L1 / L2<br>40 Channels  | Y             | -                     |

Link to ROS 2 driver:  
[https://github.com/polyexplore/ROS2_Driver](https://github.com/polyexplore/ROS2_Driver)

Link to company website:  
[https://www.polyexplore.com/](https://www.polyexplore.com/)
