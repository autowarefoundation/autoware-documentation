# MORAI Sim: Drive

!!! note

    Any kind of for-profit activity with the trial version of the MORAI SIM:Drive is strictly prohibited.

## Hardware requirements

| Minimum PC Specs |                                                      |
| :--------------- | :--------------------------------------------------- |
| OS               | Windows 10, Ubuntu 20.04, Ubuntu 18.04, Ubuntu 16.04 |
| CPU              | Intel i5-9600KF or AMD Ryzen 5 3500X                 |
| RAM              | DDR4 16GB                                            |
| GPU              | RTX2060 Super                                        |

| Required PC Specs |                                                      |
| :---------------- | :--------------------------------------------------- |
| OS                | Windows 10, Ubuntu 20.04, Ubuntu 18.04, Ubuntu 16.04 |
| CPU               | Intel i9-9900K or AMD Ryzen 7 3700X (or higher)      |
| RAM               | DDR4 64GB (or higher)                                |
| GPU               | RTX2080Ti or higher                                  |

## Application and Download

Only for AWF developers, trial license for 3 months can be issued.
Download the [application form](https://drive.google.com/file/d/1SO9hAr2-828MNl410xSABp3znHaR-AWV/view?usp=sharing) and send to [Hyeongseok Jeon](#technical-support)

After the trial license is issued, you can login to MORAI Sim:Drive via Launchers ([Windows](https://drive.google.com/file/d/1NMd2kInUALXYosRMtOHDPPGou9yCWMKK/view?usp=sharing)/[Ubuntu](https://drive.google.com/file/d/1qmA_1eUDyNJ85AeAzSxZRQaDbR_Sc76R/view?usp=sharing))

CAUTION: Do not use the Launchers in the following manual


	cd ~/${autoware installed directory}
	source /opt/ros/humble/setup.bash
	source install/setup.bash

	cd ~/${autoware installed directory}/src/universe/autoware.universe/tools/simulator_test/simulator_compatibility_test/test_sim_common_manual_testing/
	python3 -m pytest .