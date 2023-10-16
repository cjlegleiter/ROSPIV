# ROSPIV
Robot Operating System Particle Image Velocimetry: ROSPIV

## Description
ROSPIV is a complement to the Toolbox for River Velocimetry using Images from Aircraft, or TRiVIA for short, that is intended to facilitate estimation of surface flow velocities in river channels from various types of remotely sensed data acquired with a nadir-viewing geometry (i.e., looking straight down). Spatially distributed information on flow speed is derived using a computational method known as Particle Image Velocimetry (PIV). Whereas the TRiVIA end user software package provides a graphical user interface conducive to post-processing, the ROSPIV branch is intended for real-time application.

## Installation
For guidance on setting up a ROS environment on a Windows computer, please refer to [ROS4windows.pdf](https://code.usgs.gov/wma/osd/trivia/-/blob/ROSPIV/ROSPIV/ROS4windows.pdf). The user's guide included in this branch provides detailed instructions on how to set up and use the ROSPIV network: [ROSPIVusersGuide.pdf](https://code.usgs.gov/wma/osd/trivia/-/blob/ROSPIV/ROSPIV/ROSPIVusersGuide.pdf).

## Usage
For an example application of the ROSPIV network that illustrates how to use the network using data sets included in this repository or freely available through the USGS ScienceBase catalog, please refer to: [ROSPIVusersGuide.pdf](https://code.usgs.gov/wma/osd/trivia/-/blob/ROSPIV/ROSPIV/ROSPIVusersGuide.pdf).

The ROSPIV folder on the ROSPIV branch of the parent TRiVIA repository contains the MATLAB source code and related helper functions used to develop the ROSPIV network: [ROSPIV](https://code.usgs.gov/wma/osd/trivia/-/tree/ROSPIV/ROSPIV). 

## Support
For guidance on setting up a ROS environment on a Windows computer, please refer to [ROS4windows.pdf](https://code.usgs.gov/wma/osd/trivia/-/blob/ROSPIV/ROSPIV/ROS4windows.pdf). The user's guide included in this branch provides detailed instructions on how to set up and use the ROSPIV network: [ROSPIVusersGuide.pdf](https://code.usgs.gov/wma/osd/trivia/-/blob/ROSPIV/ROSPIV/ROSPIVusersGuide.pdf).

For technical questions or issues regarding the ROSPIV network, please contact the lead developer at [cjl@usgs.gov](mailto:cjl@usgs.gov).

## Roadmap
This initial release of the ROSPIV network focuses on estimating surface flow velocities from nadir-viewing image time series. Possibilities for future enhancement of the software include allowing for other types of sensors, incorporating data from a laser range finder so that pixel sizes can be calculated as images are acquired, and improving run time.

## Contributing
If you have additional feature requests or would like to collaborate to improve the ROSPIV network, please contact the lead developer at [cjlo@usgs.gov](mailto:cjl@usgs.gov).

## Authors and acknowledgment
Carl J. Legleiter ([cjl@usgs.gov](mailto:cjl@usgs.gov)) is the lead developer for the ROSPIV network and is supported through the USGS Water Mission Area's Next Generation Water Observing System (NGWOS). Michael Dille assisted with conversion of an existing PIV codebase to a ROS-based implementation. The core PIV algorithm included in the ROSPIV network was adapted from the [PIVlab Toolbox](https://github.com/Shrediquette/PIVlab) developed by William Thielicke and described in the following publications:
- Thielicke, W., & Stamhuis, E. J. (2014). PIVlab - Towards User-friendly, Affordable and Accurate Digital Particle Image Velocimetry in MATLAB. Journal of Open Research Software, 2(1), e30. [https://doi.org/10.5334/jors.bl](https://doi.org/10.5334/jors.bl)
- Thielicke, W., & Sonntag, R. (2021). Particle Image Velocimetry for MATLAB: Accuracy and enhanced algorithms in PIVlab. Journal of Open Research Software, 9(1), 12. [https://doi.org/10.5334/jors.334](https://doi.org/10.5334/jors.334)

## License
The ROSPIV network is licensed under the Creative Commons Zero v1.0 Universal [LICENSE.txt](https://code.usgs.gov/wma/osd/trivia/-/blob/ROSPIV/ROSPIV/LICENSE.txt). 

## Project status
The ROSPIV project is ongoing and the lead developer is committed to refining the initial version, adding new features, and responding to user needs.

## USGS project metadata
The USGS IPDS record associated with this software is: IP-144279

The Digital Object Identifier (DOI) for this software is: [https://doi.org/10.5066/P9AD3VT3](https://doi.org/10.5066/P9AD3VT3)

## Disclaimer
This software has been approved for release by the U.S. Geological Survey (USGS). Although the software has been subjected to rigorous review, the USGS reserves the right to update the software as needed pursuant to further analysis and review. No warranty, expressed or implied, is made by the USGS or the U.S. Government as to the functionality of the software and related material nor shall the fact of release constitute any such warranty. Furthermore, the software is released on condition that neither the USGS nor the U.S. Government shall be held liable for any damages resulting from its authorized or unauthorized use.

Any use of trade, firm, or product names is for descriptive purposes only and does not imply endorsement by the U.S. Government.

## Citation
This information product is in the public domain, however citation is appreciated.

Suggested citation:
Legleiter, C.J., 2023, TRiVIA - Toolbox for River Velocimetry using Images from Aircraft (ver. 1.7.0, April, 2023): U.S. Geological software release, https://doi.org/10.5066/P9AD3VT3.