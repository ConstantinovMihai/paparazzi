# README: FlyingStars (Group 5, 2022-2023)

![Happy Kevin](https://github.com/kmalkow/paparazzi/blob/assets/flyingstars.jpeg?raw=true)

## Introduction
This repository contains the work of **FlyingStars** (Group 5) for the course **AE4317 Autonomous Flight of Micro Air Vehicles**. This branch ([**FlyingStars_FINAL**](https://github.com/ConstantinovMihai/paparazzi/tree/FlyingStars_FINAL)) contains the final implementation of the code that we used during the competition, and runs specifically on the drone.

The main approach of our group was to detect the floor and green plants, avoid orange obstacles and detect divergence size in the CyberZoo. Based on the detections, the drone avoids the obstacles and finds the best path to continue flying.

In the end, we came **second!** in the competition of the academic year 2022-2023.

## Implementations
Throughout our prototyping and testing, we developed many different versions of our code with methods being used in different combinations and other avenues being explored. All of this work is in this repository and can be found through the various branches.

In the end, we had five working versions with different combinations of methods as detailed below:

- [**FlyingStars_A**](https://github.com/ConstantinovMihai/paparazzi/tree/FlyingStars_A): Orange Detection + Floor Colour Detection
- [**FlyingStars_B**](https://github.com/ConstantinovMihai/paparazzi/tree/FlyingStars_B): Orange Detection + Floor Colour Detection + Divergence Size
- [**FlyingStars_C**](https://github.com/ConstantinovMihai/paparazzi/tree/FlyingStars_C): Orange Detection + Floor Colour Detection + Plant Colour Detection + Divergence Size
- [**FlyingStars_C_SimVersion**](https://github.com/ConstantinovMihai/paparazzi/tree/FlyingStars_C_SimVersion): Orange Detection + Floor Colour Detection + Plant Colour Detection + Divergence Size (tuned for simulator)
- [**FlyingStars_D**](https://github.com/ConstantinovMihai/paparazzi/tree/FlyingStars_D): Orange Detection + Floor Colour Detection + Divergence Size + Divergence Difference