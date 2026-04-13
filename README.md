# Crane3DSim V2.0: 3D Overhead Crane Simulator

![Screenshot of the simulator in action](assets/movgrua.gif)

## Project Description

This repository hosts `Crane3DSim V2.0`, a MATLAB-based three-dimensional overhead crane simulator. Developed for dynamic analysis and control system design, `Crane3DSim` offers two main variants: one for continuous input signals and another for discrete control applications, facilitating research and development in the field of crane systems.

## Key Features

* **Full 3D Dynamic Simulation:** Models the complex dynamics of a three-dimensional overhead crane.
* **Two Operation Modes:**
    * **Continuous Simulator (`Crane3DSim_cont.m`):** Ideal for control inputs defined as continuous time functions, utilizing ODE solvers with event detection.
    * **Discrete Simulator (`Crane3DSim_disc.m`):** Designed to emulate real-world discrete control systems, allowing for the implementation of discrete-time controllers.
* **Customizable Controllers:** The discrete variant includes a designated "slot" for integrating custom Feedback (FB) or Feedforward (FF) controllers.
* **Comprehensive Visualization:** Generates detailed plots of state variables, forces, and a 3D animation of the crane's movement.
* **Identification Parameters:** Supports loading friction and PWM-to-force conversion parameters derived from experimental identification processes.

## Getting Started

### Requirements

* It has been tested in MATLAB versions 2024b and 2025a.
The `sgtitle` command in figures was introduced in 2018b, so in earlier versions it is necessary to comment out these lines when generating graphs in `Crane3DSim_cont.m` and `Crane3DSim_disc.m`. It has been tested in version 2017b.
### Running a Simulation

1.  **Clone the Repository**
2.  **Open in MATLAB:** Open MATLAB and navigate to the root folder of the repository.
3.  **Generate Parameters (Optional, you can use `Example_identification.mat`):**
    Run the `ParametersMATgenerator.m` script to create the identification parameters file. The launchers by default will load `Example_identification.mat`.
4.  **Launch Example Simulations:**
    * **Continuous Simulation:** Open and run `Launcher_cont.m`. You can modify simulation parameters and PWM input signals directly within this file.
    * **Discrete Simulation:** Open and run `Launcher_disc.m`. Similar to the continuous version, you can configure parameters and PWM input matrices here. To implement your own discrete controller, you will need to modify the designated section within `Crane3DSim_disc.m`.

## Key Inputs and Outputs

### Continuous Simulator (`Crane3DSim_cont.m`)

* **Inputs:**
    * `frictions`: Structure containing static and dynamic friction coefficients.
    * `IMOT`, `IMOTx`, `IMOTy`: Motor equivalent inertias.
    * `ks`: Vector `[X_PWM_TO_F Y_PWM_TO_F Z_PWM_TO_F]` for PWM to force conversion.
    * `uPWM_x`, `uPWM_y`, `uPWM_z`: Function handles `@(t)` for continuous PWM inputs.
    * `masas`: Vector `[mc mw ms]` with payload, trolley, and rail masses.
    * `x0`: Initial state vector.
    * `tsimul`: Total simulation time.
    * `g`: Gravity constant.
    * `ph_limits`: Physical limits of the crane.

* **Outputs:**
    * `T_out`: Time vector of simulation results.
    * `DATA_OUT`: Matrix containing all state variables and derived quantities.
    * `ESTADO_OUT`: State of static friction and movement per axis.

### Discrete Simulator (`Crane3DSim_disc.m`)

* **Inputs:**
    * `fricciones`: Identical friction structure as in the continuous version.
    * `IMOT`, `IMOTx`, `IMOTy`: Motor equivalent inertias.
    * `ks`: Vector `[X_PWM_TO_F Y_PWM_TO_F Z_PWM_TO_F]`.
    * `uPWM_x_matriz`, `uPWM_y_matriz`, `uPWM_r_matriz`: Matrices defining discrete PWM inputs over time.
    * `masas`: Mass vector.
    * `x0`: Initial state vector.
    * `tsimul`: Total simulation time.
    * `T_sample`: Discrete sampling period.
    * `g`: Gravity constant.
    * `ph_limits`: Physical limits.
    * `refs`: Matrix of reference values for control.

* **Outputs:**
    * `T_out`: Time vector of simulation results.
    * `DATA_OUT`: Matrix containing all state variables and derived quantities.
    * `ESTADO_OUT`: State of static friction and movement per axis.

### Important Note on Friction Units:

The simulator is designed to receive friction parameters in units derived from the identification process (effectively in PWM-scaled units). If you wish to introduce friction parameters directly in International System (SI) units (Newtons for forces, N·s/m for dynamic friction coefficients), you must **remove the respective multiplications** by `X_PWM_TO_F`, `Y_PWM_TO_F`, and `Z_PWM_TO_F` in the `%% FRICTION` section of the `Crane3DSim_cont.m` and `Crane3DSim_disc.m` files.

## Additional Documentation

For a more detailed guide on setup, usage, and key functionalities of the simulator, please refer to the documentation:

* [Crane3DSim User Manual (PDF)](Crane3DSim_Guide.pdf)

## How to cite

The paper explaining the model, the identification process, and the results has been submitted for review in a journal.
In the meantime, you can use the preprint hosted on Zenodo.

> J. Vicente-Martinez y E. Ramirez-Laboreo, «A hybrid dynamic model and parameter estimation method for accurately simulating overhead cranes with friction». Zenodo, sep. 01, 2025. doi: 10.5281/zenodo.17043988.

```bibtex
@misc{vicente_martinez_2025_17043988,
  author       = {Vicente-Martinez, Jorge and
                  Ramirez-Laboreo, Edgar},
  title        = {A hybrid dynamic model and parameter estimation
                   method for accurately simulating overhead cranes
                   with friction
                  },
  month        = sep,
  year         = 2025,
  publisher    = {Zenodo},
  doi          = {10.5281/zenodo.17043988},
  url          = {https://doi.org/10.5281/zenodo.17043988},
}
```



[![DOI](https://zenodo.org/badge/DOI/10.5281/zenodo.17043988.svg)](https://doi.org/10.5281/zenodo.17043988)




## Licence

GNU GPLv3 


---
