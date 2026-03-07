# VAIT - Visuotactile-guided Adaptive In-hand Twisting

**Visuotactile-guided Adaptive In-hand Twisting (VAIT)** is a robust robotic control framework designed for the high-precision manipulation of ultra-thin slender objects, such as surgical needles, catheters, and guidewires. By integrating high-resolution visuotactile sensing with multi-degree-of-freedom (DoF) actuation, VAIT enables stable grasping and continuous axial rotation (twisting) of objects ranging from 5 mm rods down to 0.2 mm filaments.

## Key Features

* **Dual-Regime Control Model**: Features a unified piecewise control law that adaptively switches between **Open Configuration (OC)** for kinematic rolling and **Closed Configuration (CC)** for deformation-dominated sliding.
* **Visuotactile Force Tracking**: Leverages shear stress maps from optical flow to provide real-time external force estimation ($F_x, F_y$) calibrated against a Bota MiniONE Pro sensor.
* **Safety Release Mechanism**: Implements an automatic "overload" protection that triggers a gripper release when forces exceed a defined threshold (e.g., 1 N), essential for medical interventions like acupuncture and vascular surgery.
* **High-Precision Accuracy**: Achieves sub-4° rolling-angle accuracy by compensating for silicone viscoelastic creep, geometric misalignment, and initial motion dead zones.

## Project Structure

* **`src/`**: Core system logic.
    * `core/`: Unified Rotational Pose Model and regime-switching logic.
    * `sensors/`: Interfaces for **DW-Tac W (Daimon)** visuotactile sensors, including depth-based orientation estimation and shear integral ($S_x, S_y$) computation.
    * `actuators/`: Multi-DoF control for linear motors (transversal translation) and the gripper motor (clamping).
* **`scripts/`**:
    * `analysis/`: Calibration tools for identifying empirical coefficients ($K_{OC}, C_{OC}, K_{CC}, A_{CC}, C_{CC}$).
    * `control/`: Real-time teleoperation and autonomous intervention scripts.
* **`data/`**: Datasets for force calibration and regression results across varying object diameters (0.2 mm to 5.0 mm).

## Hardware Specifications

The framework is optimized for the following hardware stack:
* **Sensing**: 2x Daimon DW-Tac W Visuotactile Sensors (VS1, VS2).
* **Actuation**: 
    * 2x Independent Linear Motors (28HS32) for sensor translation (LM1, LM2).
    * 1x Damiao Gripper Motor (DM-J4310) for clamping (GM).
    * 3x Precision Linear Guides (TGN5C & MGN20).



## Mathematical Model

The system uses a unified pose model $\Theta_{deg}(d, n, \theta)$ to calculate the required motor steps $n$ for a target angle based on the object diameter $d$ and orientation $\theta$:

$$
\Theta_{deg}(d, n, \theta) = 
\begin{cases} 
0.53\left(\frac{720sn}{\pi d}\cos\theta\right) - 0.43 & \text{(Open Configuration)} \\
0.53\left(\frac{720sn}{\pi d}\right) - \frac{11.73}{d} - 0.55 & \text{(Closed Configuration)} 
\end{cases}
$$

## Installation & Usage

### Requirements
* **Python**: 3.10+
* **Acceleration**: CUDA 12.x and `cupy-cuda12x` (required for real-time marker tracking/optical flow).
* **Environment**: `conda create -n ssr python=3.10`

### Quick Start
1.  **Configuration**: Edit `config/system_config.yaml` to specify hardware IPs and coefficients.
2.  **Calibration**: Use scripts in `scripts/analysis/` to process force regression data.
3.  **Execution**: Run `scripts/control/` for real-time intervention tasks.