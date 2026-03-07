# VAIT - Visuotactile-Assisted Intervention Technology

Visuotactile-Assisted Intervention Technology (VAIT) is a comprehensive robotic control and data analysis framework designed for advanced sensorimotor intervention tasks. It integrates visuotactile sensing (DAIMON sensors), force sensors (Bota), and robotic manipulation (Universal Robots) to perform precision tasks such as catheter navigation, acupuncture, and tactile exploration.

## Project Structure

* **`src/`**: Contains the core logic for the project.
  * `core/`: Core system components and data structures.
  * `sensors/`: Interfaces for processing visuotactile sensor data, encoder integration, and EtherCAT-based force sensing.
  * `actuators/`: Interfaces for robotic control, such as Universal Robots via UR-RTDE.
* **`scripts/`**: Executable scripts for running and evaluating the system.
  * `control/`: Scripts for real-time control loops and robotic interventions.
  * `analysis/`: Scripts for analyzing recorded sensor/force data (e.g., force regression).
  * `tests/`: Testing and validation scripts.
* **`data/`**: Datasets collected during interventions, force tests (press force, pull force, angle data, regression results), and their respective visualizations.
* **`demo/`**: Demonstration recordings, images, and specific use-cases (e.g., `acupuncture`, `catheter`).
* **`config/`**: Configuration files (e.g., `system_config.yaml`) defining operational parameters.

## Installation

This project has been updated to use `pyproject.toml` for modern standard compliance. 

### Creating the Environment

It is recommended to use `conda` or another virtual environment manager (such as `venv`). The original working environment is named `ssr`.

```bash
conda create -n ssr python=3.10
conda activate ssr
```

### Installing Dependencies

Install the core package and its dependencies using `pip`:

```bash
pip install -e .
```

Hardware-specific dependencies (e.g., `ur-rtde`, `pysoem`, `wmi` for Windows) are automatically included. 

**Note for DAIMON usage:** To utilize GPU acceleration for visuotactile processing, you must ensure that CUDA 12.x matches with `cupy-cuda12x`.

## Usage

1. **Configuration**: Edit `config/system_config.yaml` to specify the hardware configuration, robotic IP addresses, sensor parameters, and data saving directories.
2. **Analysis**: Use the scripts inside `scripts/analysis/` to process regression data located in the `data/` directories.
3. **Execution**: Run the main intervention tasks located in `scripts/control/`.