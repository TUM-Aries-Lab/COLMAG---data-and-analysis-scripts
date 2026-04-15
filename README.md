# COLMAG Data and Analysis Scripts

MATLAB analysis code and experiment assets for COLMAG studies on magnetic sensing in collaborative robotics.

This repository combines:
- experiment datasets across multiple acquisition sessions,
- MATLAB scripts used for analysis, visualization, and figure generation,
- helper functions for plotting and statistical processing,
- ROS bag recordings and derived `.mat` outputs for selected studies.

The codebase covers several experiment families, including avoidance, safety, guidance, magnetic tracking, calibration, and learning-based magnetic-field estimation.

## Important note about data download

Large data assets in this repository are tracked with **Git LFS**.

That means `.csv`, `.bag`, and `.mat` files may appear as small text pointer files if you downloaded the repository as a ZIP archive or exported it without fetching LFS objects. In that case, you do **not** yet have the actual experimental data.

**To obtain the real files, use Git LFS in a cloned checkout**:

```bash
git lfs install
git clone <repo-url>
cd <repo-folder>
git lfs pull
```

Tracked extensions in this repository:
- `.csv`
- `.bag`
- `.mat`

## Repository layout

```text
.
├── README.md
├── LICENSE
├── testScript.m
├── scripts/
│   ├── Avoidance_data_analysis_01.m
│   ├── Avoidance_data_analysis_02.m
│   ├── Avoidance_data_analysis_03.m
│   ├── Avoidance_data_analysis_04.m
│   ├── Avoidance_data_analysis_05.m
│   ├── Avoidance_data_analysis_06.m
│   ├── Demo_Analysis.m
│   ├── Dynamic_Calibration_Bdata_analysis.m
│   ├── Dynamic_Calibration_Bdata_analysis2.m
│   ├── Guidance_data_analysis_01.m
│   ├── Guidance_data_analysis_02.m
│   ├── Guidance_data_analysis_03.m
│   ├── Magnet_Tracking_analysis.m
│   ├── Safety_data_analysis_01.m
│   ├── Safety_data_analysis_02.m
│   ├── Safety_data_analysis_03.m
│   ├── Safety_data_analysis_04.m
│   └── StaticCalibrationPanel.m
├── dependencies/
│   ├── dabarplot.m
│   ├── daboxplot.m
│   ├── daviolinplot.m
│   ├── Levenetest.m
│   ├── SpectralArcLength.m
│   ├── swtest.m
│   └── plot2svg-master/
└── data/
    ├── session_01/
    │   ├── avoidance_exp/
    │   ├── demo_experiment/
    │   ├── experiments_mod_1/
    │   ├── experiments_mod_2/
    │   └── safety_experiments/
    ├── session_02/
    ├── session_03/
    │   └── DataStop/
    │       ├── 08ms/
    │       ├── 09ms/
    │       └── 10ms/
    ├── session_04/
    │   ├── 1_0ms/
    │   └── Reference/
    ├── session_05/
    └── session_06/
```

## Data summary

The repository currently references **122 Git LFS-tracked data files** for an approximate total payload of **7.62 GiB**. It should weight around **16 Gb**.

| Session | Main contents | Files | Approx. size* |
|---|---|---:|---:|
| `session_01` | Avoidance, demo, guidance, and safety CSV logs | 33 | 3.02 GiB |
| `session_02` | Avoidance, calibration, and magnetic-tracking CSV logs | 15 | 3.54 GiB |
| `session_03` | Stop-response CSV batches (`08ms`, `09ms`, `10ms`) | 56 | 0.78 GiB |
| `session_04` | ROS bag files for stop/safety experiments | 13 | 0.19 GiB |
| `session_05` | ROS bag + neural-network result file | 2 | 0.06 GiB |
| `session_06` | ROS bag + derived tremor/statistics MAT files | 3 | 0.02 GiB |

\* Sizes are estimated from Git LFS metadata.

## Script overview

### Avoidance analysis

| Script | Purpose |
|---|---|
| `Avoidance_data_analysis_01.m` | Segment and visualize avoidance trajectories for selected conditions. |
| `Avoidance_data_analysis_02.m` | Compare avoidance performance with compact summary plots. |
| `Avoidance_data_analysis_03.m` | Run statistical analysis for radius-based avoidance experiments. |
| `Avoidance_data_analysis_04.m` | Extend avoidance analysis with timing and velocity features. |
| `Avoidance_data_analysis_05.m` | Study avoidance behavior versus obstacle distance/radius. |
| `Avoidance_data_analysis_06.m` | Reconstruct and visualize a 2D velocity field from avoidance trials. |

### Safety analysis

| Script | Purpose |
|---|---|
| `Safety_data_analysis_01.m` | Segment and visualize safety experiment trajectories. |
| `Safety_data_analysis_02.m` | Compare stop distance and stop time across conditions. |
| `Safety_data_analysis_03.m` | Batch-process CSV stop experiments and summarize minimum-distance behavior. |
| `Safety_data_analysis_04.m` | Analyze stop behavior directly from ROS bag recordings. |

### Guidance and demo analysis

| Script | Purpose |
|---|---|
| `Guidance_data_analysis_01.m` | Analyze guidance study results, including questionnaire and trajectory metrics. |
| `Guidance_data_analysis_02.m` | Compare completion-time behavior between guidance modalities. |
| `Guidance_data_analysis_03.m` | Extract tremor/vibration metrics from still segments in ROS bag data. |
| `Demo_Analysis.m` | Inspect one demo recording and visualize end-effector motion and state evolution. |

### Tracking, calibration, and learning

| Script | Purpose |
|---|---|
| `Magnet_Tracking_analysis.m` | Aggregate 3D magnetic tracking performance and computation-time trends. |
| `StaticCalibrationPanel.m` | Perform static magnetometer calibration and visualize ellipsoid correction. |
| `Dynamic_Calibration_Bdata_analysis.m` | Compare dynamically calibrated and non-calibrated magnetic-field measurements. |
| `Dynamic_Calibration_Bdata_analysis2.m` | Build an LSTM-based pipeline for magnetic-field estimation from robot-link motion. |

### Utility

| Script | Purpose |
|---|---|
| `testScript.m` | Smoke-test the repository by running the scripts in test mode. |

### MATLAB

The script headers indicate the project was tested on **MATLAB R2025b**. In practice, many scripts should also run on earlier releases, but scripts using modern name-value syntax are best suited to **R2021a or newer**.

### Toolboxes Requirements

Toolbox requirements vary by script. Across the repository, the following toolboxes are used:
- Signal Processing Toolbox
- Statistics and Machine Learning Toolbox
- Curve Fitting Toolbox
- ROS Toolbox
- Sensor Fusion and Tracking Toolbox
- Navigation Toolbox
- System Identification Toolbox
- Deep Learning Toolbox

Not every script needs every toolbox. Check the header at the top of the specific script you want to run.

### Included helper functions

The `dependencies/` folder contains local plotting and statistics helpers used by the scripts, including:
- `dabarplot`
- `daboxplot`
- `daviolinplot`
- `swtest`
- `Levenetest`
- `SpectralArcLength`
- `plot2svg`

## Quick start

1. Fetch the repository with **Git LFS** so that the real data files are available.
2. Open MATLAB in the **repository root**.
3. Run the smoke test:

```matlab
testScript
```

4. Run a specific analysis script from the `scripts/` folder, for example:

```matlab
run("scripts/Magnet_Tracking_analysis.m")
run("scripts/Avoidance_data_analysis_01.m")
```

Make sure to be in the script folder path when running the scripts individually.
Most scripts expose user-editable configuration variables near the top of the file, such as selected trial index, file name, condition, or visualization options.

## Notes on usage

- Several scripts assume the repository structure is preserved exactly as provided.
- Some analyses rely on **interactive selections** (`ginput`) or manual point picking.
- `Dynamic_Calibration_Bdata_analysis2.m` can be computationally expensive when training is enabled.
- Some scripts work on CSV telemetry, while others require ROS bag files and precomputed `.mat` outputs.
- File and folder naming is part of the workflow, so renaming data assets may break hard-coded references.

## Reproducibility notes

- The repository is organized primarily by **session**, not by publication figure.
- Data-processing choices such as trimming, smoothing, segment selection, and condition selection are implemented inside the scripts.
- The script headers provide a useful starting point for understanding each file's goal, dependencies, and expected outputs.

## License

This project is released under the **MIT License**. See [`LICENSE`](LICENSE) for details.

## Citation

If this repository contributes to your work, please cite the associated publication:

> F. Masiero et al., *Human-Aware Motion Framework Exploiting Magnetic Sensing in Collaborative Robotics*.