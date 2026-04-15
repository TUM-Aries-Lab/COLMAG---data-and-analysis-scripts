# COLMAG_CodeDataAnalysis

A structured repository of CSV-based experimental recordings collected across multiple COLMAG sessions. The dataset is organized by session and experiment type, and captures time-series telemetry related to magnetic measurements, end-effector kinematics, forces, and multi-sensor readings.

> **Current scope:** this repository snapshot contains data files and scripts for analysis and visualization.

## Overview

This repository appears to support offline analysis of experimental runs involving:

- magnetic position and orientation channels
- end-effector position, velocity, and force channels
- multi-sensor measurements (`s1` to `s4` for main detector, `s1` to `s8` for peripheral detector)
- extended telemetry in selected files, including normalized sensor values, timing/fit metrics, matrix-style outputs, and state labels

The data is grouped into two sessions and several experiment categories such as avoidance, safety, demo, and modular experiments.

## Dataset at a glance

- **Total files:** 36 CSV files
- **Approximate size:** 3.36 GB
- **Approximate total samples:** 6.18 million rows
- **Approximate recorded duration:** 1.72 hours of telemetry
- **Observed sampling cadence:** approximately 1 kHz in representative files, inferred from timestamp spacing

## Repository structure

```text
COLMAG_CodeDataAnalysis/
├── README.md
└── data/
    ├── session_01/
    │   ├── avoidance_exp/        # 6 CSV files
    │   ├── demo_experiment/      # 3 CSV files
    │   ├── experiments_mod_1/    # 9 CSV files
    │   ├── experiments_mod_2/    # 9 CSV files
    │   └── safety_experiments/   # 6 CSV files
    └── session_02/
        └── *.csv                 # 3 CSV files
```

## Folder summary

| Path | Files | Approx. rows | Approx. duration | Schema |
|---|---:|---:|---:|---|
| `data/session_01/avoidance_exp` | 6 | 1,856,839 | 30.9 min | base |
| `data/session_01/demo_experiment` | 3 | 515,377 | 8.6 min | extended + state |
| `data/session_01/experiments_mod_1` | 9 | 1,125,969 | 18.8 min | base |
| `data/session_01/experiments_mod_2` | 9 | 1,175,709 | 19.6 min | base |
| `data/session_01/safety_experiments` | 6 | 1,040,397 | 17.3 min | base |
| `data/session_02` | 3 | 462,754 | 7.7 min | extended / advanced |

## Data schema

Three schema levels are present in the CSV files.

### 1. Base schema

Present in most files, with 28 columns:

- `time`
- `mag_pos_x`, `mag_pos_y`, `mag_pos_z`
- `mag_orien_x`, `mag_orien_y`, `mag_orien_z`
- `ee_x`, `ee_y`, `ee_z`
- `v_ee_x`, `v_ee_y`, `v_ee_z`
- `F_ee_x`, `F_ee_y`, `F_ee_z`
- `s1_x`, `s1_y`, `s1_z`
- `s2_x`, `s2_y`, `s2_z`
- `s3_x`, `s3_y`, `s3_z`
- `s4_x`, `s4_y`, `s4_z`

### 2. Extended schema

Adds derived or auxiliary fields to the base schema:

- normalized sensor channels: `n_s1_*`, `n_s2_*`, `n_s3_*`, `n_s4_*`
- timing / quality metrics: `ls_compute_time`, `r2`

These files contain 42 columns in total.

### 3. Advanced schema

Present in selected files and extends the previous schema with a flattened 16-element matrix-style output:

- `O_T_EE_1` through `O_T_EE_16`

Some demo files also include:

- `state`

These files contain 58 or 59 columns, depending on whether `state` is present.

## File naming convention

The filenames are descriptive and usually encode the experiment category, setup type, run number, and a timestamp suffix.

Examples:

- `Avoidance_REAL1_exp_09_07_15_57_32.csv`
- `SAFETY_VIRTUAL_3_exp_09_07_18_27_01.csv`
- `data_em1_S01.csv`
- `Demo_exp_02_18_20_10_41.csv`

In practical terms:

- `REAL` / `VIRTUAL` likely distinguish experiment modes or environments
- `em1` / `em2` refer to modular experiment groups
- `S01` to `S09` appear to identify subject IDs, sequence IDs, or run IDs
- trailing numeric groups act as timestamp-like identifiers

## Quick start

### Python example

```python
from pathlib import Path
import pandas as pd

file_path = Path("data/session_01/avoidance_exp/Avoidance_REAL1_exp_09_07_15_57_32.csv")
df = pd.read_csv(file_path)

# Create a relative time axis for analysis
df["t"] = df["time"] - df["time"].iloc[0]

# Example inspection
print(df.head())
print(df.columns.tolist())
print(df[["t", "ee_x", "ee_y", "ee_z"]].describe())
```

### MATLAB example

```matlab
T = readtable('data/session_01/avoidance_exp/Avoidance_REAL1_exp_09_07_15_57_32.csv');
T.t = T.time - T.time(1);

head(T)
summary(T(:, {'t','ee_x','ee_y','ee_z'}))
```

## Suggested analysis workflow

A typical workflow for working with this dataset is:

1. Select a session and experiment family.
2. Load one or more CSV files.
3. Convert `time` to a relative timeline.
4. Inspect the channels relevant to your study, such as end-effector motion, force, or sensor trajectories.
5. Compare runs across `REAL` and `VIRTUAL` conditions or across experimental modules.
6. Export cleaned subsets or derived features for downstream modeling or statistical analysis.

## Notes and caveats

- Several files start with an initial zero-valued segment before dynamic measurements appear. This is likely a warm-up, synchronization, or idle phase and may need trimming during preprocessing.
- Column availability is not fully uniform across all files. Always inspect the header before batch processing.
- The semantic meaning of advanced fields such as `O_T_EE_*`, `state`, and `ls_compute_time` should be verified against the original acquisition or analysis pipeline.
- Filename capitalization is not completely uniform across all runs, so case-sensitive workflows should be written carefully.

## Recommended next additions

To make the repository easier to reuse and reproduce, the following additions would strengthen it significantly:

- analysis scripts or notebooks
- a data dictionary for each column
- experiment protocol notes
- subject/run metadata
- a license file
- citation information for publications using this dataset

## License

MIT License

## Citation

F. Masiero et al., "Human-Aware Motion Framework Exploiting Magnetic Sensing in Collaborative Robotics".
