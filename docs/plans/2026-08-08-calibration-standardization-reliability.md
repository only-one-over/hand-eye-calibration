# Calibration Standardization and Reliability Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Make the Qt6 hand-eye calibration tool reject ambiguous input, calculate physically meaningful RMSE, validate independent data, and export traceable calibration results.

**Architecture:** Keep all internal poses canonical as Rodrigues radians and meters with explicit frame-direction labels. Conversion happens at IO/adapter boundaries. `CalibrationService` returns a result plus per-sample residuals and a reliability report; UI only renders those results. Exporters consume the canonical dataset and include metadata.

**Tech Stack:** C++17, Qt6 Widgets/Core, OpenCV 4.x calib3d/core, CMake.

---

### Task 1: Extend canonical data types and metadata

**Files:** `src/domain/calibration_types.h`

- Add rotation representation, angle unit, translation unit, frame-direction, robot/camera metadata, validation samples, per-sample residuals, reliability report, pass threshold, and recommendation fields.
- Keep `PoseSample` canonical: Rodrigues in radians, translation in meters.
- Make Eye-To-Hand unavailable in the first reliable release; retain the enum for future implementation.

### Task 2: Implement pose conversion and input normalization

**Files:** `src/core/pose_conversion.h`, `src/core/pose_conversion.cpp`, `src/io/pose_adapter.h`, `src/io/pose_adapter.cpp`

- Convert Rodrigues, Euler XYZ, RPY ZYX, and quaternion input into canonical Rodrigues radians.
- Convert degrees/radians and mm/m explicitly; never infer units from numeric magnitude.
- Parse common robot adapter text formats: Generic 6D, UR `[x,y,z,rx,ry,rz]`, KUKA/FANUC `XYZABC/XYZWPR`.
- Return line-level errors and frame-direction metadata.

### Task 3: Replace residual math with reliability reports

**Files:** `src/core/dataset_validator.*`, `src/core/calibration_service.*`

- Detect insufficient samples, non-finite values, duplicate poses, low rotation diversity, parallel relative axes, and near-zero relative motion.
- Compute each sample’s predicted fixed target pose and compare it against a reference/robust mean pose.
- Compute true RMSE as `sqrt(mean(error^2))` separately for rotation degrees and translation meters.
- Compute average and maximum residuals, pass/fail status, and outlier candidates.
- Evaluate independent validation samples without refitting the calibration transform.

### Task 4: Add all-algorithm recommendation and synthetic truth tests

**Files:** `src/core/synthetic_dataset.*`, `src/core/calibration_service.*`, `src/main.cpp`

- Generate deterministic ground-truth data and check matrix direction plus numeric error.
- Run all five methods automatically.
- Recommend the lowest validation score among passing methods; otherwise recommend the lowest reliable training score and mark the dataset failed.
- Extend `--smoke-test` to verify matrix round-trip, true RMSE, outlier detection, and independent validation.

### Task 5: Add traceable multi-format exports

**Files:** `src/io/dataset_io.*`, `src/mainwindow.*`, `README.md`

- Persist robot, camera, units, input formats, directions, algorithms, timestamps, thresholds, residuals, and recommendation in JSON/YAML.
- Add TXT, C++, and Python matrix exporters for the selected/recommended result.
- Preserve the current CSV schema with a metadata header and document the canonical contract.

### Task 6: Update Qt UI

**Files:** `src/mainwindow.*`, `src/models/calibration_session_model.*`

- Add explicit input format/unit controls, robot/camera fields, validation-data import, reliability summary, residual/outlier table, and export actions.
- Disable Eye-To-Hand in the mode selector with an explanatory label.
- Show algorithm recommendation, average/max/RMSE values, pass/fail status, and matrix direction.

### Task 7: Verify and document

- Configure/build with the known Qt6/OpenCV installation.
- Run smoke tests and inspect generated export files.
- Run the GUI startup path and confirm disabled Eye-To-Hand, input controls, and result summary.
