# Hand-Eye Calibration MVP Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Build a Qt 6 Widgets desktop tool that imports hand-eye pose pairs, runs five OpenCV hand-eye algorithms, evaluates residuals, and exports results.

**Architecture:** Keep `MainWindow` focused on presentation. Store samples/results in a Qt-facing session model, isolate transforms and validation in utility code, isolate OpenCV calls in `CalibrationService`, and isolate CSV/JSON persistence in `DatasetIo`. The first version is offline and deterministic, with room for camera/robot adapters later.

**Tech Stack:** C++17, Qt 6 Widgets/Core/Concurrent, OpenCV calib3d/core, CMake, Qt Linguist.

---

### Task 1: Establish the project structure and dependency contract

**Files:**
- Modify: `CMakeLists.txt`
- Create: `src/domain/calibration_types.h`
- Create: `src/core/matrix_utils.h`, `src/core/matrix_utils.cpp`

**Steps:**

1. Move the application source list to `src/` while preserving the existing Qt 6 entry point.
2. Add `find_package(OpenCV REQUIRED COMPONENTS core calib3d)` and link the imported OpenCV target or `${OpenCV_LIBS}` based on the detected package.
3. Define value types for `PoseSample`, `CalibrationResult`, `CalibrationMethod`, `CalibrationDataset`, and `CalibrationReport`.
4. Implement conversions between Rodrigues vectors, rotation matrices, and 4×4 homogeneous matrices.
5. Configure Qt Concurrent and enable C++17 warnings where supported.

### Task 2: Add deterministic data validation and synthetic samples

**Files:**
- Create: `src/core/dataset_validator.h`, `src/core/dataset_validator.cpp`
- Create: `src/core/synthetic_dataset.h`, `src/core/synthetic_dataset.cpp`
- Create: `tests/test_matrix_utils.cpp`, `tests/test_dataset_validator.cpp`

**Steps:**

1. Write tests for identity/round-trip pose conversions and invalid numeric input.
2. Validate minimum sample count, finite values, unit scale, and non-degenerate rotation diversity.
3. Generate a fixed synthetic dataset with a known camera-to-gripper transform and deterministic perturbation.
4. Run the unit tests before adding UI code.

### Task 3: Implement algorithm and residual services

**Files:**
- Create: `src/core/calibration_service.h`, `src/core/calibration_service.cpp`
- Create: `tests/test_calibration_service.cpp`

**Steps:**

1. Map the five UI methods to `cv::HandEyeCalibrationMethod`.
2. Convert validated samples to OpenCV input arrays with the documented transform directions.
3. Call `cv::calibrateHandEye` and build a `CalibrationResult` with matrix, Rodrigues vector, translation, elapsed time, and status.
4. Compute pairwise motion residuals and a report-level RMS rotation/translation error.
5. Test all five methods against the synthetic dataset and test an insufficient-data failure.

### Task 4: Implement dataset persistence and the session model

**Files:**
- Create: `src/io/dataset_io.h`, `src/io/dataset_io.cpp`
- Create: `src/models/calibration_session_model.h`, `src/models/calibration_session_model.cpp`
- Create: `tests/test_dataset_io.cpp`

**Steps:**

1. Define a documented CSV header and parser with line-level errors.
2. Implement JSON read/write for samples, selected method, units, and results.
3. Implement a `QAbstractTableModel` for samples and a model for results.
4. Add round-trip tests for CSV and JSON.

### Task 5: Build the Qt 6 main window workflow

**Files:**
- Modify: `src/mainwindow.h`, `src/mainwindow.cpp`, `src/mainwindow.ui`
- Modify: `src/main.cpp`
- Modify: `hand_eye_calibration_zh_CN.ts`

**Steps:**

1. Add menus and actions for new, import, export JSON/CSV, generate demo, calculate, calculate all, and delete selected.
2. Add a left control panel for mode/units/method and a central splitter containing sample/result tables and a log view.
3. Connect buttons to the session model and service; disable calculation until validation passes.
4. Run calculations through `QtConcurrent::run`, report progress and errors through queued signals.
5. Add matrix preview and clipboard export for the selected result.

### Task 6: Build and verify the application

**Files:**
- Modify: `README.md`
- Modify: `.gitignore` if needed

**Steps:**

1. Configure a clean Qt 6 build directory with the installed compiler and OpenCV.
2. Build the application and test targets.
3. Run unit tests and exercise the demo-data workflow manually.
4. Verify import/export round trips and no unhandled exceptions.
5. Document dependency installation, CSV schema, algorithm conventions, and known limitations.
