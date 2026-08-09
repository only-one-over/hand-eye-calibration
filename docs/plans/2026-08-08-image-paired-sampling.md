# Image-Paired Hand-Eye Sampling Implementation Plan

> **For Claude:** REQUIRED SUB-SKILL: Use superpowers:executing-plans to implement this plan task-by-task.

**Goal:** Upgrade the application so each calibration sample pairs one robot end-effector pose with one calibration-board image, then derives `target→camera` before running the existing hand-eye solvers.

**Architecture:** Keep the existing canonical pose/calibration pipeline intact and add an acquisition boundary in front of it. A sample may carry an image path and processing metadata; the board-pose estimator converts the image plus camera intrinsics into the existing canonical target pose. Legacy CSV/JSON files containing already-estimated target poses remain supported.

**Tech Stack:** Qt 6 Widgets, Qt Concurrent, OpenCV core/imgproc/imgcodecs/calib3d, C++17.

---

### Task 1: Add image-paired sample data model

**Files:**
- Modify: `src/domain/calibration_types.h`
- Modify: `src/models/calibration_session_model.cpp`

**Steps:**

1. Add calibration-board type, board dimensions, square size, camera intrinsics, image path, processing status, detected corner count, and PnP reprojection error fields.
2. Keep all existing canonical robot and target pose fields unchanged.
3. Add table columns that make unprocessed, failed, and successful image samples visible.
4. Ensure old synthetic and pose-only samples remain valid with an empty image path.

### Task 2: Implement board image pose estimation

**Files:**
- Create: `src/core/board_pose_estimator.h`
- Create: `src/core/board_pose_estimator.cpp`
- Modify: `CMakeLists.txt`

**Steps:**

1. Load the image with OpenCV and convert it to grayscale.
2. Detect chessboard corners using the configured inner-corner rows and columns.
3. Refine corners with `cornerSubPix`.
4. Build 3D board points from square size and solve `target→camera` with `solvePnP`.
5. Compute pixel reprojection RMSE and return a user-readable failure reason for missing images, invalid intrinsics, failed detection, or failed PnP.
6. Add OpenCV `imgproc` and `imgcodecs` components to CMake.

### Task 3: Add paired CSV import and processing

**Files:**
- Create: `src/io/image_sample_io.h`
- Create: `src/io/image_sample_io.cpp`
- Modify: `src/controllers/calibration_controller.h`
- Modify: `src/controllers/calibration_controller.cpp`

**Steps:**

1. Define a CSV schema containing `id,image_path` followed by the robot pose columns.
2. Parse and normalize robot poses using the existing `PoseInputSpec` and selected adapter.
3. Preserve image paths relative to the CSV directory when appropriate.
4. Add a controller operation that processes every paired image and replaces only successfully estimated target poses, while retaining failure metadata for review.
5. Clear stale calibration results whenever paired samples or board parameters change.

### Task 4: Expose the workflow in Qt Widgets

**Files:**
- Modify: `src/mainwindow.h`
- Modify: `src/mainwindow.cpp`
- Modify: `README.md`

**Steps:**

1. Add board rows, columns, square size, camera matrix, and distortion inputs.
2. Add “Import pose + image CSV” and “Process board images” actions.
3. Display sample image path, image status, corner count, and PnP reprojection RMSE.
4. Keep pose-only CSV import available for backward compatibility.
5. Explain the paired workflow and required camera intrinsics in the README.

### Task 5: Verify the full data flow

**Files:**
- Modify: `src/main.cpp`
- Modify: `docs/plans/2026-08-08-image-paired-sampling.md`

**Steps:**

1. Generate a synthetic chessboard image with known intrinsics and pose.
2. Verify corner detection and PnP recover a finite target pose with bounded reprojection RMSE.
3. Verify a paired sample can flow into the existing calibration service.
4. Run the existing multi-method smoke checks.
5. Build Debug and inspect Git status without staging unrelated user changes.
