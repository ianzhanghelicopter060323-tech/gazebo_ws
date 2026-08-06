# smart_factory_perception

This package owns RGB-D target detection, classification, depth localization,
TF transformation, and temporal filtering. It contains both the offline OCR
tools and the runtime `/cube_locator/locate` service.

## OCR layout

```text
config/ocr.yaml                         runtime defaults
models/ocr/                             local PP-OCR ONNX weights
scripts/ocr_image.py                    image-to-JSON smoke-test CLI
src/smart_factory_perception/ocr/       reusable OCR and object classification
test/                                   rule-level tests without model loading
```

The migrated model is RapidOCR 3.9.2's pretrained PP-OCRv6 small pipeline:

- `PP-OCRv6_det_small.onnx`: text detection;
- `ch_ppocr_mobile_v2.0_cls_mobile.onnx`: 0/180 degree orientation;
- `PP-OCRv6_rec_small.onnx`: multilingual text recognition.

The model runs locally through ONNX Runtime. It must not access the network
during a competition run.

## Prepared local environment

The workspace-local environment is `.venv/ocr`. It inherits the ROS Noetic
system packages and contains RapidOCR 3.9.2, ONNX Runtime 1.19.2, OpenCV
4.8.1 and OmegaConf 2.3.1. OpenCV is intentionally kept below 4.9 because
OpenCV 5 is incompatible with ROS Noetic's `cv_bridge` type mapping. Activate
it after sourcing the catkin workspace:

```bash
source devel/setup.bash
source .venv/ocr/bin/activate
```

To recreate the Python dependencies on another machine, create a Python 3.8
virtual environment with `--system-site-packages`, then install:

```bash
python -m pip install -r src/smart_factory_perception/requirements-ocr.txt
```

On Ubuntu, `python3.8-venv` is required for a normal `python3 -m venv` setup.
The ONNX files under `models/ocr` are deliberately addressed by explicit local
paths, so inference does not depend on a package cache or first-run download.

## Offline smoke test

After building/sourcing the workspace:

```bash
rosrun smart_factory_perception ocr_image \
  src/car3/models/cube/meshes/Food.png --json
```

Before the first catkin build, the equivalent source-tree command is:

```bash
PYTHONPATH=src/smart_factory_perception/src \
  .venv/ocr/bin/python \
  src/smart_factory_perception/scripts/ocr_image.py \
  src/car3/models/cube/meshes/Food.png --json
```

Use the extensionless `ocr_image` command with `rosrun`. It is a small launcher
that selects `.venv/ocr/bin/python` only for OCR. Catkin normally pins Python
scripts to `/usr/bin/python3`, which would bypass the isolated OCR environment.
Set `OCR_PYTHON=/another/python` to override the launcher when deploying to a
different computer.

The CLI tries the native image first. Only when the closed-set keyword result
is rejected does it retry at 2x and then 4x scale. JSON output reports the
selected `scale`, the selected result in the top-level fields, and every scale
under `attempts`. Detection boxes are converted back to original-image pixel
coordinates. Disable fallback for a one-scale diagnostic run with:

```bash
rosrun smart_factory_perception ocr_image IMAGE.png \
  --scale 1 --retry-scales --json
```

The classifier accepts only 食品/日用/电子 keywords from the full image;
unrelated OCR text does not enter the class score.

## Runtime RGB-D locator

`cube_locator` synchronizes RGB and depth images, finds the cube label locally,
uses the median valid depth inside the OCR box, deprojects through
`CameraInfo`, and transforms the point into `base_footprint` and `map`. A
request succeeds only when at least 3 of 5 observations agree spatially and,
when requested, on class.

```bash
roslaunch smart_factory_perception cube_locator.launch
rosservice call /cube_locator/locate \
  "{station: 35, require_classification: true}"
```

The configurable camera-axis correction in `config/cube_locator.yaml` accounts
for the Gazebo sensor render convention; it was checked against simulation
truth during calibration only. Runtime mission decisions do not read Gazebo
model state.

## Capture one Gazebo camera frame

After sourcing the workspace, save one frame from `/camera/rgb/image_raw` with:

```bash
rosrun smart_factory_perception imgsave \
  /home/ianichinose/gazebo_ws/data/close_navi/close_%04i.png
```

Each invocation keeps one subscription alive long enough to discard four
camera warmup frames, saves the fifth frame, and selects the next unused index.
Repeating the command creates `close_0000.png`, `close_0001.png`, and so on
without requiring a persistent `image_saver` service or saving a stale first
frame after the arm moves.
