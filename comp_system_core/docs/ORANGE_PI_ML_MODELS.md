# AI / ML models on the Orange Pi 5 Pro / Max: groundwork

Status: **research notes, nothing built or measured by us yet.** Written 2026-11-14.
Every number below comes from a vendor or community page (linked at the end), not from our
hardware. Section 9 is the task list for whoever picks this up next.

This file lives in the robot repo because that is where it was written. It is really about
the vision coprocessor code (`C:\CJH\Python\FRC\vision\2026`) and can move there.

## 1. Why this board

The Orange Pi 5 Pro (RK3588S) and 5 Max (RK3588) carry a **neural processing unit (NPU)**
that the Raspberry Pi 5 does not have. For our purposes the two boards are the same
computer:

| | Orange Pi 5 Pro | Orange Pi 5 Max | Raspberry Pi 5 |
|---|---|---|---|
| Chip | RK3588S | RK3588 | BCM2712 |
| CPU | 4x A76 + 4x A55 | 4x A76 + 4x A55 | 4x A76 |
| NPU | 6 TOPS, 3 cores | 6 TOPS, 3 cores | none |
| GPU | Mali-G610 | Mali-G610 | VideoCore VII |
| RAM | 4 / 8 / 16 GB | 4 / 8 / 16 GB | up to 16 GB |
| Differences that matter to us | fewer camera/PCIe lanes | 2.5 GbE, WiFi 6E, PCIe 3.0 | |

The NPU is identical on both, so a model converted for one runs on the other (the target
name is `rk3588` for both). Pick between them on I/O, not on ML speed.

What the NPU buys: a small object detector at **roughly 55 to 110 frames per second per NPU
core** (table in section 3), against a few frames per second for the same model on a Pi 5's
CPU. There are three cores, so three cameras can each have a core, or one camera can use
all three.

What it does not do: it runs **8-bit quantized convolutional networks** well. It is not a
general GPU. Large transformer models, anything needing floating point throughout, and
training are out. Training happens on a PC or in the cloud.

## 2. The toolchain, in one picture

```
 PC / Colab (x86 Linux only)                      Orange Pi (on the robot)
 ---------------------------                      ------------------------
 images + labels
      |  train (Ultralytics, PyTorch)
      v
   best.pt
      |  export to ONNX
      v
   best.onnx
      |  rknn-toolkit2: quantize to INT8
      |  using ~100+ of OUR images
      v
   best.rknn  ------------ copy -------------->   rknn-toolkit-lite2 (Python)
                                                  + librknnrt.so (runtime)
                                                       |
                                                       v
                                                  boxes -> NetworkTables
```

Facts that trip people up:

- **Conversion cannot be done on the Orange Pi or on Windows.** `rknn-toolkit2` runs on
  x86 Linux only. Use WSL2, a Linux PC or Google Colab.
- **Versions must match.** The toolkit that made the `.rknn` and the `librknnrt.so` on the
  board must be the same release, or the model fails to load or runs slowly. The model zoo
  is at release 2.5.0 and wants RKNPU2 SDK 2.5.0 or newer.
- **Quantization needs our images.** INT8 conversion measures typical activations on a
  calibration set. Calibrating on generic COCO images for a model that looks at an FRC
  field costs accuracy. Use a few hundred of our own frames.
- **INT8 can lose accuracy.** Always compare the `.rknn` against the original `.pt` on the
  same test images (section 7).
- **Post-processing runs on the CPU.** The NPU produces raw tensors; decoding boxes and
  non-max suppression are ours to write (the model zoo has reference code) or come with the
  Ultralytics runtime.

Two ways to do the conversion:

| Route | How | When |
|---|---|---|
| Ultralytics export | `model.export(format="rknn", name="rk3588", quantize=8, data="ours.yaml")` | simplest; ties us to the Ultralytics runtime on the board |
| `rknn_model_zoo` scripts | export ONNX with Rockchip's modified head, then `convert.py` | fastest on the NPU, we own the post-processing; what PhotonVision uses |

## 3. Which models, and how fast

Rockchip's own single-core numbers for the RK3588, 640x640 input, INT8, **inference only**
(no camera capture, resize or box decoding):

| Model | FPS, one NPU core | Notes |
|---|---|---|
| YOLOv6n | 109 | fastest; less common tooling |
| YOLOv5n | 85 | old but the fastest mainstream option |
| YOLOv8n | 76 | best speed/accuracy balance; widest support |
| YOLOv8n-obb | 77 | rotated boxes |
| YOLO11n | 64 | a little more accurate, a little slower |
| YOLOv10n | 64 | |
| YOLOv8n-seg | 63 | pixel masks |
| YOLOv8n-pose | 58 | keypoints |
| YOLO26n | 55 | newest; INT8 on hardware still being validated upstream |
| YOLOv8s | 39 | "small": about half the speed of "nano" |
| YOLO11s | 35 | |
| YOLOv8m | 16 | too slow for a moving robot |
| YOLO-World v2s | 21 | open vocabulary (detect by text prompt), no training |

Reading this table:

- **Start with a "nano" model.** Game pieces are simple, high-contrast shapes. Nano is very
  likely enough, and it leaves headroom.
- Expect real end-to-end rates well below these. One report of a full YOLOv8n pipeline on
  an RK3588S got about 31 FPS on one core and 46 FPS using all three, limited by the
  camera. Measuring this on our board is task T3.
- **Smaller input is the cheapest speed-up.** 640x640 is the default, not a law. 416 or 320
  roughly doubles to quadruples the rate, at the cost of range on small, far objects. Worth
  measuring because a ball at 4 m is only a few pixels wide.

Recommendation to test first: **YOLOv8n, 640x640, INT8**, then YOLO11n and a 416 input as
comparisons. Reasons: best-supported path, and it is one of the three architectures
PhotonVision accepts, which keeps that door open.

## 4. What we would actually use it for

We already know where we are on the field (odometry, AprilTags, QuestNav) and where the
fixed field elements are. ML earns its place only for things that **move**.

| Job | Model type | Classes | Difficulty | Value |
|---|---|---|---|---|
| A. Game pieces on the floor | detection | 1 (the game piece) | low | high: replaces the HSV colour filter, which breaks with lighting and with anything else the same colour |
| B. Other robots (auto defense, avoidance) | detection | `robot`, or `red_bumper` / `blue_bumper` | medium | high for defense; needs tracking on top (section 6) |
| C. Game piece clusters / "where is the most fuel" | detection + counting | same as A | low once A works | medium |
| D. Robot heading or intake direction | pose or OBB | keypoints on bumper corners | high | low until B works |
| E. Open-vocabulary experiments | YOLO-World | none (text prompts) | low to try | useful for bootstrapping labels, not for matches |

Do A first. It is the easy one, it directly improves something the robot already does
(driving to a vision target), and it builds the whole pipeline that B needs.

On B, be realistic: detecting a robot in the image is the easy half. Turning that into
"where is it on the field and where is it going" needs a distance estimate and a tracker.
Bumpers are the best target because every robot has them, they are at a known height, and
they are red or blue by rule. Detecting whole robots works but gives a poor distance.

**About the game piece zoo.** The game piece changes every January, so a model has a
one-season life. The asset to build is not a model but the *process*: capture, label,
train, convert, measure, in a weekend. For 2026 (REBUILT) the piece is FUEL, the ball our
HSV pipeline calls `yellow`; confirm dimensions in the game manual before using them for
distance. Robot and bumper models, by contrast, carry over from year to year.

## 5. How it fits the code we already have

The vision repo already publishes, per camera and per target type:

```
/Cameras/<Camera>/<label>/targets     how many
/Cameras/<Camera>/<label>/distance    metres, nearest target
/Cameras/<Camera>/<label>/rotation    degrees, left positive
/Cameras/<Camera>/<label>/strafe      metres, left positive
/Cameras/<Camera>/<label>/id
```

where `<label>` is `tags` or a colour such as `yellow`. The robot's `subsystems/vision.py`
reads exactly these, and `Vision.get_vision_target_pose()` turns them into a field pose
that `DriveToPoseCustomControl` drives to (see `DRIVE_TO_POSE_HOWTO.md`).

So **an ML detector should publish the same five topics under its own label**, for example
`/Cameras/LogitechFront/fuel/...`. Then:

- the robot code needs one new entry in `CameraConstants.k_cameras` and nothing else;
- HSV and ML can run side by side on the same camera and be compared live;
- the simulator's camera sim (`simulation/vision_sim.py`) already fakes these topics, so
  autos can be developed before the model exists.

On the coprocessor this is a new detector class next to `vision/detect_hsv.py`, returning
the same result dictionary (`targets`, `distances`, `rotations`, `strafes`, ...) so
`vision/network.py` publishes it unchanged.

Distance from a box, same idea `detect_hsv.py` uses now: a target of known real width that
fills a known fraction of the image is at a known distance.

```
distance = real_width / (2 * tan(box_width_fraction * horizontal_fov / 2))
rotation = -(box_centre_x_normalised) * horizontal_fov / 2
```

This degrades when the object is cut off by the image edge or partly hidden. A better
method for things on the floor: use the **bottom centre of the box** and the camera's known
height and tilt to intersect the floor plane. That needs no assumed size and is the one to
use for bumpers.

**Every detection, not just the nearest** (added 2026-11-14; robot side built and tested
in simulation only, nothing on a coprocessor publishes it yet). The robot's
`DriveThroughTargets` command drives one path through several game pieces, and reads them
from one more topic per camera and label:

```
/Cameras/<Camera>/<label>/field_positions    Translation2d[]  (struct array)
```

The contract, which `simulation/vision_sim.py` in both robot projects already follows:

- **Field coordinates, metres**, blue-origin, the same frame as the robot pose. The
  coprocessor already subscribes to `/SmartDashboard/Swerve/drive_pose` (`vision/network.py`);
  place each detection with it: `robot.translation() + robot_relative.rotateBy(robot.rotation())`.
- The robot-relative position needs the **camera's mount** (position and yaw on the robot),
  not just distance and bearing from the lens.
- Use the **floor-plane method** above for distance. The box-width method is wrong for a
  ball cut off by the image edge or touching its neighbour, and a field of balls is mostly
  that.
- **Nearest first.** Publish with `keepDuplicates=True` so the timestamp advances while the
  list is unchanged, and publish an **empty list** when nothing is seen. The robot ignores a
  list older than 1 s.
- The robot merges duplicates from two cameras (within 15 cm) and decides the order and the
  path itself, from `deploy/pathplanner/navgrid.json`. The coprocessor only reports where
  things are.
- Latency: the pose used should be the one from when the frame was captured. Using the
  latest pose is fine while driving straight and wrong by (turn rate x latency) while
  turning; measure before trusting it at speed.

## 6. Tracking, for defense

A detector answers "what is in this frame". Defense needs "that robot, over there, moving
that way". That is a tracker, and it runs on the CPU, not the NPU:

- **ByteTrack** is the standard light choice: it links boxes between frames and gives each
  object a stable ID. Ultralytics ships it; standalone versions exist.
- Convert each tracked box to field coordinates (floor-plane method above plus our pose),
  then a simple constant-velocity filter gives position and velocity.
- The robot side then needs a decision layer: which robot, where to stand. That is robot
  code and strategy, not ML, and it is the larger half of the work.

Expect this to be a full off-season project. Do not promise it for a competition before job
A has been reliable for a while.

## 7. Data and testing

Training data matters more than model choice.

**Where images come from**

- Our own cameras, mounted where they are on the robot, at our resolution and exposure.
  This is the data that counts. Record video on a practice field and at events.
- Roboflow Universe has public FRC sets. Found so far, none verified for quality or
  licence: an "FRC 2026" set of about 13,000 images, a "FRC Scorekeeper 2026" robot + FUEL
  model, several robot-detection sets (hundreds of images), and older game-piece sets.
  Useful for pre-training and for the robot class; not a substitute for our own images.

**How many**

- One simple class: a few hundred labelled images gets a working model; one to two thousand
  with varied lighting, distance, clutter and motion blur gets a dependable one.
- Include **empty frames and look-alikes** (yellow shirts, field tape, lights). A model
  never shown what is *not* a game piece will find game pieces everywhere.

**Splits.** Train / validation / test, and split by **recording session**, not by frame.
Neighbouring video frames are near-duplicates; splitting them randomly makes the test score
meaninglessly high.

**What to report for every model** (this is the acceptance test):

| Measure | On | Why |
|---|---|---|
| mAP50 and mAP50-95 | test set, `.pt` on PC | is the training good |
| the same | test set, `.rknn` INT8 | what quantization cost; investigate a drop of more than a few points |
| recall at 1, 2, 3, 4 m | staged shots | how far it really sees |
| false positives per minute | video of an empty field and a crowd | will it chase a shirt |
| end-to-end FPS and latency | on the Orange Pi with the real camera | the only speed that matters |
| distance error at known distances | staged shots | is the geometry right |
| NPU and CPU load, temperature after 10 min | on the board | will it throttle in a match |

A model is good enough to drive to when it holds roughly 95 percent recall inside the range
we use, with false positives rare enough that the "nearest target" is almost never a ghost.

## 8. Build it ourselves or use PhotonVision

PhotonVision already does RKNN object detection on Orange Pi 5 boards and is a real option.

| | Our vision repo + RKNN | PhotonVision |
|---|---|---|
| Models accepted | anything we can post-process | only 640x640 Ultralytics YOLOv5 / v8 / v11, quantized |
| Topics | our existing schema, robot code unchanged | PhotonVision's own; robot code needs a PhotonLib path |
| Effort | we write the detector class and post-processing | upload a `.rknn` in the web UI |
| Tags + ML together on one board | ours to schedule | supported |
| Conversion help | model zoo scripts | a ready notebook (`scripts/rknn_conversion.ipynb`), runs in Colab |

A sensible path: **use PhotonVision's conversion notebook and constraints as the recipe**
(640x640 YOLOv8n, INT8) even if we run the result in our own pipeline. Then the same
`.rknn` file works in both, and PhotonVision is a fallback that costs nothing to keep.

## 9. Groundwork tasks

Each task is self-contained and has a "done when". Do them in order; T1 to T4 need no
training and no game pieces.

**T1. Bring up the NPU.** Flash the board, install the RKNN runtime and
`rknn-toolkit-lite2`, record the OS image and every version number.
*Done when:* a script prints the runtime and driver versions and they match the toolkit
version we will convert with. Write the versions into this file.

**T2. Run a stock model.** Take the pre-converted `yolov8n` from `rknn_model_zoo`, run it
on a still image and on a live camera, draw boxes.
*Done when:* boxes appear on a person or a chair from the robot camera.

**T3. Measure speed.** For yolov5n, yolov8n and yolo11n at 640 and 416: inference-only FPS
on one core and on three, and end-to-end FPS with our camera.
*Done when:* a table of our numbers replaces the vendor table in section 3. Note NPU load
and temperature.

**T4. Prove the conversion chain.** On x86 Linux (WSL2 or Colab), convert the stock
`yolov8n.pt` to `.rknn` ourselves with INT8 calibration, and run it on the board.
*Done when:* our file gives the same boxes as the model zoo's, and the steps are written
down as commands someone else can paste.

**T5. Collect and label.** Record the game piece from the robot's cameras: near, far,
clustered, partly hidden, different lighting, plus empty and look-alike frames. Label one
class. Split by recording session.
*Done when:* at least 500 labelled images and 100 negatives, with the split recorded.

**T6. Train and convert.** Fine-tune YOLOv8n on T5, convert with our own calibration
images.
*Done when:* the section 7 table is filled in for both the `.pt` and the `.rknn`.

**T7. Publish to NetworkTables.** Add an ML detector class to the vision repo beside
`detect_hsv.py`, publishing under a new label with the existing five topics.
*Done when:* the robot's dashboard shows targets from the ML label, and distance and
rotation agree with a tape measure within the error recorded in T6.

**T8. Compare against HSV.** Run both on the same camera over the same recorded session.
*Done when:* there is a short table of misses and false positives for each, and a decision
on which one the robot uses.

**T9. Robots and bumpers.** Repeat T5 to T7 for a `robot` or bumper class, using public
datasets to start.
*Done when:* the section 7 table exists for it. Tracking (section 6) is a separate project
after this.

Rules for whoever does this work:

- Write down every version number and every command. Most lost time on this platform is
  version mismatch.
- Never report a speed without saying what was included (inference only, or camera to
  NetworkTables) and how many NPU cores.
- Never report an accuracy without saying which file (`.pt` or `.rknn`) and which test set.
- If a number in this file is replaced by a measurement, say so and date it.

## 10. Open questions

- Which OS image and kernel do our boards run, and does its NPU driver match the current
  runtime? (T1)
- Do we want tags and ML on the same board, and if so how are the three NPU cores and the
  CPU cores shared with AprilTag detection? Tag detection is CPU work, so they should not
  fight, but it has not been measured.
- What input size keeps a game piece detectable at the edge of the range we care about?
- Camera choice: a global-shutter camera reduces motion blur, which hurts ML detection the
  same way it hurts tags.
- Is the season's game piece distinct enough that HSV is already good enough? T8 answers
  that honestly; ML is not automatically better.

## 11. Things in this file that are not verified

- All FPS figures are vendor or community numbers for other RK3588 boards.
- The Roboflow datasets were found by search only; classes, quality and licences are
  unchecked.
- The 2026 game piece details are from memory of our own code (`yellow` HSV target), not
  re-read from the manual.
- Whether Ultralytics INT8 RKNN export is accurate on real hardware: upstream says it has
  only been validated in the simulator so far.

## Sources

- Rockchip model zoo, model list and RK3588 FPS table: https://github.com/airockchip/rknn_model_zoo
- Rockchip conversion toolkit: https://github.com/airockchip/rknn-toolkit2
- Ultralytics RKNN export guide: https://docs.ultralytics.com/integrations/rockchip-rknn/
- PhotonVision, Orange Pi 5 object detection: https://docs.photonvision.org/en/latest/docs/objectDetection/opi.html
- PhotonVision, RKNN benchmarks: https://docs.photonvision.org/en/latest/docs/benchmarks/rknn-model-benchmarks.html
- Seeed RK3576 / RK3588 vision benchmarks: https://sensecraft.seeed.cc/ai-lab/tutorials/rk/benchmark/cv
- Orange Pi 5 Pro overview: https://www.cnx-software.com/2024/04/12/orange-pi-5-pro-low-cost-rockchip-rk3588s-sbc-16gb-lpddr5-ram-dual-hdmi/
- Orange Pi 5 Max overview: https://www.cnx-software.com/2024/08/01/rockchip-rk3588-powered-orange-pi-5-max-sbc-features-up-to-16gb-lpddr5-2-5gbe-onboard-wifi-6e-and-bluetooth-5-3/
- Roboflow Universe FRC sets (unverified): https://universe.roboflow.com/testing-frfr/frc-2026-mldc0 , https://universe.roboflow.com/blind-assistant-model/frc-scorekeeper-2026 , https://universe.roboflow.com/frc2073/frc-robot-detection-ff9or
