# Test data

Real frames for the odometry node tests, so that they register actual imagery instead of
synthetic noise. Synthetic textures give a detector corners that match nothing between
frames, which makes a "motion" that only proves the node did not crash.

Everything here is BSD-3-Clause, same authors and same terms as the rest of this
repository.

| Path | Origin | Used by |
| --- | --- | --- |
| `stereo/rect/{left,right}/{50,60}.jpg` | [RTAB-Map `data/stereo_rect`](https://github.com/introlab/rtabmap/tree/master/data/stereo_rect) | `test_stereo_odometry.cpp` |
| `stereo/rect/stereo_{left,right}.yaml` | [RTAB-Map `data/stereo_rect`](https://github.com/introlab/rtabmap/tree/master/data/stereo_rect) | `test_stereo_odometry.cpp` |
| `stereo/raw/{left,right}/{420,425}.jpg` | frames 420 and 425 (21.00 s and 21.25 s at 20 Hz) of the [stereo indoor tutorial](https://github.com/introlab/rtabmap/wiki/Stereo-mapping) test sequence | `test_stereo_odometry.cpp` |
| `stereo/raw/stereo_{left,right}.yaml`, `stereo/raw/stereo_pose.yaml` | that rig's own calibration (`stereo_tutorial_*`) | `test_stereo_odometry.cpp` |
| `rgbd/rgb/{17,154}.jpg`, `rgbd/depth/{17,154}.png` | [RTAB-Map `data/rgbd`](https://github.com/introlab/rtabmap/tree/master/data/rgbd) | `test_rgbd_odometry.cpp` |
| `rgbd/calib/{17,154}.yaml` | [RTAB-Map `data/rgbd`](https://github.com/introlab/rtabmap/tree/master/data/rgbd) | `test_rgbd_odometry.cpp` |

They are vendored rather than read from an RTAB-Map checkout because RTAB-Map reaches us
as an installed library: the `introlab3it/rtabmap` images used by CI delete the source tree
after `make install`, and the ROS buildfarm has no network during a build. A copy here is
what makes these tests run everywhere rather than skip.

## What the frames are

Two frames of each kind is the minimum that says anything: the first initialises the
odometry at the origin, the second has to be registered against it.

- **`stereo/rect`** -- `50` and `60`, two rectified pairs of the same scene a short motion
  apart. The estimate comes out at 0.171 m, steady to well under a centimetre across runs.
  These back `corelib/test/test_odometry.cpp` upstream, so a failure here that also fails
  there is an RTAB-Map issue rather than a ROS one.
- **`stereo/raw`** -- `420` and `425`, an unrectified pair a quarter second apart, for the
  `Rtabmap/ImagesAlreadyRectified:=false` path described in `doc/stereo_odometry.md`, and
  for pinning what happens when that parameter is left at its default on distorted images.
  A quarter second of walking forward, ~0.233 m, reproduced to a fraction of a percent
  across runs. Wider gaps in the same window register too, but not reliably: 21.00 s to
  22.00 s is ~1.04 m at around 45 inliers and lost tracking outright in one run out of
  eight, where this pair holds 210 or more.
- **`rgbd`** -- `17` and `154`, two frames of a hand-held Kinect sequence, far enough apart
  that losing tracking between them is a legitimate outcome.

In every set the left image is color and the right one grayscale, as the cameras recorded
them.

Both stereo sets come from the same 640x480 rig, but **not** from the same calibration, and
the two are not interchangeable:

| | `stereo/rect` | `stereo/raw` |
| --- | --- | --- |
| `distortion_coefficients` | zeros | the lens's real plumb_bob values (~-0.34) |
| `rectification_matrix` | identity | the rotation into the rectified frame |
| `projection_matrix` fx | 487.61 | 500.22 |
| baseline (`-Tx/fx`) | 0.1197 m | 0.1197 m |

Rectification is what leaves a calibration with no distortion and an identity rotation, so
the rectified file describes the output of `stereo_image_proc`, not what the camera
produced. Handing it to a raw pair claims a distortion-free lens the images do not have:
doing that costs roughly a third of the inliers (110 against 214) and triples the reported
standard deviation. `stereo_pose.yaml` holds the rig's measured extrinsics -- ~12 cm along
x plus a few milliradians of rotation -- which the tests publish as the TF between
`camera_left` and `camera_right`, the transform the node looks up when it has to rectify
the pair itself.

## File formats

The images are copied as they are. The calibration files differ from their originals by one
line: the format directive is commented out (`#%YAML:1.0`, with no `---`), which is the ROS
flavour of the same file. `%YAML:1.0` is not a valid YAML directive, so plain YAML parsers
-- `camera_info_manager`, `rosparam`, PyYAML -- reject the original form.
`test/test_data.hpp` puts the directive back in memory before handing the text to
`cv::FileStorage`, so the files stay readable by both.

Note that they carry no OpenCV `dt` field either, so `>> cv::Mat` cannot read them;
`rows`/`cols`/`data` are read element by element, as RTAB-Map's own `CameraModel::load`
does.

The depth images are 16-bit millimetres. `rgbd/calib/*.yaml` also carries a
`local_transform` (the optical-frame-to-robot transform RTAB-Map stores with the camera
model); the ROS nodes take that from TF instead, so the tests publish it as a `base_link`
-> `camera` static transform rather than reading it here.

## Using them

`test/test_data.hpp` loads these into `cv::Mat`, `sensor_msgs/CameraInfo` and
`geometry_msgs/Transform`. CMake passes the directory as `RTABMAP_ODOM_TEST_DATA_ROOT`,
pointing into the source tree: the test binaries are not installed, and neither are these
files.
