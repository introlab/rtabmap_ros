# rtabmap_python

Python helpers for reading and writing the binary formats [RTAB-Map](https://github.com/introlab/rtabmap) uses.

RTAB-Map is a C++ library, and the data it hands to ROS is not always plain ROS types. Several `rtabmap_msgs` fields — and every blob in an `.db` database — carry a matrix in RTAB-Map's own compressed encoding rather than as a `sensor_msgs/Image` or an array. This package is the Python side of that encoding, for scripts that read those fields without going through the C++ library.

There are no nodes here. It is an `ament_python` package that installs one importable module.

## Contents

- [Module](#module)
- [Things worth knowing](#things-worth-knowing)
- [License](#license)

## Module

`rtabmap_python.cv_compression` — a single-channel `cv::Mat` to and from bytes.

| Function | Description |
|---|---|
| `compress(data)` | 1-D or 2-D numpy array → `bytearray`. |
| `uncompress(data)` | those bytes → 2-D numpy array. |

```python
import numpy as np
from rtabmap_python.cv_compression import compress, uncompress

scan = np.zeros((360, 2), dtype=np.float32)
blob = compress(scan)          # what the message field carries
restored = uncompress(blob)    # (360, 2) float32
```

The encoding is a zlib stream followed by a 12-byte trailer holding rows, cols and the element type as three `int32`. It matches `compressData()` and `uncompressData()` in RTAB-Map's `corelib/src/Compression.cpp` byte for byte, so either side can read what the other wrote. The [module docstring](rtabmap_python/cv_compression.py) has the exact layout, and the generated [Python API reference](https://docs.ros.org/en/jazzy/p/rtabmap_python/) renders it alongside the two functions.

## Things worth knowing

**The result is read-only.** `uncompress` views the decompressed buffer instead of copying it, so the array it returns has `writeable=False` and assigning into it raises. Call `.copy()` if you need to modify it.

**Single-channel only.** The C++ encoder packs the channel count into the type code; the tables here cover the single-channel depths `CV_8U` through `CV_64F`. A multi-channel matrix written by the C++ side raises `KeyError` rather than decoding wrongly. So does an unsupported dtype on the way in — `int64` and `float16` have no encoding.

**The trailer is host-endian**, because the C++ side writes raw `int`s. A blob is not portable between machines of opposite endianness.

## License

BSD-3-Clause. See the [repository root](https://github.com/introlab/rtabmap_ros#license).
