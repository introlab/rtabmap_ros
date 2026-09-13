# Copyright 2025 matlabbe
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#    * Redistributions of source code must retain the above copyright
#      notice, this list of conditions and the following disclaimer.
#
#    * Redistributions in binary form must reproduce the above copyright
#      notice, this list of conditions and the following disclaimer in the
#      documentation and/or other materials provided with the distribution.
#
#    * Neither the name of the matlabbe nor the names of its
#      contributors may be used to endorse or promote products derived from
#      this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.


"""
Compress numpy arrays into RTAB-Map's ``cv::Mat`` wire format.

RTAB-Map stores and transmits matrices -- images, laser scans, descriptors -- as a zlib
payload followed by a 12-byte trailer recording the shape and the element type. Database
blobs and the compressed fields of ``rtabmap_msgs`` messages both use it.

The layout is::

    [ zlib stream of the elements in C order ][ rows ][ cols ][ type ]
                                               int32   int32   int32

The three trailer fields are written with ``struct`` format ``'iii'`` -- native byte order
and size, matching the C++ side's raw ``int`` writes. That makes the encoding
**host-endian**, so a blob does not travel between machines of opposite endianness.

``type`` is the OpenCV depth of the elements: 0 ``CV_8U``, 1 ``CV_8S``, 2 ``CV_16U``,
3 ``CV_16S``, 4 ``CV_32S``, 5 ``CV_32F``, 6 ``CV_64F``.

These two functions are the Python side of that format. They match ``compressData()`` and
``uncompressData()`` in RTAB-Map's ``corelib/src/Compression.cpp`` byte for byte, so a
matrix written by either side can be read by the other.

Single-channel matrices only. The C++ encoder packs the channel count into the type code
alongside the depth; the tables here cover the single-channel depths ``CV_8U`` through
``CV_64F``, which is what the codes 0 to 6 mean.
"""


import struct
import zlib

import numpy as np


def compress(data):
    """
    Compress a 1-D or 2-D array into RTAB-Map's format.

    :param data: a single-channel array whose dtype is one of ``uint8``, ``int8``,
        ``uint16``, ``int16``, ``int32``, ``float32`` or ``float64``. A 1-D array of
        length ``n`` is recorded as a 1-by-``n`` matrix, which is the shape
        :func:`uncompress` gives back. Any memory layout is accepted; the bytes are
        always written in C order.
    :returns: a ``bytearray`` holding the zlib payload followed by the trailer described
        in the module docstring.
    :raises AssertionError: if ``data`` has more than two dimensions.
    :raises KeyError: if its dtype is not one of the seven above -- ``int64`` and
        ``float16`` have no encoding in this format.
    """
    assert data.ndim == 1 or data.ndim == 2

    dim1 = 1
    if data.ndim == 1:
        dim1 = 1
        dim2 = len(data)
    else:
        dim1 = data.shape[0]
        dim2 = data.shape[1]

    numpy_type_to_cvtype = {
        'uint8': 0,
        'int8': 1,
        'uint16': 2,
        'int16': 3,
        'int32': 4,
        'float32': 5,
        'float64': 6,
    }

    compressed_data = bytearray(zlib.compress(data.tobytes()))
    compressed_data.extend(
        struct.pack('iii', dim1, dim2, numpy_type_to_cvtype[data.dtype.name])
    )

    return compressed_data


def uncompress(data):
    """
    Restore an array written by :func:`compress` or by RTAB-Map's C++ side.

    :param data: a bytes-like object laid out as :func:`compress` returns.
    :returns: a 2-D array of the recorded shape and dtype. It is 2-D even when
        :func:`compress` was handed a 1-D array, and it is **read-only**: it views the
        decompressed buffer instead of copying it, so call ``.copy()`` before writing.
    :raises KeyError: if the trailer's type code is not a single-channel depth 0 to 6,
        which is what a multi-channel matrix from the C++ side encodes to.
    """
    cvtype_to_numpy_type = {
        0: 'uint8',
        1: 'int8',
        2: 'uint16',
        3: 'int16',
        4: 'int32',
        5: 'float32',
        6: 'float64',
    }
    out = zlib.decompress(data[: len(data) - 3 * 4])
    rows, cols, datatype = struct.unpack_from('iii', data, offset=len(data) - 3 * 4)
    data = np.frombuffer(out, dtype=cvtype_to_numpy_type[datatype])
    return data.reshape((rows, cols))
