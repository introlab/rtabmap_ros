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


import struct
import zlib

import numpy as np


def compress(data):
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
