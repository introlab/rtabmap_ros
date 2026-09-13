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


"""Tests for :mod:`rtabmap_python.cv_compression`."""

import struct
import zlib

import numpy as np
import pytest

from rtabmap_python.cv_compression import compress, uncompress


# The single-channel depths the format encodes, with the codes RTAB-Map's
# serializeMatType() gives them: CV_8U, CV_8S, CV_16U, CV_16S, CV_32S, CV_32F, CV_64F.
SUPPORTED_TYPES = [
    ('uint8', 0),
    ('int8', 1),
    ('uint16', 2),
    ('int16', 3),
    ('int32', 4),
    ('float32', 5),
    ('float64', 6),
]

# Three int32: rows, cols, type code.
TRAILER_SIZE = 3 * 4


@pytest.mark.parametrize('dtype,code', SUPPORTED_TYPES)
def test_roundtrip_preserves_shape_dtype_and_values(dtype, code):
    """Every supported depth survives a compress/uncompress cycle unchanged."""
    data = np.arange(12, dtype=dtype).reshape(3, 4)

    result = uncompress(compress(data))

    assert result.shape == (3, 4)
    assert result.dtype == np.dtype(dtype)
    assert np.array_equal(result, data)


@pytest.mark.parametrize('dtype,code', SUPPORTED_TYPES)
def test_trailer_records_rows_cols_and_type_code(dtype, code):
    """The last 12 bytes are rows, cols and the type code, as the C++ side writes them."""
    data = np.zeros((3, 4), dtype=dtype)

    trailer = bytes(compress(data)[-TRAILER_SIZE:])

    assert trailer == struct.pack('iii', 3, 4, code)


def test_payload_is_plain_zlib_of_the_c_order_bytes():
    """Everything before the trailer is a zlib stream, so the C++ side can inflate it."""
    data = np.arange(6, dtype=np.uint8)

    payload = bytes(compress(data)[:-TRAILER_SIZE])

    assert zlib.decompress(payload) == data.tobytes()


def test_compress_returns_a_bytearray():
    """The return type is a bytearray, which is what the message fields expect."""
    assert isinstance(compress(np.zeros(4, dtype=np.uint8)), bytearray)


def test_one_dimensional_input_comes_back_as_a_single_row():
    """A 1-D array is recorded as 1-by-n, so the roundtrip is not shape-preserving."""
    data = np.arange(5, dtype=np.float32)

    result = uncompress(compress(data))

    assert result.shape == (1, 5)
    assert np.array_equal(result.ravel(), data)


@pytest.mark.parametrize('shape', [(1, 5), (5, 1), (2, 3)])
def test_two_dimensional_shapes_are_preserved_exactly(shape):
    """Rows and cols are recorded separately, so no 2-D shape is transposed or flattened."""
    data = np.arange(5 if 1 in shape else 6, dtype=np.uint8).reshape(shape)

    assert uncompress(compress(data)).shape == shape


def test_non_contiguous_input_roundtrips():
    """A transposed view is written in C order, so it reads back as the same matrix."""
    data = np.arange(12, dtype=np.int16).reshape(3, 4).T
    assert not data.flags.c_contiguous

    result = uncompress(compress(data))

    assert result.shape == (4, 3)
    assert np.array_equal(result, data)


def test_empty_array_roundtrips_as_an_empty_row():
    """An empty array is not a special case; it comes back as a 1-by-0 matrix."""
    result = uncompress(compress(np.array([], dtype=np.uint8)))

    assert result.shape == (1, 0)
    assert result.dtype == np.uint8


def test_uncompressed_array_is_read_only():
    """uncompress() views the decompressed buffer rather than copying it."""
    result = uncompress(compress(np.arange(4, dtype=np.uint8)))

    assert not result.flags.writeable
    with pytest.raises(ValueError):
        result[0, 0] = 1
    # .copy() is the way out, as the docstring says.
    assert result.copy().flags.writeable


def test_accepts_bytes_as_well_as_bytearray():
    """uncompress() reads whatever compress() produced, converted or not."""
    data = np.arange(8, dtype=np.uint16).reshape(2, 4)

    result = uncompress(bytes(compress(data)))

    assert np.array_equal(result, data)


def test_larger_matrix_roundtrips():
    """A matrix big enough to actually exercise zlib, with non-trivial content."""
    rng = np.random.default_rng(42)
    data = rng.integers(0, 255, size=(120, 160), dtype=np.uint8)

    assert np.array_equal(uncompress(compress(data)), data)


def test_rejects_more_than_two_dimensions():
    """The format has no encoding for a third dimension, so compress() refuses one."""
    with pytest.raises(AssertionError):
        compress(np.zeros((2, 2, 2), dtype=np.uint8))


@pytest.mark.parametrize('dtype', ['int64', 'uint64', 'float16'])
def test_rejects_unsupported_dtype(dtype):
    """Depths outside CV_8U..CV_64F have no type code and raise rather than truncate."""
    with pytest.raises(KeyError):
        compress(np.zeros(4, dtype=dtype))


def test_rejects_unknown_type_code():
    """A trailer from a multi-channel C++ matrix encodes a code this table lacks."""
    payload = bytes(compress(np.zeros((2, 2), dtype=np.uint8))[:-TRAILER_SIZE])
    # What serializeMatType() returns for a 3-channel CV_8U matrix: depth + ((cn - 1) << 3).
    forged = bytearray(payload) + struct.pack('iii', 2, 2, 0 + ((3 - 1) << 3))

    with pytest.raises(KeyError):
        uncompress(forged)
