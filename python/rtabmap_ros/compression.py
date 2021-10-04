
import zlib
import struct
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

    numpy_type_to_cvtype = {'uint8': 0, 'int8': 1, 'uint16': 2,
                            'int16': 3, 'int32': 4, 'float32': 5,
                            'float64': 6}

    compressed_data = bytearray(zlib.compress(data.tobytes()))
    compressed_data.extend(struct.pack("iii", dim1, dim2, numpy_type_to_cvtype[data.dtype.name]))

    return compressed_data

def uncompress(bytes):
    cvtype_to_numpy_type = {0: 'uint8', 1: 'int8', 2: 'uint16',
                            3: 'int16', 4: 'int32', 5: 'float32',
                            6: 'float64'}
    out = zlib.decompress(bytes[:len(bytes)-3*4])
    rows, cols, datatype = struct.unpack_from("iii", bytes, offset=len(bytes)-3*4)
    data = np.frombuffer(out, dtype=cvtype_to_numpy_type[datatype])
    return data.reshape((rows, cols))

