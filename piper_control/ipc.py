"""Bounded local JSON/NumPy framing shared by isolated worker processes."""
import base64
import json
import struct
import numpy as np

MAX_MESSAGE_BYTES = 128 * 1024 * 1024


def _encode(value):
    if isinstance(value, np.ndarray):
        if value.dtype.hasobject:
            raise ValueError("Object arrays cannot cross the policy boundary")
        arr = np.ascontiguousarray(value)
        return {"__array__": base64.b64encode(arr.tobytes()).decode("ascii"),
                "dtype": arr.dtype.str, "shape": list(arr.shape)}
    if isinstance(value, np.generic):
        return value.item()
    raise TypeError("Cannot encode %s" % type(value).__name__)



def _decode(value):
    if "__array__" in value:
        dtype = np.dtype(value["dtype"])
        if dtype.hasobject or dtype.kind not in "buif":
            raise ValueError("Invalid transport dtype")
        shape = value["shape"]
        if len(shape) > 6 or any(type(n) is not int or n < 0 for n in shape):
            raise ValueError("Invalid transport shape")
        return np.frombuffer(base64.b64decode(value["__array__"], validate=True),
                             dtype=dtype).reshape(shape).copy()
    return value



def _owned(value):
    """Detach queued commands from mutable producer/camera buffers."""
    if isinstance(value, np.ndarray):
        return value.copy()
    if isinstance(value, dict):
        return {key: _owned(item) for key, item in value.items()}
    if isinstance(value, (tuple, list)):
        return [_owned(item) for item in value]
    return value



def write_message(stream, value):
    payload = json.dumps(value, default=_encode, allow_nan=False, separators=(",", ":")).encode()
    if len(payload) > MAX_MESSAGE_BYTES:
        raise ValueError("Policy message exceeds 128 MiB")
    stream.write(struct.pack("!I", len(payload)))
    stream.write(payload)
    stream.flush()



def _read_exact(stream, count):
    data = bytearray()
    while len(data) < count:
        block = stream.read(count - len(data))
        if not block:
            raise EOFError("Policy worker closed its pipe")
        data.extend(block)
    return bytes(data)



def read_message(stream):
    size = struct.unpack("!I", _read_exact(stream, 4))[0]
    if size > MAX_MESSAGE_BYTES:
        raise ValueError("Policy message exceeds 128 MiB")
    return json.loads(_read_exact(stream, size), object_hook=_decode)
