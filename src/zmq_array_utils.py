import zmq
import numpy as np

def recv_array(socket, flags=0, copy=True, track=False):
    """recv a numpy array"""
    # md = socket.recv_json(flags=flags)
    msg = socket.recv(flags=flags, copy=copy, track=track)
    buf = buffer(msg)
    A = np.frombuffer(buf, dtype=np.float64)
    return np.copy(A)

def send_array(socket, A, flags=0, copy=True, track=False):
    return socket.send(A, flags, copy=copy, track=track)