import ctypes
import mmap
import os


class data_shared(ctypes.Structure):
    _fields_ = [
        ("t", ctypes.c_float),
        ("x", ctypes.c_float * 4),
        ("u", ctypes.c_float * 2),
        ("r", ctypes.c_float * 4),
        ("copy_now", ctypes.c_int),
    ]


fd = os.open("/dev/shm/states_controller", os.O_RDWR)
mm = mmap.mmap(
    fd, ctypes.sizeof(data_shared), mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE
)

data = data_shared.from_buffer(mm)

while 1:
    if data.copy_now == 1:
        print(data.t, list(data.x), list(data.u), list(data.r))
        data.copy_now = 0
