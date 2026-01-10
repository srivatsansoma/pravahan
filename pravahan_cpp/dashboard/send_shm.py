import ctypes
import mmap
import os


class data_shared(ctypes.Structure):
    _fields_ = [
        ("vertical_velocity", ctypes.c_float),
        ("throttle", ctypes.c_float),
        ("copy_now", ctypes.c_int),
    ]


fd = os.open("/dev/shm/controls", os.O_RDWR | os.O_CREAT)
mm = mmap.mmap(
    fd, ctypes.sizeof(data_shared), mmap.MAP_SHARED, mmap.PROT_READ | mmap.PROT_WRITE
)

data = data_shared.from_buffer(mm)