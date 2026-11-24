"""Exact sampled-ray implementation. No approximation or FMA contraction."""
import ctypes
from pathlib import Path
import numpy as np
from envs import FastV2Env,rotation
D=ctypes.POINTER(ctypes.c_double);U=ctypes.POINTER(ctypes.c_uint8)
lib=ctypes.CDLL(str(Path(__file__).with_name('lidar_native.so')))
lib.cast.argtypes=[U,ctypes.c_int,ctypes.c_int,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,ctypes.c_double,D,ctypes.c_int,D,ctypes.c_int,ctypes.c_double,D]
lib.cast.restype=None
class NativeV2Env(FastV2Env):
 def lidar(self):
  origin=self.pose[:2]+rotation(self.pose[2])@self.lidar_offset;angles=self.angles+self.pose[2];dirs=np.ascontiguousarray(np.c_[np.cos(angles),np.sin(angles)])
  result=np.empty(self.beams,dtype=np.float64);cells=self.laser_grid.blocked
  assert cells.flags.c_contiguous
  lib.cast(cells.ctypes.data_as(U),cells.shape[1],cells.shape[0],self.laser_grid.resolution,*self.size,*origin,dirs.ctypes.data_as(D),self.beams,self._ranges.ctypes.data_as(D),len(self._ranges),self.max_range,result.ctypes.data_as(D))
  result+=self.np_random.normal(0,self.sensor_noise,self.beams)
  return np.where(self.mask,np.clip(result,0,self.max_range)/self.max_range,0).astype(np.float32),self.mask.astype(np.float32)
