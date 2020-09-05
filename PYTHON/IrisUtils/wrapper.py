from ctypes import *
import json

lib = cdll.LoadLibrary('../../CC/Sounder/libsounder_module.so')

class Config(object):
    def __init__(self,val):
        lib.Config_new.argtypes = [c_char_p]
	lib.Config_new.restype = c_void_p
        self.obj = lib.Config_new(c_char_p(val))

class Sounder(object):
    def __init__(self,configfile):
        config = Config(configfile)
        lib.Recorder_start.argtypes = [c_void_p]
        lib.Recorder_start.restype = c_void_p
        lib.Recorder_getRecordedFrameNum.argtypes = [c_void_p]
        lib.Recorder_getRecordedFrameNum.restype = c_int
        lib.Recorder_getTraceFileName.argtypes = [c_void_p]
        lib.Recorder_getTraceFileName.restype = c_char_p
	lib.Recorder_new.argtypes = [c_void_p]
	lib.Recorder_new.restype = c_void_p
        self.obj = lib.Recorder_new(config.obj)

    def start(self):
        lib.Recorder_start(self.obj)

    def getMaxFrame(self):
        return lib.Recorder_getRecordedFrameNum(self.obj)

    def getTraceFileName(self):
        return lib.Recorder_getTraceFileName(self.obj)
