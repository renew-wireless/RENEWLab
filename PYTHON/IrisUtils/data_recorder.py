"""
 data_recorder.py

 Class used to record data into h5file

---------------------------------------------------------------------
 Copyright © 2018-2019. Rice University.
 RENEW OPEN SOURCE LICENSE: http://renew-wireless.org/license
 ---------------------------------------------------------------------
"""

import numpy as np
import h5py
import time
import datetime
import logging
import os


class DataRecorder:

    def __init__(self, tag, serial, freq, lna, pga, tia, lna1, lna2, attn, numSamps,latitude,longitude,elevation):
        self.num_frame = 1
        self.frame_number = 0
        self.numSamps = numSamps
        self.tag = tag
        self.serial = serial
        self.tia = tia 
        self.pga = pga 
        self.lna = lna 
        self.lna1 = lna1 
        self.lna2 = lna2 
        self.attn = attn
        self.freq = freq
        self.latitude = latitude
        self.longitude = longitude
        self.elevation = elevation
        
    def init_h5file(self, filename=None, init_datasets=True):
        """
        Initialize a new h5file with the correct data structure and attributes.
        We do this outside of __init__ so that multiple files can be collected without reinitializing.
        """

        #set the hdf5 filename
        if filename is None:
            self.filename = ("RX-"+ str(self.tag)+"-"+time.strftime("%Y-%m-%d-%H-%M-%S.hdf5"))
        else:
            self.filename = filename

        #create hdf5 file and setup the structure and attributes.
        if init_datasets:
            self.h5file = h5py.File(self.filename, "w")
            self.init_datasets()
            self.h5file.attrs['serial'] = self.serial
            self.h5file.attrs['frame_length'] = self.numSamps
            self.h5file.attrs['num_frames'] = 0
            self.h5file.attrs['frequency'] = self.freq
            self.h5file.attrs['start_time'] = time.time()
            self.h5file.attrs['start_clock'] = time.clock()
            self.h5file.attrs['rxgain_pga'] = self.pga
            self.h5file.attrs['rxgain_tia'] = self.tia
            self.h5file.attrs['rxgain_lna'] = self.lna
            self.h5file.attrs['rxgain_lna1'] = self.lna1
            self.h5file.attrs['rxgain_lna2'] = self.lna2
            self.h5file.attrs['rxgain_attn'] = self.attn
            self.h5file.attrs['latitude'] = self.latitude
            self.h5file.attrs['longitude'] = self.longitude
            self.h5file.attrs['elevation'] = self.elevation
            self.h5file.attrs['comment'] = self.tag
            self.close()    
        else:    
            self.h5file = h5py.File(self.filename, "r+")

    def init_datasets(self, group=None):
        self.h5group = "" if group is None else group
        group = self.h5file if group is None else self.h5file.create_group(group)
        self.samples = group.create_dataset("Samples", (self.num_frame, 2, self.numSamps),
                dtype=np.complex64, maxshape=(None, 2, self.numSamps), chunks=(self.num_frame, 2, self.numSamps))
        self.timeref = group.create_dataset("Time", (self.num_frame, 2),
                dtype=np.uint64, maxshape=(None, 2), chunks=(self.num_frame, 2))
        group['Samples'].attrs['column_names'] = [a.encode('utf8') for a in ['Frame', 'Ant', 'Samples'] ]


    def save_frame(self, frame, timestamp):
        self.init_h5file(filename=self.filename, init_datasets=False)
        self.samples = self.h5file['Samples']
        self.timeref = self.h5file['Time']
        cur_len = self.samples.shape[0]
        self.frame_number = cur_len + 1
        self.samples.resize(self.frame_number, axis=0)
        self.timeref.resize(self.frame_number, axis=0)
        #Write the buffers to disk
        self.samples[self.frame_number-1,:] = frame
        self.timeref[self.frame_number-1,0] = np.int64(time.time()) #np.int64(time.clock()*1e7)
        self.timeref[self.frame_number-1,1] = timestamp
        self.h5file.attrs['num_frames'] = self.frame_number
        self.close()


    def stop(self):
        if self.h5file is not None and self.h5file.id: #it seems the easy way to check if the h5file is open is just if file.id
            total_frames = self.samples.shape[0]  
            self.h5file.attrs['num_frames'] = total_frames
            print('Closing h5file %s with %d frames recorded.' % (self.filename, total_frames) )
            self.h5file.flush()
            self.h5file.close()

    def close(self):
        self.h5file.flush()
        self.h5file.close()

