#!/usr/bin/env python

import serial
import struct
import time
import threading
import sys
from scipy.signal import butter, lfilter
from dataclasses import dataclass

import pyvisa
import numpy as np
from collections import namedtuple

import pyinotify
import time
import os

#macros 
DUAL_PROBE = False           #set True if using 2 magnetometers
#SINGLE_PROBE_OFFSET = -35    #in uT, used when no seconds probe is used the measure the DC offset in the FOV
PRINT_ACQUISITION = False     #set True to print ADC output to terminal
PRINT_COMPENSATION = False   #set True to print supply input to terminal
OUTPUT_NAME = 'data/output.txt'
CORRECTION_PATH = 'data/correction_status.txt'
OFFSET_PATH = 'data/offset.txt'
end_time = -1
calibration_time = 10

#constants for acquisition
Vref = 2.5
pga = 1
PORTNAME = '/dev/ttyACM0'
SCALING = 1 / 0.01  # Bartington is 10 mV/uT --> 1 / 0.01 uT/V

#constants for supply control
VISANAME = 'USB0::10893::4354::MY61005594::0::INSTR'
UT_TO_AMP = 1 / 50.17 #A / uT

#constants for real time low pass filter
F_SAMPLING = 1000   
F_CUTOFF = 10
N = 4       #filter order
F_NYQUIST = F_SAMPLING/2
F_CUTOFF_NORM = float(F_CUTOFF)/float(F_NYQUIST)
B, A = butter(N, F_CUTOFF_NORM, btype='low', analog=False)  #numerator and denominator of the LPF transfer function
BUFFER_SIZE = int(F_SAMPLING/2)       #size of the data buffer used for filtering, increases as cutoff frequency decreases 

#Define initial values
f = open(OFFSET_PATH,"r")
SINGLE_PROBE_OFFSET = int(f.readline())
f.close()

f = open(CORRECTION_PATH,"r")
correction_ON = int(f.readline())
f.close()

#Change correction status if file changed its value
def correction_status_changed(ev):
    global correction_ON
    try:
        f = open(CORRECTION_PATH,"r")
        correction_ON = int(f.readline())
        f.close()

        print ("Correction Changed{}".format(correction_ON))
    except Exception as e:
        print (e)

#Change offset value if file changed its value
def offset_changed(ev):
    global SINGLE_PROBE_OFFSET
    try:
        f = open(OFFSET_PATH,"r")
        SINGLE_PROBE_OFFSET = int(f.readline())
        f.close()

        print ("Offset Changed {}".format(SINGLE_PROBE_OFFSET))
    except Exception as e:
        print (e)


#threads for check filesystem 
def correction_thread(self):
    pyinotify.Notifier(correction_status_wm).loop()
def offset_thread(self):
    pyinotify.Notifier(offset_wm).loop()

#start threads to monitor file changes
correction_status_wm = pyinotify.WatchManager()
correction_status_wm.add_watch(CORRECTION_PATH, pyinotify.IN_MODIFY, correction_status_changed)
correction_thread = threading.Thread(target=correction_thread, args=(1,))
correction_thread.start()

offset_wm = pyinotify.WatchManager()
offset_wm.add_watch(OFFSET_PATH, pyinotify.IN_MODIFY, offset_changed)
offset_thread = threading.Thread(target=offset_thread, args=(1,))
offset_thread.start()

def correction_thread(self):
    pyinotify.Notifier(correction_status_wm).loop()

def offset_thread(self):
    pyinotify.Notifier(offset_wm).loop()

#---Definition of functions and types---
def open_serial_connection(port):
    while True:
        try:
            return serial.Serial(port, 500000)
        except serial.SerialException:
            print(f"Failed to open serial connection on {port}. Retrying in 1 second...")
            time.sleep(1)

def init_f():
    f = open(OUTPUT_NAME, 'w')
    f.write('Seconds\tB1_field[uT]\tB2_field[uT]\tB1_mean[uT]\tB2_mean[uT]\tB1_filtered[uT]\n')
    f.flush()
    return f

def write_f(sample):
    global f, B1_mean, B2_mean
    sample_string = f'{sample.us_counter*1e-6:.5f}\t{sample.B1_field:.4f}\t{sample.B2_field:.4f}\t{B1_mean:.4f}\t{B2_mean:.4f}\t{sample.B1_filtered:.4f}\n'
    f.write (sample_string)
    f.flush()
'''
def check_arguments ():
    if len(sys.argv) != 3:
        print ("Invalid arguments")
        print(f'Usage: python3 {sys.argv[0]} calibration_duration acquisition_duration')
        print ('Set acquisition_duration to -1 for infinite duration')
        sys.exit(1)
'''
#Launch kst
os.system("kst2")
#function ran by supply_thread
def supply_control (B1_mean, B_offset):  
    supply_write ('VOLT 5, (@1)')
    if correction_ON:
        supply_write('OUTP ON, (@1)')
    while not stop_flag.is_set():
        try:
            if correction_ON:
                with lock:
                    supply_input = -((B1_filtered-B1_mean)+B_offset)*UT_TO_AMP
                if supply_input <= 0:
                    print ("Supply error: cannot output negative current")
                    time.sleep (1)
                else:
                    string = f'CURR {supply_input:.5f}, (@1)'
                    supply_write (string)
                    time.sleep (0.05)    
            else:
                string = 'OUTP OFF, (@1)'
                supply_write (string)
                while not correction_ON and not stop_flag.is_set():
                    time.sleep (1)
                string = 'OUTP ON, (@1)'
                supply_write (string)
        except KeyboardInterrupt:
            break

    supply_write ('OUTP OFF, (@1)')    
    supply.close()

def supply_init():
    rm = pyvisa.ResourceManager('@py')  
    supply = rm.open_resource (VISANAME) #init DC supply
    supply.write_termination = '\n'
    supply.read_termination = '\n'
    supply.write ('OUTP OFF, (@1)') 
    return supply

def supply_write (string):
    supply.write (string)
    if PRINT_COMPENSATION:
        print (string)

def calibration (duration):
    start_time = time.time()
    end_time = start_time + duration

    data1 = np.array([])
    data2 = np.array([])

    print ('Offset: [{}] correction: [{}]\n'.format(SINGLE_PROBE_OFFSET, correction_ON))
    print ('---Calibration start--- \n')
    print (f'Wait {duration} seconds\n')
    sample = SerialData()

    while time.time() < end_time:
        sample = update_values (True)
        data1 = np.append (data1, sample.B1_filtered)
        data2 = np.append (data2, sample.B2_field)      #B2 will end up not being used 
    
    B1_mean = np.mean (data1)
    B2_mean = np.mean (data2)

    print ('Calibration finished\n')

    return sample.B1_field, sample.B1_filtered, sample.B2_field, B1_mean, B2_mean

def get_serial ():      #gets serial data from arduino and writes it in output file
    global ser, f
    while True:
        try:
            ADC1code_little_endian = ser.read (3)     #2*3 bytes of ADC data, 3 bytes for sample counter, 4 bytes for microseconds counter (int)
            if DUAL_PROBE:
                ADC2code_little_endian = ser.read (3)
            else: 
                ADC2code_little_endian = b'\x00\x00\x00'
            counter_bytes = ser.read (3)
            us_bytes = ser.read (4)
            status_byte = ser.read (1)

            if PRINT_ACQUISITION:
                ADC1code = struct.unpack ('<I', ADC1code_little_endian + b'\x00') [0]       # unpacks as an unsigned integer ('I') in little-endian ('<'), with one padding byte (b'\00') to match 32 bits
                ADC2code = struct.unpack ('<I', ADC2code_little_endian + b'\x00') [0]
                counter = int.from_bytes(counter_bytes, byteorder='little', signed=False)
            
            us = int.from_bytes(us_bytes, byteorder='little', signed=False)
            status = int.from_bytes(status_byte, byteorder='little', signed=False)
            #converts ADCcode to signed integer for scaling to voltage
            ADC1signed = int.from_bytes(ADC1code_little_endian, byteorder='little', signed=True)      
            ADC2signed = int.from_bytes(ADC2code_little_endian, byteorder='little', signed=True)
            #scaling from ADC code to voltage
            voltage1 = (ADC1signed/0x7FFFFF) * ((2*Vref) / pga)     
            voltage2 = (ADC2signed/0x7FFFFF) * ((2*Vref) / pga)
            #scaling from voltage to magnetic field (in uT)
            B1_field = voltage1 * SCALING
            if DUAL_PROBE:     
                B2_field = voltage2 * SCALING  
            else:
                B2_field = 0   

            if PRINT_ACQUISITION:
                hex_data = f'0x{ADC1code:06X}\t0x{ADC2code:06X}\t{counter}\t{us*1e-6:.5f}\t{voltage1:.7f}\t{voltage2:.7f}\t{B1_field:.4f}\t{B2_field:.4f}\t{status:b}'
                print (hex_data)
            
            sample = SerialData (B1_field, B2_field, us, None, status_byte)
            return sample
        
        except serial.SerialException:
            ser.close()
            f.close()
            ser = open_serial_connection(PORTNAME)
            f = init_f() 
            print("Serial connection reestablished")

def update_values (write):        #gets serial, computes filtered values and write them to output file if specified
    global data1_buffer, B1_filtered, correction_ON

    sample = get_serial ()
    data1_buffer.append(sample.B1_field)
    data1_buffer.pop(0)

    sample.B1_filtered = lfilter (B,A,data1_buffer)[-1]
    with lock:
        B1_filtered = sample.B1_filtered
        #correction_ON = bool ((sample.status[0] >> 0) & 1)
    if write:
        write_f(sample)

    return sample

def prefill_buffer():
    data_buffer = []
    for _ in range(BUFFER_SIZE):
        new_value = get_serial().B1_field
        data_buffer.append(new_value)
    return data_buffer

@dataclass
class SerialData:
    B1_field: float = -1
    B2_field: float = -1
    us_counter: float = -1
    B1_filtered: float = -1
    status : str = None

#---Beginning of the program---
B1_filtered = B1_mean = B2_mean = 0
data1_buffer = []
lock = threading.Lock() #for sharing of variables between main thread and supply_thread
stop_flag = threading.Event()

#--Initialization--
#check_arguments()
supply = supply_init()                  #init communication with DC supply
ser = open_serial_connection(PORTNAME)  #init serial connection with arduino 
f = init_f()                            #open output file
data1_buffer = prefill_buffer()         #prefil buffer prior to filtering of input1

B1_field, B1_filtered, B2_field, B1_mean, B2_mean = calibration(float(calibration_time))
print (f'B1_mean: {B1_mean}\tB2_mean: {B2_mean}\n')

#start the thread controlling the DC supply
supply_thread = threading.Thread(target=supply_control, args = (B1_mean,B2_mean if DUAL_PROBE else SINGLE_PROBE_OFFSET), daemon=True)
supply_thread.start()

#main thread acquiring measurements from magnetometer 
start_time = time.time()
settling_delay = 1      #gives a few seconds for DC supply to stabilize 
acquisition_begin_time = start_time + settling_delay

'''
if float(sys.argv[2]) == -1:
    end_time = -1
else:
    end_time = acquisition_begin_time + float(sys.argv[2])
'''

data1 = np.array([])
data2 = np.array([])

while time.time()<end_time or end_time == -1:
    try:
        sample = update_values(True)

        if time.time()>start_time+settling_delay:
            data1 = np.append (data1, sample.B1_field)
            if DUAL_PROBE:
                data2 = np.append (data2, sample.B2_field)
    except KeyboardInterrupt:
        break

stop_flag.set()
f.close()
ser.close()
supply_thread.join()

print (f'---End of {time.time()-acquisition_begin_time} seconds acquisition---\n')
print (f'B1_mean = {np.mean(data1)} uT\tB1_std = {np.std(data1)} uT\tB1_peaktopeak = {np.ptp(data1)} uT\n')
if DUAL_PROBE:
    print (f'B2_mean = {np.mean(data2)} uT\tB2_std = {np.std(data2)} uT\tB2_peaktopeak = {np.ptp(data2)} uT\n')

