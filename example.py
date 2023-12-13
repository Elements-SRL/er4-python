import er4CommLib_python as er4
import time
import numpy as np
from tail_recursion import tail_recursive

# This is an example usage of a device that uses er4.
# A lot of functions, return a tuple Err, .. where Err is an ErrorCode.
# If the ErrorCode is "Success" the function has worked as expected.
# Some function could return different error codes due to the different nature of the devices so be sure to check this values.

class Device:
    def __init__(self, ch_indexes, number_of_channels):
        self.number_of_channels = number_of_channels
        self.ch_indexes = ch_indexes
        self.number_of_channels = len(ch_indexes)

    # Value of current in nA
    @staticmethod
    def acquire(ch_idx):
        time.sleep(0.2)
        # clean all data from the device
        er4.purgeData()
        data = []
        # 1250 is the number of packets that I want to have, roughly one second of data 
        while len(data) < 1250:
            # get the status of the data queue
            err, qs = er4.getQueueStatus()
            if err != er4.ErrorCode.Success:
                # wait for data
                time.sleep(0.01)
                continue
            # read data from the queue
            err, packets_read, buffer = er4.readData(qs.availableDataPackets)
            # convert values from int to currents (start add 1 because the first element is the voltage, other values are currents)
            #  returns a tuple, ErrorCode, float
            tuples = [er4.convertCurrentValue(int(d[ch_idx + 1]), ch_idx) for d in buffer]
            data_in_channel = [t[1] for t in tuples if t[0] == er4.ErrorCode.Success]
            data = data + data_in_channel
        # return the average of the last n samples
        return sum(data) / len(data)
    
    # parallel version of the previous function
    def parallel_acquire(self, n_of_readings) -> np.ndarray:
        @tail_recursive
        # Value of current in nA
        def _parallel_acquire(data: np.ndarray) -> np.ndarray:
            # wait for a bit to let data accumulate
            time.sleep(0.1)
            if data.shape[1] > n_of_readings:
                return np.mean(data, axis=1)/1e9
            err, qs = er4.getQueueStatus()
            if err != er4.ErrorCode.Success:
                return _parallel_acquire(data)
            err, packets_read, buffer = er4.readData(qs.availableDataPackets)
            # print("avail data ", qs.availableDataPackets, " packets read ", packets_read)
            data_without_voltage = np.array([d[1:] for d in buffer])
            ch_measurement_in_rows = np.transpose(data_without_voltage)
            measurements = np.array([
                [er4.convertCurrentValue(int(m), idx) for m in measurements_for_one_ch]
                for measurements_for_one_ch, idx in zip(ch_measurement_in_rows, range(ch_measurement_in_rows.shape[0]))
            ])
            data_in_float = [[t[1] for t in list_of_measurements if t[0] == er4.ErrorCode.Success] for
                             list_of_measurements
                             in
                             measurements]
            return _parallel_acquire(np.hstack((data, data_in_float)))

        time.sleep(0.2)
        er4.purgeData()
        return _parallel_acquire(np.empty((len(self.ch_indexes), 0)))

    def set_dacs_in(self, voltage):
        # create a measurement:
        #  first argument the value to set
        #  second argument is the prefix (u, m, None, k, etc...)
        #  third argument is the unit of Measurement as a str
        v_dac_in_measurement = er4.Measurement(voltage, er4.UnitPfxNone, "V")
        # this will set the internal dacs to the initial measurement, will set all dacs if ch_index == num_channels
        er4.setVoltageOffset(len(self.ch_indexes), v_dac_in_measurement)

    # connect to the first listed device
    @staticmethod
    def connect():
        # detect the devices connected via usb
        #  returns a list of str and an ErrorCode
        err, devices = er4.detectDevices()
        if len(devices) > 0:
            # connect to the first device
            er4.connect(devices[0])
            print("successfully connected to " + devices[0])
            return True
        else:
            # if no device is been found exit
            print("no device connected, exiting in 5 seconds")
            time.sleep(5)
            return False

    # set the initial configuration of the device (in this example we are using an e16e)
    @staticmethod
    def configure():
        # set the current range
        #  first argument is the index of the range (0: 200pA, 1: 2nA, 2: 20nA, 3: 200nA)
        #  second argument is the channel to apply the current ragne to, 16 means all the channels
        er4.setCurrentRange(0, 16)
        #  set the voltage range
        #   first argument is the index of the range (0: 500mV )
        er4.setVoltageRange(0)
        #  set the voltage range
        #   first argument is the index of the range ()
        #   your device will return an error because it has np voltage reference range
        er4.setVoltageReferenceRange(0)
        # set the voltage low pass filter
        #   first argument is the index of the low pass filter range()
        #   your device will return an error because it has np voltage reference range
        er4.setVoltageReferenceLpf(1)
        # set the sampling rate
        #   first argument is the index of the sampling rate(0: 1.25kHz, 1: 5kHz, 2: 10kHz, 3: 20kHz, 4: 50kHz, 5: 100kHz, 6: 200kHz,)
        er4.setSamplingRate(0)
    
    # set voltage of the external dac
    @staticmethod
    def set_v_ref(voltage):
        v_dac_ext_measurement = er4.Measurement(voltage, er4.UnitPfxNone, "V")
        # apply the voltage to the external dac
        if er4.applyDacExt(v_dac_ext_measurement) != er4.ErrorCode.Success:
            print("error setting vref")

    # disconnect the device
    @staticmethod
    def disconnect():
        # disconnect the device
        er4.disconnect()
