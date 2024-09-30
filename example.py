import time
import er4_python_wrapper as c4
import sys
import numpy as np
import matplotlib.pyplot as plt
from tempfile import TemporaryFile


def list_protocols(protocols: list, protocol_name: str):
    if len(protocols) == 0:
        print("No " + protocol_name.lower() + " available for this protocol")
    else:
        for v in protocols:
            print(protocol_name + str(v))


def print_protocol_info(names: list[str], ranges: list[c4.RangedMeasurement]):
    for n, r in zip(names, ranges):
        print("Voltage name: " + n)
        print("Range:" + r.niceLabel())
        print("min:" + str(r.min))
        print("max:" + str(r.max))


err, devices = c4.detectDevices()
if err != c4.Success:
    print("connection failed")
    sys.exit()

err = c4.connect(devices)
if err != c4.Success:
    print("connection failed")
    sys.exit()

c4.setRawDataFilter(c4.Measurement(1.0, c4.UnitPfxNone, "Hz"), True, False)

e, v_channels, i_channels, gp_channels = c4.getChannelsNumber()
print(e, v_channels, i_channels, gp_channels)

e, current_ranges, default_options = c4.getCurrentRanges()
for cr in current_ranges:
    print(cr.niceLabel())

e, sampling_rates, default_options = c4.getSamplingRates()
for sr in sampling_rates:
    print(sr.niceLabel())

c4.setCurrentRange(1, i_channels)
c4.setSamplingRate(0)
print("Start digital offset compensation")
c4.digitalOffsetCompensation(i_channels, True)
time.sleep(5)
c4.digitalOffsetCompensation(i_channels, False)
print("Stop digital offset compensation")
e, names, images, voltages, times, slopes,frequencies, adimensionals = c4.getProtocolList()
for n in names:
    print(n)

protocol_id = 3
conduct_prot_voltages = voltages[protocol_id]
conduct_prot_times = times[protocol_id]
conduct_prot_slopes = slopes[protocol_id]
conduct_prot_frequencies = frequencies[protocol_id]
conduct_prot_adimensionals = adimensionals[protocol_id]

list_protocols(conduct_prot_voltages, "Voltages")
list_protocols(conduct_prot_times, "Times")
list_protocols(conduct_prot_slopes, "Slopes")
list_protocols(conduct_prot_frequencies, "Frequencies")
list_protocols(conduct_prot_adimensionals, "Adimensionals")

print("Available conductance protocol features to select with the previous indexes")
e, v_names, v_ranges, default_values = c4.getProtocolVoltage()
print_protocol_info(v_names, v_ranges)

e, t_names, t_ranges, default_values = c4.getProtocolTime()
print_protocol_info(t_names, t_ranges)

e, a_names, a_ranges, default_values = c4.getProtocolAdimensional()
print_protocol_info(a_names, a_ranges)

c4.selectVoltageProtocol(protocol_id)

v_hold = c4.Measurement(2.0, c4.UnitPfxMilli, "V")
c4.setProtocolVoltage(0, v_hold)

v_pulse = c4.Measurement(100.0, c4.UnitPfxMilli, "V")
c4.setProtocolVoltage(1, v_pulse)

v_step = c4.Measurement(20.0, c4.UnitPfxMilli, "V")
c4.setProtocolVoltage(2, v_step)

t_hold = c4.Measurement(5.0, c4.UnitPfxMilli, "s")
c4.setProtocolTime(0, t_hold)

t_pulse = c4.Measurement(40.0, c4.UnitPfxMilli, "s")
c4.setProtocolTime(1, t_pulse)

adim_n = c4.Measurement(10.0, c4.UnitPfxNone, "")
c4.setProtocolAdimensional(0, adim_n)

adim_n_r = c4.Measurement(0.0, c4.UnitPfxNone, "")
c4.setProtocolAdimensional(1, adim_n_r)

print("Start voltage protocol")
if c4.applyVoltageProtocol() != c4.Success:
    print("Cannot apply protocol")
    c4.disconnect()
    sys.exit()

data_to_read = 12500
data_read = 0

i_data = np.array([])
v_data = np.array([])

_, i_range = c4.getCurrentRange(0)
_, v_range = c4.getVoltageRange()

v_m = v_range.step
print(v_range.step)
i_m = i_range.step
print(i_range.step)

c4.purgeData()

total_channels = v_channels + i_channels + gp_channels
while len(i_data) < data_to_read:
    e, q = c4.getQueueStatus()
    available_data = q.availableDataPackets
    if available_data > 0:
        read_error, data = c4.readData(available_data)
        np_buffer = np.array(data, copy=False)
        data_matrix = np_buffer.reshape((-1, total_channels)).transpose()
        # getting the first voltage channel
        v_data = np.append(v_data, data_matrix[0].astype(np.int16) * v_m)
        # getting the first current channel
        i_data = np.append(i_data, data_matrix[v_channels].astype(np.int16) * i_m)
        if gp_channels > 0:
            i_data = np.append(i_data, (data_matrix[v_channels + i_channels] * i_m) + min)

# disconnect from the device
c4.disconnect()

ifile = TemporaryFile()
vfile = TemporaryFile()
np.save(ifile, i_data)
np.save(vfile, v_data)

# Only needed to simulate closing & reopening file
_ = ifile.seek(0)
_ = vfile.seek(0)
# data read from file
read_i_data = np.load(ifile)
read_v_data = np.load(vfile)
plt.plot(read_i_data)
plt.show()
plt.plot(read_v_data)
plt.show()
