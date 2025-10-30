# Python wrapper

## Documentation
This project is a port of the C++ official library to Python. The detailed documentation resides in the ```include``` directory, with ```er4commlib.h``` being the most crucial file to consult, as it describes every callable function. For this reason, this repository includes that file, along with the C++ file that defines the mapping between the C++ and Python code using pybind11 (```main.cpp```).

Almost every function returns an error code, which should be used to verify whether the function call had the desired effect.
### Notable differences
The most significant difference between the C++ code and the Python code is the handling of parameters. The C++ code extensively uses parameters passed by reference, while the Python code does not.
For example, the function ```getChannelsNumber``` in C++ is defined as:

```cpp
ErrorCodes_t getChannelsNumber(
    uint32_t &voltageChannelsNum,
    uint32_t &currentChannelsNum,
    uint32_t &gpChannelsNum);
```
and its usage would be something like the follwing:
```cpp
uint32_t voltageChannelsNum, currentChannelsNum, gpChannelsNum;
auto err = er4CommLib::er4cgetChannelsNumber(voltageChannelsNum, currentChannelsNum, gpChannelsNum)
```
In this case, the function modifies the values for the voltage and current channels, and the error code indicates whether these values are reliable.

In Python, the same function does not accept any parameters but directly returns a tuple. 
The first element of the tuple is always the error code, and the subsequent elements are the values that would have been passed by reference.

The equivalent Python code looks like this:
```python
import er4_python_wrapper as c4
...

e, v_channels, i_channels, gp_channels = c4.getChannelsNumber()
``` 

Other functions have both input parameters (needed for the function) and output parameters (values that are returned). For example:
```cpp
ErrorCodes_t getCurrentRange(RangedMeasurement_t &currentRange, uint16_t channelIdx = 0);
```
This function "returns" the current range for the given channle index. 
In Python, the equivalent function would look like this:
```python
import er4_python_wrapper as c4
...
ch_index = 0
_, i_range = c4.getCurrentRange(ch_index)
``` 

In general, all parameters passed by reference and modified by the function in the Python wrapper are returned as a tuple, along with the error code. The number and order of the input parameters in the various functions remain unchanged.


### Reading the data
The communication library returns values that are **16 bit unsigned integers**, to get the real values they need to be multiplied by the resolution of the respective range and treated as **16 bit signed integers**.
The commlib exposes 2 methods to do so but for efficiency reason we strongly recommend to get the resolution from the range you want to scale and then using numpy for the multiplications, as shown in the following snippet.
```python
import er4_python_wrapper as c4
i_data = np.array([])
v_data = np.array([])
gp_data = np.array([])

_, i_range = c4.getCurrentRange(0)
_, v_range = c4.getVoltageRange()
_, gp_range = c4.getGpRange(0)
# getting the data resolution, needed to go from the int version of the data to the usable float value
v_m = v_range.step
i_m = i_range.step
gp_m = gp_range.step
e, v_channels, i_channels, gp_channels = c4.getChannelsNumber()
total_channels = v_channels + i_channels + gp_channels
# remove all previous data
c4.purgeData()
# get at least 2 seconds of data
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
        # some devices might have general purpose channels
        if gp_channels > 0:
            # the data needs to be used in a slightly different way
            gp_data = np.append(gp_data, (data_matrix[v_channels + i_channels] * gp_m) + gp_m.min)
```
From this snippet you can notice other 4 things:
1. We put the data from the *commlib* in a *np.array* using the flag ```copy=False```, this gives the wrapper the possibility to use the C++ data without having to make fresh and costly copies.
2. The data returned by the commlib is organized in sectors. Specifically, there are **n** voltage values (where n is the number of the voltage channels) followed by **m** current values (where m is the number of current channels). After the current data, there might be **p** general puprpose values (where p is the number of general purpose channels). To organize the data more conveniently, we reshape the matrix to the total number of channelsas shown in the snippet, placing all the values in columns. By calling transpose, we organize them into rows. In this way the rows `[0,n[` will contain the voltages, the rows `[n, n + m[` will contain the currents and there might be gp values in the rows `[n+m, n+m+p[`.
3. The data needs to be interpreted as **16-bit signed integer** for **current** and **voltage** values, while it needs to be interpreted as **16-bit unsigned integer** for **general purpose** values.
4. To obtain correct current and voltage data it is only necessary to multiply the raw data by the data resolution. However, to work with general purpose channels, there is an extra step where we have to sum the minimum of the range.

## Working with the wrapper
### Installation
Currently the Python wrapper of the e4 only works on Windows systems.

#### Windows
##### Requirements
For the example to work, you need to install the USB driver provided by FTDI. This can be done by either installing EDR4, or by installing the driver directly from https://ftdichip.com/drivers/d2xx-drivers/, version 2.12.36.20 x64 for Windows.

In either case, the procedure correctly ends only once the current amplifier is plugged to to the PC via a USB cable.

- Python **3.11.7**
- custom dlls v0.18.0:
    - You can download them from [here](https://elements-ic.com/wp-content/uploads/2024/02/python_3_11_7_dlls.zip)

> :warning: **The dlls must be used only with Python version 3.11.7**

Once installed locate the python interpreter, should be under the following path ```C:\Users\<User>\AppData\Local\Programs\Python310```

Open a **command prompt**.

Navigate to the folder you wish to install the calibrator to.

Now you need to use Python to create a **virtual environment** also called **venv**.

Be sure to use the right version of Python to create the virtual environment, for this reason go to the previously located path and copy the absolute path of the **python.exe** file like in the following image.

![Python Path](./images/pp.png "Python Path")

```
"abs/path/of/python.exe" -m venv .\
```
After the creation of the venv activate it by typing: ```Scripts\activate```

Now that the venv is active you need to install all the dependency from the requirements.txt using ```pip install -r path\to\requirements.txt``` then copy all the files from the **dlls** folder into the **Lib\site-packages** folder that has been created with the venv.

### Usage
The typical sequence of commands is:
- detectDevices
- connect
- configure: you can configure it in various way (sampling rate, current range ecc.)
- read data/run your experiment
- disconnect

To work with the er4-python open the command prompt.

Activate the virtual environment with: ```Scripts\activate```

Run the script : ```python .\your_script.py```

You can find an example with some callable functions in this directory.
