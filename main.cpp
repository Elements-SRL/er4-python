#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <string>
#include <vector>
#include "er4commlib.h"

#define WRAP_0_ARGS(rType, fname) \
    rType fname() override { PYBIND11_OVERRIDE(rType, MessageDispatcher, fname); }

#define WRAP_0_ARGS_PURE(rType, fname) \
    rType fname() override { PYBIND11_OVERRIDE_PURE(rType, MessageDispatcher, fname); }

#define WRAP_0_ARGS_RET_ERROR_CODES(fname) \
    er4CommLib::ErrorCodes_t fname() override { PYBIND11_OVERRIDE(er4CommLib::ErrorCodes_t, MessageDispatcher, fname); }

#define WRAP_0_ARGS_RET_ERROR_CODES_PURE(fname) \
    er4CommLib::ErrorCodes_t fname() override { PYBIND11_OVERRIDE_PURE(er4CommLib::ErrorCodes_t, MessageDispatcher, fname); }

#define PARTIAL_WRAP_N_ARGS_RET_ERROR_CODES(fname, ...) \
    PYBIND11_OVERRIDE(er4CommLib::ErrorCodes_t, MessageDispatcher, fname, __VA_ARGS__);

#define PARTIAL_WRAP_N_ARGS_RET_ERROR_CODES_PURE(fname, ...) \
    PYBIND11_OVERRIDE_PURE(er4CommLib::ErrorCodes_t, MessageDispatcher, fname, __VA_ARGS__);

#define GENERAL_GET(fname, inType) \
    .def(#fname, [](MessageDispatcher &self) {\
    inType inArg;\
    auto err = self.fname(inArg);\
    return std::make_tuple(err, inArg);\
    })

#define GET_RANGED_MEASUREMENT(fname) GENERAL_GET(fname, RangedMeasurement_t)
#define GET_RANGED_MEASUREMENT_VEC(fname) GENERAL_GET(fname, std::vector<RangedMeasurement_t>)
#define GET_MEASUREMENT(fname) GENERAL_GET(fname, Measurement_t)
#define GET_MEASUREMENT_VEC(fname) GENERAL_GET(fname, std::vector<Measurement_t>)
#define GET_U32(fname) GENERAL_GET(fname, uint32_t)
#define GET_U16(fname) GENERAL_GET(fname, uint16_t)
#define GET_RANGED_MEASUREMENT_AND_U32(fname) \
    .def(#fname, [](MessageDispatcher &self) {\
    RangedMeasurement_t range;\
    uint32_t i;\
    auto err = self.fname(range, i);\
    return std::make_tuple(err, range, i);\
    })
#define GET_STRING(fname) GENERAL_GET(fname, std::string)
#define GET_STRING_VEC(fname) GENERAL_GET(fname, std::vector<std::string>)
#define GET_COMPENSATION_CONTROL(fname) GENERAL_GET(fname, CompensationControl)

namespace py = pybind11;
uint16_t * data;
uint16_t * unfilteredData;
uint32_t voltageChannelsNumber, currentChannelsNumber, gpChannelsNumber, totalChannelsNumber;
std::vector<double> dData;

class I16Buffer {
public:
    I16Buffer(int16_t* data, size_t size) : data(data), size(size) {}
    I16Buffer(uint16_t* data, size_t size) : data((int16_t*)data), size(size) {}

    py::buffer_info get_buffer() {
        return py::buffer_info(
                    data,                                  // Pointer to buffer
                    sizeof(int16_t),                       // Size of one scalar
                    py::format_descriptor<int16_t>::format(),// Python struct-style format descriptor
                    1,                                     // Number of dimensions
                    { size },                              // Buffer dimensions
                    { sizeof(int16_t) }                    // Strides (in bytes) for each index
                    );

    }
private:
    int16_t* data;
    size_t size;
};

class U16Buffer {
public:
    U16Buffer(uint16_t* data, size_t size) : data(data), size(size) {}

    py::buffer_info get_buffer() {
        return py::buffer_info(
                    data,                                  // Pointer to buffer
                    sizeof(uint16_t),                       // Size of one scalar
                    py::format_descriptor<uint16_t>::format(),// Python struct-style format descriptor
                    1,                                     // Number of dimensions
                    { size },                              // Buffer dimensions
                    { sizeof(uint16_t) }                    // Strides (in bytes) for each index
                    );

    }
private:
    uint16_t* data;
    size_t size;
};

PYBIND11_MODULE(er4_python_wrapper, m) {
    m.def("detectDevices", []() {
        std::vector <std::string> deviceIds;
        auto err = er4CommLib::detectDevices(deviceIds);
        return std::make_tuple(err, deviceIds);
    }, "Detect plugged in devices");
    m.def("connect", [](std::vector <std::string> deviceIds) {
        auto err = er4CommLib::connect(deviceIds);
        if (err != er4CommLib::Success){
            return err;
        }
        er4CommLib::getChannelsNumber(voltageChannelsNumber, currentChannelsNumber, gpChannelsNumber);
        totalChannelsNumber=voltageChannelsNumber+currentChannelsNumber+gpChannelsNumber;
        return err;
    });
    m.def("disconnect", &er4CommLib::disconnect);
    m.def("sendCommands", &er4CommLib::sendCommands);
    m.def("readData", [](unsigned int dataToRead){
        unsigned int dataRead;
        auto ret = er4CommLib::readData(dataToRead, dataRead, data);
        return std::make_tuple(ret, U16Buffer(data, dataRead*totalChannelsNumber));
    });
    m.def("readAllData", [](unsigned int dataToRead){
        unsigned int dataRead;
        auto ret = er4CommLib::readAllData(dataToRead, dataRead, data, unfilteredData);
        const auto actualDataRead = dataRead*totalChannelsNumber;
        return std::make_tuple(ret, U16Buffer(data, actualDataRead), U16Buffer(unfilteredData, actualDataRead));
    });
    m.def("convertVoltageValue", [](uint16_t uintValue){
        double fltValue;
        auto ret = er4CommLib::convertVoltageValue(uintValue, fltValue);
        return std::make_tuple(ret, fltValue);
    });
    m.def("convertCurrentValue", [](uint16_t uintValue, uint16_t channelIdx){
        double fltValue;
        auto ret = er4CommLib::convertCurrentValue(uintValue, channelIdx, fltValue);
        return std::make_tuple(ret, fltValue);
    });
    m.def("convertGpValue", [](uint16_t uintValue, uint16_t channelIdx){
        double fltValue;
        auto ret = er4CommLib::convertGpValue(uintValue, channelIdx, fltValue);
        return std::make_tuple(ret, fltValue);
    });
    m.def("setVoltageRange", &er4CommLib::setVoltageRange);
    m.def("setVoltageReferenceRange", &er4CommLib::setVoltageReferenceRange);
    m.def("setCurrentRange", &er4CommLib::setCurrentRange);
    m.def("setGpRange", &er4CommLib::setGpRange);
    m.def("setSamplingRate", &er4CommLib::setSamplingRate);
    m.def("setOversamplingRatio", &er4CommLib::setOversamplingRatio);
    m.def("setVoltageStimulusLpf", &er4CommLib::setVoltageStimulusLpf);
    m.def("setVoltageReferenceLpf", &er4CommLib::setVoltageReferenceLpf);
    m.def("selectStimulusChannel", &er4CommLib::selectStimulusChannel);
    m.def("digitalOffsetCompensation", &er4CommLib::digitalOffsetCompensation);
    m.def("digitalOffsetCompensationAutostop", &er4CommLib::digitalOffsetCompensationAutostop);
    m.def("zap", &er4CommLib::zap);
    m.def("switchChannelOn", &er4CommLib::switchChannelOn);
    m.def("getSwitchedOnChannels", [](unsigned int idx){
        uint32_t channelMask;
        auto ret = er4CommLib::getSwitchedOnChannels(channelMask);
        return std::make_tuple(ret, channelMask);
    });
    m.def("turnOnDigitalOutput", &er4CommLib::turnOnDigitalOutput);
    m.def("turnLedOn", &er4CommLib::turnLedOn);
    m.def("enableFrontEndResetDenoiser", &er4CommLib::enableFrontEndResetDenoiser);
    m.def("resetDevice", &er4CommLib::resetDevice);
    m.def("holdDeviceReset", &er4CommLib::holdDeviceReset);
//  m.def("resetDigitalOffsetCompensation", &er4CommLib::resetDigitalOffsetCompensation)
    m.def("sendCommands", &er4CommLib::sendCommands);
    m.def("selectVoltageProtocol", &er4CommLib::selectVoltageProtocol);
    m.def("applyVoltageProtocol", &er4CommLib::applyVoltageProtocol);
    m.def("setProtocolVoltage", &er4CommLib::setProtocolVoltage);
    m.def("setProtocolTime", &er4CommLib::setProtocolTime);
    m.def("setProtocolSlope", &er4CommLib::setProtocolSlope);
    m.def("setProtocolFrequency", &er4CommLib::setProtocolFrequency);
    m.def("setProtocolAdimensional", &er4CommLib::setProtocolAdimensional);
    m.def("checkSelectedProtocol", [](unsigned int idx){
        std::string message;
        auto ret = er4CommLib::checkSelectedProtocol(idx, message);
        return std::make_tuple(ret, message);
    });
    m.def("checkProtocolVoltage", [](unsigned int idx, er4CommLib::Measurement_t voltage){
        std::string message;
        auto ret = er4CommLib::checkProtocolVoltage(idx, voltage, message);
        return std::make_tuple(ret, message);
    });
    m.def("checkProtocolTime", [](unsigned int idx, er4CommLib::Measurement_t time){
        std::string message;
        auto ret = er4CommLib::checkProtocolTime(idx, time, message);
        return std::make_tuple(ret, message);
    });
    m.def("checkProtocolSlope", [](unsigned int idx, er4CommLib::Measurement_t slope){
        std::string message;
        auto ret = er4CommLib::checkProtocolSlope(idx, slope, message);
        return std::make_tuple(ret, message);
    });
    m.def("checkProtocolFrequency", [](unsigned int idx, er4CommLib::Measurement_t frequency){
        std::string message;
        auto ret = er4CommLib::checkProtocolFrequency(idx, frequency, message);
        return std::make_tuple(ret, message);
    });
    m.def("checkProtocolAdimensional", [](unsigned int idx, er4CommLib::Measurement_t adimensional){
        std::string message;
        auto ret = er4CommLib::checkProtocolAdimensional(idx, adimensional, message);
        return std::make_tuple(ret, message);
    });
    m.def("setVoltageOffset", &er4CommLib::setVoltageOffset);
    m.def("checkVoltageOffset", &er4CommLib::checkVoltageOffset);
    m.def("applyInsertionPulse", &er4CommLib::applyInsertionPulse);
    m.def("applyReferencePulse", &er4CommLib::applyReferencePulse);
    m.def("applyReferencePulseTrain", &er4CommLib::applyReferencePulseTrain);
    m.def("overrideReferencePulse", &er4CommLib::overrideReferencePulse);
    m.def("setRawDataFilter", &er4CommLib::setRawDataFilter);
    m.def("applyDacExt", &er4CommLib::applyDacExt);
    m.def("setFastReferencePulseProtocolWave1Voltage", &er4CommLib::setFastReferencePulseProtocolWave1Voltage);
    m.def("setFastReferencePulseProtocolWave1Time", &er4CommLib::setFastReferencePulseProtocolWave1Time);
    m.def("setFastReferencePulseProtocolWave2Voltage", &er4CommLib::setFastReferencePulseProtocolWave2Voltage);
    m.def("setFastReferencePulseProtocolWave2Time", &er4CommLib::setFastReferencePulseProtocolWave2Time);
    m.def("setFastReferencePulseProtocolWave2Duration", &er4CommLib::setFastReferencePulseProtocolWave2Duration);
    m.def("setFastReferencePulseProtocolWave2Period", &er4CommLib::setFastReferencePulseProtocolWave2Period);
    m.def("setFastReferencePulseProtocolWave2PulseNumber", &er4CommLib::setFastReferencePulseProtocolWave2PulseNumber);
    m.def("setCustomFlag", &er4CommLib::setCustomFlag);
    m.def("setCustomDouble", &er4CommLib::setCustomDouble);
    m.def("resetWasherError", &er4CommLib::resetWasherError);
    m.def("setWasherPresetSpeeds", &er4CommLib::setWasherPresetSpeeds);
    m.def("startWasher", &er4CommLib::startWasher);
    m.def("updateWasherState", &er4CommLib::updateWasherState);
    m.def("updateWasherPresetSpeeds", &er4CommLib::updateWasherPresetSpeeds);
    m.def("setCompensationsChannel", &er4CommLib::setCompensationsChannel);
    m.def("turnCFastCompensationOn", &er4CommLib::turnCFastCompensationOn);
    m.def("setCFastCompensationOptions", &er4CommLib::setCFastCompensationOptions);
    m.def("setCFastCapacitance", &er4CommLib::setCFastCapacitance);
    m.def("setTtlPulseTrain", &er4CommLib::setTtlPulseTrain);
    m.def("startTtlPulseTrain", &er4CommLib::startTtlPulseTrain);
    m.def("setDebugBit", &er4CommLib::setDebugBit);
    m.def("setDebugByte", &er4CommLib::setDebugByte);
    m.def("getDeviceInfo", [](){
        uint8_t deviceVersion, deviceSubversion;
        uint32_t firmwareVersion;
        auto ret = er4CommLib::getDeviceInfo(deviceVersion, deviceSubversion, firmwareVersion);
        return std::make_tuple(ret, deviceVersion, deviceSubversion, firmwareVersion);
    });
    m.def("getQueueStatus", [](){
        er4CommLib::QueueStatus_t status;
        auto ret = er4CommLib::getQueueStatus(status);
        return std::make_tuple(ret, status);
    });
    m.def("purgeData", &er4CommLib::purgeData);
    m.def("getChannelsNumber", [](){
        uint32_t voltageChannelsNumber, currentChannelsNumber, gpChannelsNumber;
        auto ret = er4CommLib::getChannelsNumber(voltageChannelsNumber, currentChannelsNumber, gpChannelsNumber);
        return std::make_tuple(ret, voltageChannelsNumber, currentChannelsNumber, gpChannelsNumber);
    });
    m.def("getCurrentRanges", [](){
        std::vector <er4CommLib::RangedMeasurement_t> currentRanges;
        std::vector <uint16_t> defaultOptions;
        auto ret = er4CommLib::getCurrentRanges(currentRanges, defaultOptions);
        return std::make_tuple(ret, currentRanges, defaultOptions);
    });
    m.def("getCurrentRange", [](uint16_t channelIdx){
        er4CommLib::RangedMeasurement_t currentRange;
        auto ret = er4CommLib::getCurrentRange(currentRange, channelIdx);
        return std::make_tuple(ret, currentRange);
    });
    m.def("hasIndependentCurrentRanges", &er4CommLib::hasIndependentCurrentRanges);
    m.def("getGpRanges", [](){
        std::vector <std::vector <er4CommLib::RangedMeasurement_t>> gpRanges;
        std::vector <uint16_t> defaultOptions;
        std::vector <std::string> names;
        auto ret = er4CommLib::getGpRanges(gpRanges, defaultOptions, names);
        return std::make_tuple(ret, gpRanges, defaultOptions, names);
    });
    m.def("getGpRange", [](uint16_t channelIdx){
        er4CommLib::RangedMeasurement_t gpRange;
        auto ret = er4CommLib::getGpRange(gpRange, channelIdx);
        return std::make_tuple(ret, gpRange, channelIdx);
    });
    m.def("getVoltageRanges", [](){
        std::vector <er4CommLib::RangedMeasurement_t> voltageRanges;
        uint16_t defaultOption;
        std::vector <std::string> extensions;
        auto ret = er4CommLib::getVoltageRanges(voltageRanges, defaultOption, extensions);
        return std::make_tuple(ret, voltageRanges, defaultOption, extensions);
    });
    m.def("getVoltageRange", [](){
        er4CommLib::RangedMeasurement_t voltageRange;
        auto ret = er4CommLib::getVoltageRange(voltageRange);
        return std::make_tuple(ret, voltageRange);
    });
    m.def("getVoltageReferenceRanges", [](){
        std::vector <er4CommLib::RangedMeasurement_t> ranges;
        uint16_t defaultOption;
        auto ret = er4CommLib::getVoltageReferenceRanges(ranges, defaultOption);
        return std::make_tuple(ret, ranges, defaultOption);
    });
    m.def("getVoltageReferenceRange", [](){
        er4CommLib::RangedMeasurement_t range;
        auto ret = er4CommLib::getVoltageReferenceRange(range);
        return std::make_tuple(ret, range);
    });
    m.def("getSamplingRates", [](){
        std::vector <er4CommLib::Measurement_t> samplingRates;
        uint16_t defaultOption;
        auto ret = er4CommLib::getSamplingRates(samplingRates, defaultOption);
        return std::make_tuple(ret, samplingRates, defaultOption);
    });
    m.def("getSamplingRate", [](){
        er4CommLib::Measurement_t samplingRate;
        auto ret = er4CommLib::getSamplingRate(samplingRate);
        return std::make_tuple(ret, samplingRate);
    });
    m.def("getRealSamplingRates", [](){
        std::vector <er4CommLib::Measurement_t> samplingRates;
        auto ret = er4CommLib::getRealSamplingRates(samplingRates);
        return std::make_tuple(ret, samplingRates);
    });
    m.def("getRealSamplingRate", [](){
        er4CommLib::Measurement_t samplingRate;
        auto ret = er4CommLib::getSamplingRate(samplingRate);
        return std::make_tuple(ret, samplingRate);
    });
    m.def("getOversamplingRatios", [](){
        std::vector <uint16_t> oversamplingRatios;
        auto ret = er4CommLib::getOversamplingRatios(oversamplingRatios);
        return std::make_tuple(ret, oversamplingRatios);
    });
    m.def("getOversamplingRatio", [](){
        uint16_t oversamplingRatio;
        auto ret = er4CommLib::getOversamplingRatio(oversamplingRatio);
        return std::make_tuple(ret, oversamplingRatio);
    });
    m.def("getVoltageStimulusLpfs", [](){
        uint16_t defaultOption;
        int16_t voltageRangeId;
        std::vector <er4CommLib::Measurement_t> filterOptions;
        auto ret = er4CommLib::getVoltageStimulusLpfs(filterOptions, defaultOption, voltageRangeId);
        return std::make_tuple(ret, filterOptions, defaultOption, voltageRangeId);
    });
    m.def("getVoltageReferenceLpfs", [](){
        uint16_t defaultOption;
        int16_t voltageRangeId;
        std::vector <er4CommLib::Measurement_t> filterOptions;
        auto ret = er4CommLib::getVoltageReferenceLpfs(filterOptions, defaultOption, voltageRangeId);
        return std::make_tuple(ret, filterOptions, defaultOption, voltageRangeId);
    });
    m.def("hasSelectStimulusChannel", [](){
        bool selectStimulusChannelFlag, singleChannelSSCFlag;
        auto ret = er4CommLib::hasSelectStimulusChannel(selectStimulusChannelFlag, singleChannelSSCFlag);
        return std::make_tuple(ret, selectStimulusChannelFlag, singleChannelSSCFlag);
    });
    m.def("hasDigitalOffsetCompensation", [](){
        bool digitalOffsetCompensationFlag, singleChannelDOCFlag, selectableDOCAutostopFlag;
        auto ret = er4CommLib::hasDigitalOffsetCompensation(digitalOffsetCompensationFlag, singleChannelDOCFlag, selectableDOCAutostopFlag);
        return std::make_tuple(ret, digitalOffsetCompensationFlag, singleChannelDOCFlag, selectableDOCAutostopFlag);
    });
    m.def("hasZap", [](){
        bool zappableDeviceFlag, singleChannelZapFlag;
        auto ret = er4CommLib::hasZap(zappableDeviceFlag, singleChannelZapFlag);
        return std::make_tuple(ret, zappableDeviceFlag, singleChannelZapFlag);
    });
    m.def("hasChannelOn", [](){
        bool channelOnFlag, singleChannelOnFlag;
        auto ret = er4CommLib::hasChannelOn(channelOnFlag, singleChannelOnFlag);
        return std::make_tuple(ret, channelOnFlag, singleChannelOnFlag);
    });
    m.def("getSwitchedOnChannels", [](){
        uint32_t channelsMask;
        auto ret = er4CommLib::getSwitchedOnChannels(channelsMask);
        return std::make_tuple(ret, channelsMask);
    });
    m.def("hasDigitalOffsetCompensationReset", &er4CommLib::hasDigitalOffsetCompensationReset);
    m.def("hasDigitalOutput", &er4CommLib::hasDigitalOutput);
    m.def("hasFrontEndResetDenoiser", &er4CommLib::hasFrontEndResetDenoiser);
    m.def("getProtocolList", [](){
        std::vector <std::string> names, images;
        std::vector <std::vector <uint16_t>> voltages, times, slopes, frequencies, adimensionals;
        auto ret = er4CommLib::getProtocolList(names, images, voltages, times, slopes, frequencies, adimensionals);
        return std::make_tuple(ret, names, images, voltages, times, slopes, frequencies, adimensionals);
    });
    m.def("getTriangularProtocolIdx", [](){
        uint16_t idx;
        auto ret = er4CommLib::getTriangularProtocolIdx(idx);
        return std::make_tuple(ret, idx);
    });
    m.def("getSealTestProtocolIdx", [](){
        uint16_t idx;
        auto ret = er4CommLib::getSealTestProtocolIdx(idx);
        return std::make_tuple(ret, idx);
    });
    m.def("getProtocolVoltage",  [](){
        std::vector <std::string> voltageNames;
        std::vector <er4CommLib::RangedMeasurement_t> ranges;
        std::vector <er4CommLib::Measurement_t> defaultValues;
        auto ret = er4CommLib::getProtocolVoltage(voltageNames, ranges, defaultValues);
        return std::make_tuple(ret, voltageNames, ranges, defaultValues);
    });
    m.def("getProtocolTime", [](){
        std::vector <std::string> timeNames;
        std::vector <er4CommLib::RangedMeasurement_t> ranges;
        std::vector <er4CommLib::Measurement_t> defaultValues;
        auto ret = er4CommLib::getProtocolTime(timeNames, ranges, defaultValues);
        return std::make_tuple(ret, timeNames, ranges, defaultValues);
    });
    m.def("getProtocolSlope", [](){
        std::vector <std::string> slopeNames;
        std::vector <er4CommLib::RangedMeasurement_t> ranges;
        std::vector <er4CommLib::Measurement_t> defaultValues;
        auto ret = er4CommLib::getProtocolSlope(slopeNames, ranges, defaultValues);
        return std::make_tuple(ret, slopeNames, ranges, defaultValues);
    });
    m.def("getProtocolFrequency", [](){
        std::vector <std::string> frequencyNames;
        std::vector <er4CommLib::RangedMeasurement_t> ranges;
        std::vector <er4CommLib::Measurement_t> defaultValues;
        auto ret = er4CommLib::getProtocolFrequency(frequencyNames, ranges, defaultValues);
        return std::make_tuple(ret, frequencyNames, ranges, defaultValues);
    });
    m.def("getProtocolAdimensional", [](){
        std::vector <std::string> adimensionalNames;
        std::vector <er4CommLib::RangedMeasurement_t> ranges;
        std::vector <er4CommLib::Measurement_t> defaultValues;
        auto ret = er4CommLib::getProtocolAdimensional(adimensionalNames, ranges, defaultValues);
        return std::make_tuple(ret, adimensionalNames, ranges, defaultValues);
    });
    m.def("getVoltageOffsetControls", [](){
        er4CommLib::RangedMeasurement_t voltageRange;
        auto ret = er4CommLib::getVoltageOffsetControls(voltageRange);
        return std::make_tuple(ret, voltageRange);
    });
    m.def("getInsertionPulseControls", [](){
        er4CommLib::RangedMeasurement_t voltageRange, durationRange;
        auto ret = er4CommLib::getInsertionPulseControls(voltageRange, durationRange);
        return std::make_tuple(ret, voltageRange);
    });
    m.def("hasReferencePulseControls", [](){
        bool referencePulseImplemented, overrideReferencePulseImplemented;
        auto ret = er4CommLib::hasReferencePulseControls(referencePulseImplemented, overrideReferencePulseImplemented);
        return std::make_tuple(ret, referencePulseImplemented, overrideReferencePulseImplemented);
    });
    m.def("getReferencePulseControls", [](){
        er4CommLib::RangedMeasurement_t voltageRange, durationRange;
        auto ret = er4CommLib::getReferencePulseControls(voltageRange, durationRange);
        return std::make_tuple(ret, voltageRange, durationRange);
    });
    m.def("hasReferencePulseTrainControls", [](){
        bool referencePulseImplemented, overrideReferencePulseImplemented;
        auto ret = er4CommLib::hasReferencePulseTrainControls(referencePulseImplemented, overrideReferencePulseImplemented);
        return std::make_tuple(ret, referencePulseImplemented, overrideReferencePulseImplemented);
    });
    m.def("getReferencePulseTrainControls", [](){
        er4CommLib::RangedMeasurement_t voltageRange, durationRange, periodRange;
        uint16_t pulsesNumber;
        auto ret = er4CommLib::getReferencePulseTrainControls(voltageRange, durationRange, periodRange, pulsesNumber);
        return std::make_tuple(ret, voltageRange, durationRange, periodRange, pulsesNumber);
    });
    m.def("getEdhFormat", [](){
        std::string format;
        auto ret = er4CommLib::getEdhFormat(format);
        return std::make_tuple(ret, format);
    });
    m.def("getRawDataFilterCutoffFrequency", [](){
        er4CommLib::RangedMeasurement_t range;
        er4CommLib::Measurement_t defaultValue;
        auto ret = er4CommLib::getRawDataFilterCutoffFrequency(range, defaultValue);
        return std::make_tuple(ret, range, defaultValue);
    });
    m.def("getLedsNumber", [](){
        uint16_t ledsNumber;
        auto ret = er4CommLib::getLedsNumber(ledsNumber);
        return std::make_tuple(ret, ledsNumber);
    });
    m.def("getLedsColors", [](){
        std::vector <uint32_t> ledsColors;
        auto ret = er4CommLib::getLedsColors(ledsColors);
        return std::make_tuple(ret, ledsColors);
    });
    m.def("getFastReferencePulseProtocolWave1Range",  [](){
        er4CommLib::RangedMeasurement_t voltageRange, timeRange;
        uint16_t nPulse;
        auto ret = er4CommLib::getFastReferencePulseProtocolWave1Range(voltageRange, timeRange, nPulse);
        return std::make_tuple(ret, voltageRange, timeRange, nPulse);
    });
    m.def("getFastReferencePulseProtocolWave2Range", [](){
        er4CommLib::RangedMeasurement_t voltageRange, timeRange, durationRange;
        uint16_t nPulse;
        auto ret = er4CommLib::getFastReferencePulseProtocolWave2Range(voltageRange, timeRange, durationRange, nPulse);
        return std::make_tuple(ret, voltageRange, timeRange, durationRange, nPulse);
    });
    m.def("getFastReferencePulseTrainProtocolWave2Range", [](){
        er4CommLib::RangedMeasurement_t voltageRange, timeRange, durationRange, waitTimeRange;
        uint16_t pulsesPerTrain, nPulse;
        auto ret = er4CommLib::getFastReferencePulseTrainProtocolWave2Range(voltageRange, timeRange, durationRange, waitTimeRange, pulsesPerTrain, nPulse);
        return std::make_tuple(ret, voltageRange, timeRange, durationRange, waitTimeRange, pulsesPerTrain, nPulse);
    });
    m.def("getCalibrationEepromSize", [](){
        uint32_t size;
        auto ret = er4CommLib::getCalibrationEepromSize(size);
        return std::make_tuple(ret, size);
    });
    m.def("writeCalibrationEeprom", &er4CommLib::writeCalibrationEeprom);
    m.def("readCalibrationEeprom", [](std::vector <uint32_t> address,  std::vector <uint32_t> size){
    std::vector <uint32_t> value;
    auto ret = er4CommLib::readCalibrationEeprom(value, address, size);
        return std::make_tuple(ret, value, address, size);
    });
    m.def("getCustomFlags", [](){
        std::vector <std::string> customFlags;
        std::vector <bool> customFlagsDefault;
        auto ret = er4CommLib::getCustomFlags(customFlags, customFlagsDefault);
            return std::make_tuple(ret, customFlags, customFlagsDefault);
        });
    m.def("getCustomDoubles", [](){
        std::vector <std::string> customDoubles;
        std::vector <er4CommLib::RangedMeasurement_t> customDoublesRanges;
        std::vector <double> customDoublesDefault;
        auto ret = er4CommLib::getCustomDoubles(customDoubles, customDoublesRanges, customDoublesDefault);
            return std::make_tuple(ret, customDoubles, customDoublesRanges, customDoublesDefault);
        });
    m.def("hasNanionTemperatureController", &er4CommLib::hasNanionTemperatureController);
    m.def("getTemperatureControllerRange", [](){
        int minTemperature, maxTemperature;
        auto ret = er4CommLib::getTemperatureControllerRange(minTemperature, maxTemperature);
            return std::make_tuple(ret, minTemperature, maxTemperature);
        });
    m.def("hasWasherControls", &er4CommLib::hasWasherControls);
    m.def("getWasherSpeedRange", [](){
        er4CommLib::RangedMeasurement_t range;
        auto ret = er4CommLib::getWasherSpeedRange(range);
            return std::make_tuple(ret, range);
        });
    m.def("getWasherStatus", [](){
        er4CommLib::WasherStatus_t status;
        er4CommLib::WasherError_t error;
        auto ret = er4CommLib::getWasherStatus(status, error);
            return std::make_tuple(ret, status, error);
        });
    m.def("getWasherPresetSpeeds", [](){
        std::vector <int8_t> speedValue;
        auto ret = er4CommLib::getWasherPresetSpeeds(speedValue);
            return std::make_tuple(ret, speedValue);
        });
    m.def("hasCFastCompensation", &er4CommLib::hasCFastCompensation);
    m.def("getCFastCompensationOptions", [](){
        std::vector <std::string> options;
        auto ret = er4CommLib::getCFastCompensationOptions(options);
            return std::make_tuple(ret, options);
        });
    m.def("getCFastCapacitanceControl", [](){
        er4CommLib::CompensationControl_t control;
        auto ret = er4CommLib::getCFastCapacitanceControl(control);
            return std::make_tuple(ret, control);
        });
    m.def("hasTtlPulseTrain", &er4CommLib::hasTtlPulseTrain);
    m.def("getVoltageOffsetCompensations", [](){
        std::vector <er4CommLib::Measurement_t> offsets;
        auto ret = er4CommLib::getVoltageOffsetCompensations(offsets);
            return std::make_tuple(ret, offsets);
        });

    py::class_<I16Buffer>(m, "I16Buffer", py::buffer_protocol())
            .def_buffer(&I16Buffer::get_buffer);

    py::class_<U16Buffer>(m, "U16Buffer", py::buffer_protocol())
            .def_buffer(&U16Buffer::get_buffer);

    py::class_<er4CommLib::Measurement_t>(m, "Measurement")
            .def(py::init<double, er4CommLib::UnitPfx_t, std::string>())
            .def_readonly("value", &er4CommLib::Measurement_t::value)
            .def_readonly("prefix", &er4CommLib::Measurement_t::prefix)
            .def_readonly("unit", &er4CommLib::Measurement_t::unit)
            .def("getNoPrefixValue", &er4CommLib::Measurement_t::getNoPrefixValue)
            .def("getPrefix", &er4CommLib::Measurement_t::getPrefix)
            .def("getFullUnit", &er4CommLib::Measurement_t::getFullUnit)
            .def("multiplier", &er4CommLib::Measurement_t::multiplier)
            .def("label", &er4CommLib::Measurement_t::label)
            .def("niceLabel", &er4CommLib::Measurement_t::niceLabel)
            .def("convertValue", [=](er4CommLib::Measurement_t &self, double newMultiplier) {
                self.convertValue(newMultiplier);
    })
            .def("convertValues", [=](er4CommLib::Measurement_t &self, er4CommLib::UnitPfx_t newPrefix) {
                self.convertValue(newPrefix);
    })
            .def("nice", &er4CommLib::Measurement_t::nice);

    py::class_<er4CommLib::RangedMeasurement_t>(m, "RangedMeasurement")
            .def(py::init<double, double, double, er4CommLib::UnitPfx_t, std::string>())
            .def_readonly("min", &er4CommLib::RangedMeasurement_t::min)
            .def_readonly("max", &er4CommLib::RangedMeasurement_t::max)
            .def_readonly("step", &er4CommLib::RangedMeasurement_t::step)
            .def_readonly("prefix", &er4CommLib::RangedMeasurement_t::prefix)
            .def_readonly("unit", &er4CommLib::RangedMeasurement_t::unit)
            .def("steps", [=](er4CommLib::RangedMeasurement_t &self) {
                return self.steps();
    })
            .def("multiplier", [=](er4CommLib::RangedMeasurement_t &self) {
                return self.multiplier();
    })
            .def("getPrefix", &er4CommLib::RangedMeasurement_t::getPrefix)
            .def("getFullUnit", &er4CommLib::RangedMeasurement_t::getFullUnit)
            .def("valueLabel", &er4CommLib::RangedMeasurement_t::valueLabel)
            .def("label", &er4CommLib::RangedMeasurement_t::label)
            .def("convertValues", py::overload_cast<er4CommLib::UnitPfx_t>(&er4CommLib::RangedMeasurement_t::convertValues))
            .def("convertValues", py::overload_cast<double>(&er4CommLib::RangedMeasurement_t::convertValues))
            .def("delta", &er4CommLib::RangedMeasurement_t::delta)
            .def("decimals", &er4CommLib::RangedMeasurement_t::decimals)
            .def("getMax", &er4CommLib::RangedMeasurement_t::getMax)
            .def("getMin", &er4CommLib::RangedMeasurement_t::getMin)
            .def("getClosestIndex", &er4CommLib::RangedMeasurement_t::getClosestIndex)
            .def("niceLabel", &er4CommLib::RangedMeasurement_t::niceLabel)
            .def("includes", &er4CommLib::RangedMeasurement_t::includes);

    py::enum_<er4CommLib::UnitPfx_t>(m, "UnitPfx")
            .value("UnitPfxFemto",      er4CommLib::UnitPfxFemto)
            .value("UnitPfxPico",       er4CommLib::UnitPfxPico)
            .value("UnitPfxNano",       er4CommLib::UnitPfxNano)
            .value("UnitPfxMicro",      er4CommLib::UnitPfxMicro)
            .value("UnitPfxMilli",      er4CommLib::UnitPfxMilli)
            .value("UnitPfxNone",       er4CommLib::UnitPfxNone)
            .value("UnitPfxKilo",       er4CommLib::UnitPfxKilo)
            .value("UnitPfxMega",       er4CommLib::UnitPfxMega)
            .value("UnitPfxGiga",       er4CommLib::UnitPfxGiga)
            .value("UnitPfxTera",       er4CommLib::UnitPfxTera)
            .value("UnitPfxPeta",       er4CommLib::UnitPfxPeta)
            .export_values();

    py::enum_<er4CommLib::ErrorCodes_t>(m, "ErrorCodes")
            .value("Success",                               er4CommLib::Success)
            .value("ErrorNoDeviceFound",                    er4CommLib::ErrorNoDeviceFound)
            .value("ErrorListDeviceFailed",                 er4CommLib::ErrorListDeviceFailed)

            .value("ErrorEepromAlreadyConnected",           er4CommLib::ErrorEepromAlreadyConnected)
            .value("ErrorEepromConnectionFailed",           er4CommLib::ErrorEepromConnectionFailed)
            .value("ErrorEepromDisconnectionFailed",        er4CommLib::ErrorEepromDisconnectionFailed)
            .value("ErrorEepromNotConnected",               er4CommLib::ErrorEepromNotConnected)
            .value("ErrorEepromReadFailed",                 er4CommLib::ErrorEepromReadFailed)
            .value("ErrorEepromWriteFailed",                er4CommLib::ErrorEepromWriteFailed)
            .value("ErrorEepromNotRecognized",              er4CommLib::ErrorEepromNotRecognized)
            .value("ErrorEepromInvalidAddress",             er4CommLib::ErrorEepromInvalidAddress)

            .value("ErrorInitializationFailed",             er4CommLib::ErrorInitializationFailed)
            .value("ErrorDeviceTypeNotRecognized",          er4CommLib::ErrorDeviceTypeNotRecognized)
            .value("ErrorDeviceAlreadyConnected",           er4CommLib::ErrorDeviceAlreadyConnected)
            .value("ErrorDeviceNotConnected",               er4CommLib::ErrorDeviceNotConnected)
            .value("ErrorDeviceConnectionFailed",           er4CommLib::ErrorDeviceConnectionFailed)
            .value("ErrorFtdiConfigurationFailed",          er4CommLib::ErrorFtdiConfigurationFailed)
            .value("ErrorDeviceDisconnectionFailed",        er4CommLib::ErrorDeviceDisconnectionFailed)
            .value("ErrorDeviceCommunicationFailed",        er4CommLib::ErrorDeviceCommunicationFailed)
            .value("ErrorConnectingDifferentDevices",       er4CommLib::ErrorConnectingDifferentDevices)

            .value("ErrorSendMessageFailed",                er4CommLib::ErrorSendMessageFailed)
            .value("ErrorCommandNotImplemented",            er4CommLib::ErrorCommandNotImplemented)
            .value("ErrorValueOutOfRange",                  er4CommLib::ErrorValueOutOfRange)

            .value("WarningNoDataAvailable",                er4CommLib::WarningNoDataAvailable)
            .value("WarningNotEnoughDataAvailable",         er4CommLib::WarningNotEnoughDataAvailable)
            .value("ErrorInvalidProtocolParameters",        er4CommLib::ErrorInvalidProtocolParameters)

            .value("ErrorFeatureNotImplemented",            er4CommLib::ErrorFeatureNotImplemented)
            .value("ErrorUpgradesNotAvailable",             er4CommLib::ErrorUpgradesNotAvailable)

            .value("ErrorExpiredDevice",                    er4CommLib::ErrorExpiredDevice)

            .value("ErrorUnknown",                          er4CommLib::ErrorUnknown)
            .export_values();

    py::enum_<er4CommLib::WasherStatus_t>(m, "WasherStatus")
            .value("WasherIdle",        er4CommLib::WasherIdle)
            .value("WasherUnsaved",     er4CommLib::WasherUnsaved)
            .value("WasherSaving",      er4CommLib::WasherSaving)
            .export_values();


    py::enum_<er4CommLib::WasherError_t>(m, "WasherError")
            .value("WasherOk",                  er4CommLib::WasherOk)
            .value("WasherTimeout",             er4CommLib::WasherTimeout)
            .value("WasherPower",               er4CommLib::WasherPower)
            .value("WasherCommunication",       er4CommLib::WasherCommunication)
            .value("WasherExecution",           er4CommLib::WasherExecution)
            .value("WasherOverload",            er4CommLib::WasherOverload)
            .value("WasherChecksumError",       er4CommLib::WasherChecksumError)
            .value("WasherIllFormedMessage",    er4CommLib::WasherIllFormedMessage)
            .export_values();

    py::class_<er4CommLib::QueueStatus_t>(m, "QueueStatus")
            .def(py::init<unsigned int, bool,bool,bool,bool,bool,bool>())
            .def_readonly("availableDataPackets",      &er4CommLib::QueueStatus_t::availableDataPackets)
            .def_readonly("bufferOverflowFlag",        &er4CommLib::QueueStatus_t::bufferOverflowFlag)
            .def_readonly("lostDataFlag",              &er4CommLib::QueueStatus_t::lostDataFlag)
            .def_readonly("saturationFlag",            &er4CommLib::QueueStatus_t::saturationFlag)
            .def_readonly("currentRangeIncreaseFlag",  &er4CommLib::QueueStatus_t::currentRangeIncreaseFlag)
            .def_readonly("currentRangeDecreaseFlag",  &er4CommLib::QueueStatus_t::currentRangeDecreaseFlag)
            .def_readonly("communicationErrorFlag",    &er4CommLib::QueueStatus_t::communicationErrorFlag);
}
