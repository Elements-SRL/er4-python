#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include "messagedispatcher.h"

namespace py = pybind11;
using namespace er4CommLib;

static int mdCounter = 0;
static std::vector <MessageDispatcher *> mds;

PYBIND11_MODULE(er4CommLib_python, m) {
    m.doc() = "Typical order of calls\n"
              "detectDevices();\n"
              "connect();\n"
              "<use the device>\n"
              "disconnect();";

    m.def("detectDevices", []() {
        std::vector <std::string> deviceIds;
        ErrorCode err = MessageDispatcher::detectDevices(deviceIds);
        return std::make_tuple(err, deviceIds);
    }, "Detect plugged in devices");

    m.def("connect", [](std::string deviceId) {
        MessageDispatcher * md = nullptr;
        ErrorCode err = MessageDispatcher::connectDevice(deviceId, md);
        if (md == nullptr) {
            return std::make_tuple(err, -1);

        } else {
            mds.push_back(md);
            return std::make_tuple(err, mdCounter++);
        }
    }, "Connect to device <arg0>");

    m.def("disconnect", [](int mdIdx) {
        ErrorCode err = mds[mdIdx]->disconnectDevice();
        delete mds[mdIdx];
        mds[mdIdx] = nullptr;
        return err;
    }, py::arg() = 0, "Disconnect from device");

    m.def("setVoltageOffset", [](unsigned int chIdx, Measurement_t meas, int mdIdx) {
        return mds[mdIdx]->setVoltageOffset(chIdx, meas);
    }, py::arg(), py::arg(), py::arg() = 0, "Set an offset voltage on the input pin of the selected channel");

    m.def("applyDacExt", [](Measurement_t meas, int mdIdx) {
        return mds[mdIdx]->applyDacExt(meas);
    }, py::arg(), py::arg() = 0, "Set the voltage on the reference pin");

    m.def("setCurrentRange", [](uint16_t rangeIdx, uint16_t channelIdx, int mdIdx) {
        return mds[mdIdx]->setCurrentRange(rangeIdx, channelIdx);
    }, py::arg(), py::arg(), py::arg() = 0, "Set the selected current range for the selected channel");

    m.def("setVoltageRange", [](uint16_t rangeIdx, int mdIdx) {
        return mds[mdIdx]->setVoltageRange(rangeIdx);
    }, py::arg(), py::arg() = 0, "Set the selected voltage range for the input pins stimula");

    m.def("setVoltageReferenceRange", [](uint16_t rangeIdx, int mdIdx) {
        return mds[mdIdx]->setVoltageReferenceRange(rangeIdx);
    }, py::arg(), py::arg() = 0, "Set the selected voltage range for the reference pin stimulus");

    m.def("setVoltageReferenceLpf", [](uint16_t filterIdx, int mdIdx) {
        return mds[mdIdx]->setVoltageReferenceLpf(filterIdx, true);
    }, py::arg(), py::arg() = 0, "Set the selected voltage filter for the reference pin stimulus");

    m.def("setSamplingRate", [](uint16_t srIdx, int mdIdx) {
        return mds[mdIdx]->setSamplingRate(srIdx);
    }, py::arg(), py::arg() = 0, "Set the selected sampling rate for the current readout");

    m.def("getQueueStatus", [](int mdIdx) {
        QueueStatus_t status;
        ErrorCode err = mds[mdIdx]->getQueueStatus(status);
        return std::make_tuple(err, status);
    }, py::arg() = 0, "Get the status of the received data buffer");

    m.def("purgeData", [](int mdIdx) {
        return mds[mdIdx]->purgeData();
    }, py::arg() = 0, "Purge the received data buffer from old samples");

    m.def("writeCalibrationEeprom", [](std::vector <uint32_t> value, std::vector <uint32_t> address, std::vector <uint32_t> size, int mdIdx) {
        return mds[mdIdx]->writeCalibrationEeprom(value, address, size);
    }, py::arg(), py::arg(), py::arg(), py::arg() = 0, "Purge the received data buffer from old samples");

    m.def("setVCVoltageOffset", [](uint32_t chIdx, uint32_t value, int mdIdx) {
        std::vector<RangedMeasurement_t> currentRanges;
        std::vector<uint16_t> defaultOptions;
        auto err = mds[mdIdx]->getCurrentRanges(currentRanges, defaultOptions);
        if (err != ErrorCode::Success) {
            return err;
        }
        uint32_t vChNum;
        uint32_t cChNum;
        mds[mdIdx]->getChannelsNumber(vChNum, cChNum);
        if(value < 0 || value > 1023) {
            return ErrorValueOutOfRange;
        }
//        dynamically get the number of modalities.
//        2 rows for each channel, one for slow, one for fast, 2 cols for each current range, one for gain, one for offset, nChannel of
        auto addr = (cChNum*2)*(currentRanges.size()*2) + (chIdx*4 + 2);
        printf("%llu\n", addr);
//        return mds[mdIdx]->writeCalibrationEeprom({value}, {addr}, {2});
        return ErrorCode::Success;
    }, py::arg(), py::arg(), py::arg() = 0, "Purge the received data buffer from old samples");

    m.def("setCurrentGain", [](uint32_t chIdx, uint32_t value, int mdIdx) {
        std::vector<RangedMeasurement_t> currentRanges;
        std::vector<uint16_t> defaultOptions;
        auto err = mds[mdIdx]->getCurrentRanges(currentRanges, defaultOptions);
        if (err != ErrorCode::Success) {
            return err;
        }
        uint32_t vChNum;
        uint32_t cChNum;
        mds[mdIdx]->getChannelsNumber(vChNum, cChNum);
        if(value < 0 || value > 1023) {
            return ErrorValueOutOfRange;
        }
//        dynamically get the number of modalities.
//        2 rows for each channel, one for slow, one for fast, 2 cols for each current range, one for gain, one for offset, nChannel of
        auto addr = (cChNum*2)*(currentRanges.size()*2) + (chIdx*4 + 2);
        printf("%llu\n", addr);
//        return mds[mdIdx]->writeCalibrationEeprom({value}, {addr}, {2});
        return ErrorCode::Success;
    }, py::arg(), py::arg(), py::arg() = 0, "Purge the received data buffer from old samples");

    m.def("setCurrentOffset", [](uint32_t chIdx, uint32_t value, int mdIdx) {
        std::vector<RangedMeasurement_t> currentRanges;
        std::vector<uint16_t> defaultOptions;
        auto err = mds[mdIdx]->getCurrentRanges(currentRanges, defaultOptions);
        if (err != ErrorCode::Success) {
            return err;
        }
        uint32_t vChNum;
        uint32_t cChNum;
        mds[mdIdx]->getChannelsNumber(vChNum, cChNum);
        if(value < 0 || value > 1023) {
            return ErrorValueOutOfRange;
        }
//        dynamically get the number of modalities.
//        2 rows for each channel, one for slow, one for fast, 2 cols for each current range, one for gain, one for offset, nChannel of
        auto addr = (cChNum*2)*(currentRanges.size()*2) + (chIdx*4 + 2);
        printf("%llu\n", addr);
//        return mds[mdIdx]->writeCalibrationEeprom({value}, {addr}, {2});
        return ErrorCode::Success;
    }, py::arg(), py::arg(), py::arg() = 0, "Purge the received data buffer from old samples");

    m.def("readData", [](unsigned int dataToRead, int mdIdx) {
        unsigned int dataRead;
        uint16_t * buffer;
        ErrorCode err = mds[mdIdx]->getDataPackets(buffer, dataToRead, dataRead);
        uint32_t vChNum;
        uint32_t cChNum;
        mds[mdIdx]->getChannelsNumber(vChNum, cChNum);
        uint32_t totChNum = vChNum+cChNum;
        std::vector <std::vector <double>> bufferOut(dataRead);
        uint32_t idx = 0;
        for (uint32_t pkIdx = 0; pkIdx < dataRead; pkIdx++) {
            bufferOut[pkIdx].resize(totChNum);
            for (uint32_t chIdx = 0; chIdx < totChNum; chIdx++) {
                bufferOut[pkIdx][chIdx] = buffer[idx++];
            }
        }
        return std::make_tuple(err, dataRead, bufferOut);
    }, py::arg(), py::arg() = 0, "Get the status of the received data buffer");

    m.def("getChannelsNumber", [](int mdIdx) {
        uint32_t vChNum;
        uint32_t cChNum;
        auto err = mds[mdIdx]->getChannelsNumber(vChNum, cChNum);
        return std::make_tuple(err, vChNum, cChNum);
    }, py::arg() = 0, "Get the number of channels");

    m.def("getSamplingRates", [](int mdIdx) {
        std::vector<Measurement_t> samplingRates;
        uint16_t defaultOption;
        auto err = mds[mdIdx]->getSamplingRates(samplingRates, defaultOption);
        return std::make_tuple(err, samplingRates, defaultOption);
    }, py::arg() = 0, "Get the sampling rate for the current device");

    m.def("getSamplingRate", [](int mdIdx) {
        Measurement_t samplingRate;
        auto err = mds[mdIdx]->getSamplingRate(samplingRate);
        return std::make_tuple(err, samplingRate);
    }, py::arg() = 0, "Get the sampling rate for the current device");

    m.def("setSamplingRate", [](uint16_t samplingRateIdx, int mdIdx){
        auto err = mds[mdIdx]->setSamplingRate(samplingRateIdx, true);
        return std::make_tuple(err);
    }, py::arg(), py::arg() = 0, "Set the sampling rate for the current device");

    m.def("getVoltageRanges",[](int mdIdx){
        std::vector<RangedMeasurement_t> voltageRanges;
        uint16_t defaultOption;
        std::vector<string> strings_to_ignore;
        auto err = mds[mdIdx]->getVoltageRanges(voltageRanges, defaultOption, strings_to_ignore);
        return std::make_tuple(err, voltageRanges, defaultOption);
    }, py::arg() = 0, "Get the voltage ranges");

    m.def("getCurrentRanges", [](int mdIdx){
        std::vector<RangedMeasurement_t> currentRanges;
        std::vector<uint16_t> defaultOptions;
        auto err = mds[mdIdx]->getCurrentRanges(currentRanges, defaultOptions);
        return std::make_tuple(err, currentRanges, defaultOptions[0]);
    }, py::arg() = 0, "Get the current ranges");

    m.def("getCurrentRange", [](int mdIdx){
        RangedMeasurement_t currentRange;
//        todo POSSO USARE 0?
        auto err = mds[mdIdx]->getCurrentRange(currentRange, 0);
        return std::make_tuple(err, currentRange);
    }, py::arg() = 0, "Get the current range for channel 0");

    m.def("setCurrentRange", [](uint16_t currentRangeIdx, int mdIdx){
        uint32_t vChNum;
        uint32_t cChNum;
        auto err = mds[mdIdx]->getChannelsNumber(vChNum, cChNum);
        err = mds[mdIdx]->setCurrentRange(currentRangeIdx, cChNum);
        return std::make_tuple(err);
    },  py::arg(), py::arg() = 0, "Set the current range for all channels");

    m.def("convertCurrentValue", [](uint16_t intValue, uint16_t channelIdx, int mdIdx) {
        double fltValue;
        ErrorCode err = mds[mdIdx]->convertCurrentValue(intValue, channelIdx, fltValue);
        return std::make_tuple(err, fltValue);
    }, py::arg(), py::arg(), py::arg() = 0, "Get the status of the received data buffer");

    m.def("applyConstantProtocol", [](Measurement meas, int mdIdx) {
        ErrorCode err = mds[mdIdx]->selectVoltageProtocol(0);
        if (err != ErrorCode::Success) {
            return err;
        }
        err = mds[mdIdx]->setProtocolVoltage(0, meas);
        if (err != ErrorCode::Success) {
            return err;
        }
        return mds[mdIdx]->applyVoltageProtocol();
    }, py::arg(), py::arg() = 0, "Apply a constant voltage");


    py::enum_<ErrorCodes_t>(m, "ErrorCode")
            .value("Success",                           Success)
            .value("ErrorNoDeviceFound",                ErrorNoDeviceFound)
            .value("ErrorListDeviceFailed",             ErrorListDeviceFailed)
            .value("ErrorEepromAlreadyConnected",       ErrorEepromAlreadyConnected)
            .value("ErrorEepromConnectionFailed",       ErrorEepromConnectionFailed)
            .value("ErrorEepromDisconnectionFailed",    ErrorEepromDisconnectionFailed)
            .value("ErrorEepromNotConnected",           ErrorEepromNotConnected)
            .value("ErrorEepromReadFailed",             ErrorEepromReadFailed)
            .value("ErrorEepromNotRecognized",          ErrorEepromNotRecognized)
            .value("ErrorInitializationFailed",         ErrorInitializationFailed)
            .value("ErrorDeviceTypeNotRecognized",      ErrorDeviceTypeNotRecognized)
            .value("ErrorDeviceAlreadyConnected",       ErrorDeviceAlreadyConnected)
            .value("ErrorDeviceNotConnected",           ErrorDeviceNotConnected)
            .value("ErrorDeviceConnectionFailed",       ErrorDeviceConnectionFailed)
            .value("ErrorFtdiConfigurationFailed",      ErrorFtdiConfigurationFailed)
            .value("ErrorDeviceDisconnectionFailed",    ErrorDeviceDisconnectionFailed)
            .value("ErrorDeviceCommunicationFailed",    ErrorDeviceCommunicationFailed)
            .value("ErrorSendMessageFailed",            ErrorSendMessageFailed)
            .value("ErrorCommandNotImplemented",        ErrorCommandNotImplemented)
            .value("ErrorValueOutOfRange",              ErrorValueOutOfRange)
            .value("WarningNoDataAvailable",            WarningNoDataAvailable)
            .value("WarningNotEnoughDataAvailable",     WarningNotEnoughDataAvailable)
            .value("ErrorInvalidProtocolParameters",    ErrorInvalidProtocolParameters)
            .value("ErrorFeatureNotImplemented",        ErrorFeatureNotImplemented)
            .value("ErrorUpgradesNotAvailable",         ErrorUpgradesNotAvailable)
            .value("ErrorExpiredDevice",                ErrorExpiredDevice)
            .value("ErrorUnknown",                      ErrorUnknown)
            .export_values();

    py::enum_<UnitPfx_t>(m, "UnitPfx")
            .value("UnitPfxFemto",      UnitPfxFemto)
            .value("UnitPfxPico",       UnitPfxPico)
            .value("UnitPfxNano",       UnitPfxNano)
            .value("UnitPfxMicro",      UnitPfxMicro)
            .value("UnitPfxMilli",      UnitPfxMilli)
            .value("UnitPfxNone",       UnitPfxNone)
            .value("UnitPfxKilo",       UnitPfxKilo)
            .value("UnitPfxMega",       UnitPfxMega)
            .value("UnitPfxGiga",       UnitPfxGiga)
            .value("UnitPfxTera",       UnitPfxTera)
            .value("UnitPfxPeta",       UnitPfxPeta)
            .export_values();

    py::class_<Measurement_t>(m, "Measurement")
            .def(py::init<double, UnitPfx_t, std::string>())
            .def_readonly("value", &Measurement_t::value)
            .def_readonly("prefix", &Measurement_t::prefix)
            .def_readonly("unit", &Measurement_t::unit);

    py::class_<QueueStatus_t>(m, "QueueStatus")
            .def(py::init<>())
            .def_readonly("availableDataPackets", &QueueStatus_t::availableDataPackets)
            .def_readonly("bufferOverflowFlag", &QueueStatus_t::bufferOverflowFlag)
            .def_readonly("lostDataFlag", &QueueStatus_t::lostDataFlag)
            .def_readonly("saturationFlag", &QueueStatus_t::saturationFlag)
            .def_readonly("currentRangeIncreaseFlag", &QueueStatus_t::currentRangeIncreaseFlag)
            .def_readonly("currentRangeDecreaseFlag", &QueueStatus_t::currentRangeDecreaseFlag)
            .def_readonly("communicationErrorFlag", &QueueStatus_t::communicationErrorFlag);

    py::class_<RangedMeasurement_t>(m, "RangedMeasurement")
            .def(py::init<double, double, double, UnitPfx_t, std::string>())
            .def_readonly("min", &RangedMeasurement_t::min)
            .def_readonly("max", &RangedMeasurement_t::max)
            .def_readonly("step", &RangedMeasurement_t::step)
            .def_readonly("prefix", &RangedMeasurement_t::prefix)
            .def_readonly("unit", &RangedMeasurement_t::unit);
}
