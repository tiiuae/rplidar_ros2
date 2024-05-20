#include "hal/abs_rxtx.h"
#include "hal/event.h"
#include "hal/locker.h"
#include "hal/socket.h"
#include "hal/thread.h"
#include "hal/types.h"
#include "rplidar_cmd.h"
#include "rplidar_driver.h"
#include "sdkcommon.h"

#include <cstddef>
#include <cstdint>
#include <fuzzer/FuzzedDataProvider.h>
#include <iostream>
#include <iterator>
#include <memory>
#include <ostream>
#include <stddef.h>
#include <stdint.h>
#include <string>
#include <vector>

#define MAX_ALLOC 1024 * 1000

extern "C" int LLVMFuzzerTestOneInput(const uint8_t *data, size_t size) {
  FuzzedDataProvider fdp(data, size);

  auto driverType = fdp.ConsumeBool()
                        ? rp::standalone::rplidar::DRIVER_TYPE_TCP
                        : rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT;
  auto drv = rp::standalone::rplidar::RPlidarDriver::CreateDriver(driverType);

  if (!drv) {
    return 0;
  }

  if (fdp.remaining_bytes() >= 2) {
    if (driverType == rp::standalone::rplidar::DRIVER_TYPE_SERIALPORT) {
      std::string portPath = fdp.ConsumeRandomLengthString(256);
      uint32_t baudrate = fdp.ConsumeIntegral<uint32_t>();
      drv->connect(portPath.c_str(), baudrate);
    } else {
      std::string ip = fdp.ConsumeRandomLengthString(15);
      uint32_t port = fdp.ConsumeIntegral<uint32_t>();
      drv->connect(ip.c_str(), port);
    }
  }

  drv->isConnected();

  if (fdp.ConsumeBool()) {
    drv->reset();
  }

  if (fdp.ConsumeBool()) {
    drv->clearNetSerialRxCache();
  }

  if (fdp.ConsumeBool()) {
    std::vector<rp::standalone::rplidar::RplidarScanMode> outModes;
    drv->getAllSupportedScanModes(outModes);
  }

  if (fdp.ConsumeBool()) {
    uint16_t outMode;
    drv->getTypicalScanMode(outMode);
  }

  if (fdp.ConsumeBool()) {
    bool force = fdp.ConsumeBool();
    bool useTypicalScan = fdp.ConsumeBool();
    rp::standalone::rplidar::RplidarScanMode outUsedScanMode;
    drv->startScan(force, useTypicalScan, 0, &outUsedScanMode);
  }

  if (fdp.ConsumeBool()) {
    bool force = fdp.ConsumeBool();
    uint16_t scanMode = fdp.ConsumeIntegral<uint16_t>();
    rp::standalone::rplidar::RplidarScanMode outUsedScanMode;
    drv->startScanExpress(force, scanMode, 0, &outUsedScanMode);
  }

  if (fdp.ConsumeBool()) {
    rplidar_response_device_health_t health;
    drv->getHealth(health);
  }

  if (fdp.ConsumeBool()) {
    rplidar_response_device_info_t info;
    drv->getDeviceInfo(info);
  }

  if (fdp.ConsumeBool()) {
    // NOTE: DEPRECATED API
    rplidar_response_sample_rate_t rateInfo;
    drv->getSampleDuration_uS(rateInfo);
  }

  if (fdp.ConsumeBool()) {
    uint16_t pwm = fdp.ConsumeIntegral<uint16_t>();
    drv->setMotorPWM(pwm);
  }

  if (fdp.ConsumeBool()) {
    drv->startMotor();
  }

  if (fdp.ConsumeBool()) {
    drv->stopMotor();
  }

  if (fdp.ConsumeBool()) {
    bool support;
    drv->checkMotorCtrlSupport(support);
  }

  if (fdp.ConsumeBool()) {
    // NOTE: DEPRECATED API
    bool inExpressMode = fdp.ConsumeBool();
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    float frequency;
    bool is4kmode;
    drv->getFrequency(inExpressMode, count, frequency, is4kmode);
  }

  if (fdp.ConsumeBool()) {
    rp::standalone::rplidar::RplidarScanMode scanMode;
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    float frequency;
    drv->getFrequency(scanMode, count, frequency);
  }

  if (fdp.ConsumeBool()) {
    bool force = fdp.ConsumeBool();
    drv->startScanNormal(force);
  }

  if (fdp.ConsumeBool()) {
    // NOTE: DEPRECATED API
    bool support;
    drv->checkExpressScanSupported(support);
  }

  if (fdp.ConsumeBool()) {
    drv->stop();
  }

  if (fdp.ConsumeBool()) {
    // NOTE: DEPRECATED API
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    auto nodebuffer = std::unique_ptr<rplidar_response_measurement_node_t[]>(
        new rplidar_response_measurement_node_t[count]);
    auto timeout = static_cast<uint32_t>(fdp.ConsumeIntegral<uint16_t>());
    drv->grabScanData(nodebuffer.get(), count, timeout);
  }

  if (fdp.ConsumeBool()) {
    // NOTE: DEPRECATED API
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    auto nodebuffer = std::unique_ptr<rplidar_response_measurement_node_t[]>(
        new rplidar_response_measurement_node_t[count]);
    drv->ascendScanData(nodebuffer.get(), count);
  }

  if (fdp.ConsumeBool()) {
    // NOTE: DEPRECATED API
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    auto nodebuffer = std::unique_ptr<rplidar_response_measurement_node_t[]>(
        new rplidar_response_measurement_node_t[count]);
    drv->getScanDataWithInterval(nodebuffer.get(), count);
  }

  if (fdp.ConsumeBool()) {
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    auto nodebuffer = std::unique_ptr<rplidar_response_measurement_node_hq_t[]>(
        new rplidar_response_measurement_node_hq_t[count]);
    auto timeout = static_cast<uint32_t>(fdp.ConsumeIntegral<uint16_t>());
    drv->grabScanDataHq(nodebuffer.get(), count, timeout);
  }

  if (fdp.ConsumeBool()) {
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    auto nodebuffer = std::unique_ptr<rplidar_response_measurement_node_hq_t[]>(
        new rplidar_response_measurement_node_hq_t[count]);
    drv->ascendScanData(nodebuffer.get(), count);
  }

  if (fdp.ConsumeBool()) {
    size_t count = fdp.ConsumeIntegralInRange<size_t>(0, MAX_ALLOC);
    auto nodebuffer = std::unique_ptr<rplidar_response_measurement_node_hq_t[]>(
        new rplidar_response_measurement_node_hq_t[count]);
    drv->getScanDataWithIntervalHq(nodebuffer.get(), count);
  }

  rp::standalone::rplidar::RPlidarDriver::DisposeDriver(drv);

  return 0;
}
