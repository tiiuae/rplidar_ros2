/*
 *  RPLIDAR SDK
 *
 *  Copyright (c) 2009 - 2014 RoboPeak Team
 *  http://www.robopeak.com
 *  Copyright (c) 2014 - 2019 Shanghai Slamtec Co., Ltd.
 *  http://www.slamtec.com
 *
 */
/*
 * Redistribution and use in source and binary forms, with or without 
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, 
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, 
 *    this list of conditions and the following disclaimer in the documentation 
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, 
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR 
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR 
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, 
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, 
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; 
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR 
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, 
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#pragma once

#include <gz/msgs.hh>
#include <gz/transport.hh>

namespace rp { namespace standalone{ namespace rplidar {

class RPlidarDriverSim : public RPlidarDriverImplCommon
{
public:

    RPlidarDriverSim();
    virtual ~RPlidarDriverSim();
    virtual u_result connect(const char * worldSlashModel, _u32, _u32 flag = 0);
    virtual void disconnect() {};

    virtual u_result reset(_u32);
    virtual u_result clearNetSerialRxCache();
    virtual u_result getAllSupportedScanModes(std::vector<RplidarScanMode>& outModes, _u32 timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result getTypicalScanMode(_u16& outMode, _u32 timeoutInMs = DEFAULT_TIMEOUT);
    virtual u_result startScan(bool force, bool useTypicalScan, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL);
    virtual u_result startScanExpress(bool force, _u16 scanMode, _u32 options = 0, RplidarScanMode* outUsedScanMode = NULL, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result getHealth(rplidar_response_device_health_t & health, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result getDeviceInfo(rplidar_response_device_info_t & info, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result setMotorPWM(_u16 pwm);
    virtual u_result startMotor();
    virtual u_result stopMotor();
    virtual u_result checkMotorCtrlSupport(bool & support, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result getFrequency(const RplidarScanMode& scanMode, size_t count, float & frequency);
    virtual u_result startScanNormal(bool force, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result stop(_u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result grabScanDataHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count, _u32 timeout = DEFAULT_TIMEOUT);
    virtual u_result ascendScanData(rplidar_response_measurement_node_hq_t * nodebuffer, size_t count);
    virtual u_result getScanDataWithIntervalHq(rplidar_response_measurement_node_hq_t * nodebuffer, size_t & count);

private:
    void _scanCallback(const gz::msgs::LaserScan &scan);

    gz::transport::Node _node;
    std::string _worldName;
    std::string _modelName;

    gz::msgs::LaserScan _scanInput;
    gz::msgs::LaserScan _scanOutput;
    bool _scanReady;
};

}}}