/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

/**
 * @ingroup icub_hardware_modules
 * \defgroup analogSensorEth
 *
 * To Do: add description
 *
 */

#ifndef __robotran_yarp_force_torque_driver_h__
#define __robotran_yarp_force_torque_driver_h__

#include <yarp/dev/DeviceDriver.h>
#include <yarp/dev/IAnalogSensor.h>

#include <yarp/os/Semaphore.h>
#include <string>
#include <list>
#include <map>

#include <boost/concept_check.hpp>

#include <yarp/os/Semaphore.h>
#include <yarp/os/RateThread.h>
#include <string>
#include <list>
#include <map>

#include <boost/concept_check.hpp>

#include "MBSdataStruct.h"
#include <yarp/dev/PolyDriverList.h>

namespace yarp{
    namespace dev{
        class RobotranYarpForceTorqueDriver;
    }
}

namespace robotran {
    class RobotranYarpForceTorque;
}

#define _CLEAN_WAY_

class IRobotran
{
public:
    virtual void onUpdate(const MBSdataStruct * /*_info*/) = 0;// update forcetorque_data with the simulation data
};

typedef int AnalogDataFormat;
/*! class yarp::dev::fakebotFTsensor
 *
 */
class yarp::dev::RobotranYarpForceTorqueDriver:
                                      public yarp::dev::IAnalogSensor,
#ifdef _CLEAN_WAY_
                                      public yarp::dev::DeviceDriver,
#else
                                      public yarp::dev::PolyDriver,
#endif
                                      public IRobotran
{
private:

////////////////////
    // parameters
    int             _channels;
    short           _useCalibration;

    short status;

    double timeStamp;
    yarp::os::Semaphore mutex;
    yarp::sig::Vector data;

    //yarp::sig::Vector forcetorque_data; //buffer for forcetorque sensor data

    yarp::os::Bottle initMsg;
    yarp::os::Bottle speedMsg;
    yarp::os::Bottle closeMsg;
    std::string deviceIdentifier;

    // Read useful data from config and check fir correctness
    bool fromConfig(yarp::os::Searchable &config);

public:

    RobotranYarpForceTorqueDriver();
    ~RobotranYarpForceTorqueDriver();

    bool open(yarp::os::Searchable &config);
    bool close();

    void onUpdate(const MBSdataStruct * /*_info*/);// update forcetorque_data with the simulation data
    //void onUpdate(const MBSdataStruct MBSdata );// update forcetorque_data with the simulation data
    // robotran is a namespace. it should have some

    //IAnalogSensor interface
    virtual int read(yarp::sig::Vector &out);
    virtual int getState(int ch);
    virtual int getChannels();
    virtual int calibrateChannel(int ch, double v);
    virtual int calibrateSensor();
    virtual int calibrateSensor(const yarp::sig::Vector& value);

    virtual int calibrateChannel(int ch);

    void setDeviceId(std::string id)
    {
        deviceIdentifier=id;
    }

    std::string getDeviceId()
    {
        return deviceIdentifier;
    }
};


#endif
