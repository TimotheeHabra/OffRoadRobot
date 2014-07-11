/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifdef YARP

/// general purpose stuff.

#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <yarp/dev/PolyDriver.h>

#include <string>
#include <iostream>
#include <iterator>
#include <string.h>


/// specific to this device driver.
#include <ForceTorqueDriver.h>


#ifdef WIN32
#pragma warning(once:4355)
#endif

using namespace yarp;
using namespace yarp::os;
using namespace yarp::dev;

inline bool NOT_YET_IMPLEMENTED(const char *txt)
{
    return false;
}

//generic function that check is key1 is present in input bottle and that the result has size elements
// return true/false
static inline bool validate(Bottle &input, Bottle &out, const std::string &key1, const std::string &txt, int size)
{
    size++;  // size includes also the name of the parameter
    Bottle &tmp=input.findGroup(key1.c_str(), txt.c_str());
    if (tmp.isNull())
    {
        fprintf(stderr, "%s not found\n", key1.c_str());
        return false;
    }

    if( tmp.size() != size)
    {
        fprintf(stderr, "%s incorrect number of entries\n", key1.c_str());
        return false;
    }

    out=tmp;

    return true;
}

bool RobotranYarpForceTorqueDriver::fromConfig(yarp::os::Searchable &_config)
{
    return true;
}

bool RobotranYarpForceTorqueDriver::close()
{
    return true;
}

RobotranYarpForceTorqueDriver::RobotranYarpForceTorqueDriver()
{
    _useCalibration=0;
    _channels=6;
    data.resize(_channels);

    status=IAnalogSensor::AS_OK;
}

RobotranYarpForceTorqueDriver::~RobotranYarpForceTorqueDriver()
{

}

bool RobotranYarpForceTorqueDriver::open(yarp::os::Searchable &config)
{
    Property prop;
//    std::string str=config.toString().c_str();

//    if(!fromConfig(config))
//        return false;

//    prop.fromString(str.c_str());

    data.resize(30); // number of DoFs

    return true;
}


/*! Read a vector from the sensor.
 * @param out a vector containing the sensor's last readings.
 * @return AS_OK or return code. AS_TIMEOUT if the sensor timed-out.
 **/
int RobotranYarpForceTorqueDriver::read(yarp::sig::Vector &out)
{
    // This method gives data to the analogServer

    mutex.wait();
    status = AS_OK;

    //data=MBSdataStruct.Qq[];

    out.resize(data.size());

    out = data;
    mutex.post();
    return status;
}

int RobotranYarpForceTorqueDriver::getState(int ch)
{
    printf("getstate\n");
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::getChannels()
{
     return _channels;
}

int RobotranYarpForceTorqueDriver::calibrateSensor()
{
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::calibrateSensor(const yarp::sig::Vector& value)
{
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::calibrateChannel(int ch)
{
    return AS_OK;
}

int RobotranYarpForceTorqueDriver::calibrateChannel(int ch, double v)
{
    return AS_OK;
}

void RobotranYarpForceTorqueDriver::onUpdate(const MBSdataStruct * MBSdata)
{
    // output going to the forcetorque_data with the simulation data
    // input coming from robotran::UpdateSensor
    for (int i =1; i<30; i++)
        data[i] = MBSdata->Qq[i];

    return ;
}

#endif