/*
 * Copyright (C) 2007-2013 Istituto Italiano di Tecnologia ADVR & iCub Facility
 * Authors: Enrico Mingo, Alessio Rocchi, Mirko Ferrati, Silvio Traversaro and Alessandro Settimi
 * CopyPolicy: Released under the terms of the LGPLv2.1 or later, see LGPL.TXT
 */

#ifndef ROBOTRAN_YARP_CONTROLBOARD_H
#define ROBOTRAN_YARP_CONTROLBOARD_H

#include <yarp/sig/all.h>
#include <yarp/os/all.h>
#include <yarp/os/Network.h>
#include <yarp/os/Property.h>
#include <yarp/dev/Drivers.h>
#include <yarp/dev/PolyDriver.h>
#include <yarp/dev/ControlBoardInterfaces.h>
#include <yarp/dev/IControlMode.h>
#include <yarp/sig/Vector.h>
#include <yarp/os/Time.h>
#include <yarp/dev/IControlLimits2.h>
#include <yarp/dev/Wrapper.h>

//Robotran info
#include "MBSdataStruct.h"
#include "simu_def.h"


const double ROBOT_POSITION_TOLERANCE = 0.9;

namespace yarp {
    namespace dev {
        class RobotranYarpMotionControl;
    }
    
}


class yarp::dev::RobotranYarpMotionControl:
    public DeviceDriver,
    public IPositionControl2,
    public IVelocityControl,
    public IEncodersTimed,
    public IControlMode,
    public IPositionDirect,
    public IControlLimits2
{
public:
    
    RobotranYarpMotionControl();

    virtual ~RobotranYarpMotionControl();

    /**
     * robotran stuff
     */
    bool robotran_init();
    void updateToYarp(const MBSdataStruct * MBSdata);
    void updateFromYarp(MBSdataStruct *MBSdata);

    /**
     * Yarp interfaces start here
     */
    
    //DEVICE DRIVER
    virtual bool open(yarp::os::Searchable& config);    
    virtual bool close();
    
    //ENCODERS
    virtual bool getEncoder(int j, double *v); //WORKS
    virtual bool getEncoders(double *encs); //WORKS    
    virtual bool resetEncoder(int j); //WORKS
    virtual bool resetEncoders(); //WORKS
    virtual bool setEncoder(int j, double val); //WORKS
    virtual bool setEncoders(const double *vals); //WORKS 
    
    virtual bool getEncoderSpeed(int j, double *sp); //NOT TESTED
    virtual bool getEncoderSpeeds(double *spds); //NOT TESTED 
    
    virtual bool getEncoderAcceleration(int j, double *spds); //NOT IMPLEMENTED
    virtual bool getEncoderAccelerations(double *accs); //NOT IMPLEMENTED

    // ENCODERS TIMED
    virtual bool getEncodersTimed(double *encs, double *time);
    virtual bool getEncoderTimed(int j, double *encs, double *time);

    //POSITION CONTROL
    virtual bool stop(int j); //WORKS
    virtual bool stop(); //WORKS
    virtual bool positionMove(int j, double ref); //WORKS
    virtual bool getAxes(int *ax); // WORKS
    virtual bool positionMove(const double *refs); //WORKS
    /// @arg sp [deg/sec]
    virtual bool setRefSpeed(int j, double sp); //WORKS
    virtual bool getRefSpeed(int j, double *ref); //WORKS
    virtual bool getRefSpeeds(double *spds); //WORKS
    
    virtual bool relativeMove(int j, double delta); //NOT TESTED
    virtual bool relativeMove(const double *deltas); //NOT TESTED
    virtual bool checkMotionDone(int j, bool *flag); //NOT TESTED
    virtual bool checkMotionDone(bool *flag); //NOT TESTED
    virtual bool setPositionMode(); //NOT TESTED

    // POS 2
    virtual bool positionMove(const int n_joint, const int *joints, const double *refs);
    virtual bool relativeMove(const int n_joint, const int *joints, const double *deltas);
    virtual bool checkMotionDone(const int n_joint, const int *joints, bool *flags);
    virtual bool setRefSpeeds(const int n_joint, const int *joints, const double *spds);
    virtual bool setRefAccelerations(const int n_joint, const int *joints, const double *accs);
    virtual bool getRefSpeeds(const int n_joint, const int *joints, double *spds);
    virtual bool getRefAccelerations(const int n_joint, const int *joints, double *accs);
    virtual bool stop(const int n_joint, const int *joints);

    /// @arg spds [deg/sec]
    virtual bool setRefSpeeds(const double *spds); //NOT TESTED
    
    virtual bool setRefAcceleration(int j, double acc); //NOT IMPLEMENTED
    virtual bool setRefAccelerations(const double *accs); //NOT IMPLEMENTED
    virtual bool getRefAcceleration(int j, double *acc); //NOT IMPLEMENTED
    virtual bool getRefAccelerations(double *accs); //NOT IMPLEMENTED
    
    //VELOCITY CONTROL
    virtual bool setVelocityMode(); //NOT TESTED
    virtual bool velocityMove(int j, double sp); //NOT TESTED    
    virtual bool velocityMove(const double *sp); //NOT TESTED    
    
    //CONTROL MODE
    virtual bool setPositionMode(int j); //WORKS    
    virtual bool setVelocityMode(int j){return false;}; //WORKS
    virtual bool getControlMode(int j, int *mode){return false;}; //WORKS
    
    virtual bool setTorqueMode(int j){return false;}; //NOT TESTED 
    virtual bool getControlModes(int *modes){return false;}; //NOT TESTED
    
    virtual bool setImpedancePositionMode(int j){return false;};//NOT IMPLEMENTED
    virtual bool setImpedanceVelocityMode(int j){return false;}; //NOT IMPLEMENTED
    virtual bool setOpenLoopMode(int j){return false;}; //NOT IMPLEMENTED
    

    /*IMPEDANCE CTRL
    virtual bool getImpedance(int j, double *stiffness, double *damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedance(int j, double stiffness, double damping); // [Nm/deg] & [Nm*sec/deg]
    virtual bool setImpedanceOffset(int j, double offset);
    virtual bool getImpedanceOffset(int j, double* offset);
    virtual bool getCurrentImpedanceLimit(int j, double *min_stiff, double *max_stiff, double *min_damp, double *max_damp);
    */

    /*
     * IPidControl Interface methods
     *
    virtual bool setPid (int j, const Pid &pid);
    virtual bool setPids (const Pid *pids);
    virtual bool setReference (int j, double ref);
    virtual bool setReferences (const double *refs);
    virtual bool setErrorLimit (int j, double limit);
    virtual bool setErrorLimits (const double *limits);
    virtual bool getError (int j, double *err);
    virtual bool getErrors (double *errs);
    virtual bool getPid (int j, Pid *pid);
    virtual bool getPids (Pid *pids);
    virtual bool getReference (int j, double *ref);
    virtual bool getReferences (double *refs);
    virtual bool getErrorLimit (int j, double *limit);
    virtual bool getErrorLimits (double *limits);
    virtual bool resetPid (int j);
    virtual bool disablePid (int j);
    virtual bool enablePid (int j);
    virtual bool setOffset (int j, double v);
    */

    // IPOSITION DIRECT
    bool setPositionDirectMode();
    bool setPosition(int j, double ref);
    bool setPositions(const int n_joint, const int *joints, double *refs);
    bool setPositions(const double *refs);

    // ICONTROLLIMITS2
    bool  setVelLimits (int axis, double min, double max) {return false;};
    bool  getVelLimits (int axis, double *min, double *max) {return false;};
    bool  setLimits (int axis, double min, double max) {return false;};
    bool  getLimits (int axis, double *min, double *max); 
    
private:
	
    /* PID structures */
    struct PID {
        double p;
        double i;
        double d;
        double maxInt;
        double maxOut;
    };
    
    unsigned int numberOfJoints;

    /**
     * The ROBOTRAN position of each joints, readonly from MBSdataStruct
     */
    yarp::sig::Vector pos;
    yarp::sig::VectorOf<int> jointID_map;
    yarp::sig::VectorOf<int> motorID_map;
    yarp::sig::Vector max_pos, min_pos;
    double simu_time;
    yarp::sig::VectorOf<int> controlMode;

    //Contains the parameters of the device contained in the yarpConfigurationFile .ini file
    
     /**
     * The GAZEBO desired position of each joints, (output of trajectory interp)
     */
    yarp::sig::Vector desiredPosition;
    
    /**
     * The zero position is the position of the GAZEBO joint that will be read as the starting one
     * i.e. getEncoder(j)=zeroPosition+gazebo.getEncoder(j);
     */
    yarp::sig::Vector zeroPosition;

    yarp::sig::Vector vel, speed, acc, amp, torque;
    yarp::os::Semaphore pos_lock;
    yarp::sig::Vector referenceSpeed, referencePosition, referenceAcceleraton, referenceTorque;

};

#endif //ROBOTRAN_YARP_CONTROLBOARD_H
