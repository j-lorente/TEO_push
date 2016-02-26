// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*  Juan Lorente                                */
/*  Masters' Thesis                             */
/*  Push-recovery experiments with robot TEO    */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <time.h>
#include <deque>

#include "pid.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
#define Zcom 103.6602 //Distance to COM in Z axis [cm]

//Low-pass Filter
#define samples 100 //Number of samples for computing average

//PID parameters
#define dt 0.05 //Loop interval time [assumtion: s]
#define max 10 //Maximum output value
#define min -10 //Minimum output value
#define Kp 0.1 //Proportional gain
#define Kd 0.01 //Derivative gain
#define Ki 0.001 //Integral gain
#define setpoint 0 //Desired value [cm]

#include "ratethread.h"

int main(int argc, char *argv[])
{

    //CONSTRUCT PID CONTROLLER
    PID pidcontroller(dt, max, min, Kp, Kd, Ki); //P?PI?PD?

    //INITIALISE AND CHECK YARP
    Network yarp;
    if ( ! yarp.checkNetwork() ) {
        fprintf(stderr,"[error] %s found no YARP network (try running \"yarp detect --write\").\n",argv[0]);
        return -1;
    } else printf("[success] YARP network found.\n");

    //OPEN YARP PORT
    BufferedPort<Bottle> readPort;          //YARP port for reading from sensor
    readPort.open("/inertial:i");
    Time::delay(10);  //Wait for port to open [s]

    //CONNECT TO IMU
    Network::connect("/inertial", "/inertial:i");
    Time::delay(0.5);  //Wait for ports connect [s]

    //CONNECT TO ROBOT LEFT LEG
    Property optionsLeftLeg;                                //YARP class for storing name-value (key-value) pairs
    optionsLeftLeg.put("device","remote_controlboard");     //YARP device
    optionsLeftLeg.put("remote","/teo/leftLeg");            //To what will be connected
    optionsLeftLeg.put("local","/juan/leftLeg");            //How will be called on YARP network
    PolyDriver deviceLeftLeg(optionsLeftLeg);               //YARP multi-use driver with the given options
    if(!deviceLeftLeg.isValid())
    {
      printf("[error] /teo/leftLeg device not available.\n");
      deviceLeftLeg.close();
      Network::fini();
      return 1;
    }
    IVelocityControl *velLeftLeg;                 //Velocity controller
    if ( ! deviceLeftLeg.view(velLeftLeg) )
    {
        printf("[error] Problems acquiring robot left leg IVelocityControl interface.\n");
        return false;
    } else printf("[success] TEO_push acquired robot left leg IVelocityControl interface.\n");
    velLeftLeg->setVelocityMode();

    //CONNECT TO ROBOT RIGHT LEG
    Property optionsRightLeg;                                //YARP class for storing name-value (key-value) pairs
    optionsRightLeg.put("device","remote_controlboard");      //YARP device
    optionsRightLeg.put("remote","/teo/rightLeg");            //To what will be connected
    optionsRightLeg.put("local","/juan/rightLeg");            //How will be called on YARP network
    PolyDriver deviceRightLeg(optionsRightLeg);               //YARP multi-use driver with the given options
    if(!deviceRightLeg.isValid())
    {
      printf("[error] /teo/rightLeg device not available.\n");
      deviceRightLeg.close();
      Network::fini();
      return 1;
    }
    IVelocityControl *velRightLeg;                 //Velocity controller
    if ( ! deviceRightLeg.view(velRightLeg) )
    {
        printf("[error] Problems acquiring robot right leg IVelocityControl interface.\n");
        return false;
    } else printf("[success] TEO_push acquired robot right leg IVelocityControl interface.\n");
    velRightLeg->setVelocityMode();

    //CONTROL LOOP
    MyRateThread myRateThread;
    myRateThread.set(velRightLeg, velLeftLeg, &pidcontroller, &readPort);
//    myRateThread.set(&pidcontroller, &readPort);
    myRateThread.start();

    //WAIT FOR ENTER AND EXIT LOOP
    char c;
    do {
        c=getchar();
    } while(c != '\n');
    myRateThread.stop();
    Time::delay(0.5);  //Wait for thread to stop [s]

    //CLOSE PORTS AND DEVICES
    deviceRightLeg.close();
    deviceLeftLeg.close();
    readPort.close();

    return 0;
}
