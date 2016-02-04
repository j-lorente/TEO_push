// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/* Juan Lorente - Masters' Thesis */
/* Push-recovery experiments with robot TEO */

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>

#include "pid.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
#define Zcom 103.6602 //Distance to COM in Z axis [cm]

//PID parameters
#define dt 0.01 //Loop interval time [assumtion: s]
#define max 10 //Maximum output value
#define min -10 //Minimum output value
#define Kp 0.079987 //Proportional gain
#define Kd 0 //Derivative gain
#define Ki 0.0015869 //Integral gain
#define setpoint 0 //Desired value [cm]

#include "ratethread.h"

int main(int argc, char *argv[])
{

    //Construct PID Controller
    PID pidcontroller(dt, max, min, Kp, Kd, Ki); //PI controller (Kd=0)

    //Initialise and check YARP
    Network yarp;
    if ( ! yarp.checkNetwork() ) {
        fprintf(stderr,"[error] %s found no YARP network (try running \"yarp detect --write\").\n",argv[0]);
        return -1;
    } else printf("[success] YARP network found.\n");

    //Connect to robot left leg
//    Property optionsLeftLeg;                                //YARP class for storing name-value (key-value) pairs
//    optionsLeftLeg.put("device","remote_controlboard");     //YARP device
//    optionsLeftLeg.put("remote","/teo/leftLeg");            //To what will be connected
//    optionsLeftLeg.put("local","/juan/leftLeg");            //How will be called on YARP network
//    PolyDriver deviceLeftLeg(optionsLeftLeg);               //YARP multi-use driver with the given options
//    if(!deviceLeftLeg.isValid())
//    {
//      printf("[error] /teo/leftLeg device not available.\n");
//      deviceLeftLeg.close();
//      Network::fini();
//      return 1;
//    }
//    IVelocityControl *velLeftLeg;                 //Velocity controller
//    if ( ! deviceLeftLeg.view(velLeftLeg) )
//    {
//        printf("[error] Problems acquiring robot left leg IVelocityControl interface.\n");
//        return false;
//    } else printf("[success] TEO_push acquired robot left leg IVelocityControl interface.\n");
//    velLeftLeg->setVelocityMode();

//    //Connect to robot right leg
//    Property optionsRightLeg;;                                //YARP class for storing name-value (key-value) pairs
//    optionsRightLeg.put("device","remote_controlboard");      //YARP device
//    optionsRightLeg.put("remote","/teo/rightLeg");            //To what will be connected
//    optionsRightLeg.put("local","/juan/rightLeg");            //How will be called on YARP network
//    PolyDriver deviceRightLeg(optionsRightLeg);               //YARP multi-use driver with the given options
//    if(!deviceRightLeg.isValid())
//    {
//      printf("[error] /teo/rightLeg device not available.\n");
//      deviceRightLeg.close();
//      Network::fini();
//      return 1;
//    }
//    IVelocityControl *velRightLeg;                 //Velocity controller
//    if ( ! deviceRightLeg.view(velRightLeg) )
//    {
//        printf("[error] Problems acquiring robot right leg IVelocityControl interface.\n");
//        return false;
//    } else printf("[success] TEO_push acquired robot right leg IVelocityControl interface.\n");
//    velRightLeg->setVelocityMode();

    //Control loop
    MyRateThread myRateThread;
//    myRateThread.setVelRightLeg(velRightLeg);
//    myRateThread.setVelLeftLeg(velLeftLeg);
    myRateThread.setPid(&pidcontroller);
    myRateThread.start();

    printf("Enter value to exit...\n");
    char c;
    cin >> c;  //This line is blocking - Wait for value

    myRateThread.stop();
//    deviceRightLeg.close();
//    deviceLeftLeg.close();
    Time::delay(0.5);  //Wait for thread to stop [s]

    return 0;

}
