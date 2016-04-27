// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

/*  Juan Lorente                                                             */
/*  Masters' Thesis                                                          */
/*  2D EQUILIBRIUM CONTROL BY INERTIAL PERCEPTION WITH HUMANOID ROBOT TEO    */

#include <math.h>
#include <yarp/os/all.h>
#include <yarp/dev/all.h>
#include <fstream>
#include <deque>

#include "pid.h"

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

//General constants
#define pi 3.14159265358979323846
#define g 9.81

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis [cm]
#define Ycom 0 //Distance to COM in Y axis [cm]
#define Zcom 103.6602 //Distance to COM in Z axis [cm]

//Low-pass Filter
#define samples 15       //Number of samples for computing average

//PID parameters
#define dt 0.05 //Loop interval time [s]
#define max 10 //Maximum output value
#define min -10 //Minimum output value
    //Ankle parameters
#define Kp_ankle 0.1 //Proportional gain
#define Kd_ankle 0.01 //Derivative gain
#define Ki_ankle 0.001 //Integral gain
    //Hip parameters
#define Kp_hip 10 //Proportional gain
#define Kd_hip 0.01 //Derivative gain
#define Ki_hip 0.001 //Integral gain

#define seconds 1 //DELAY FOR EXPERIMENTS

#include "ratethread.h"

int main(int argc, char *argv[])
{

    //CONSTRUCT PID CONTROLLERS
    PID pidcontroller_ankle_s(dt, max, min, Kp_ankle, Kd_ankle, Ki_ankle);    //Ankle Sagittal PID
    PID pidcontroller_ankle_f(dt, max, min, Kp_ankle, Kd_ankle, Ki_ankle);    //Ankle Frontal PID
    PID pidcontroller_hip(dt, max, min, Kp_hip, Kd_hip, Ki_hip);              //Hip PID

    //INITIALISE YARP
    Network yarp;
    if ( ! yarp.checkNetwork() ){
        cerr << "[error] YARP network not found." << endl;
        return -1;
    } else cout << "[success] YARP network found." << endl;

    //OPEN YARP PORT FOR READING FROM SENSOR
    BufferedPort<Bottle> readPort;
    readPort.open("/inertial:i");
    Time::delay(0.5);

    //CONNECT TO IMU
    Network::connect("/inertial", "/inertial:i");
    if ( NetworkBase::isConnected("/inertial", "/inertial:i") == false ){
        cerr << "[error] Couldn't connect to YARP port /inertial:i." << endl;
    } else cout << "[success] Connected to IMU." << endl;
    Time::delay(0.5);

    //CONNECT TO ROBOT LEFT LEG
    Property optionsLeftLeg;                                //YARP class for storing name-value (key-value) pairs
    optionsLeftLeg.put("device","remote_controlboard");     //YARP device
    optionsLeftLeg.put("remote","/teo/leftLeg");            //To what will be connected
    optionsLeftLeg.put("local","/local/leftLeg");           //How will be called on YARP network
    PolyDriver deviceLeftLeg(optionsLeftLeg);               //YARP multi-use driver with the given options
    if(!deviceLeftLeg.isValid())
    {
        cout << "[error] /teo/leftLeg device not available." << endl;
        deviceLeftLeg.close();
        Network::fini();
        return 1;
    }
            ///Velocity controller
    IVelocityControl *velLeftLeg;
    if ( ! deviceLeftLeg.view(velLeftLeg) )
    {
        cout << "[error] Problems acquiring robot left leg IVelocityControl interface." << endl;
        return false;
    } else cout << "[success] Robot left leg IVelocityControl interface acquired." << endl;
    velLeftLeg->setVelocityMode();

    //CONNECT TO ROBOT RIGHT LEG
    Property optionsRightLeg;                               //YARP class for storing name-value (key-value) pairs
    optionsRightLeg.put("device","remote_controlboard");    //YARP device
    optionsRightLeg.put("remote","/teo/rightLeg");          //To what will be connected
    optionsRightLeg.put("local","/local/rightLeg");         //How will be called on YARP network
    PolyDriver deviceRightLeg(optionsRightLeg);             //YARP multi-use driver with the given options
    if(!deviceRightLeg.isValid())
    {
      cout << "[error] /teo/rightLeg device not available." << endl;
      deviceRightLeg.close();
      Network::fini();
      return 1;
    }
            ///Velocity controller
    IVelocityControl *velRightLeg;
    if ( ! deviceRightLeg.view(velRightLeg) )
    {
        cout << "[error] Problems acquiring robot right leg IVelocityControl interface." << endl;
        return false;
    } else cout << "[success] Robot right leg IVelocityControl interface acquired." << endl;
    velRightLeg->setVelocityMode();

    //CONNECT TO ROBOT TRUNK
    Property optionsTrunk;                                  //YARP class for storing name-value (key-value) pairs
    optionsTrunk.put("device","remote_controlboard");       //YARP device
    optionsTrunk.put("remote","/teo/trunk");                //To what will be connected
    optionsTrunk.put("local","/local/trunk");               //How will be called on YARP network
    PolyDriver deviceTrunk(optionsTrunk);                   //YARP multi-use driver with the given options
    if(!deviceTrunk.isValid())
    {
      cout << "[error] /teo/trunk device not available." << endl;
      deviceTrunk.close();
      Network::fini();
      return 1;
    }
            ///Velocity controller
    IVelocityControl *velTrunk;
    if ( ! deviceTrunk.view(velTrunk) )
    {
       cout << "[error] Problems acquiring robot trunk IVelocityControl interface." << endl;
       return false;
    } else cout << "[success] Robot trunk IVelocityControl interface acquired." << endl;
    velTrunk->setVelocityMode();
            ///Position controller
    IPositionControl *posTrunk;
    if ( ! deviceTrunk.view(posTrunk) )
    {
       cout << "[error] Problems acquiring robot trunk IPositionControl interface." << endl;
       return false;
    } else cout << "[success] Robot trunk IPositionControl interface acquired." << endl;
    posTrunk->setPositionMode();
            ///Encoders
    IEncoders *encTrunk;
    if( ! deviceTrunk.view(encTrunk) )
    {
        cout << "[error] Problems acquiring robot trunk Encoders interface." << endl;
        return false;
    }else cout << "[success] Robot trunk Encoders interface acquired." << endl;


    //DELAY FOR EXPERIMENTS
    Time::delay(seconds);

    //SAGITTAL CONTROL THREAD
    MyRateThread SagittalThread;
    SagittalThread.set("sagittal", &readPort, velRightLeg, velLeftLeg, velTrunk, posTrunk, encTrunk,
                       &pidcontroller_ankle_s, &pidcontroller_hip);
    SagittalThread.start();

    //FRONTAL CONTROL THREAD
    MyRateThread FrontalThread;
    FrontalThread.set("frontal", &readPort, velRightLeg, velLeftLeg, velTrunk, posTrunk, encTrunk,
                      &pidcontroller_ankle_f, &pidcontroller_hip);
    FrontalThread.start();

    //WAIT FOR ENTER AND EXIT LOOP
    char c;
    do {
        c=getchar();
    } while(c != '\n');
    SagittalThread.stop();
    FrontalThread.stop();
    Time::delay(0.5);

    //CLOSE PORTS AND DEVICES
    deviceRightLeg.close();
    deviceLeftLeg.close();
    deviceTrunk.close();
    readPort.close();

    return 0;
}

