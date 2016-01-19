// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace std;
using namespace yarp::os;
using namespace yarp::dev;

//Origin of coordinates established in the middle point between the feet
#define Xcom 0 //Distance to COM in X axis (cm)
#define Ycom 0 //Distance to COM in Y axis (cm)
#define Zcom 103.6602 //Distance to COM in Z axis (cm)
//PID parameters
#define dt 0.1 //Loop interval time
#define max 12 //Maximum value of the manipulated variable
#define min -12 //Minimum value of the manipulated variable
#define Kp 1 //Proportional gain
#define Kd 0.001 //Derivative gain
#define Ki 0 //Integral gain
#define setpoint 0 //Desired value
//Motor
#define max_vel 30384 //Maximum velocity (deg/s)

int main()
{
    double x, y, z, x2, y2, z2;

    //Construct PID Controller
    PID pidcontroller(dt, max, min, Kp, Kd, Ki); //PD controller (Ki=0)

    //Initialise YARP
    Network yarp;

    //YARP port for reading from sensor
    BufferedPort<Bottle> readPort;
    readPort.open("/inertial:i");

    Time::delay(1); //Wait 1s for port to open

    //Connect to IMU
    Network::connect("/inertial", "/inertial:i");

    //Connect to robot left leg
    Property optionsLeftLeg;                                //YARP class for storing name-value (key-value) pairs
    optionsLeftLeg.put("device","remote_controlboard");     //YARP device
    optionsLeftLeg.put("remote","/teo/leftLeg");            //To what will be connected
    optionsLeftLeg.put("local","/juan/leftLeg");            //How will be called on YARP network
    PolyDriver deviceLeftLeg(optionsLeftLeg);               //YARP multi-use driver with the given options
    if(!deviceLeftLeg.isValid())
    {
      printf("/teo/leftLeg device not available.\n");
      deviceLeftLeg.close();
      Network::fini();
      return 1;
    }
    IVelocityControl *velLeftLeg;                 //Velocity controller
    if ( ! deviceLeftLeg.view(velLeftLeg) )
    {
        printf("[warning] Problems acquiring robot left leg IVelocityControl interface\n");
        return false;
    } else printf("[success] TEO_push acquired robot left leg IVelocityControl interface\n");
    velLeftLeg->setVelocityMode();

    //Connect to robot right leg
    Property optionsRightLeg;;                                //YARP class for storing name-value (key-value) pairs
    optionsRightLeg.put("device","remote_controlboard");      //YARP device
    optionsRightLeg.put("remote","/teo/rightLeg");            //To what will be connected
    optionsRightLeg.put("local","/juan/rightLeg");            //How will be called on YARP network
    PolyDriver deviceRightLeg(optionsRightLeg);               //YARP multi-use driver with the given options
    if(!deviceRightLeg.isValid())
    {
      printf("/teo/rightLeg device not available.\n");
      deviceRightLeg.close();
      Network::fini();
      return 1;
    }
    IVelocityControl *velRightLeg;                 //Velocity controller
    if ( ! deviceRightLeg.view(velRightLeg) )
    {
        printf("[warning] Problems acquiring robot right leg IVelocityControl interface\n");
        return false;
    } else printf("[success] TEO_push acquired robot right leg IVelocityControl interface\n");
    velRightLeg->setVelocityMode();


    while (true)
    {

        Bottle *input = readPort.read();

        if (input != NULL)
        {
            x = input->get(3).asDouble(); //Linear acceleration in X (m/s^2)
            y = input->get(4).asDouble(); //Linear acceleration in Y (m/s^2)
            z = input->get(5).asDouble(); //Linear acceleration in Z (m/s^2)
        }

        //Transformation from sensor coordinates to robot coordinates
        x2 = -x;
        y2 = y;
        z2 = -z;

        //Calculation of the Zero-Moment Point
        double Xzmp = Xcom - (Zcom / z2)*x2; //ZMP X coordinate (cm)
        double Yzmp = Ycom - (Zcom / z2)*y2; //ZMP Y coordinate (cm)
        printf("ZMP = (%f, %f) cm\n", Xzmp, Yzmp);

        //PID
        double actual_value = Xzmp;
        double pid_output = pidcontroller.calculate(setpoint, actual_value);
        printf("PID output: %f\n", pid_output);

        //Transform to range of velocity (deg/s)
        double velocity = (((pid_output - min) * (max_vel - (-max_vel))) / (max - min)) + (-max_vel);
        printf("Velocity output: %f deg/s\n\n", velocity);

        //Send motor torque through YARP
        velLeftLeg->velocityMove(4, velocity);  //Fourth motor. Velocity in deg/s.
        velRightLeg->velocityMove(4, velocity);  //Fourth motor. Velocity in deg/s.

        Time::delay(1.0/5.0); //Delay in seconds
    }

    return 0;
}
