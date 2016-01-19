// -*- mode:C++; tab-width:4; c-basic-offset:4; indent-tabs-mode:nil -*-

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include "pid.h"

#include <yarp/os/all.h>
#include <yarp/dev/all.h>

using namespace std;
using namespace yarp::os;

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

    //YARP port for sending data
    Port writePort;
    writePort.open("/sender");

    Time::delay(1); //Wait for ports to open

    //Connect input and output ports
    Network::connect("/inertial", "/inertial:i");
    Network::connect("/sender", "/reader"); //Reader port must be initialized to read data

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
        Bottle send;
        send.addDouble(Xzmp);
        send.addDouble(velocity);
        writePort.write(send);

        Time::delay(1.0/5.0); //Delay in seconds
    }

    return 0;
}




















