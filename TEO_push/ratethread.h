#ifndef _ratethread_H_
#define _ratethread_H_

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {}  // Conversion to [ms]

    void run()
    {

       //Read sensor
       Bottle *input = readPort->read();
       if (input == NULL)
       {
            printf("[error] No data yet...\n");
            return;
        }
        x = input->get(3).asDouble(); //Linear acceleration in X [m/s^2]
        y = input->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        z = input->get(5).asDouble(); //Linear acceleration in Z [m/s^2]

        //Transformation from sensor coordinates to robot coordinates
        x2 = -x;
        y2 = y;
        z2 = -z;
        printf("Acceleration in X: %f m/s^2\n",x2);
        printf("Acceleration in Y: %f m/s^2\n",y2);
        printf("Acceleration in Z: %f m/s^2\n",z2);

        //Calculation of the Zero-Moment Point
        double Xzmp = Xcom - (Zcom / z2)*x2; //ZMP X coordinate [cm]
        double Yzmp = Ycom - (Zcom / z2)*y2; //ZMP Y coordinate [cm]
        printf("\nZMP = (%f, %f) cm\n", Xzmp, Yzmp);

        //PID
        double actual_value = Xzmp;
        if (*first_zmp==0)
        {
            double setpoint = Xzmp;
            *first_zmp = 1;
        }
        //double setpoint = 0; //Desired value [cm]
        double pid_output = pidcontroller->calculate(setpoint, actual_value);
        printf("PID output: %f\n", pid_output);

        //Send motor torque through YARP
//        velLeftLeg->velocityMove(4, velocity);  //Fourth motor. Velocity [deg/s].
//        velRightLeg->velocityMove(4, velocity);  //Fourth motor. Velocity [deg/s].
        velLeftArm->velocityMove(3, pid_output);
        velRightArm->velocityMove(3, -pid_output);

        /* This is for plot with python */
        //Bottle send;
        //send.addDouble(Xzmp);
        //send.addDouble(pid_output);
        //writePort.write(send);

        printf("\nEnter value to exit...\n");
        cout << "*******************************" << endl << endl;

    }

//    void setVelRightLeg(IVelocityControl *value)
//    {
//        velRightLeg = value;
//    }

//    void setVelLeftLeg(IVelocityControl *value)
//    {
//        velLeftLeg = value;
//    }

    void setVelRightArm(IVelocityControl *value)
    {
        velRightArm = value;
    }

    void setVelLeftArm(IVelocityControl *value)
    {
        velLeftArm = value;
    }

    void setPid(PID *value)
    {
        pidcontroller = value;
    }

    void setReadPort(BufferedPort<Bottle> *value)
    {
        readPort = value;
    }

    void setFirstZMP(int *value)
    {
        first_zmp = value;
    }

private:
    BufferedPort<Bottle> *readPort;                  //YARP port for reading from sensor
    PID *pidcontroller;                             //PID controller
//    IVelocityControl *velRightLeg, *velLeftLeg;     //Velocity controllers
    IVelocityControl *velRightArm, *velLeftArm;     //Velocity controllers
    double x,y,z,x2,y2,z2, setpoint;
    int *first_zmp;

    /* This is for plot with python */
    //Port writePort;                                 //YARP port for sending output
};

#endif
