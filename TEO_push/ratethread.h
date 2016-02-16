#ifndef _ratethread_H_
#define _ratethread_H_

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <vector>

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) { // Conversion to [ms]
        x.resize(20);
        y.resize(20);
        z.resize(20);
    }

    void run()
    {

        samples = 20; //Number of samples for low-pass filter

       //Read sensor
       for(int i=0;i<samples;i++)
       {
           Bottle *input = readPort->read();
           if (input == NULL)
           {
                printf("[error] No data from sensor...\n");
                return;
            }
            x[i] = input->get(3).asDouble(); //Linear acceleration in X [m/s^2]
            y[i] = input->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
            z[i] = input->get(5).asDouble(); //Linear acceleration in Z [m/s^2]
       }

        //Low-pass Filter (Average)
       x_avg=0.0;
       y_avg=0.0;
       z_avg=0.0;
       for(int i=0;i<samples;i++)
       {
           x_avg = x_avg + x[i];
           y_avg = y_avg + y[i];
           z_avg = z_avg + z[i];
       }
        x_avg = x_avg / samples;
        y_avg = y_avg / samples;
        z_avg = z_avg / samples;

        for(int i=0;i<samples;i++)
        {
            printf("X[%d]: %f m/s^2\n",i,x[i]);
        }
        printf("X_average: %f m/s^2\n",x_avg);

        //Transformation from sensor coordinates to robot coordinates
        x_r = -x_avg;
        y_r = y_avg;
        z_r = -z_avg;
        printf("Acceleration in X: %f m/s^2\n",x_r);
        printf("Acceleration in Y: %f m/s^2\n",y_r);
        printf("Acceleration in Z: %f m/s^2\n",z_r);

        //Calculation of the Zero-Moment Point
        Xzmp = Xcom - (Zcom / z_r)*x_r; //ZMP X coordinate [cm]
        Yzmp = Ycom - (Zcom / z_r)*y_r; //ZMP Y coordinate [cm]
        printf("\nZMP = (%f, %f) cm\n", Xzmp, Yzmp);

        //PID
        actual_value = Xzmp;
        if (first_zmp==0)
        {
            setpoint = Xzmp; //Desired value [cm]
            first_zmp = 1;
        }
        //setpoint = 0; //Desired value [cm]
        pid_output = pidcontroller->calculate(setpoint, actual_value);
        pid_output = -pid_output;
        printf("Setpoint: %f\n", setpoint);
        printf("PID output: %f\n", pid_output);

        //Send motor torque through YARP
//        velLeftLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].
//        velRightLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].

        /* This is for plot with python */
        Bottle send;
        send.addDouble(Xzmp);
        send.addDouble(setpoint);
        writePort->write(send);

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

    void setPid(PID *value)
    {
        pidcontroller = value;
    }

    void setReadPort(BufferedPort<Bottle> *value)
    {
        readPort = value;
    }

    void setWritePort(Port *value)
    {
        writePort = value;
    }

    void setFirstValues()
    {
        first_zmp = 0;
    }

private:
    BufferedPort<Bottle> *readPort;                 //YARP port for reading from sensor
    PID *pidcontroller;                             //PID controller
//    IVelocityControl *velRightLeg, *velLeftLeg;     //Velocity controllers
    vector<double> x,y,z;
    double x_avg,y_avg,z_avg,x_r,y_r,z_r;
    int first_zmp, samples;
    double Xzmp, Yzmp, actual_value, setpoint, pid_output;
    Port *writePort;  /* This is for plot with python */   //YARP port for sending output
};

#endif
