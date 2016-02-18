#ifndef _ratethread_H_
#define _ratethread_H_

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {} // Conversion to [ms]

    void run()
    {
       //RESET OF AVERAGE VARIABLES
       x_sensor = 0.0;
       y_sensor = 0.0;
       z_sensor = 0.0;

//       //READ SENSOR
//       for(int i=0;i<samples;i++)
//       {
//           Bottle *input = readPort->read();
//           if (input==NULL)
//           {
//                printf("[error] No data from sensor...\n");
//                return;
//            }
//            x_sensor = x_sensor + input->get(3).asDouble(); //Linear acceleration in X [m/s^2]
//            y_sensor = y_sensor + input->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
//            z_sensor = z_sensor + input->get(5).asDouble(); //Linear acceleration in Z [m/s^2]
//       }

//        //Low-pass Filter (Average)
//        x = x_sensor / samples;
//        y = y_sensor / samples;
//        z = z_sensor / samples;

        //READ SENSOR
        Bottle *input = readPort->read();
        if (input==NULL)
        {
            printf("[error] No data from sensor...\n");
            return;
        }
        x = input->get(3).asDouble(); //Linear acceleration in X [m/s^2]
        y = input->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        z = input->get(5).asDouble(); //Linear acceleration in Z [m/s^2]

        //TRANSFORMATION FROM SENSOR COORDINATES TO ROBOT COORDINATES
        x_robot = -x;
        y_robot = y;
        z_robot = -z;
        printf("Acceleration in X: %f m/s^2\n",x_robot);
        printf("Acceleration in Y: %f m/s^2\n",y_robot);
        printf("Acceleration in Z: %f m/s^2\n",z_robot);

        //CALCULATION OF THE ZERO MOMENT POINT
        Xzmp = Xcom - (Zcom / z_robot) * x_robot; //ZMP X coordinate [cm]
        Yzmp = Ycom - (Zcom / z_robot) * y_robot; //ZMP Y coordinate [cm]
        printf("\nZMP = (%f, %f) cm\n", Xzmp, Yzmp);

        //PID
        actual_value = Xzmp;
        if (first_zmp==0)
        {
            setpoint = Xzmp; //Get initial position as setpoint [cm]
            first_zmp = 1;
        }
        pid_output = pidcontroller->calculate(setpoint, actual_value); //PROBAR PONIENDO AQUI EL MENOS (-)
        pid_output = -pid_output;
        printf("Setpoint: %f\n", setpoint);
        printf("PID output: %f\n", pid_output);

        //SEND MOTOR VELOCITY THROUGH YARP
//        velLeftLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].
//        velRightLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].

        //SAVE DATA IN EXTERNAL FILE
        ofstream out;
        out.open("data.txt",ios::app);
        out << Xzmp ;
        out << setpoint << endl;
        out.close();

        printf("\nPress ENTER to exit...\n\n");
        cout << "*******************************" << endl << endl;

    }

    void setVels(IVelocityControl *value, IVelocityControl *value0)
    {
        velRightLeg = value;
        velLeftLeg = value0;
    }

    void setPid(PID *value)
    {
        pidcontroller = value;
    }

    void setPorts(BufferedPort<Bottle> *value)
    {
        readPort = value;
    }

    void setFirstValues()
    {
        first_zmp = 0;
    }

private:
    BufferedPort<Bottle> *readPort;                                         //YARP port for reading from sensor
    PID *pidcontroller;                                                     //PID controller
    IVelocityControl *velRightLeg, *velLeftLeg;                             //Velocity controllers
    int first_zmp;
    double Xzmp, Yzmp, actual_value, setpoint, pid_output;
    double x, y, z, x_sensor, y_sensor, z_sensor, x_robot, y_robot, z_robot;
};

#endif
