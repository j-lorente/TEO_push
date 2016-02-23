#ifndef _ratethread_H_
#define _ratethread_H_

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {    // Conversion to [ms]
        x_sensor.resize(samples);
        y_sensor.resize(samples);
        z_sensor.resize(samples);
        first_iteration = 0;
        iteration = 1;
        act_loop_acum = 0;
    }

    void run()
    {
        getInitialTime(first_iteration);

        x_sensor.push_front(1);
        x_sensor.pop_back();

        cout << "mydeque contains:";
          for (deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
            cout << ' ' << *it;
          cout << '\n';


//       //RESET OF AVERAGE VARIABLES
//       x_sensor = 0.0;
//       y_sensor = 0.0;
//       z_sensor = 0.0;

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

        //CONVERSION FROM SENSOR COORDINATES TO ROBOT COORDINATES
        x_robot = -x;
        y_robot = y;
        z_robot = -z;

        //CALCULATION OF THE ZERO MOMENT POINT
        Xzmp = Xcom - (Zcom / z_robot) * x_robot; //ZMP X coordinate [cm]
        Yzmp = Ycom - (Zcom / z_robot) * y_robot; //ZMP Y coordinate [cm]

        //CALCULATE ZMP ERROR
        ZMPerror = getError();

        //PID
        actual_value = Xzmp;
        if (first_iteration==0)
        {
            setpoint = Xzmp; //Get initial position as setpoint [cm]
        }
        pid_output = - pidcontroller->calculate(setpoint, actual_value);

        //SEND MOTOR VELOCITY THROUGH YARP
//        velLeftLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].
//        velRightLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].

        //saveInFile(first_iteration, act_time, Xzmp, setpoint); //Save data in external file

        first_iteration = 1; //Not first iteration anymore

        getCurrentTime(init_time, init_loop, act_loop_acum, act_loop_avg, iteration);

        printData(x_robot, y_robot, z_robot, Xzmp, Yzmp, ZMPerror, setpoint, pid_output, act_time, act_loop_avg);

        cout << endl << "Press ENTER to exit..." << endl;
        cout << "*******************************" << endl << endl;

        iteration++;
    }

    void getInitialTime(int first_iteration)
    {
        if (first_iteration==0)
        {
            init_time = Time::now();
        }
        init_loop = Time::now();
    }

    void getCurrentTime(double init_time, double init_loop, double act_loop_acum,
                        double act_loop_average, int iteration)
    {
        act_time = Time::now() - init_time;
        act_loop = Time::now() - init_loop;
        act_loop_acum = act_loop_acum + act_loop;
        act_loop_avg = act_loop_acum / iteration;
    }

    void printData(double x_robot, double y_robot, double z_robot, double Xzmp, double Yzmp, double ZMPerror,
                   double setpoint, double pid_output, double act_time, double act_loop_avg)
    {
        cout << "Acceleration in X = " << x_robot << " m/s^2" << endl;
        cout << "Acceleration in Y = " << y_robot << " m/s^2" << endl;
        cout << "Acceleration in Z = " << z_robot << " m/s^2" << endl << endl;
        cout << "ZMP = (" << Xzmp << ", " << Yzmp << ") cm" << endl;
        //cout << "ZMP(x) error = +/- " << ZMPerror << " cm" << endl;
        cout << "Setpoint = " << setpoint << endl;
        cout << "PID output = " << pid_output << endl << endl;
        cout << "Loop time: " << act_loop_avg*1000 << " ms" << endl;
        cout << "Absolute time: " << int(act_time) << " s" << endl;
    }

    double getError()
    {
        if (first_iteration==0)
        {
            maxZMP = Xzmp;
            minZMP = Xzmp;
        }
        if (Xzmp > maxZMP)
        {
            maxZMP = Xzmp;
        }
        if (Xzmp < minZMP)
        {
            minZMP = Xzmp;
        }
        return (maxZMP - minZMP)/2;
    }

    void saveInFile(int first_iteration, int act_time, double Xzmp, double setpoint)
    {
        ofstream out;
        if (first_iteration==0) {out.open("data.txt",ios::trunc);}  //The first time deletes previous content
        else {out.open("data.txt",ios::app);}                       //The following times appends data to the file
        out << act_time << " " << Xzmp << " " << setpoint << endl;
        out.close();
    }

//    void set(IVelocityControl *value, IVelocityControl *value0, PID *value1, BufferedPort<Bottle> *value2)
//    {
//        velRightLeg = value;
//        velLeftLeg = value0;
//        pidcontroller = value1;
//        readPort = value2;
//    }

    void set(PID *value1, BufferedPort<Bottle> *value2)
    {
        pidcontroller = value1;
        readPort = value2;
    }

private:
    BufferedPort<Bottle> *readPort;
    PID *pidcontroller;
//    IVelocityControl *velRightLeg, *velLeftLeg;

    int first_iteration;
    int iteration;
    double init_time, act_time, init_loop, act_loop, act_loop_avg, act_loop_acum;

    double Xzmp, Yzmp, actual_value, setpoint, pid_output;
    double maxZMP, minZMP, ZMPerror;
    double x, y, z, x_robot, y_robot, z_robot;
    //double x_sensor, y_sensor, z_sensor;

    deque<double> x_sensor, y_sensor, z_sensor;
};

#endif
