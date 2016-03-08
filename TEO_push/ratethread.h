#ifndef _ratethread_H_
#define _ratethread_H_

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {    // Conversion to [ms]
        x_sensor.resize(samples);
        y_sensor.resize(samples);
        z_sensor.resize(samples);
        iteration = 1;
    }

    void run()
    {
        getInitialTime();

        //READ SENSOR
        Bottle *input = readPort->read();
        if (input==NULL)
        {
             printf("[error] No data from sensor...\n");
             return;
        }
        x_acc = input->get(3).asDouble();
        x_sensor.push_front(x_acc); //Linear acceleration in X [m/s^2]
        x_sensor.pop_back();
        y_sensor.push_front(input->get(4).asDouble()); //Linear acceleration in Y [m/s^2]
        y_sensor.pop_back();
        z_sensor.push_front(input->get(5).asDouble()); //Linear acceleration in Z [m/s^2]
        z_sensor.pop_back();

         //LOW-PASS FILTER
         x = 0.0;
         y = 0.0;
         z = 0.0;
         for(deque<double>::iterator it = x_sensor.begin(); it != x_sensor.end(); it++)
             x = x + *it;
         for(deque<double>::iterator it = y_sensor.begin(); it != y_sensor.end(); it++)
             y = y + *it;
         for(deque<double>::iterator it = z_sensor.begin(); it != z_sensor.end(); it++)
             z = z + *it;
         x = x / samples;
         y = y / samples;
         z = z / samples;

        //CONVERSION FROM SENSOR COORDINATES TO ROBOT COORDINATES
        x_robot = -x;
        y_robot = y;
        z_robot = -z;

        //CALCULATION OF THE ZERO MOMENT POINT
        Xzmp = Xcom - (Zcom / z_robot) * x_robot; //ZMP X coordinate [cm]
        Yzmp = Ycom - (Zcom / z_robot) * y_robot; //ZMP Y coordinate [cm]

//        //CALCULATE ZMP ERROR
//        ZMPerror = getError();

        //DETERMINING STRATEGY
        //lin_vel = (x_sensor.at(0) - x_sensor.at(1)) * dt;
        lin_vel = x_sensor.at(0) * dt;
        w = sqrt(g / (Zcom / 100));
        capture_point = (lin_vel / w) + (Xzmp / 100);
        cout << "Linear velocity: " << lin_vel << " m/s" << endl;
        cout << "g: " << g << " m/s²" << endl;
        cout << "Zcom: " << Zcom/100 << " m" << endl;
        cout << "w: " << w << " rad/s" << endl;
        cout << "Xcom: " << Xzmp/100 << " m" << endl;
        cout << "Capture point: " << capture_point << " m" << endl;
        if (capture_point < 0.12 && capture_point > -0.12) {cout << "ANKLE STRATEGY" << endl;}
        else {cout << "HIP STRATEGY" << endl;}

//        //PID
        actual_value = Xzmp;
//        if (iteration==1)
//        {
//            setpoint = Xzmp; //Get initial position as setpoint [cm]
//        }
        pid_output_ankle = - pidcontroller_ankle->calculate(setpoint, actual_value);
        pid_output_hip = pidcontroller_hip->calculate(setpoint, actual_value);

        // [PRUEBA DE ACELERACIONES]
//        double acc_max = (g / (Zcom / 100)) * (0.12);
//        cout << "acc_max: " << acc_max << " m/s²" << endl;
//        if (x_robot < acc_max && x_robot > -acc_max) {cout << "ANKLE STRATEGY" << endl;}
//        else {cout << "HIP STRATEGY" << endl;}

        //JOINTS CONTROL
//        if () //Ankle strategy
//        {
//            velLeftLeg->velocityMove(4, pid_output_ankle);   //Motor number. Velocity [deg/s].
//            velRightLeg->velocityMove(4, pid_output_ankle);  //Motor number. Velocity [deg/s].
//        }
//        else //Hip strategy
//        {
//            velLeftLeg->velocityMove(4, pid_output_ankle);   //Motor number. Velocity [deg/s].
//            velRightLeg->velocityMove(4, pid_output_ankle);  //Motor number. Velocity [deg/s].
//            velTrunk->velocityMove(1, pid_output_hip);       //Motor number. Velocity [deg/s].
//        }

        saveInFile(); //Save data in external file

        getCurrentTime();

        //printData(); //Save relevant data in external file for posterior plotting

        cout << endl << "Press ENTER to exit..." << endl;
        cout << "*******************************" << endl << endl;

        iteration++;
    }

    void getInitialTime()
    {
        if (iteration==1)
        {
            init_time = Time::now();
        }
        init_loop = Time::now();
    }

    void getCurrentTime()
    {
        act_time = Time::now() - init_time;
        act_loop = Time::now() - init_loop;
    }

//    void printData()
//    {
//        cout << "Acceleration in X = " << x_robot << " m/s^2" << endl;
//        cout << "Acceleration in Y = " << y_robot << " m/s^2" << endl;
//        cout << "Acceleration in Z = " << z_robot << " m/s^2" << endl << endl;
//        cout << "ZMP = (" << Xzmp << ", " << Yzmp << ") cm" << endl;
//        //cout << "ZMP(x) error = +/- " << ZMPerror << " cm" << endl;
//        cout << "Setpoint = " << setpoint << endl;
//        cout << "PID output = " << pid_output_ankle << endl << endl;
//        cout << "Loop time: " << act_loop << " ms" << endl;
//        cout << "Absolute time: " << int(act_time) << " s" << endl;
//    }

//    double getError()
//    {
//        if (iteration==1)
//        {
//            maxZMP = Xzmp;
//            minZMP = Xzmp;
//        }
//        if (Xzmp > maxZMP)
//        {
//            maxZMP = Xzmp;
//        }
//        if (Xzmp < minZMP)
//        {
//            minZMP = Xzmp;
//        }
//        return (maxZMP - minZMP)/2;               3.0763
//    }

    void saveInFile()
    {
        ofstream out;
        if (iteration==1) {out.open("data.txt",ios::trunc);}        //The first time deletes previous content
        else {out.open("data.txt",ios::app);}                       //The following times appends data to the file
        //out << act_time << " " << x << " " << pid_output_ankle << " " << pid_output_hip << " ";
        //out << setpoint << " " << Xzmp << endl;
        out << act_time << " " << x_sensor.at(0) << " " << x << " ";
        if (capture_point < 0.12 && capture_point > -0.12)
            out << 0 << " ";
        else
            out << 1 << " ";
        out << lin_vel << " " << g << " " << Zcom/100 << " " << Xzmp/100 << " " << capture_point << endl;
        out.close();
    }

//    void set(IVelocityControl *value, IVelocityControl *value0, IVelocityControl *value1,
//             PID *value2, PID *value3, BufferedPort<Bottle> *value4)
//    {
//        velRightLeg = value;
//        velLeftLeg = value0;
//        velTrunk = value1;
//        pidcontroller_ankle = value2;
//        pidcontroller_hip = value3;
//        readPort = value4;
//    }

    void set(PID *value, PID *value0, BufferedPort<Bottle> *value1)
    {
        pidcontroller_ankle = value;
        pidcontroller_hip = value0;
        readPort = value1;
    }

private:
    BufferedPort<Bottle> *readPort;
    PID *pidcontroller_ankle, *pidcontroller_hip;
    IVelocityControl *velRightLeg, *velLeftLeg, *velTrunk;

    int iteration;
    double capture_point;
    double x, y, z, x_robot, y_robot, z_robot, x_acc;
    double init_time, act_time, init_loop, act_loop;
    double Xzmp, Yzmp, actual_value, pid_output_ankle, pid_output_hip;
     //double setpoint;
    double maxZMP, minZMP, ZMPerror;
    double lin_vel, w;

    deque<double> x_sensor, y_sensor, z_sensor;
};

#endif
