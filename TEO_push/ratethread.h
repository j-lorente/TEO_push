#ifndef _ratethread_H_
#define _ratethread_H_

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {    //Conversion to [ms]
        x_sensor.resize(samples);
        y_sensor.resize(samples);
        z_sensor.resize(samples);
        iteration = 1;
    }

    virtual void threadRelease()
    {
        if (plane.compare("sagittal") == 0) cout << "RATETHREAD: Sagittal thread closed." << endl;
        else cout << "RATETHREAD: Frontal thread closed." << endl;
    }

    void run()
    {
        cout << "it: " << iteration << endl;
        getInitialTime();

        //READ SENSOR
        Bottle *input = readPort->read();
        if (input==NULL)
        {
            cout << "[error] No data from sensor..." << endl;
            return;
        }
        acc_x = input->get(3).asDouble(); //Linear acceleration in X [m/s^2]
        x_sensor.push_front(acc_x);
        x_sensor.pop_back();
        acc_y = input->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
        y_sensor.push_front(acc_y);
        y_sensor.pop_back();
        acc_z = input->get(5).asDouble(); //Linear acceleration in Z [m/s^2]
        z_sensor.push_front(acc_z);
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

        //ZERO MOMENT POINT COMPUTATION
        Xzmp = Xcom - (Zcom / z_robot) * x_robot; //ZMP X coordinate [cm]
        Yzmp = Ycom - (Zcom / z_robot) * y_robot; //ZMP Y coordinate [cm]

        if (plane.compare("sagittal") == 0)     //SAGITTAL STRATEGY
        {
            //DETERMINING STRATEGY
            lin_vel = x_sensor.at(0) * dt;
            w = sqrt(g / (Zcom / 100));
            capture_point = (lin_vel / w) + (Xzmp / 100);

            //PID
            actual_value = Xzmp;
            if (iteration==1){ setpoint = Xzmp; } //Get initial position as setpoint [cm]
            pid_output_ankle = pidcontroller_ankle->calculate(setpoint, actual_value);
            pid_output_hip = pidcontroller_hip->calculate(setpoint, actual_value);

            //Get encoder (Needed because trunk has relative encoders)
            getTrunkEncoders();

            //CONTROL
            if (capture_point < 0.12 && capture_point > -0.12)      ///Ankle strategy
            {
                velLeftLeg->velocityMove(4, -pid_output_ankle);     //Left Ankle
                Time::delay(0.01);
                velRightLeg->velocityMove(4, -pid_output_ankle);    //Right Ankle
                Time::delay(0.01);
                posTrunk->positionMove(1, initial_encoder);         //Hip
                Time::delay(0.01);
            }
            else                                                    ///Hip strategy
            {
                velLeftLeg->velocityMove(4, -pid_output_ankle);     //Left Ankle
                Time::delay(0.01);
                velRightLeg->velocityMove(4, -pid_output_ankle);    //Right Ankle
                Time::delay(0.01);
                //velTrunk->velocityMove(1, pid_output_hip);        //Hip
                position = encoders[1] + pid_output_hip * dt;
                posTrunk->positionMove(1, position);
                Time::delay(0.01);
            }
        }
        else //FRONTAL STRATEGY
        {
            //PID
            actual_value = Yzmp;
            if (iteration==1) { setpoint = Yzmp; } //Get initial position as setpoint [cm]
            pid_output_ankle = pidcontroller_ankle->calculate(setpoint, actual_value);

            //CONTROL
            velRightLeg->velocityMove(5, -pid_output_ankle);      //Right Hip
            Time::delay(0.01);
            velRightLeg->velocityMove(1, -pid_output_ankle);      //Right Ankle
            Time::delay(0.01);
            velLeftLeg->velocityMove(5, pid_output_ankle);        //Left Hip
            Time::delay(0.01);
            velLeftLeg->velocityMove(1, pid_output_ankle);        //Right Ankle
            Time::delay(0.01);
        }

        saveInFile();

        getCurrentTime();

        printData();

        cout << endl << "Press ENTER to exit..." << endl;
        cout << "*******************************" << endl << endl;

        iteration++;
    }

    void getInitialTime()
    {
        if (iteration==1){init_time = Time::now();}
        init_loop = Time::now();
        it_time = init_loop - it_prev;
        it_prev = init_loop;
    }

    void getCurrentTime()
    {
        act_time = Time::now() - init_time;
        act_loop = Time::now() - init_loop;
    }

    void getTrunkEncoders()
    {
        //Get joints
        posTrunk->getAxes(&joints);
        encoders.resize(joints);

        //Get encoders
        if (iteration == 1)
        {
            cout << "Reading encoders..." << endl;
            while ( ! encTrunk->getEncoders(encoders.data()) ){}
            initial_encoder = encoders[1];
            cout << "Trunk encoder [" << 1 << "]: " << initial_encoder << endl << endl;
        }
        else { encTrunk->getEncoders(encoders.data()); }
    }

    void printData()
    {
        if (plane.compare("sagittal") == 0)
        {
            cout << "Acceleraction in X: " << acc_x << " m/s²" << endl;
            cout << "Acceleraction in Y: " << acc_y << " m/s²" << endl;
            cout << "Acceleraction in Z: " << acc_z << " m/s²" << endl << endl;
            cout << "PID output hip: " << pid_output_hip << " deg/s" << endl;
            cout << "Hip position command: " << position << " deg" << endl << endl;
            cout << "Iteration time: " << act_loop*1000 << " ms" << endl;
            cout << "Time between iterations: " << it_time*1000 << " ms" << endl;
            cout << "Absolute time: " << int(act_time) << " s" << endl;
        }
    }

    void saveInFile()
    {
        ofstream out;

        if (plane.compare("sagittal") == 0)
        {
            if (iteration==1) {out.open("sagittal_data.txt",ios::trunc);} //The first time deletes previous content
            else {out.open("sagittal_data.txt",ios::app);} //The following times appends data to the file
        }
        else
        {
            if (iteration==1) {out.open("frontal_data.txt",ios::trunc);} //The first time deletes previous content
            else {out.open("frontal_data.txt",ios::app);} //The following times appends data to the file
        }

        out << act_time << " ";
        out << actual_value << " ";
        out << setpoint << " ";

        if (plane.compare("sagittal") == 0)
        {
            if(capture_point < 0.12 && capture_point > -0.12)
                out << 0 << endl;
            else
                out << 5 << endl;
        }
        else out << endl;

        out.close();
    }

    void set(std::string _plane, BufferedPort<Bottle> *_readPort, IVelocityControl *_velRightLeg,
             IVelocityControl *_velLeftLeg, IVelocityControl *_velTrunk, IPositionControl *_posTrunk,
             IEncoders *_encTrunk, PID *_pidcontroller_ankle, PID *_pidcontroller_hip)
    {
        plane = _plane;
        velRightLeg = _velRightLeg;
        velLeftLeg = _velLeftLeg;
        velTrunk = _velTrunk;
        posTrunk = _posTrunk;
        pidcontroller_ankle = _pidcontroller_ankle;
        pidcontroller_hip = _pidcontroller_hip;
        readPort = _readPort;
        encTrunk = _encTrunk;
    }

private:
    BufferedPort<Bottle> *readPort;
    PID *pidcontroller_ankle, *pidcontroller_hip;
    IVelocityControl *velTrunk, *velRightLeg, *velLeftLeg;
    IPositionControl *posTrunk;
    IEncoders *encTrunk;

    double acc_x, acc_y, acc_z;

    int iteration, joints;
    std::string plane;
    double x, y, z, x_robot, y_robot, z_robot, Xzmp, Yzmp;
    double init_time, act_time, init_loop, act_loop, it_prev, it_time;
    double actual_value, setpoint, pid_output_ankle, pid_output_hip;
    double capture_point, lin_vel, w, initial_encoder;

    vector<double> encoders;

    deque<double> x_sensor, y_sensor, z_sensor;

    double position;
};

#endif
