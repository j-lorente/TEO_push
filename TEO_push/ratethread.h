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
             cout << "[error] No data from sensor..." << endl;
             return;
        }
        x_sensor.push_front(input->get(3).asDouble()); //Linear acceleration in X [m/s^2]
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

        if (plane == 0) //SAGITTAL PLANE
        {
            //DETERMINING STRATEGY
            lin_vel = x_sensor.at(0) * dt;
            w = sqrt(g / (Zcom / 100));
            capture_point = (lin_vel / w) + (Xzmp / 100);

            //PID
            actual_value = Xzmp;
            if (iteration==1){ setpoint = Xzmp; } //Get initial position as setpoint [cm]
            pid_output_ankle = pidcontroller_ankle->calculate(setpoint, actual_value);
            //pid_output_hip = pidcontroller_hip->calculate(setpoint, actual_value);

            getTrunkEncoders(); //Get encoders (Needed because trunk has relative encoders)

            //JOINTS CONTROL
            if (capture_point < 0.12 && capture_point > -0.12) //Ankle strategy
            {
                velRightLeg->velocityMove(4, -pid_output_ankle); //Right Leg
                velLeftLeg->velocityMove(4, -pid_output_ankle); //Left Leg
                posTrunk->positionMove(1,initial_encoder); //Hip
            }
            else //Hip strategy
            {
                velRightLeg->velocityMove(4, -pid_output_ankle); //Right Leg
                velLeftLeg->velocityMove(4, -pid_output_ankle); //Left Leg
                //velTrunk->velocityMove(1, pid_output_hip); //Hip
                //posTrunk->positionMove(1,pid_output_hip);
            }
        }
        else //FRONTAL PLANE
        {
            //PID
            actual_value = Yzmp;
            if (iteration==1) { setpoint = Yzmp; } //Get initial position as setpoint [cm]
            pid_output_ankle = pidcontroller_ankle->calculate(setpoint, actual_value);

            //JOINTS CONTROL
            velRightLeg->velocityMove(5, -pid_output_ankle);
            velRightLeg->velocityMove(1, -pid_output_ankle);
            velLeftLeg->velocityMove(5, pid_output_ankle);
            velLeftLeg->velocityMove(1, pid_output_ankle);
        }

        //saveInFile(); //Save relevant data in external file for posterior plotting

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
            while ( ! encTrunk->getEncoders(encoders.data())){}
            for(int i=0;i<joints;i++){
                cout << "Trunk encoder [" << i << "]: " << encoders[i] << endl;}
            initial_encoder = encoders[1];
        }
        else
        {
            if (! encTrunk->getEncoders(encoders.data())){
                cout << "[error] Problem acquiring robot trunk encoders." << endl;}
            else
            {
                for(int i=0;i<joints;i++){
                    cout << "Trunk encoder [" << i << "]: " << encoders[i] << endl;}
            }
        }
        cout << endl;
    }

    void printData()
    {
        cout << "Iteration time: " << act_loop*1000 << " ms" << endl;
        cout << "Time between iterations: " << it_time*1000 << " ms" << endl;
        cout << "Absolute time: " << int(act_time) << " s" << endl;
    }

//    void saveInFile()
//    {
//        ofstream out;
//        if (iteration==1) {out.open("data.txt",ios::trunc);}    //The first time deletes previous content
//        else {out.open("data.txt",ios::app);}                   //The following times appends data to the file
//        out << act_time << " ";
//        out << x_acc << " ";
//        out << x << " ";
//        out << setpoint_x << " ";
//        out << Xzmp << " ";
//        if(capture_point < 0.12 && capture_point > -0.12)
//            out << 0 << endl;
//        else
//            out << 5 << endl;
//        out.close();
//    }

    void set(int value0, IVelocityControl *value1, IVelocityControl *value2, IVelocityControl *value3, IPositionControl *value4,
             PID *value5, PID *value6, BufferedPort<Bottle> *value7, IEncoders *value8)
    {
        plane = value0;
        velRightLeg = value1;
        velLeftLeg = value2;
        velTrunk = value3;
        posTrunk = value4;
        pidcontroller_ankle = value5;
        pidcontroller_hip = value6;
        readPort = value7;
        encTrunk = value8;
    }

private:
    BufferedPort<Bottle> *readPort;
    PID *pidcontroller_ankle, *pidcontroller_hip;
    IVelocityControl *velTrunk, *velRightLeg, *velLeftLeg;
    IPositionControl *posTrunk;
    IEncoders *encTrunk;

    int iteration, joints, plane;
    double x, y, z, x_robot, y_robot, z_robot, Xzmp, Yzmp;
    double init_time, act_time, init_loop, act_loop, it_prev, it_time;
    double actual_value, setpoint, pid_output_ankle, pid_output_hip;
    double capture_point, lin_vel, w, initial_encoder;

    vector<double> encoders;

    deque<double> x_sensor, y_sensor, z_sensor;
};

#endif
