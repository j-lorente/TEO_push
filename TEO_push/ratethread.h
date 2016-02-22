#ifndef _ratethread_H_
#define _ratethread_H_

#include <sys/timeb.h>

class MyRateThread : public RateThread
{
public:
    MyRateThread() : RateThread(dt*1000.0) {} // Conversion to [ms]

    void run()
    {

        //GET INITIAL TIME
        if (first_iteration==0)
        {
            init_time = getMilliCount();
        }
        init_loop = getMilliCount();

       //RESET OF AVERAGE VARIABLES
       x_sensor = 0.0;
       y_sensor = 0.0;
       z_sensor = 0.0;

       //READ SENSOR
       for(int i=0;i<samples;i++)
       {
           Bottle *input = readPort->read();
           if (input==NULL)
           {
                printf("[error] No data from sensor...\n");
                return;
            }
            x_sensor = x_sensor + input->get(3).asDouble(); //Linear acceleration in X [m/s^2]
            y_sensor = y_sensor + input->get(4).asDouble(); //Linear acceleration in Y [m/s^2]
            z_sensor = z_sensor + input->get(5).asDouble(); //Linear acceleration in Z [m/s^2]
       }

        //Low-pass Filter (Average)
        x = x_sensor / samples;
        y = y_sensor / samples;
        z = z_sensor / samples;

        //CONVERSION FROM SENSOR COORDINATES TO ROBOT COORDINATES
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

        //CALCULATE ZMP ERROR
        ZMPerror = getError();
        cout << "ZMP(x) error: +/- " << ZMPerror << " cm" << endl;

        //PID
        actual_value = Xzmp;
        if (first_iteration==0)
        {
            setpoint = Xzmp; //Get initial position as setpoint [cm]
        }
        pid_output = - pidcontroller->calculate(setpoint, actual_value);
        printf("Setpoint: %f\n", setpoint);
        printf("PID output: %f\n", pid_output);

        //SEND MOTOR VELOCITY THROUGH YARP
//        velLeftLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].
//        velRightLeg->velocityMove(4, pid_output);  //Motor number. Velocity [deg/s].

        //GET CURRENT TIME
        act_time = getMilliSpan(init_time);
        act_loop = getMilliSpan(init_loop);
        cout << "Absolute time: " << act_time << " ms" << endl;
        cout << "Loop time: " << act_loop << " ms" << endl;

        //SAVE DATA IN EXTERNAL FILE
        saveInFile(first_iteration, act_time, Xzmp, setpoint);

        //NOT FIRST ZMP ANYMORE
        first_iteration = 1;

        printf("\nPress ENTER to exit...\n\n");
        cout << "*******************************" << endl << endl;

    }

    double getError(){
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

    void saveInFile(int first_iteration, int act_time, double Xzmp, double setpoint){
        ofstream out;
        if (first_iteration==0) {out.open("data.txt",ios::trunc);}    //The first time deletes previous content
        else {out.open("data.txt",ios::app);}                   //The following times appends data to the file
        out << act_time << " " << Xzmp << " " << setpoint << endl;
        out.close();
    }

    int getMilliCount(){
        timeb tb;
        ftime(&tb);
        int nCount = tb.millitm + (tb.time & 0xfffff) * 1000;
        return nCount;
    }

    int getMilliSpan(int nTimeStart){
        int nSpan = getMilliCount() - nTimeStart;
        if(nSpan < 0)
            nSpan += 0x100000 * 1000;
        return nSpan;
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
        first_iteration = 0;
    }

private:
    BufferedPort<Bottle> *readPort;                                         //YARP port for reading from sensor
    PID *pidcontroller;                                                     //PID controller
    IVelocityControl *velRightLeg, *velLeftLeg;                             //Velocity controllers
    int first_iteration;
    double Xzmp, Yzmp, actual_value, setpoint, pid_output;
    double x, y, z, x_sensor, y_sensor, z_sensor, x_robot, y_robot, z_robot;
    int init_time, act_time, init_loop, act_loop;
    double maxZMP, minZMP, ZMPerror;
};

#endif
