#ifndef _PID_H_
#define _PID_H_

class PID
{
    public:
        // dt -  loop interval time (s)
        // max - maximum value of manipulated variable
        // min - minimum value of manipulated variable
        // Kp -  proportional gain
        // Kd -  derivative gain
        // Ki -  Integral gain
        PID( double dt, double max, double min, double Kp, double Kd, double Ki );

        // Returns the manipulated variable given a setpoint and the actual value
        double calculate( double setpoint, double actual_value );

        ~PID();

    private:
        double _dt;
        double _max;
        double _min;
        double _Kp;
        double _Kd;
        double _Ki;
        double _pre_error;
        double _integral;
};

#endif
