#ifndef NULL_PID_H
#define PID_H

class PID
{
    private:
        float _kp;
        float _ki;
        float _kd;

        float _PID;
        float _dt;

        float _error;
        float _current; 

    public:
        PID(float kp, float ki, float kd, float dt, float target, float current);

        //pid constants
        float getKp(){return _kp;}
        float getKi(){return _ki;}
        float getKd(){return _kd;}

        //calculate pid values and return pid gain
        void calcPID(float target, float current, double dt, double error, double pre_ierror);
        float getPID(){return _PID;}        
};

#endif