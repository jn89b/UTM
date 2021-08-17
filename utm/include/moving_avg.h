#ifndef NULL_MOVING_AVG_H
#define MOVING_AVG_H

#include <queue>

class MovingAverage
{
    private:
        
        double kf_x;
        double kf_y;
        double kf_mag;
        
        double kfx_avg;
        double kfy_avg;
                
        double running_total;
        
        const int window_size = 25;
        std::queue<int> buffer;
        double _avg_mag;

    public:
        MovingAverage(double x, double y); 

        void init_vals(double x, double y);
        double compute_avg(double input);

        //double compute_avg(double x);
        //double compute_avg(double y);        

        double get_kfx(){return kfx_avg;}
        double get_kfy(){return kfy_avg;}

        double compute_mag(double x, double y);

        double get_avg_mag(){return _avg_mag;}   
};

#endif