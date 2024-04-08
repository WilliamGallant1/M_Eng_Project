
#include <ros/ros.h>
#include <array>
#include <iterator>
#include <stdexcept>
#include <vector>

typedef std::array<double,6> ind_coefs_; //6 coefficients needed only for each dimension
typedef std::array<double,6> state; //6 angles

class quintic_poly
{
    public:
    void init(const double& start_time, const state& start_state, const double& end_time, const state& end_state);
    void generate_powers(int size, const double& time, double *powers);
    void compute_coefficients(const double& start_pos, const double& end_pos, const double& duration, ind_coefs_ &coefficients);
    void sampling(double current_time,const ind_coefs_ &coefficients, double &position, double& velocity, double& acceleration);

    //private:
    std::array<ind_coefs_,6> coefs_; //array of array = matrix, size 6x6

};

void quintic_poly::init(const double& start_time, const state& start_state, const double& end_time, const state& end_state)
{
    //x(t) = a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5

    double duration = end_time - start_time;

    typedef typename std::array<ind_coefs_,6>::iterator Iterator;
    int count = 0;

    for(Iterator coefs_it = coefs_.begin(); coefs_it != coefs_.end(); ++coefs_it) //fills up my coefs_ 6x6 matrix
    {
        //should access start_state[0] first, then [1], then [2] to fill out coefs_
        compute_coefficients(start_state[count],end_state[count],duration,*coefs_it);
        count++;
    }
}

void quintic_poly::generate_powers(int size, const double& time, double *powers)
{
    powers[0] = 1.0;
    for (int i = 1; i <=size; i++)
    {
        powers[i] = powers[i-1]*time; //powers[1] = powers[0]*t ; powers[2] = powers[1]*t ; powers[3] = powers[2]*t ...
    }
}

void quintic_poly::compute_coefficients(const double& start_pos, const double& end_pos, const double& duration, ind_coefs_ &coefficients)
{
    double T[6];
    generate_powers(5,duration,T); //generates T[0],T[1],T[2],T[3],T[4],T[5]
    coefficients[0] = start_pos; //a0
    coefficients[1] = 0.0; //a1 -- assume no initial velocity
    coefficients[2] = 0.0; //a2 -- assume no initial acceleration
    coefficients[3] = (-20.0*start_pos + 20.0*end_pos)/ (2.0*T[3]); //a3
    coefficients[4] = (30.0*start_pos - 30.0*end_pos) / (2.0*T[4]); //a4
    coefficients[5] = (-12.0*start_pos + 12.0*end_pos) / (2.0*T[5]); //a5

}

void quintic_poly::sampling(double current_time,const ind_coefs_ &coefficients, double &position, double &velocity, double& acceleration)
{
    double T[6];
    generate_powers(5,current_time,T);
    //now that I have my coefficients, I can evaluate each time where my angles should be
    position = coefficients[0]*T[0] + coefficients[1]*T[1] + coefficients[2]*T[2] + coefficients[3]*T[3]
    + coefficients[4]*T[4] + coefficients[5]*T[5]; //a0 + a1*t + a2*t^2 + a3*t^3 + a4*t^4 + a5*t^5
    velocity = coefficients[1]*T[0] + 2.0*coefficients[2]*T[1] + 3.0*coefficients[3]*T[2]
    + 4.0*coefficients[4]*T[3] + 5.0*coefficients[5]*T[4]; //a1 + 2*a2*t + 3*a3*t^2 + 4*a4*t^3 + 5*a5*t^4
    acceleration = 2.0*coefficients[2] + 6.0*coefficients[3]*T[1] + 12.0*coefficients[4]*T[2]
    + 20.0*coefficients[5]*T[3]; //2*a2 + 6*a3*t + 12*a4*t^2 + 20*a5*t^3
}