#define PID_H
 
#include "time_utils.h"
#include "foc_utils.h"
 
/**
 *  PID controller class
 */
class PosPidController
{
public:
    /**
     *  
     * @param P - Proportional gain 
     * @param I - Integral gain
     * @param D - Derivative gain 
     * @param ramp - Maximum speed of change of the output value
     * @param limit - Maximum output value
     */
    PosPidController(float P, float I, float D, float ramp, float limit);
    ~PosPidController() = default;
 
    float operator() (float error);
 
    float P; //!< Proportional gain 
    float I; //!< Integral gain 
    float D; //!< Derivative gain 
    float proportional = 0.0;
    float integral = 0.0;
    float derivative = 0.0;
    float integral_prev; //!< last integral component value
    float output_ramp; //!< Maximum speed of change of the output value
    float limit; //!< Maximum output value
    float error_prev; //!< last tracking error value
    unsigned long timestamp_prev; //!< Last execution timestamp
    float output_prev;  //!< last pid output value
};
