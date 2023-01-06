#include <iostream>
#include "pid.h"
#include "software/logger/logger.h"
#include "proto/message_translation/tbots_protobuf.h"

PID::PID(double dt, double max, double min, double Kp, double Kd, double Ki)
        : pimpl(dt, max, min, Kp, Kd, Ki)
{
}

double PID::calculate(double setpoint, double pv)
{
    return pimpl.calculate(setpoint, pv);
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double dt, double max, double min, double Kp, double Kd, double Ki) :
        _dt(dt),
        _max(max),
        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0)
{
    CHECK(_min < _max) << "PID min must be less than max";
}

double PIDImpl::calculate(double setpoint, double pv)
{
    // TODO: Fix the first output being super large!
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * _dt;
    double Iout = _Ki * _integral;

    // Derivative term
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    std::map<std::string, double> plotjuggler_values;
    plotjuggler_values.insert({"PID_P_" + std::to_string(_Kd) + "out", Pout});
    plotjuggler_values.insert({"PID_D_" + std::to_string(_Kd) + "out", Dout});
    plotjuggler_values.insert({"PID_D_" + std::to_string(_Kd) + "derror", error - _pre_error});
    LOG(PLOTJUGGLER) << *createPlotJugglerValue(plotjuggler_values);

    // Calculate total output
    double output = Pout + Iout + Dout;

    // Restrict to max/min
    if (output > _max)
    {
//        std::cout << "output: " << output << " -> max clamped: " << _max << std::endl;
        output = _max;
    }
    else if (output < _min)
    {
//        std::cout << "output: " << output << " -> min clamped: " << _min << std::endl;
        output = _min;
    }

    // Save error to previous error
    _pre_error = error;

    return output;
}
