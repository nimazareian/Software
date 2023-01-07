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
        _integral(0),
        _first_run(true)
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
    if (_first_run)
    {
        _pre_error = error;
        _first_run = false;
    }
    double derivative = (error - _pre_error) / _dt;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    std::map<std::string, double> plotjuggler_values;
    plotjuggler_values.insert({"PID_P_" + std::to_string(_Kd) + "out", Pout});
    plotjuggler_values.insert({"PID_D_" + std::to_string(_Kd) + "out", Dout});
    plotjuggler_values.insert({"PID_D_" + std::to_string(_Kd) + "derror", error - _pre_error});
    plotjuggler_values.insert({"PID_" + std::to_string(_Kd) + "output_no_clamp", output});

    // Restrict to max/min
    if (output > _max)
    {
        output = _max;
    }
    else if (output < _min)
    {
        output = _min;
    }

    // Save error to previous error
    _pre_error = error;

    plotjuggler_values.insert({"PID_" + std::to_string(_Kd) + "output", output});
    LOG(PLOTJUGGLER) << *createPlotJugglerValue(plotjuggler_values);
    return output;
}
