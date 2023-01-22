#include <iostream>
#include "pid.h"
#include "software/logger/logger.h"
#include "proto/message_translation/tbots_protobuf.h"

PID::PID(double max_delta_s, double Kp, double Kd, double Ki)
        : pimpl(max_delta_s, Kp, Kd, Ki)
{
}

double PID::calculate(double setpoint, double pv, double dt_s, const std::string& name)
{
    return pimpl.calculate(setpoint, pv, dt_s, name);
}


/**
 * Implementation
 */
PIDImpl::PIDImpl(double max_delta_s, double Kp, double Kd, double Ki) :
//        _dt(dt),
        _max_delta_s(max_delta_s),
//        _min(min),
        _Kp(Kp),
        _Kd(Kd),
        _Ki(Ki),
        _pre_error(0),
        _integral(0),
        _first_run(true)
{
}

double PIDImpl::calculate(double setpoint, double pv, double dt_s, const std::string& name)
{
    // TODO: Fix the first output being super large!
    // Calculate error
    double error = setpoint - pv;

    // Proportional term
    double Pout = _Kp * error;

    // Integral term
    _integral += error * dt_s;
    double Iout = _Ki * _integral;

    // Derivative term
    if (_first_run)
    {
        _pre_error = error;
        _first_run = false;
    }
    double derivative = (error - _pre_error) / dt_s;
    double Dout = _Kd * derivative;

    // Calculate total output
    double output = Pout + Iout + Dout;

    std::map<std::string, double> plotjuggler_values;
    plotjuggler_values.insert({"PID_P_" + name + "out", Pout});
    plotjuggler_values.insert({"PID_D_" + name + "out", Dout});
    plotjuggler_values.insert({"PID_D_" + name + "derror", error - _pre_error});
    plotjuggler_values.insert({"PID_" + name + "output_no_clamp", output});

    // Restrict to max/min
    output = std::clamp(output, -_max_delta_s * dt_s, _max_delta_s * dt_s);

    // Save error to previous error
    _pre_error = error;

    plotjuggler_values.insert({"PID_" + name + "output", output});
    LOG(PLOTJUGGLER) << *createPlotJugglerValue(plotjuggler_values);
    return output;
}
