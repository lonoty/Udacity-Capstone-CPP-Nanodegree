#include "control_system.h"

control_sys::control_sys(double kp, double ki, double kd ) : _kp(kp), _ki(ki) ,_kd(kd){}
control_sys::control_sys(double kp, double ki, double kd, double refresh) : _kp(kp), _ki(ki) ,_kd(kd),  _refresh_time(refresh){}
control_sys::control_sys(double kp, double ki, double kd, double refresh, int satLOW, int satHIGH, double reference) : _kp(kp), _ki(ki) ,_kd(kd), _refresh_time(refresh), _saturationHIGH(satHIGH), _saturationLOW(satLOW), _reference(reference) {}
int control_sys::update(int feedback)
{
//get the error
double error = _reference - feedback;
//get the integral
integral = integral + (error * _refresh_time);
//get the derivative
double derivative = (error - _prev_error)/ _refresh_time;
// get the output of the control_sys
double output = _kp * error + _ki*integral + _kd*derivative;
// set the actual errors to the previous ones
_prev_error = error;
_prev_derivative = derivative;
return saturate((int)output);
}

int control_sys::saturate(int value)
{
  if(value > _saturationHIGH){
    return _saturationHIGH;
  }
  if(value < _saturationLOW){
    return _saturationLOW;
  }
  return value;
}

void control_sys::set_all_param(double kp, double ki, double kd) 
{
  _kp = kp;
  _ki = ki;
  _kd = kd;
}

void control_sys::set_kd(double kd)
{
_kd = kd;
}

void control_sys::set_kp(double kp)
{
_kp = kp;
}

void control_sys::set_ki(double ki)
{
_kp = ki;
}

void control_sys::set_reference(double reference)
{
_reference = reference;
}

void control_sys::set_saturation_range(int satLOW, int satHIGH)
{
_saturationHIGH = satHIGH;
_saturationLOW = satLOW;
}

double control_sys::get_reference()
{
return _reference;
}

int control_sys::get_saturation_low()
{
return _saturationLOW;
}

int control_sys::get_saturation_high()
{
return _saturationHIGH;
}

double control_sys::get_kp()
{
return _kp;
}

double control_sys::get_ki()
{
return _ki;
}

double control_sys::get_kd()
{
return _kd;
}
