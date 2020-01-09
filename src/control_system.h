#ifndef CTL_SYSTEM_SYSTEM_H
#define CTL_SYSTEM_SYSTEM_H
enum control_type{
PID = 0,
I_PD,
};
class  control_sys{
  public:
    control_sys();
    control_sys(double kp, double ki, double kd);
    control_sys(double kp, double ki, double kd, double refresh);
    control_sys(double kp, double ki, double kd, double refresh, int satLOW, int satHIGH, double reference);
    int update(int feedback);
    int saturate(int value);

    //getters and setters
    void set_reference(double reference);
    void set_saturation_range(int satLOW, int satHIGH);
    void set_all_param(double kp , double ki, double kd);
    void set_kp(double kp);
    void set_ki(double ki);
    void set_kd(double kd);

    double get_reference();
    int get_saturation_low();
    int get_saturation_high();
    double get_kp();
    double get_ki();
    double get_kd();



  private:
   int saturate();
   //refresh time of the system
   double _refresh_time{.1};
   //tuning parameters of system
   double _kp{1}, _kd{1}, _ki{1};
   //saturation range (this helpes the system have limit so that the output is controlled)
   int _saturationLOW{0}, _saturationHIGH{100};
   //preveious values to take deviative and integral value
   double _prev_derivative{0}, _prev_error{0};
   //accumulation of the error to get integral
   double integral;
   //reference of the system
   double _reference{0};

};


#endif
