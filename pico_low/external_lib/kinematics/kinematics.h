#define LINO_BASE MECANUM // Mecanum drive robot

#define MOTOR_MAX_RPM 140 // motor's max RPM
#define MAX_RPM_RATIO                                                          \
  0.85 // max RPM allowed for each MAX_RPM_ALLOWED = MOTOR_MAX_RPM *
       // MAX_RPM_RATIO

// Для бесщеточного мотора с встроенным контроллером
#define BRUSHLESS_MOTORS
#define MAX_RPM 107 // Максимальные обороты мотора (в об/мин)

// Для щеточного мотора
// #define BRUSH_MOTORS
// #define MOTOR_OPERATING_VOLTAGE 24          // motor's operating voltage
// (used to calculate max RPM) #define MOTOR_POWER_MAX_VOLTAGE 12          //
// max voltage of the motor's power source (used to calculate max RPM) #define
// MOTOR_POWER_MEASURED_VOLTAGE 12     // current voltage reading of the power
// connected to the motor (used for calibration)

#define COUNTS_PER_REV1 144000  // wheel1 encoder's no of ticks per rev
#define COUNTS_PER_REV2 144000  // wheel2 encoder's no of ticks per rev
#define COUNTS_PER_REV3 144000  // wheel3 encoder's no of ticks per rev
#define COUNTS_PER_REV4 144000  // wheel4 encoder's no of ticks per rev
#define WHEEL_DIAMETER 0.083    // wheel's diameter in meters
#define LR_WHEELS_DISTANCE 0.22 // distance between left and right wheels
// #define PWM_BITS 10                          // PWM Resolution of the
// microcontroller #define PWM_FREQUENCY 20000                 // PWM Frequency

#ifndef KINEMATICS_H
#define KINEMATICS_H

// #include "Arduino.h"

#define RPM_TO_RPS 1 / 60

// Про классы в С++ - http://cppstudio.com/post/439/   ,
// https://learntutorials.net/ru/cplusplus/topic/508/классы---структуры

double constrain(double x, double min, double max);

class Kinematics {
public:
  enum base { DIFFERENTIAL_DRIVE, SKID_STEER, MECANUM };

  base base_platform_;

  struct rpm {
    double motor1;
    double motor2;
    double motor3;
    double motor4;
  }; // motor;

  struct velocities {
    double linear_x;
    double linear_y;
    double angular_z;
  };

  struct pwm {
    int motor1;
    int motor2;
    int motor3;
    int motor4;
  };
  Kinematics(base robot_base, int motor_max_rpm, double max_rpm_ratio,
#ifdef BRUSH_MOTORS
             double motor_operating_voltage, double motor_power_max_voltage,
#endif
#ifdef BRUSHLESS_MOTORS
             double max_rpm,
#endif
             double wheel_diameter, double wheels_y_distance);
  [[nodiscard]] velocities getVelocities(double rpm1, double rpm2, double rpm3,
                                         double rpm4) const;
  rpm getRPM(double linear_x, double linear_y, double angular_z);
  [[nodiscard]] double getMaxRPM() const;

private:
  [[nodiscard]] rpm calculateRPM(double linear_x, double linear_y,
                                 double angular_z) const;
  static int getTotalWheels(base robot_base);

  double max_rpm_;
  double wheels_y_distance_;
  double pwm_res_{};
  double wheel_circumference_;
  int total_wheels_;
};

#endif
