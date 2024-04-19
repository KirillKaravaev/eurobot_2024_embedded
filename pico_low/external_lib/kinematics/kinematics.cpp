#include "kinematics.h"
#include <cmath>

// Функция ограничения области допустимых значений. Возвращает всегда a<x<b
double constrain(double x, double min, double max) {
  if (x < min)
    x = min;
  else if (x > max)
    x = max;
  return x;
}

Kinematics::Kinematics(base robot_base, int motor_max_rpm, double max_rpm_ratio,
#ifdef BRUSH_MOTORS
                       double motor_operating_voltage,
                       double motor_power_max_voltage,
#endif
#ifdef BRUSHLESS_MOTORS
                       double max_rpm,
#endif
                       double wheel_diameter, double wheels_y_distance)
    : base_platform_(robot_base), wheels_y_distance_(wheels_y_distance),
      wheel_circumference_(3.1415 * wheel_diameter),
      total_wheels_(getTotalWheels(robot_base)) {
#ifdef BRUSH_MOTORS
  motor_power_max_voltage =
      constrain(motor_power_max_voltage, 0, motor_operating_voltage);
  max_rpm_ =
      ((motor_power_max_voltage / motor_operating_voltage) * motor_max_rpm) *
      max_rpm_ratio;
#endif

#ifdef BRUSHLESS_MOTORS
  max_rpm_ = MAX_RPM;
#endif
}

Kinematics::rpm Kinematics::calculateRPM(double linear_x, double linear_y,
                                         double angular_z) const {

  double tangential_vel = angular_z * (wheels_y_distance_ / 2.0);

  // convert m/s to m/min
  double linear_vel_x_mins = linear_x * 60.0;
  double linear_vel_y_mins = linear_y * 60.0;
  // convert rad/s to rad/min
  double tangential_vel_mins = tangential_vel * 60.0;

  double x_rpm = linear_vel_x_mins / wheel_circumference_;
  double y_rpm = linear_vel_y_mins / wheel_circumference_;
  double tan_rpm = tangential_vel_mins / wheel_circumference_;

  double a_x_rpm = std::fabs(x_rpm);
  double a_y_rpm = std::fabs(y_rpm);
  double a_tan_rpm = std::fabs(tan_rpm);

  double xy_sum = a_x_rpm + a_y_rpm;
  double xtan_sum = a_x_rpm + a_tan_rpm;

  // calculate the scale value how much each target velocity
  // must be scaled down in such cases where the total required RPM
  // is more than the motor's max RPM
  // this is to ensure that the required motion is achieved just with slower
  // speed
  if (xy_sum >= max_rpm_ && angular_z == 0) {
    double vel_scaler = max_rpm_ / xy_sum;

    x_rpm *= vel_scaler;
    y_rpm *= vel_scaler;
  }

  else if (xtan_sum >= max_rpm_ && linear_y == 0) {
    double vel_scaler = max_rpm_ / xtan_sum;

    x_rpm *= vel_scaler;
    tan_rpm *= vel_scaler;
  }

  Kinematics::rpm rpm{};
  // Mecanum wheels
  /*
      //calculate for the target motor RPM and direction
      //front-left motor
      rpm.motor1 = x_rpm - y_rpm - tan_rpm;
      rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

      //front-right motor
      rpm.motor2 = x_rpm + y_rpm + tan_rpm;
      rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

      //rear-left motor
      rpm.motor3 = x_rpm + y_rpm - tan_rpm;
      rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

      //rear-right motor
      rpm.motor4 = x_rpm - y_rpm + tan_rpm;
      rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);
  */
  // Omni wheels 4WD (square frame)
  // the coordinate system of the robot frame
  //                  y
  //                  / |\
//                  |
  //         (2) //----|----\\ (1)
  //             :     |     :
  //      _______:_____|_____:_______\ x
  //             :     |     :       /
  //             :     |     :
  //         (3) \\----|----// (4)
  //                   |
  //                   |
  // for more information about kinematics model -
  // https://www.youtube.com/watch?v=-wzl8XJopgg

  double scaler = sqrt(2) / 2; // the coordinate system of the robot frame
  //                                                                  // y
  // calculate for the target motor RPM and direction                  // /|\  1
  // motor                                                           // |
  rpm.motor1 =
      scaler * (-x_rpm + y_rpm) + tan_rpm; //        (2) //----|----\\ (1)
  rpm.motor1 =
      constrain(rpm.motor1, -max_rpm_, max_rpm_); //            :     |     :
  //                                                                  //
  //                                                                  _______:_____|_____:_______\
  //                                                                  x
  // 2 motor                                                           // : | :
  // /
  rpm.motor2 = scaler * (-x_rpm - y_rpm) + tan_rpm; //            :     |     :
  rpm.motor2 = constrain(rpm.motor2, -max_rpm_,
                         max_rpm_); //        (3) \\----|----// (4)
  //                                                                  // |
  // 3 motor                                                           // |
  rpm.motor3 = scaler * (x_rpm - y_rpm) +
               tan_rpm; // for more information about kinematics model -
                        // https://www.youtube.com/watch?v=-wzl8XJopgg
  rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

  // 4 motor
  rpm.motor4 = scaler * (x_rpm + y_rpm) + tan_rpm;
  rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

  return rpm;
}

Kinematics::rpm Kinematics::getRPM(double linear_x, double linear_y,
                                   double angular_z) {
  if (base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER) {
    linear_y = 0;
  }

  return calculateRPM(linear_x, linear_y, angular_z);
  ;
}

Kinematics::velocities Kinematics::getVelocities(double rpm1, double rpm2,
                                                 double rpm3,
                                                 double rpm4) const {
  Kinematics::velocities vel{};
  double average_rps_x;
  double average_rps_y;
  double average_rps_a;

  if (base_platform_ == DIFFERENTIAL_DRIVE) {
    rpm3 = 0.0;
    rpm4 = 0.0;
  }

  // convert average revolutions per minute to revolutions per second
  average_rps_x =
      ((double)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_) / 60.0; // RPM
  vel.linear_x = average_rps_x * wheel_circumference_;              // m/s

  // convert average revolutions per minute in y axis to revolutions per second
  average_rps_y =
      ((double)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_) / 60.0; // RPM
  if (base_platform_ == MECANUM)
    vel.linear_y = average_rps_y * wheel_circumference_; // m/s
  else
    vel.linear_y = 0;

  // convert average revolutions per minute to revolutions per second
  average_rps_a = ((double)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_) / 60.0;
  vel.angular_z = (average_rps_a * wheel_circumference_) /
                  (wheels_y_distance_ / 2.0); //  rad/s

  return vel;
}

int Kinematics::getTotalWheels(base robot_base) {
  switch (robot_base) {
  case DIFFERENTIAL_DRIVE:
    return 2;
  case SKID_STEER:
    return 4;
  case MECANUM:
    return 4;
  default:
    return 2;
  }
}

double Kinematics::getMaxRPM() const { return max_rpm_; }

/*
#include "kinematics.h"

double constrain(double x, double min, double max){
    if(x < min) x = min;
    else if(x > max) x = max;
    return x;
}

double PI = 3.14159;

Kinematics(base robot_base, int motor_max_rpm, double max_rpm_ratio,
                       double motor_operating_voltage, double
motor_power_max_voltage, double wheel_diameter, double wheels_y_distance)
{
    base_platform_ = robot_base;
    wheels_y_distance_ = wheels_y_distance;
    wheel_circumference_ = PI * wheel_diameter;
    total_wheels_ = getTotalWheels(robot_base);

    motor_power_max_voltage = constrain(motor_power_max_voltage, 0,
motor_operating_voltage); max_rpm_ =  ((motor_power_max_voltage /
motor_operating_voltage) * motor_max_rpm) * max_rpm_ratio; }Kinematics;


rpm->Kinematics calculateRPM(double linear_x, double linear_y, double angular_z)
{
    double tangential_vel = angular_z * (wheels_y_distance_ / 2.0);

    double linear_vel_x_mins = linear_x * 60.0;
    double linear_vel_y_mins = linear_y * 60.0;

    double tangential_vel_mins = tangential_vel * 60.0;
    double x_rpm = linear_vel_x_mins / wheel_circumference_;
    double y_rpm = linear_vel_y_mins / wheel_circumference_;
    double tan_rpm = tangential_vel_mins / wheel_circumference_;
    double a_x_rpm = fabs(x_rpm);
    double a_y_rpm = fabs(y_rpm);
    double a_tan_rpm = fabs(tan_rpm);
    double xy_sum = a_x_rpm + a_y_rpm;
    double xtan_sum = a_x_rpm + a_tan_rpm;

    if(xy_sum >= max_rpm_ && angular_z == 0)
    {
        double vel_scaler = max_rpm_ / xy_sum;
        x_rpm *= vel_scaler;
        y_rpm *= vel_scaler;
    }
    else if(xtan_sum >= max_rpm_ && linear_y == 0)
    {
        double vel_scaler = max_rpm_ / xtan_sum;
        x_rpm *= vel_scaler;
        tan_rpm *= vel_scaler;
    }

    rpm rpm;
    rpm.motor1 = x_rpm - y_rpm - tan_rpm;
    rpm.motor1 = constrain(rpm.motor1, -max_rpm_, max_rpm_);

    rpm.motor2 = x_rpm + y_rpm + tan_rpm;
    rpm.motor2 = constrain(rpm.motor2, -max_rpm_, max_rpm_);

    rpm.motor3 = x_rpm + y_rpm - tan_rpm;
    rpm.motor3 = constrain(rpm.motor3, -max_rpm_, max_rpm_);

    rpm.motor4 = x_rpm - y_rpm + tan_rpm;
    rpm.motor4 = constrain(rpm.motor4, -max_rpm_, max_rpm_);

    return rpm;
}

rpm Kinematics->getRPM(double linear_x, double linear_y, double angular_z)
{
    if(base_platform_ == DIFFERENTIAL_DRIVE || base_platform_ == SKID_STEER)
    {
        linear_y = 0;
    }
    return calculateRPM(linear_x, linear_y, angular_z);
}

velocities Kinematics->getVelocities(double rpm1, double rpm2, double rpm3,
double rpm4)
{
    velocities vel;
    double average_rps_x;
    double average_rps_y;
    double average_rps_a;

    if(base_platform_ == DIFFERENTIAL_DRIVE)
    {
        rpm3 = 0.0;
        rpm4 = 0.0;
    }

    average_rps_x = ((double)(rpm1 + rpm2 + rpm3 + rpm4) / total_wheels_)
/ 60.0; vel.linear_x = average_rps_x * wheel_circumference_;

    average_rps_y = ((double)(-rpm1 + rpm2 + rpm3 - rpm4) / total_wheels_)
/ 60.0; if(base_platform_ == MECANUM) vel.linear_y = average_rps_y *
wheel_circumference_; else vel.linear_y = 0;

    average_rps_a = ((double)(-rpm1 + rpm2 - rpm3 + rpm4) / total_wheels_)
/ 60.0; vel.angular_z =  (average_rps_a * wheel_circumference_) /
(wheels_y_distance_ / 2.0);

    return vel;
}

int Kinematics::getTotalWheels(base robot_base)
{
    switch(robot_base)
    {
        case DIFFERENTIAL_DRIVE:    return 2;
        case SKID_STEER:            return 4;
        case MECANUM:               return 4;
        default:                    return 2;
    }
}

double Kinematics::getMaxRPM()
{
    return max_rpm_;
}

#endif

*/