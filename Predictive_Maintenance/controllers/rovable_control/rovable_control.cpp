#include "rovable_control.h"
#include "control_constants.h"

// Global variables
double goal_angle = 0;

const char *motors_names[2] = {"left motor", "right motor"};
const char *distance_sensors_names[2] = {"left distance sensor", "right distance sensor"};
const char *encoder_names[2] = {"left encoder", "right encoder"};

int ll_robot_state;
const double allowable_angle_error = PI / 20;
const int straight_ticks = 75;
int straight_counter = 0;
double *imu_result = (double *)malloc(sizeof(double) * 3);

WbDeviceTag dist_sensors[2];
WbDeviceTag gps;
WbDeviceTag imu;
WbDeviceTag motors[2];

// Sensor Functions

/**
 * @brief Gets the distance value from a distance sensor.
 *
 * @param tag The device tag of the distance sensor.
 * @return The distance value.
 */
double getDistance(WbDeviceTag tag)
{
  if (wb_distance_sensor_get_sampling_period(tag) == 0)
  {
    wb_distance_sensor_enable(tag, (TIME_STEP));
    return 9999.9;
  }
  else
  {
    double distance_value = wb_distance_sensor_get_value(tag);
    return distance_value;
  }
}

/**
 * @brief Gets the touch value from a touch sensor.
 *
 * @param tag The device tag of the touch sensor.
 * @return The touch value.
 */
int getTouch(WbDeviceTag tag)
{
  if (wb_touch_sensor_get_sampling_period(tag) == 0)
  {
    wb_touch_sensor_enable(tag, (TIME_STEP));
    return 0;
  }
  else
  {
    int touch_value = wb_touch_sensor_get_value(tag);
    return touch_value;
  }
}

// Spec Functions

/**
 * @brief Gets the largest dimension of the robot.
 *
 * @return The largest dimension.
 */
double largestDimension()
{
  return ROBOT_LENGTH;
}

/**
 * @brief Sets the speed of the robot's motors.
 *
 * @param left The speed of the left motor.
 * @param right The speed of the right motor.
 */
void robot_set_speed(double left, double right)
{
  wb_motor_set_velocity(motors[LEFT], left);
  wb_motor_set_velocity(motors[RIGHT], right);
}

/**
 * @brief Sets a message with the distance value from a sensor.
 *
 * @param msg_idx The message index.
 * @param msg_value The message value.
 * @param sensor_idx The sensor index.
 */
void setMessageWithDistance(int msg_idx, char msg_value, int sensor_idx)
{
  msg_idx += 1;
  if (UID >= 10)
  {
    msg_idx += 1;
  }
  message[msg_idx] = msg_value;
  strcpy(distance_message, message);
  char value[32];
  double val = getDistance(dist_sensors[sensor_idx]);
  snprintf(value, sizeof value, "-%0.0lf", val);
  strcat(distance_message, value);
}

/**
 * @brief Sets up the robot by initializing sensors and motors.
 */
void setup()
{
  for (int i = 0; i < 2; i++)
  {
    motors[i] = wb_robot_get_device(motors_names[i]);
    wb_motor_set_position(motors[i], INFINITY);
    dist_sensors[i] = wb_robot_get_device(distance_sensors_names[i]);
    wb_distance_sensor_enable(dist_sensors[i], TIME_STEP);
  }

  gps = wb_robot_get_device("gps");
  imu = wb_robot_get_device("imu");

  wb_gps_enable(gps, (TIME_STEP));
  wb_inertial_unit_enable(imu, (TIME_STEP));

  WbDeviceTag electromagnet = wb_robot_get_device("electromagnet");
  wb_motor_set_position(electromagnet, INFINITY);
  wb_motor_set_velocity(electromagnet, 1);

  ll_robot_state = STRAIGHT;
}

/**
 * @brief Gets the sign of a number.
 *
 * @param i The number.
 * @return The sign of the number: 1 for positive, -1 for negative, 0 for zero.
 */
int sign_of(double i)
{
  if (i > 0)
  {
    return 1;
  }
  else if (i < 0)
  {
    return -1;
  }
  else
  {
    return 0;
  }
}

/**
 * @brief Computes the real modulo operation.
 *
 * @param a The dividend.
 * @param b The divisor.
 * @return The remainder after division.
 */
double real_fmod(double a, double b)
{
  double val = sign_of(a) * fmod(abs(a), b);
  if (val < 0)
  {
    return val + b;
  }
  else
  {
    return val;
  }
}

/**
 * @brief Normalizes an angle to the range [-PI, PI].
 *
 * @param angle The angle to normalize.
 * @return The normalized angle.
 */
double normalize_angle(double angle)
{
  return real_fmod(angle + PI, 2 * PI) - PI;
}

/**
 * @brief Gets the GPS values.
 *
 * @return The GPS values.
 */
const double *gps_get_values()
{
  return wb_gps_get_values(gps);
}

/**
 * @brief Gets the roll, pitch, and yaw values from the inertial unit.
 *
 * @return The roll, pitch, and yaw values.
 */
const double *inertial_unit_get_roll_pitch_yaw()
{
  const double *result = wb_inertial_unit_get_roll_pitch_yaw(imu);
  imu_result[0] = result[0];
  imu_result[1] = result[1];
  imu_result[2] = normalize_angle(PI + result[2]);
  return imu_result;
}

/**
 * @brief Updates the low-level state of the robot based on the current state and goal angle.
 *
 * @param hl_robot_state The high-level robot state.
 */
void update_low_level_state(int &hl_robot_state)
{
  const double *roll_pitch_yaw = inertial_unit_get_roll_pitch_yaw();
  double error = normalize_angle(goal_angle - roll_pitch_yaw[2]);
  switch (ll_robot_state)
  {
  case STOP:
  {
    hl_robot_state = LL_DONE;
    ll_robot_state = STRAIGHT;
    break;
  }
  case STRAIGHT:
  {
    if (straight_counter > straight_ticks)
    {
      ll_robot_state = STOP;
      straight_counter = 0;
    }
    else if (abs(error) < allowable_angle_error)
    {
      straight_counter++;
    }
    else if (error < 0)
    {
      ll_robot_state = TURNING_RIGHT;
    }
    else
    {
      ll_robot_state = TURNING_LEFT;
    }
    break;
  }
  case TURNING_LEFT:
  {
    if (error <= 0 && error > -allowable_angle_error)
    {
      ll_robot_state = STRAIGHT;
      straight_counter = 0;
    }
    else if (error < 0)
    {
      ll_robot_state = TURNING_RIGHT;
    }
    break;
  }
  case TURNING_RIGHT:
  {
    if (error >= 0 && error < allowable_angle_error)
    {
      ll_robot_state = STRAIGHT;
      straight_counter = 0;
    }
    else if (error > 0)
    {
      ll_robot_state = TURNING_RIGHT;
    }
    break;
  }
  }
}

/**
 * @brief Performs the action corresponding to the current low-level state.
 */
void perform_state()
{
  switch (ll_robot_state)
  {
  case STRAIGHT:
  {
    robot_set_speed(MAX_SPEED, MAX_SPEED);
    break;
  }
  case TURNING_LEFT:
  {
    robot_set_speed(-MAX_SPEED, MAX_SPEED);
    break;
  }
  case TURNING_RIGHT:
  {
    robot_set_speed(MAX_SPEED, -MAX_SPEED);
    break;
  }
  case STOP:
  {
    robot_set_speed(0, 0);
    break;
  }
  }
}

/**
 * @brief Sets the goal angle for the robot.
 *
 * @param steering_angle The desired steering angle.
 */
void set_angle(double steering_angle)
{
  const double *roll_pitch_yaw = inertial_unit_get_roll_pitch_yaw();
  goal_angle = normalize_angle(steering_angle + roll_pitch_yaw[2]);
}
