import time
import threading
import struct
import os
import errno
import fcntl

from a_star import AStar
from lsm6 import LSM6

BALANCE_CMD_PATH = '/tmp/balance_cmd'
BALANCE_ODM_PATH = '/tmp/balance_odm'


# This code was developed for a Balboa unit using 50:1 motors
# and 45:21 plastic gears, for an overall gear ratio of 111.
# Adjust the ratio below to scale various constants in the
# balancing algorithm to match your robot.
GEAR_RATIO = 111

# This constant limits the maximum motor speed.  If your gear
# ratio is lower than what we used, or if you are testing
# changes to the code, you might want to reduce it to prevent
# your robot from zooming away when things go wrong.
MOTOR_SPEED_LIMIT = 300

# This constant relates the angle to its rate of change for a
# robot that is falling from a nearly-vertical position or
# rising up to that position.  The relationship is nearly
# linear.  For example, if you have the 80mm wheels it should be
# about 140, which means that the angle in millidegrees is ~140
# times its rate of change in degrees per second; when the robot
# has fallen by 90 degrees it will be moving at about
# 90,000/140 = 642 deg/s.  See the end of Balancer.ino for one
# way to calibrate this value.
ANGLE_RATE_RATIO = 140

# The following three constants define a PID-like algorithm for
# balancing.  Each one determines how much the motors will
# respond to the corresponding variable being off from zero.
# See the code in Balance.cpp for exactly how they are used.  To
# get it balancing from scratch, start with them all at zero and
# adjust them as follows:

# ANGLE_RESPONSE determines the response to a combination of
# angle and angle_rate; the combination measures how far the
# robot is from a stable trajectory.  To test this, use your
# hand to flick the robot up from a resting position.  With a
# value that is too low, it won't stop itself in time; a high
# value will cause it to slam back into the ground or oscillate
# wildly back and forth.  When ANGLE_RESPONSE is adjusted
# properly, the robot will move just enough to stop itself.
# However, after stopping itself, it will be moving and keep
# moving in the same direction, usually driving faster and
# faster until it reaches its maximum motor speed and falls
# over.  That's where the next constants come in.
ANGLE_RESPONSE = 12

# DISTANCE_RESPONSE determines how much the robot resists being
# moved away from its starting point.  Counterintuitively, this
# constant is positive: to move forwards, the robot actually has
# to first roll its wheels backwards, so that it can *fall*
# forwards.  When this constant is adjusted properly, the robot
# will no longer zoom off in one direction, but it will drive
# back and forth a few times before falling down.
DISTANCE_RESPONSE = 90

# DISTANCE_DIFF_RESPONSE determines the response to differences
# between the left and right motors, preventing undesired
# rotation due to differences in the motors and gearing.  Unlike
# DISTANCE_REPONSE, it should be negative: if the left motor is
# lagging, we need to increase its speed and decrease the speed
# of the right motor.  If this constant is too small, the robot
# will spin left and right as it rocks back and forth; if it is
# too large it will become unstable.
DISTANCE_DIFF_RESPONSE = -50

# SPEED_RESPONSE supresses the large back-and-forth oscillations
# caused by DISTANCE_RESPONSE.  Increase this until these
# oscillations die down after a few cycles; but if you increase
# it too much it will tend to shudder or vibrate wildly.
SPEED_RESPONSE = 3300

# The balancing code is all based on a 100 Hz update rate; if
# you change this, you will have to adjust many other things.
UPDATE_TIME = 0.01

# Take 100 measurements initially to calibrate the gyro.
CALIBRATION_ITERATIONS = 100
class balance_cmd:
  def __init__(self):
    # speed (int16)
    self.left_speed = 0
    self.right_speed = 0
    # flag (int8)
    self.flag_1 = 0
    self.flag_2 = 0

class balance_odm:
  def __init__(self):
    # imu (float32)
    self.ax = 0.0
    self.ay = 0.0
    self.az = 0.0
    self.gx = 0.0
    self.gy = 0.0
    self.gz = 0.0
    # enc (int16)
    self.left_enc = 0
    self.right_enc = 0
    # condition for debug (int8)
    self.status_1 = 0
    self.status_2 = 0
    # cmd_echo speed for debug (int16)
    self.left_speed = 0
    self.right_speed = 0

class Balancer:
  def __init__(self):
    self.a_star = AStar()
    self.imu = LSM6()

    self.g_y_zero = 0
    self.angle = 0 # degrees
    self.angle_rate = 0 # degrees/s
    self.distance_left = 0
    self.speed_left = 0
    self.drive_left = 0
    self.last_counts_left = 0
    self.distance_right = 0
    self.speed_right = 0
    self.drive_right = 0
    self.last_counts_right = 0
    self.motor_speed = 0
    self.balancing = False
    self.calibrated = False
    self.running = False
    self.next_update = 0
    self.update_thread = None

    self.cur_cmd = balance_cmd()
    self.cur_status = 0
    self.adj_speed_left = 0
    self.adj_speed_right = 0

    self.lock = threading.Lock()

  def setup(self):
    self.imu.enable()
    time.sleep(1) # wait for IMU readings to stabilize

    # calibrate the gyro
    total = 0
    for _ in range(CALIBRATION_ITERATIONS):
      self.imu.read()
      total += self.imu.g.y
      time.sleep(0.001)
    self.g_y_zero = total / CALIBRATION_ITERATIONS
    self.calibrated = True

  def start(self):
    if self.calibrated:
      if not self.running:
        self.running = True
        self.next_update = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
        self.update_thread = threading.Thread(target=self.update_loop, daemon=True)
        self.update_thread.start()
    else:
      raise RuntimeError("IMU not enabled/calibrated; can't start balancer")

  def stop(self):
    if self.running:
      self.running = False
      self.update_thread.join()


  def stand_up(self):
    if self.calibrated:
      sign = 1

      if self.imu.a.z < 0:
        sign = -1

      self.stop()
      self.reset()
      self.imu.read()
      self.a_star.motors(-sign*MOTOR_SPEED_LIMIT, -sign*MOTOR_SPEED_LIMIT)
      time.sleep(0.4)
      self.a_star.motors(sign*MOTOR_SPEED_LIMIT, sign*MOTOR_SPEED_LIMIT)

      for _ in range(40):
        time.sleep(UPDATE_TIME)
        self.update_sensors()
        print(self.angle)
        if abs(self.angle) < 60:
          break

      self.motor_speed = sign*MOTOR_SPEED_LIMIT
      self.reset_encoders()
      self.start()
      self.cur_status = 1
      # self.write_odm()


    else:
      raise RuntimeError("IMU not enabled/calibrated; can't stand up")

  def read_cmd(self):
    #read
    print("read_cmd")
    cmd_path = BALANCE_CMD_PATH
    try:
        # print("open", cmd_path)
        with open(cmd_path, "rb") as fin:
        #with os.fdopen(os.open(cmd_path, os.O_RDONLY | os.O_NONBLOCK), "rb") as fin:
            print("read", cmd_path)
            cmd = fin.read()
            if len(cmd) == 0:
                # print ("Read 0")
                return
            balance_cmd = struct.unpack('hhbb', cmd)
            self.cur_cmd.left_speed = balance_cmd[0]
            self.cur_cmd.right_speed = balance_cmd[1]
            self.cur_cmd.flag_1 = balance_cmd[2]
            self.cur_cmd.flag_2 = balance_cmd[3]
            print ("ls", self.cur_cmd.left_speed, "rs", self.cur_cmd.right_speed, "s1", self.cur_cmd.flag_1, "s2", self.cur_cmd.flag_2)
    except IOError:
        print ("Cmd read error")
    self.adj_speed_left = self.cur_cmd.left_speed
    self.adj_speed_right = self.cur_cmd.right_speed

  def write_odm(self):
    #write
    odm = balance_odm()
    with self.lock:
      odm.ax = self.imu.a.x
      odm.ay = self.imu.a.y
      odm.az = self.imu.a.z
      odm.gx = self.imu.g.x
      odm.gy = self.imu.g.y
      odm.gz = self.imu.g.z

      odm.left_enc = self.last_counts_left
      odm.right_enc = self.last_counts_right
      odm.status_1 = self.cur_status
      odm.status_2 = self.cur_cmd.flag_2
      #debug
      odm.left_speed = self.cur_cmd.left_speed
      odm.right_speed = self.cur_cmd.right_speed
      
      print("ax", odm.ax, "ay", odm.ay, "az", odm.az, "gx", odm.gx, "gy", odm.gy, "gz", odm.gz)
      print("le", odm.left_enc, "re", odm.right_enc, "s1", odm.status_1, "s2", odm.status_2, "ls", odm.left_speed, "rs", odm.right_speed)

    odata = struct.pack('6fhhbbhh', odm.ax, odm.ay, odm.az, odm.gx, odm.gy, odm.gz, odm.left_enc, odm.right_enc, odm.status_1, odm.status_2, odm.left_speed, odm.right_speed)

    odm_path = BALANCE_ODM_PATH
    try:
        print("open", odm_path)
        with open(odm_path, "wb") as fout:
        #with os.fdopen(os.open(odm_path, os.O_WRONLY), "wb") as fout:
            print("write", odm_path)
            fout.write(odata)
    except IOError as err:
        print ("Odm write error", err)
        pass

  def update_loop(self):
    while self.running:
      self.update_sensors()
      self.do_drive_ticks()
      #self.read_cmd(self)

      if self.imu.a.x < 2000:
        # If X acceleration is low, the robot is probably close to horizontal.
        self.reset()
        self.balancing = False
      else:
        self.balance()
        self.balancing = True

      #self.write_odm()
      # Perform the balance updates at 100 Hz.
      self.next_update += UPDATE_TIME
      now = time.clock_gettime(time.CLOCK_MONOTONIC_RAW)
      time.sleep(max(self.next_update - now, 0))

    # stop() has been called and the loop has exited. Stop the motors.
    self.a_star.motors(0, 0)

  def update_sensors(self):
    with self.lock:
      self.imu.read()
      self.integrate_gyro()
      self.integrate_encoders()

  def integrate_gyro(self):
    # Convert from full-scale 1000 deg/s to deg/s.
    self.angle_rate = (self.imu.g.y - self.g_y_zero) * 35 / 1000

    self.angle += self.angle_rate * UPDATE_TIME

  def integrate_encoders(self):
    (counts_left, counts_right) = self.a_star.read_encoders()

    self.speed_left = subtract_16_bit(counts_left, self.last_counts_left)
    self.distance_left += self.speed_left
    self.last_counts_left = counts_left

    self.speed_right = subtract_16_bit(counts_right, self.last_counts_right)
    self.distance_right += self.speed_right
    self.last_counts_right = counts_right

#  def drive(self, left_speed, right_speed):
#    self.drive_left = left_speed
#    self.drive_right = right_speed

  def drive(self):
    with self.lock:
      self.drive_left = self.adj_speed_left
      self.drive_right = self.adj_speed_right

  def do_drive_ticks(self):
    with self.lock:
      self.distance_left -= self.drive_left
      self.distance_right -= self.drive_right
      self.speed_left -= self.drive_left
      self.speed_right -= self.drive_right

  def reset(self):
    self.motor_speed = 0
    self.reset_encoders()
    self.a_star.motors(0, 0)

    if abs(self.angle_rate) < 2:
      # It's really calm, so assume the robot is resting at 110 degrees from vertical.
      if self.imu.a.z > 0:
        self.angle = 110
      else:
        self.angle = -110

  def reset_encoders(self):
    self.distance_left = 0
    self.distance_right = 0

  def balance(self):
    # Adjust toward angle=0 with timescale ~10s, to compensate for
    # gyro drift.  More advanced AHRS systems use the
    # accelerometer as a reference for finding the zero angle, but
    # this is a simpler technique: for a balancing robot, as long
    # as it is balancing, we know that the angle must be zero on
    # average, or we would fall over.
    self.angle *= 0.999

    # This variable measures how close we are to our basic
    # balancing goal - being on a trajectory that would cause us
    # to rise up to the vertical position with zero speed left at
    # the top.  This is similar to the fallingAngleOffset used
    # for LED feedback and a calibration procedure discussed at
    # the end of Balancer.ino.
    #
    # It is in units of degrees, like the angle variable, and
    # you can think of it as an angular estimate of how far off we
    # are from being balanced.
    rising_angle_offset = self.angle_rate * ANGLE_RATE_RATIO/1000 + self.angle

    # Combine risingAngleOffset with the distance and speed
    # variables, using the calibration constants defined in
    # Balance.h, to get our motor response.  Rather than becoming
    # the new motor speed setting, the response is an amount that
    # is added to the motor speeds, since a *change* in speed is
    # what causes the robot to tilt one way or the other.
    self.motor_speed += (
      + ANGLE_RESPONSE*1000 * rising_angle_offset
      + DISTANCE_RESPONSE * (self.distance_left + self.distance_right)
      + SPEED_RESPONSE * (self.speed_left + self.speed_right)
      ) / 100 / GEAR_RATIO

    if self.motor_speed > MOTOR_SPEED_LIMIT:
      self.motor_speed = MOTOR_SPEED_LIMIT
    if self.motor_speed < -MOTOR_SPEED_LIMIT:
      self.motor_speed = -MOTOR_SPEED_LIMIT

    # Adjust for differences in the left and right distances; this
    # will prevent the robot from rotating as it rocks back and
    # forth due to differences in the motors, and it allows the
    # robot to perform controlled turns.
    distance_diff = self.distance_left - self.distance_right

    self.a_star.motors(
      int(self.motor_speed + distance_diff * DISTANCE_DIFF_RESPONSE / 100),
      int(self.motor_speed - distance_diff * DISTANCE_DIFF_RESPONSE / 100))


def subtract_16_bit(a, b):
  diff = (a - b) & 0xFFFF
  if (diff & 0x8000):
    diff -= 0x10000
  return diff

balancer = Balancer()

if __name__ == "__main__":
  try:
    balancer.a_star.play_notes("o4l16ceg>c8")
    balancer.setup()
    balancer.stand_up()
    print ("go main loop")
    for i in range(1000):
      time.sleep(0.19) # wait for IMU readings to stabilize
      balancer.read_cmd()
      #balancer.drive(balancer.adj_speed_left, balancer.adj_speed_right)
      balancer.drive()
      balancer.write_odm()
      #print(balancer.angle)
      print(i, balancer.adj_speed_left)
    balancer.stop()
  except:
    print('exception')
    print('stop')
    balancer.stop()
    time.sleep(1)
    raise
