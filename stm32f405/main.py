
from bus_device import *
from failsafe import *
from util import *
from encoder import Encoder
from servo_decoder import ServoDecoder
from servo import Servo
from laser import LaserRangeSensor
from imu import BNO085_IMU
import micropython
import struct
from machine import Pin, ADC, I2C


class Crawler:

    # Ports
    I2C_PORT = 2
    IMU_UART_PORT = 6
    RPI_UART_PORT = 1
    FAILSAFE_UART_PORT = 4

    # Quadrature Encoder on motor
    ENCODER_A_PIN = 'A0'
    ENCODER_B_PIN = 'A1'

    # RC Receiver
    RADIO_STEERING_PIN = 'A2'
    RADIO_MOTOR_PIN = 'A3'
    RADIO_TIMER = 2
    RADIO_STEERING_CHANNEL = 3
    RADIO_MOTOR_CHANNEL = 4

    # ADC for electronics battery
    BATTERY_VOLTAGE_PIN = 'A5'

    # PWM Out for steering and motor control
    STEERING_SERVO_PIN = 'A6'
    MOTOR_CONTROL_PIN = 'A7'
    PWM_TIMER = 3
    PWM_STEERING_CHANNEL = 1
    PWM_MOTOR_CHANNEL = 2

    # ToF LIDAR
    FRONT_LIDAR_SHUTDOWN_PIN = 'C0'
    FRONT_LIDAR_ADDRESS = 0x30
    BACK_LEFT_LIDAR_SHUTDOWN_PIN = 'C13'
    BACK_LEFT_LIDAR_ADDRESS = 0x31
    BACK_CENTER_LIDAR_SHUTDOWN_PIN = 'C14'
    BACK_CENTER_LIDAR_ADDRESS = 0x32
    BACK_RIGHT_LIDAR_SHUTDOWN_PIN = 'C15'
    BACK_RIGHT_LIDAR_ADDRESS = 0x33

    # Control
    CONTROL_MODE_AUTONOMOUS = 0
    CONTROL_MODE_STOPPED = 1
    CONTROL_MODE_RC = 2


    def __init__(self):
        self.encoder = Encoder(Pin(self.ENCODER_A_PIN, Pin.IN), Pin(self.ENCODER_B_PIN, Pin.IN))
        self.steering_decoder = ServoDecoder(Pin(self.RADIO_STEERING_PIN, Pin.IN), self.RADIO_TIMER, self.RADIO_STEERING_CHANNEL)
        self.motor_decoder = ServoDecoder(Pin(self.RADIO_MOTOR_PIN, Pin.IN), self.RADIO_TIMER, self.RADIO_MOTOR_CHANNEL)
        self.steering_servo = Servo(Pin(self.STEERING_SERVO_PIN, Pin.OUT), self.PWM_TIMER, self.PWM_STEERING_CHANNEL)
        self.motor_servo = Servo(Pin(self.MOTOR_CONTROL_PIN, Pin.OUT), self.PWM_TIMER, self.PWM_MOTOR_CHANNEL)
        # self.battery = ADC(Pin(self.BATTERY_VOLTAGE_PIN))
        # self.i2c = I2C(self.I2C_PORT)
        # self.front_range_sensor = LaserRangeSensor('Front', self.i2c, self.FRONT_LIDAR_ADDRESS, self.FRONT_LIDAR_SHUTDOWN_PIN, LaserRangeSensor.LASER_SENSOR_VL6180X)
        # self.back_left_range_sensor = LaserRangeSensor('Back Left', self.i2c, self.BACK_LEFT_LIDAR_ADDRESS, self.BACK_LEFT_LIDAR_SHUTDOWN_PIN, LaserRangeSensor.LASER_SENSOR_VL53L1X)
        # self.back_center_range_sensor = LaserRangeSensor('Back Left', self.i2c, self.BACK_CENTER_LIDAR_ADDRESS, self.BACK_CENTER_LIDAR_SHUTDOWN_PIN, LaserRangeSensor.LASER_SENSOR_VL53L1X)
        # self.back_right_range_sensor = LaserRangeSensor('Back Left', self.i2c, self.BACK_RIGHT_LIDAR_ADDRESS, self.BACK_RIGHT_LIDAR_SHUTDOWN_PIN, LaserRangeSensor.LASER_SENSOR_VL53L1X)
        self.imu = BNO085_IMU(self.IMU_UART_PORT)
        self.failsafe = Failsafe(self.handle_failsafe_packet, self.FAILSAFE_UART_PORT)
        self.device = BusDevice(self, self.RPI_UART_PORT)
        self.mode = self.CONTROL_MODE_RC
        self.desired_speed = 0
        self.desired_turn_rate = 0
        self.voltage = 0

    def handle_failsafe_packet(self, packet):
        pass

    def update_control_table(self):
        struct.pack_into("<hhhHHHHlBB", self.device.controlTable, CONTROL_IMU_PITCH_LOW, 
            self.imu.pitch, self.imu.roll, self.imu.yaw, 0, 0, 0, 0, self.encoder.value, 
            self.mode, self.voltage)
        if self.mode == self.CONTROL_MODE_RC:
            struct.pack_into("<bb", self.device.controlTable, CONTROL_ROBOT_DESIRED_SPEED, 
                self.desired_speed, self.desired_turn_rate)

    # input speed is from -100 to 100
    # output speed needs to be from 1000 to 2000
    def motor_speed_from(self, speed):
        return ((speed + 100) * 5) + 1000

    # input steering is from -100 to 100
    # output steering needs to be from 1000 to 2000
    def steering_position_from(self, steering):
        return ((steering + 100) * 5) + 1000

    def run(self):
        cycle_metro = Metro(20)
        while True:
            if self.imu.ready():
                self.imu.update()
            if cycle_metro.check():
                self.failsafe.update()
                self.update_control_table()
                self.device.update()
                if self.mode == self.CONTROL_MODE_RC:
                    # In RC mode we get speed & steering from reading the receiver pulses
                    self.motor_servo.position(self.motor_decoder.pulse_width)
                    self.steering_servo.position(self.steering_decoder.pulse_width)
                else:
                    # In Autonomous mode we get speed & steering from the control table
                    self.desired_speed, self.desired_turn_rate = struct.unpack_from("<bb", self.device.controlTable, CONTROL_ROBOT_DESIRED_SPEED)
                    self.motor_servo.position(self.motor_speed_from(self.desired_speed))
                    self.steering_servo.position(self.steering_position_from(self.desired_turn_rate))


micropython.alloc_emergency_exception_buf(100)

Crawler().run()
