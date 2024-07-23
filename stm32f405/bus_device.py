
from machine import UART
from Packet import Packet
from util import *


BIOLOID_CMD_PING = 1
BIOLOID_CMD_READ = 2
BIOLOID_CMD_WRITE = 3
BIOLOID_CMD_RESET = 6

CONTROL_MODEL_NUMBER_LOW = 0
CONTROL_MODEL_NUMBER_HIGH = 1
CONTROL_FIRMWARE_VERSION = 2
CONTROL_ID = 3
CONTROL_BAUD_RATE = 4
CONTROL_RETURN_DELAY_TIME = 5
CONTROL_STATUS_RETURN_LEVEL = 16

CONTROL_LED = 25
CONTROL_IMU_PITCH_LOW = 26
CONTROL_IMU_PITCH_HIGH = 27
CONTROL_IMU_ROLL_LOW = 28
CONTROL_IMU_ROLL_HIGH = 29
CONTROL_IMU_YAW_LOW = 30
CONTROL_IMU_YAW_HIGH = 31

CONTROL_FRONT_RANGE_LOW = 32
CONTROL_FRONT_RANGE_HIGH = 33
CONTROL_BACK_LEFT_RANGE_LOW = 34
CONTROL_BACK_LEFT_RANGE_HIGH = 35
CONTROL_BACK_CENTER_RANGE_LOW = 36
CONTROL_BACK_CENTER_RANGE_HIGH = 37
CONTROL_BACK_RIGHT_RANGE_LOW = 38
CONTROL_BACK_RIGHT_RANGE_HIGH = 39

CONTROL_ENCODER_VALUE_0 = 40
CONTROL_ENCODER_VALUE_1 = 41
CONTROL_ENCODER_VALUE_2 = 42
CONTROL_ENCODER_VALUE_3 = 43

CONTROL_ROBOT_MODE = 44
CONTROL_ROBOT_VOLTAGE = 45

CONTROL_ROBOT_DESIRED_SPEED = 46
CONTROL_ROBOT_DESIRED_TURN_RATE = 47

CONTROL_TABLE_SIZE = 48

PACKET_ERROR_RESERVED = 0x80         # Reserved - set to zero
PACKET_ERROR_INSTRUCTION = 0x40      # Undefined instruction
PACKET_ERROR_OVERLOAD = 0x20         # Max torque can't control applied load
PACKET_ERROR_CHECKSUM = 0x10         # Checksum of instruction packet incorrect
PACKET_ERROR_RANGE = 0x08            # Instruction is out of range
PACKET_ERROR_OVERHEATING = 0x04      # Internal temperature is too high
PACKET_ERROR_ANGLE_LIMIT = 0x02      # Goal position is outside of limit range
PACKET_ERROR_INPUT_VOLTAGE = 0x01    # Input voltage out of range
PACKET_ERROR_NONE = 0x00             # No Error

DEVICE_ID = 140


class BusDevice:
  def __init__(self, owner, uart_number):
    self.port = UART(uart_number, 1000000, rxbuf=1000)
    self.initControlTable()
    self.packet = Packet()
    self.packet.register_callback(self.packetReceived)
    self.owner = owner
    # flush the UART
    while self.port.any():
      self.port.read()

  def initControlTable(self):
    self.controlTable = [0] * CONTROL_TABLE_SIZE
    self.controlTable[CONTROL_MODEL_NUMBER_LOW] = 13
    self.controlTable[CONTROL_MODEL_NUMBER_HIGH] = 67
    self.controlTable[CONTROL_FIRMWARE_VERSION] = 1
    self.controlTable[CONTROL_ID] = DEVICE_ID
    self.controlTable[CONTROL_BAUD_RATE] = 1
    self.controlTable[CONTROL_RETURN_DELAY_TIME] = 0
    self.controlTable[CONTROL_STATUS_RETURN_LEVEL] = 2
    self.controlTable[CONTROL_LED] = 0

  def packetReceived(self, receivedPacket):
    # print('got packet for {}'.format(receivedPacket.id))
    if receivedPacket.id == DEVICE_ID:
      if receivedPacket.command == BIOLOID_CMD_PING:
        self.handlePing(receivedPacket)
      elif receivedPacket.command == BIOLOID_CMD_READ:
        self.handleRead(receivedPacket)
      elif receivedPacket.command == BIOLOID_CMD_WRITE:
        self.handleWrite(receivedPacket)

  def handlePing(self, packet):
    # print('Got ping')
    self.owner.logger.info('BIOLOID_HEAD - Got ping')
    # pyb.udelay(self.controlTable[CONTROL_RETURN_DELAY_TIME] * 2)
    response_buffer = bytearray([0xFF, 0xFF, DEVICE_ID, 0x02, 0, ((DEVICE_ID + 2) ^ 255)])
    self.port.write(response_buffer)

  def handleRead(self, packet):
    # self.owner.logger.info('BIOLOID_HEAD - Got read')
    start = packet.parameters[0]
    length = packet.parameters[1]
    if start < 0 or start + length > CONTROL_TABLE_SIZE:
      self.owner.logger.error('handleRead out of range: {}: {}'.format(start, length))
      return self.sendErrorResponse(packet, PACKET_ERROR_RANGE)
    # pyb.udelay(self.controlTable[CONTROL_RETURN_DELAY_TIME] * 2)
    response_buffer = bytearray([0xFF, 0xFF, DEVICE_ID, length + 2, 0])
    crc = DEVICE_ID + length + 2
    for index in range(start, start + length):
      byte = self.controlTable[index]
      response_buffer.append(byte)
      crc += byte
    response_buffer.append((crc & 255) ^ 255)
    self.port.write(response_buffer)
    # print('{} handleRead {} bytes at {}'.format(DEVICE_ID, length, start))

  def sendErrorResponse(self, packet, error):
    response_buffer = bytearray([0xFF, 0xFF, DEVICE_ID, 2, error])
    crc = DEVICE_ID + 2
    response_buffer.append((crc & 255) ^ 255)
    self.port.write(response_buffer)

  def handleWrite(self, packet):
    # self.owner.logger.info('BIOLOID_HEAD - Got write')
    start = packet.parameters[0]
    length = packet.parameters[1]
    if start < 0 or start + length > CONTROL_TABLE_SIZE:
      self.owner.logger.error('handleWrite out of range: {}: {}'.format(start, length))
      return self.sendErrorResponse(packet, PACKET_ERROR_RANGE)
    # pyb.udelay(self.controlTable[CONTROL_RETURN_DELAY_TIME] * 2)
    response_buffer = bytearray([0xFF, 0xFF, DEVICE_ID, 2, 0])
    crc = DEVICE_ID + 2
    response_buffer.append((crc & 255) ^ 255)
    for index in range(start, start + length):
      self.controlTable[index] = packet.parameters[index + 1]
    self.port.write(response_buffer)
    # print('{} handleWrite {} bytes at {}'.format(DEVICE_ID, length, start))

  def update(self):
    try:
      while self.port.any():
        self.packet.process_byte(self.port.read(1))
    except Exception as ex:
      self.owner.logger.warn('Exception on packet read: {} (resetting packet state)'.format(ex))
      self.packet.reset_state()
