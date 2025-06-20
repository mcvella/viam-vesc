import asyncio
import struct
import time
from dataclasses import dataclass
from typing import (Any, ClassVar, Dict, Final, List, Mapping, Optional,
                    Sequence, Tuple)

import serial
from typing_extensions import Self
from viam.components.motor import *
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict


class Vesc(Motor, EasyResource):
    # To enable debug-level logging, either run viam-server with the --debug option,
    # or configure your resource/machine to display debug logs.
    MODEL: ClassVar[Model] = Model(ModelFamily("mcvella", "vesc"), "vesc")

    # VESC packet IDs
    COMM_FW_VERSION = 0x32
    COMM_JUMP_TO_BOOTLOADER = 0x33
    COMM_ERASE_NEW_APP = 0x34
    COMM_WRITE_NEW_APP_DATA = 0x35
    COMM_GET_VALUES = 0x27
    COMM_SET_DUTY = 0x00
    COMM_SET_CURRENT = 0x01
    COMM_SET_CURRENT_BRAKE = 0x02
    COMM_SET_RPM = 0x03
    COMM_SET_POS = 0x04
    COMM_SET_HANDBRAKE = 0x05
    COMM_SET_DETECT = 0x06
    COMM_SET_SERVO_POS = 0x07
    COMM_SET_MCCONF = 0x28
    COMM_GET_MCCONF = 0x29
    COMM_GET_MCCONF_DEFAULT = 0x2A
    COMM_SET_APPCONF = 0x2B
    COMM_GET_APPCONF = 0x2C
    COMM_GET_APPCONF_DEFAULT = 0x2D
    COMM_SAMPLE_PRINT = 0x2E
    COMM_TERMINAL_CMD = 0x2F
    COMM_PRINT = 0x30
    COMM_ROTOR_POSITION = 0x31
    COMM_EXPERIMENT_SAMPLE = 0x33
    COMM_DETECT_MOTOR_PARAM = 0x34
    COMM_DETECT_MOTOR_R_L = 0x35
    COMM_DETECT_MOTOR_FLUX_LINKAGE = 0x36
    COMM_DETECT_ENCODER = 0x37
    COMM_DETECT_HALL_FOC = 0x38
    COMM_REBOOT = 0x39
    COMM_ALIVE = 0x3A
    COMM_GET_DECODED_PPM = 0x3B
    COMM_GET_DECODED_ADC = 0x3C
    COMM_GET_DECODED_CHUK = 0x3D
    COMM_FORWARD_CAN = 0x3E
    COMM_SET_CHUCK_DATA = 0x3F
    COMM_CUSTOM_APP_DATA = 0x40
    COMM_NRF_START_PAIRING = 0x41
    COMM_GPD_SET_FSW = 0x42
    COMM_GPD_BUFFER_NOTIFY = 0x43
    COMM_GPD_BUFFER_SIZE_LEFT = 0x44
    COMM_GPD_SEND_BUFFER = 0x45
    COMM_GPD_RECV_BUFFER = 0x46
    COMM_GPD_SET_TX = 0x47
    COMM_GPD_SET_AWAKE = 0x48
    COMM_GET_VALUES_SETUP = 0x49
    COMM_SET_MCCONF_TEMP = 0x4A
    COMM_SET_MCCONF_TEMP_SETUP = 0x4B
    COMM_GET_VALUES_SELECTIVE = 0x4C
    COMM_GET_VALUES_SETUP_SELECTIVE = 0x4D
    COMM_EXT_NRF_PRESENT = 0x4E
    COMM_EXT_NRF_ESB_SET_CH_ADDR = 0x4F
    COMM_EXT_NRF_ESB_SEND_DATA = 0x50
    COMM_EXT_NRF_ESB_RX_DATA = 0x51
    COMM_EXT_NRF_SET_ENABLED = 0x52
    COMM_PING_CAN = 0x53
    COMM_APP_DISABLE_OUTPUT = 0x54
    COMM_TERMINAL_CMD_SYNC = 0x55
    COMM_GET_IMU_DATA = 0x56
    COMM_BM_CONNECT = 0x57
    COMM_BM_ERASE_FLASH_ALL = 0x58
    COMM_BM_WRITE_FLASH = 0x59
    COMM_BM_REBOOT = 0x5A
    COMM_BM_DISCONNECT = 0x5B
    COMM_BM_MAP_PINS_DEFAULT = 0x5C
    COMM_BM_MAP_PINS_NEW = 0x5D
    COMM_ERASE_BOOTLOADER = 0x5E
    COMM_PLOT_INIT = 0x5F
    COMM_PLOT_DATA = 0x60
    COMM_PLOT_ADD_GRAPH = 0x61
    COMM_PLOT_SET_GRAPH = 0x62
    COMM_GET_DECODED_BALANCE = 0x63
    COMM_BM_MEM_READ = 0x64
    COMM_WRITE_NEW_APP_DATA_LZO = 0x65
    COMM_WRITE_NEW_APP_DATA_ALL_CAN = 0x66
    COMM_BM_WRITE_FLASH_ALL_CAN = 0x67
    COMM_SET_CURRENT_REL = 0x68
    COMM_SET_CURRENT_BRAKE_REL = 0x69
    COMM_SET_CURRENT_HANDBRAKE = 0x6A
    COMM_SET_CURRENT_HANDBRAKE_REL = 0x6B
    COMM_GET_IMU_CALIBRATION = 0x6C
    COMM_GET_MCCONF_TEMP = 0x6D
    COMM_GET_MCCONF_TEMP_SETUP = 0x6E
    COMM_GET_APPCONF_TEMP = 0x6F
    COMM_GET_APPCONF_TEMP_SETUP = 0x70
    COMM_BM_CONNECT_ALL_CAN = 0x71
    COMM_BM_DISCONNECT_ALL_CAN = 0x72
    COMM_BM_SET_PWM_MODE = 0x73
    COMM_BM_GET_PWM_MODE = 0x74
    COMM_BM_SET_VIN_RECALC = 0x75
    COMM_BM_SET_CURRENT_OFFSET = 0x76
    COMM_BM_SET_CURRENT_OFFSET_MODE = 0x77
    COMM_BM_GET_CURRENT_OFFSET_MODE = 0x78
    COMM_BM_SET_CURRENT_IN_DIR_TOL = 0x79
    COMM_BM_GET_CURRENT_IN_DIR_TOL = 0x7A
    COMM_BM_SET_BATT_FILTER_CONST = 0x7B
    COMM_BM_GET_BATT_FILTER_CONST = 0x7C
    COMM_BM_SET_BATT_TYPE = 0x7D
    COMM_BM_GET_BATT_TYPE = 0x7E
    COMM_BM_SET_BATT_CAPACITY = 0x7F
    COMM_BM_GET_BATT_CAPACITY = 0x80
    COMM_BM_SET_BATT_CAPACITY_CYCLE = 0x81
    COMM_BM_GET_BATT_CAPACITY_CYCLE = 0x82
    COMM_BM_SET_BATT_S = 0x83
    COMM_BM_GET_BATT_S = 0x84
    COMM_BM_SET_BATT_V_START = 0x85
    COMM_BM_GET_BATT_V_START = 0x86
    COMM_BM_SET_BATT_V_END = 0x87
    COMM_BM_GET_BATT_V_END = 0x88
    COMM_BM_SET_BATT_V_CHARGED = 0x89
    COMM_BM_GET_BATT_V_CHARGED = 0x8A
    COMM_BM_SET_BATT_I_CHARGED = 0x8B
    COMM_BM_GET_BATT_I_CHARGED = 0x8C
    COMM_BM_SET_BATT_I_CHARGED_END = 0x8D
    COMM_BM_GET_BATT_I_CHARGED_END = 0x8E
    COMM_BM_SET_BATT_I_CHARGED_MAX = 0x8F
    COMM_BM_GET_BATT_I_CHARGED_MAX = 0x90
    COMM_BM_SET_BATT_TEMP_SENSOR_TYPE = 0x91
    COMM_BM_GET_BATT_TEMP_SENSOR_TYPE = 0x92
    COMM_BM_SET_BATT_TEMP_SENSOR_BETA = 0x93
    COMM_BM_GET_BATT_TEMP_SENSOR_BETA = 0x94
    COMM_BM_SET_BATT_TEMP_SENSOR_OFFSET = 0x95
    COMM_BM_GET_BATT_TEMP_SENSOR_OFFSET = 0x96
    COMM_BM_SET_BATT_TEMP_SENSOR_GAIN = 0x97
    COMM_BM_GET_BATT_TEMP_SENSOR_GAIN = 0x98
    COMM_BM_SET_BATT_TEMP_SENSOR_RESISTANCE = 0x99
    COMM_BM_GET_BATT_TEMP_SENSOR_RESISTANCE = 0x9A
    COMM_BM_SET_BATT_TEMP_SENSOR_CURRENT = 0x9B
    COMM_BM_GET_BATT_TEMP_SENSOR_CURRENT = 0x9C
    COMM_BM_SET_BATT_TEMP_SENSOR_VOLTAGE = 0x9D
    COMM_BM_GET_BATT_TEMP_SENSOR_VOLTAGE = 0x9E
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP = 0x9F
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP = 0xA0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_OFFSET = 0xA1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_OFFSET = 0xA2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_GAIN = 0xA3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_GAIN = 0xA4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_RESISTANCE = 0xA5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_RESISTANCE = 0xA6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_CURRENT = 0xA7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xA8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xA9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xAA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xAB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP = 0xAC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xAD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xAE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xAF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xB0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xBF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xC0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xC1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xC2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xC3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xC4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xC5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xC6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xC7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xC8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xC9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xCA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xCB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xCC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xCD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xCE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xCF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xD0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xD1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xD2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xD3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xD4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xD5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xD6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xD7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xD8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xD9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xDA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xDB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xDC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xDD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xDE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xDF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xE0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xE1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xE2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xE3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xE4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xE5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xE6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_CURRENT = 0xB3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_CURRENT = 0xB4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xB5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xB6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP = 0xB7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP = 0xB8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xB9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xBA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xBB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xBC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xBD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xBE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xBF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xC0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xC1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xC2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP = 0xC3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP = 0xC4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xC5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xC6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xC7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xC8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xC9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xCA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xCB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xCC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xCD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xCE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xCF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP = 0xD0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xD1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xD2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xD3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xD4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xD5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xD6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xD7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xD8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xD9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xDA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xDB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xDC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xDD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xDE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xDF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xE0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xE1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xE2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xE3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xE4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xE5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xE6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xE7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xE8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xE9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xEA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xEB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xEC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xED
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xEE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xEF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xF0
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xF1
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xF2
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xF3
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xF4
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xF5
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_OFFSET = 0xF6
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xF7
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_GAIN = 0xF8
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xF9
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_RESISTANCE = 0xFA
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xFB
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_CURRENT = 0xFC
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xFD
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_VOLTAGE = 0xFE
    COMM_BM_SET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0xFF
    COMM_BM_GET_BATT_TEMP_SENSOR_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP_TEMP = 0x100

    def __init__(self, name: str):
        super().__init__(name)
        self.serial_port = None
        self.port = None
        self.baudrate = 115200
        self.timeout = 1.0
        self._is_powered = False
        self._current_power = 0.0
        self._current_rpm = 0.0
        self._position = 0.0
        self._is_moving = False
        self._lock = asyncio.Lock()

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this Motor component.
        The default implementation sets the name from the `config` parameter and then calls `reconfigure`.

        Args:
            config (ComponentConfig): The configuration for this resource
            dependencies (Mapping[ResourceName, ResourceBase]): The dependencies (both required and optional)

        Returns:
            Self: The resource
        """
        vesc = cls(config.name)
        vesc.reconfigure(config, dependencies)
        return vesc

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        """This method allows you to validate the configuration object received from the machine,
        as well as to return any required dependencies or optional dependencies based on that `config`.

        Args:
            config (ComponentConfig): The configuration for this resource

        Returns:
            Tuple[Sequence[str], Sequence[str]]: A tuple where the
                first element is a list of required dependencies and the
                second element is a list of optional dependencies
        """
        return [], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both required and optional)
        """
        if self.serial_port:
            self.serial_port.close()
        
        # Get configuration using struct_to_dict for cleaner access
        attributes = struct_to_dict(config.attributes)
        
        # Set up VESC configuration with defaults
        self.port = str(attributes.get("port", "/dev/ttyACM0"))
        self.baudrate = int(attributes.get("baudrate", 115200))
        self.timeout = float(attributes.get("timeout", 1.0))
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=self.timeout,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.logger.info(f"Connected to VESC on {self.port} at {self.baudrate} baud")
        except Exception as e:
            self.logger.error(f"Failed to connect to VESC: {e}")
            raise

    def _crc16(self, data: bytes) -> int:
        """Calculate CRC16 for VESC packet"""
        crc = 0
        for byte in data:
            crc ^= byte << 8
            for _ in range(8):
                if crc & 0x8000:
                    crc = ((crc << 1) ^ 0x1021) & 0xFFFF
                else:
                    crc = (crc << 1) & 0xFFFF
        return crc

    def _pack_payload(self, payload: bytes) -> bytes:
        """Pack payload with VESC protocol framing"""
        if len(payload) <= 256:
            header = struct.pack('BB', 2, len(payload))  # Short packet
        else:
            header = struct.pack('BBB', 3, len(payload) >> 8, len(payload) & 0xFF)  # Long packet
        
        crc = self._crc16(payload)
        crc_bytes = struct.pack('>H', crc)
        packet = header + payload + crc_bytes + b'\x03'
        return packet

    def _send_packet(self, command_id: int, payload: bytes = b'') -> bool:
        """Send a packet to the VESC using the working protocol"""
        if not self.serial_port or not self.serial_port.is_open:
            self.logger.error("Serial port not open")
            return False
        
        try:
            # Create the full payload with command ID
            full_payload = struct.pack('B', command_id) + payload
            packet = self._pack_payload(full_payload)
            
            self.serial_port.write(packet)
            self.serial_port.flush()
            return True
        except Exception as e:
            self.logger.error(f"Failed to send packet: {e}")
            return False

    def _read_packet(self, expected_id: int = None) -> Optional[bytes]:
        """Read a packet from the VESC"""
        if not self.serial_port or not self.serial_port.is_open:
            return None
        
        try:
            # Read packet ID
            packet_id = self.serial_port.read(1)
            if not packet_id:
                return None
            
            packet_id = packet_id[0]
            
            # Read payload length (2 bytes)
            length_bytes = self.serial_port.read(2)
            if len(length_bytes) != 2:
                return None
            
            payload_length = struct.unpack('>H', length_bytes)[0]
            
            # Read payload
            payload = self.serial_port.read(payload_length)
            if len(payload) != payload_length:
                return None
            
            # Read CRC (2 bytes)
            crc_bytes = self.serial_port.read(2)
            if len(crc_bytes) != 2:
                return None
            
            received_crc = struct.unpack('>H', crc_bytes)[0]
            calculated_crc = self._crc16(struct.pack('>B', packet_id) + struct.pack('>H', payload_length) + payload)
            
            if received_crc != calculated_crc:
                self.logger.error(f"CRC mismatch: received {received_crc}, calculated {calculated_crc}")
                return None
            
            if expected_id and packet_id != expected_id:
                self.logger.error(f"Unexpected packet ID: received {packet_id}, expected {expected_id}")
                return None
            
            return payload
            
        except Exception as e:
            self.logger.error(f"Failed to read packet: {e}")
            return None

    @dataclass
    class Properties(Motor.Properties):
        position_reporting: bool = True

    async def set_power(
        self,
        power: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Set motor power as a percentage (-1.0 to 1.0)"""
        async with self._lock:
            power = max(-1.0, min(1.0, power))  # Clamp to [-1, 1]
            
            # If we're changing direction, stop first to ensure clean transition
            if hasattr(self, '_current_power') and self._current_power != 0:
                if (self._current_power > 0 and power < 0) or (self._current_power < 0 and power > 0):
                    # Direction change - stop first
                    duty_int = 0
                    stop_payload = struct.pack('>i', duty_int)
                    self._send_packet(5, stop_payload)  # COMM_SET_DUTY = 5
                    await asyncio.sleep(0.1)  # Small delay for direction change
            
            # Convert to int32 (duty * 100000) - matching working script
            duty_int = int(power * 100000)
            payload = struct.pack('>i', duty_int)
            
            if self._send_packet(5, payload):  # COMM_SET_DUTY = 5
                self._is_powered = power != 0.0
                self._current_power = power
                self._is_moving = power != 0.0
                self.logger.info(f"Set power to {power:.2f} (duty_int: {duty_int})")
            else:
                raise RuntimeError("Failed to set motor power")

    async def go_for(
        self,
        rpm: float,
        revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Go for a specific number of revolutions at a given RPM"""
        async with self._lock:
            # For VESC, we'll use RPM control and calculate the time needed
            if revolutions == 0:
                await self.stop()
                return
            
            # Set RPM using int32 encoding
            rpm_int = int(rpm)
            rpm_payload = struct.pack('>i', rpm_int)
            
            if not self._send_packet(8, rpm_payload):  # COMM_SET_RPM = 8
                raise RuntimeError("Failed to set RPM")
            
            self._current_rpm = rpm
            self._is_powered = True
            self._is_moving = True
            
            # Calculate time needed for the revolutions
            if rpm != 0:
                time_needed = abs(revolutions / rpm) * 60.0  # Convert to seconds
                
                # Wait for the specified time
                await asyncio.sleep(time_needed)
                
                # Stop the motor
                await self.stop()
                
                # Update position
                self._position += revolutions

    async def go_to(
        self,
        rpm: float,
        position_revolutions: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Go to a specific position at a given RPM"""
        target_position = position_revolutions
        current_position = self._position
        revolutions_needed = target_position - current_position
        
        await self.go_for(rpm, revolutions_needed, extra=extra, timeout=timeout, **kwargs)

    async def set_rpm(
        self,
        rpm: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Set the motor RPM"""
        async with self._lock:
            # Convert to int32
            rpm_int = int(rpm)
            rpm_payload = struct.pack('>i', rpm_int)
            
            if self._send_packet(8, rpm_payload):  # COMM_SET_RPM = 8
                self._current_rpm = rpm
                self._is_powered = rpm != 0.0
                self._is_moving = rpm != 0.0
                self.logger.info(f"Set RPM to {rpm}")
            else:
                raise RuntimeError("Failed to set RPM")

    async def reset_zero_position(
        self,
        offset: float,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Reset the zero position with an offset"""
        async with self._lock:
            self._position = offset
            self.logger.info(f"Reset zero position with offset {offset}")

    async def get_position(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> float:
        """Get the current position in revolutions"""
        return self._position

    async def get_properties(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Properties:
        """Get motor properties"""
        return self.Properties(position_reporting=True)

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        """Stop the motor"""
        async with self._lock:
            # Send zero duty cycle to stop using int32 encoding
            duty_int = 0
            payload = struct.pack('>i', duty_int)
            
            if self._send_packet(5, payload):  # COMM_SET_DUTY = 5
                self._is_powered = False
                self._current_power = 0.0
                self._current_rpm = 0.0
                self._is_moving = False
                self.logger.info("Motor stopped")
            else:
                raise RuntimeError("Failed to stop motor")

    async def is_powered(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[bool, float]:
        """Check if the motor is powered and return current power"""
        return self._is_powered, self._current_power

    async def is_moving(self) -> bool:
        """Check if the motor is moving"""
        return self._is_moving

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        """Handle custom commands"""
        command_name = command.get("command")
        
        if command_name == "get_vesc_values":
            # Get VESC telemetry values
            if self._send_packet(4):  # COMM_GET_VALUES = 4
                payload = self._read_packet(4)
                if payload:
                    # Parse VESC values (this is a simplified version)
                    # In a real implementation, you'd parse all the telemetry data
                    return {"status": "success", "data": "VESC telemetry received"}
                else:
                    return {"status": "error", "message": "Failed to read VESC values"}
            else:
                return {"status": "error", "message": "Failed to request VESC values"}
        
        elif command_name == "set_current":
            # Set motor current
            current = command.get("current", 0.0)
            if isinstance(current, (int, float)):
                # Convert to milliamps and use int32 encoding
                current_ma = int(float(current) * 1000)
                payload = struct.pack('>i', current_ma)
                if self._send_packet(6, payload):  # COMM_SET_CURRENT = 6
                    return {"status": "success", "current": current}
                else:
                    return {"status": "error", "message": "Failed to set current"}
            else:
                return {"status": "error", "message": "Invalid current value"}
        
        elif command_name == "test_direction":
            # Test direction changes with proper delays
            direction = command.get("direction", "forward")
            power = command.get("power", 0.3)
            
            # Stop first
            await self.stop()
            await asyncio.sleep(0.2)
            
            if direction == "forward":
                await self.set_power(power)
            elif direction == "reverse":
                await self.set_power(-power)
            elif direction == "stop":
                await self.stop()
            else:
                return {"status": "error", "message": "Invalid direction"}
            
            return {"status": "success", "direction": direction, "power": power}
        
        else:
            return {"status": "error", "message": f"Unknown command: {command_name}"}

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        """Get the geometries of the motor"""
        return []

