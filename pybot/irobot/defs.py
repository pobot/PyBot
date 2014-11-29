#!/usr/bin/env python
# -*- coding: utf-8 -*-

""" iRobote Create robot interface package.

Refer to the iRobot Create Open Interface reference document available on
iRobot Web site (http://www.irobot.com/filelibrary/pdfs/hrd/create/Create%20Open%20Interface_v2.pdf)

Constants definitions.
"""

__author__ = "Eric Pascual"
__email__ = "eric@pobot.org"
__copyright__ = "Copyright 2013, POBOT"
__version__ = "1.0"
__date__ = "June. 2013"
__status__ = "Development"
__license__ = "LGPL"

from collections import namedtuple

#
# Create physical characteristics
#

WHEEL_TO_WHEEL_DIST = 260 # mm
MAX_ABSOLUTE_SPEED = 500 # mm/sec

#
# Open Interface command opcodes
#

OI_SOFT_RESET				= 7
OI_START 					= 128
OI_MODE_PASSIVE				= OI_START
OI_MODE_SAFE				= 131
OI_MODE_FULL				= 132
OI_DRIVE					= 137
OI_DRIVE_DIRECT				= 145
OI_LEDS						= 139
OI_DIGITAL_OUTPUTS			= 147
OI_LOW_SIDE_DRIVERS			= 138
OI_PWM_LOW_SIDE_DRIVERS		= 144
OI_SEND_IR					= 151
OI_READ_SENSORS				= 142
OI_READ_SENSOR_LIST			= 149
OI_STREAM_SENSOR_LIST		= 148
OI_STREAM_PAUSE_RESUME		= 150
OI_SCRIPT_DEFINE			= 152
OI_SCRIPT_PLAY				= 153
OI_SCRIPT_SHOW				= 154
OI_WAIT_TIME			    = 155
OI_WAIT_DISTANCE			= 156
OI_WAIT_ANGLE				= 157
OI_WAIT_EVENT				= 158
OI_SONG_RECORD			    = 140
OI_SONG_PLAY				= 141

#
# LEDs related defines
#

# power LED color
OI_LED_GREEN	= 0
OI_LED_YELLOW	= 63
OI_LED_ORANGE	= 127
OI_LED_RED		= 255
# power LED pre-defined intensities
OI_LED_OFF		= 0
OI_LED_FULL		= 255
OI_LED_ON		= OI_LED_FULL

# "play" LED bit mask
OI_LED_PLAY		= 2
# "advance" LED bit mask
OI_LED_ADVANCE	= 8

OI_ALL_LEDS     = OI_LED_PLAY | OI_LED_ADVANCE

#
# Buttons masks and names
#

OI_BUTTONS_PLAY			= 1
OI_BUTTONS_ADVANCE		= 4
OI_ALL_BUTTONS          = OI_BUTTONS_PLAY | OI_BUTTONS_ADVANCE

OI_BUTTON_NAMES = {
    OI_BUTTONS_PLAY     : "play",
    OI_BUTTONS_ADVANCE  :  "advance"
}

#
# Digital outputs
#

OI_DOUT_0		= 1
OI_DOUT_1		= 2
OI_DOUT_2		= 4
OI_ALL_DOUTS    = OI_DOUT_0 | OI_DOUT_1 | OI_DOUT_2

#
# Low side drivers
#

OI_DRIVER_0		= 1
OI_DRIVER_1		= 2
OI_DRIVER_2		= 4
OI_ALL_DRIVERS  = OI_DRIVER_0 | OI_DRIVER_1 | OI_DRIVER_2

#
# Special values for the radius parameter of the "drive" command
#

OI_DRIVE_STRAIGHT		= 0x8000
OI_SPIN_CW				= 0xFFFF
OI_SPIN_CCW				= 0x0001

#
# Logic sensors masks
#

OI_BUMPER_RIGHT			= 0x01
OI_BUMPER_LEFT			= 0x02
OI_WHEEL_DROP_RIGHT		= 0x04
OI_WHEEL_DROP_LEFT		= 0x08
OI_CASTER_DROP			= 0x10
OI_ALL_SENSORS          = 0x1f

OI_BUMPER_NAMES = {
    OI_BUMPER_RIGHT     : 'right',
    OI_BUMPER_LEFT      : 'left'
}

OI_WHEEL_DROP_NAMES = {
    OI_WHEEL_DROP_RIGHT     : 'right wheel',
    OI_WHEEL_DROP_LEFT      : 'left wheel',
    OI_CASTER_DROP          : 'caster'
}


#
# Drivers and wheel overcurrent flags masks
#

OI_OVERCURRENT_DRV_0    = 0x02
OI_OVERCURRENT_DRV_1    = 0x01
OI_OVERCURRENT_DRV_2    = 0x04
OI_OVERCURRENT_WHEEL_R  = 0x08
OI_OVERCURRENT_WHEEL_L  = 0x10

#
# Sensor packets
#

OI_PACKET_GROUP_0					= 0

OI_PACKET_GROUP_1					= 1
OI_PACKET_GROUP_SENSOR_STATES		= OI_PACKET_GROUP_1

OI_PACKET_BUMPS_AND_WHEEL_DROPS		= 7
OI_PACKET_WALL						= 8
OI_PACKET_CLIFF_LEFT				= 9
OI_PACKET_CLIFF_FRONT_LEFT			= 10
OI_PACKET_CLIFF_FRONT_RIGHT			= 11
OI_PACKET_CLIFF_RIGHT				= 12
OI_PACKET_VIRTUAL_WALL				= 13
OI_PACKET_OVERCURRENT               = 14
OI_PACKET_UNUSED_1					= 15
OI_PACKET_UNUSED_2					= 16

OI_PACKET_GROUP_2					= 2
OI_PACKET_GROUP_UI_AND_ODOMETRY     = OI_PACKET_GROUP_2

OI_PACKET_RECEIVED_IR_BYTE			= 17
OI_PACKET_BUTTONS					= 18
OI_PACKET_DISTANCE					= 19
OI_PACKET_ANGLE						= 20

OI_PACKET_GROUP_3					= 2
OI_PACKET_GROUP_BATTERY				= OI_PACKET_GROUP_3

OI_PACKET_CHARGING_STATE			= 21
OI_PACKET_BATTERY_VOLTAGE			= 22
OI_PACKET_BATTERY_CURRENT			= 23
OI_PACKET_BATTERY_TEMPERATURE		= 24
OI_PACKET_BATTERY_CHARGE			= 25
OI_PACKET_BATTERY_CAPACITY			= 26

OI_PACKET_GROUP_4					= 4
OI_PACKET_GROUP_SIGNALS				= OI_PACKET_GROUP_4

OI_PACKET_WALL_SIGNAL				= 27
OI_PACKET_CLIFF_LEFT_SIGNAL			= 28
OI_PACKET_CLIFF_FRONT_LEFT_SIGNAL	= 29
OI_PACKET_CLIFF_FRONT_RIGHT_SIGNAL	= 30
OI_PACKET_CLIFF_RIGHT_SIGNAL		= 31
OI_PACKET_CARGO_BAY_DIGITAL_IN		= 32
OI_PACKET_CARGO_BAY_ANALOG_IN		= 33
OI_PACKET_CHARGING_SOURCES			= 34

OI_PACKET_GROUP_5					= 5
OI_PACKET_GROUP_MISC   				= OI_PACKET_GROUP_5

OI_PACKET_CURRENT_MODE				= 35
OI_PACKET_SONG_NUMBER				= 36
OI_PACKET_SONG_PLAYING				= 37
OI_PACKET_STREAM_PACKETS_COUNT		= 38
OI_PACKET_REQUESTED_VELOCITY		= 39
OI_PACKET_REQUESTED_RADIUS			= 40
OI_PACKET_REQUESTED_VELOCITY_RIGHT	= 41
OI_PACKET_REQUESTED_VELOCITY_LEFT	= 42

OI_PACKET_GROUP_6					= 6
OI_PACKET_GROUP_ALL					= OI_PACKET_GROUP_6

OI_PACKET_MIN                       = OI_PACKET_GROUP_0
OI_PACKET_MAX                       = OI_PACKET_REQUESTED_VELOCITY_LEFT

# 
# Packet sizes array, indexed by packets ids
#

OI_PACKET_SIZES = [
    26, 10, 6, 10, 14, 12, 52,
    1, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 2, 2,
    1, 2, 2, 1, 2, 2,
    2, 2, 2, 2, 2, 1, 2, 1,
    1, 1, 1, 1, 2, 2, 2, 2
]

# Packet header byte when streaming is used

OI_PACKET_HEADER		= 19

#
# Array of the npack formats for packet groups,
# indexed by the group id

PACKET_STRUCT_FORMATS = [
    None,
    '!BBBBBBBBBB',
    '!BBhh',
    '!BHhbHH',
    '!HHHHHBHB',
    '!BBBBhhhh',
    None
]
# compute the formats of compound packets
PACKET_STRUCT_FORMATS[0] = \
        PACKET_STRUCT_FORMATS[1] + \
        PACKET_STRUCT_FORMATS[2][1:] + \
        PACKET_STRUCT_FORMATS[3][1:]
PACKET_STRUCT_FORMATS[6] = \
        PACKET_STRUCT_FORMATS[0] + \
        PACKET_STRUCT_FORMATS[4][1:] + \
        PACKET_STRUCT_FORMATS[5][1:]

#
# Named tuples used for packet groups friendly unpacking
#

GroupPacket1 = namedtuple('GroupPacket1', [
    'bumps_and_wheels',
    'wall',
    'cliff_left',
    'cliff_front_left',
    'cliff_front_right',
    'cliff_right',
    'virtual_wall',
    'overcurrents',
    'unused1',
    'unused2'
])

GroupPacket2 = namedtuple('GroupPacket2', [
    'ir_byte',
    'buttons',
    'distance',
    'angle'
])

GroupPacket3 = namedtuple('GroupPacket3', [
    'charging_state',
    'voltage',
    'current',
    'battery_temp',
    'battery_charge',
    'battery_capacity'
])

GroupPacket4 = namedtuple('GroupPacket4', [
    'wall_signal',
    'cliff_left_signal',
    'cliff_front_left_signal',
    'cliff_front_right_signal',
    'cliff_right_signal',
    'user_dins',
    'user_ain',
    'charging_sources'
])

GroupPacket5 = namedtuple('GroupPacket5', [
    'oi_mode',
    'song_number',
    'song_playing',
    'nb_of_stream_packets',
    'velocity',
    'radius',
    'right_velocity',
    'left_velocity'
])

# Index of named tuples, keyed by the corresponding group packet id

GROUP_PACKET_CLASSES = [
    None,
    GroupPacket1,
    GroupPacket2,
    GroupPacket3,
    GroupPacket4,
    GroupPacket5
]

#
# Wait events
#

OI_WAIT_WHEEL_DROP              = 1
OI_WAIT_FRONT_WHEEL_DROP        = 2
OI_WAIT_LEFT_WHEEL_DROP         = 3
OI_WAIT_RIGHT_WHEEL_DROP        = 4
OI_WAIT_BUMP                    = 5
OI_WAIT_LEFT_BUMP               = 6
OI_WAIT_RIGHT_BUMP              = 7
OI_WAIT_VIRTUAL_WALL            = 8
OI_WAIT_WALL                    = 9
OI_WAIT_CLIFF                   = 10
OI_WAIT_LEFT_CLIFF              = 11
OI_WAIT_FRONT_LEFT_CLIFF        = 12
OI_WAIT_FRONT_RIGHT_CLIFF       = 13
OI_WAIT_RIGHT_CLIFF             = 14
OI_WAIT_HOME_BASE               = 15
OI_WAIT_BTN_ADVANCE             = 16
OI_WAIT_BTN_PLAY                = 17
OI_WAIT_DIN0                    = 18
OI_WAIT_DIN1                    = 19
OI_WAIT_DIN2                    = 20
OI_WAIT_DIN3                    = 21
OI_WAIT_PASSIVE_MODE            = 22

OI_WAIT_EVENT_MIN               = OI_WAIT_WHEEL_DROP
OI_WAIT_EVENT_MAX               = OI_WAIT_PASSIVE_MODE

OI_SONG_MAX_LEN = 16
