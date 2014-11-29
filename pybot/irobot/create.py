#!/usr/bin/env python
# -*- coding: utf-8 -*-
#pylint: disable=W0401,W0614

""" iRobote Create robot interface module.

This module provides the class modeling the robot and handling the communication
with the Create via its serial link.

Refer to the iRobot Create Open Interface reference document available on
iRobot Web site (http://www.irobot.com/filelibrary/pdfs/hrd/create/Create%20Open%20Interface_v2.pdf)
"""

import serial
import threading
import time
import struct
from collections import namedtuple
import logging

from .defs import *
from .utils import *
from pybot import log

__author__ = "Eric Pascual"
__email__ = "eric@pobot.org"
__copyright__ = "Copyright 2013, POBOT"
__version__ = "1.0"
__date__ = "June. 2013"
__status__ = "Development"
__license__ = "LGPL"

LedsState = namedtuple('LedsState', 'play advance pwr_color pwr_level')


class TraceOptions(object):
    TRACE_RXTX, TRACE_STREAM_FSA = range(2)
    _TRACE_FLAGS_ATTRS = ('rxtx', 'stream_fsa')

    def __init__(self, trace_format='h', flags=None):
        if trace_format == 'h':
            self.trace_fmt = lambda b : '%02x' % ord(b)
        elif trace_format == 'd':
            self.trace_fmt = lambda b : str(ord(b))
        else:
            raise ValueError('invalid trace format')

        for i, attr in enumerate(self._TRACE_FLAGS_ATTRS):
            setattr(self, attr, i in flags)


class CreateError(Exception):
    pass


class IRobotCreate(object):
    """ Models the Create robot and provides convenient methods to interact with it.

    Note that this class does not make use of the Command one to avoid stacking
    too many method calls. It could have be done to avoid duplicating in some
    way the command definitions, but the benefit didn't compensate the drawbacks.
    """

    def __init__(self, port, baudrate=57600, debug=False, simulate=False, trace=None):
        """ Constructor:

        Arguments:
            port:
                the serial port used for the serial link with it
            baudrate:
                serial link speed (default: 57600)
            debug:
                if True, activates the trace of serial data exchanges and various
                other debugging help
            simulate:
                if True, simulate data exchanges
            trace:
                display format of the communication trace. None = no trace, 'd' = decimal, 'h' = hex
                default : no trace
        """
        self._serial = serial.Serial(port, baudrate=baudrate, timeout=0.5)
        self._serial.flushInput()
        self._serial.flushOutput()

        # I/O serialization lock to be as much thread safe as possible
        self._lock = threading.Lock()

        self._debug = debug
        self._trace = trace
        if trace:
            if not isinstance(trace, TraceOptions):
                raise ValueError('trace options type mismatch')
            self._trace_func = trace.trace_fmt
        else:
            self._trace_func = None

        self._simulate = simulate

        self._log = log.getLogger(type(self).__name__)
        if self._debug:
            self._log.setLevel(logging.DEBUG)

        self._stream_listener = None
        self._timer = None

        # persistent status of the LEDs, so that it is possible to simplify the
        # state change requests by specifying only the modifications
        self._leds = None

        # Persistent settings of low side drivers PWM duty cycle
        # Initial state is supposed off (this will be set in constructor)
        self._pwm = [0]*3

        # events used to notify asynchronous operations
        self._evt_packets = threading.Event()
        self._evt_move = threading.Event()

    @property
    def serial(self):
        """ Access to the serial link instance."""
        return self._serial

    @property
    def debug_settings(self):
        """ Provides the current debug settings as a tuple containing :
            - the debug status
            - the trace settings
            - the simulate option state
        """
        return (self._debug, self._trace, self._simulate)

    @property
    def leds(self):
        """ The current state of the LEDs, as a LedState named tuple."""
        return self._leds

    @property
    def drivers_pwm(self):
        """ The current settings of the low side drivers PWM."""
        return self._pwm

    @property
    def evt_packets(self):
        """ The event object used to notify the availability of packets while in steam mode."""
        return self._evt_packets

    @property
    def evt_move(self):
        """ The event object used to notify that an asynchronous move is complete."""
        return self._evt_move

    @property
    def log(self):
        return self._log

    def _send_block(self, data):
        if type(data) is list:
            # stringify a byte list
            data = ''.join([chr(b) for b in data])

        if (self._trace_func and self._trace.rxtx) or self._simulate:
            self._log.debug(':Tx> %s', ' '.join(self._trace_func(b) for b in data))
        if self._simulate:
            return
        with self._lock:
            self._serial.flushInput()
            self._serial.write(data)
            self._serial.flushOutput()

    def _send_byte(self, byte):
        self._send_block(chr(byte))

    def _get_reply(self, nbytes):
        if self._simulate:
            print ('<Rx: -- no Rx data when running in simulated I/O mode --')
            return []

        with self._lock:
            data = ''
            cnt = 0
            maxwait = time.time() + self._serial.timeout
            while cnt < nbytes and time.time() < maxwait:
                data = data + self._serial.read(nbytes - cnt)
                cnt = len(data)

        if not data:
            raise CreateError('no reply received')

        if self._trace_func and self._trace.rxtx:
            rx = ' '.join(self._trace_func(b) for b in data)
            self._log.debug('<Rx: %s', rx)

        return data

    def start(self, mode=OI_MODE_PASSIVE):
        """ Initializes the Create.

        Includes a short pause for letting enough time to the beast for
        getting ready.

        Parameters:
            mode:
                the mode in which to place the Create (default: passive)

        Raises:
            ValueError if mode parameter is not of the expected ones
        """
        self._send_byte(OI_START)
        time.sleep(0.2)

        # check all is ok be getting the battery level
        mV, = struct.unpack('!H', self.get_sensor_packet(OI_PACKET_BATTERY_VOLTAGE))
        self._log.info('battery voltage : %.2f', mV / 1000.0)

        if mode != OI_MODE_PASSIVE:
            self.set_mode(mode)
        time.sleep(0.2)
        self.set_low_side_drivers(0)
        self.set_digital_outs(0)

    def reset(self):
        """ Soft reset of the Create."""
        self.stream_shutdown()
        self._send_byte(OI_SOFT_RESET)
        time.sleep(4)

    def shutdown(self):
        """ Shutdowns the robot actuators and streaming."""
        self.stream_shutdown()
        self.stop_move()
        self.set_low_side_drivers(0)
        self.set_digital_outs(0)

    def set_mode(self, mode):
        """ Sets the operating mode of the Create.

        Parameters:
            mode:
                the mode in which to place the Create (default: passive)

        Raises:
            ValueError if mode parameter is not of the expected ones
        """
        if mode in [OI_MODE_PASSIVE, OI_MODE_SAFE, OI_MODE_FULL]:
            self._send_byte(mode)
            # force the power LED to stay on in full mode (otherwise one may
            # think the beast has been turned off
            if mode == OI_MODE_FULL:
                self.set_leds(pwr_color=OI_LED_GREEN, pwr_level=OI_LED_FULL)
        else:
            raise ValueError('invalid mode (%s)' % mode)

    def passive_mode(self):
        """ Shorthand form for passive mode setting."""
        self.set_mode(OI_MODE_PASSIVE)

    def safe_mode(self):
        """ Shorthand form for safe mode setting."""
        self.set_mode(OI_MODE_SAFE)

    def full_mode(self):
        """ Shorthand form for full mode setting."""
        self.set_mode(OI_MODE_FULL)

    def _delayed_stop_move(self):
        if self._debug:
            self._log.debug('==> move complete')
        self._timer = None
        self.stop_move()
        # signal that the move is ended
        self._evt_move.set()

    def _cancel_delayed_stop_move(self):
        if self._timer:
            self._timer.cancel()
            self._timer = None
            if self._debug:
                self._log.debug('==> signaling move complete (in cancel)')
            self._evt_move.set()

    def _schedule_stop_move(self, delay):
        self._timer = threading.Timer(delay, self._delayed_stop_move)
        self._evt_move.clear()
        self._timer.start()
        if self._debug:
            self._log.debug('delayed stop scheduled in %s secs', self._timer.interval)

    def drive(self, velocity, radius=OI_DRIVE_STRAIGHT, distance=None):
        """ Makes the robot drive, based on speed and path curvature.

        If a distance is provided, a delayed stop is initiated (using a Timer),
        based on the time for this distance to be traveled using the requested
        speed.

        If no distance is provided, it is up to the caller to manage when to stop
        or change the move.

        In both cases, the methods exits immediately.

        Note:
            The distance checking is implemented using a poor man's strategy
            based on the ETA (estimated time of arrival).
            Another strategy would be to create a short script using a wait_distance
            command and run it. Although it has chances to be more accurate, this
            has the nasty drawback to "mute the line" while the script is running,
            and thus preventing any event (such as bumper hit) or commands to be
            exchanged.

        Parameters:
            velocity:
                robot's center velocity (in mm/sec)
            radius:
                radius (in mm) of the path to be followed
                (default: special value corresponding to a straight path
            distance:
                distance to be traveled (in mm). If provided, the method
                will exists only after this distance is supposed to be traveled,
                based on the requested speed. If no distance is provided, the
                method exists at once, leaving to robot moving.

        Raises:
            ValueError if velocity is outside valid range
        """
        if self._debug:
            self._log.debug("drive(velocity=%s, radius=%s, distance=%s)",
                            velocity, radius, distance)

        # be sure we will not get a pending stop during the move
        self._cancel_delayed_stop_move()

        if distance is not None and distance == 0:
            return

        if not -MAX_ABSOLUTE_SPEED <= velocity <= MAX_ABSOLUTE_SPEED:
            raise ValueError('invalid velocity (%f)' % velocity)

        self._send_block([OI_DRIVE] + int16_to_bytes(velocity) + int16_to_bytes(radius))
        if distance:
            self._schedule_stop_move(time_for_distance(distance, velocity))

    def drive_direct(self, left_vel, right_vel):
        """ Makes the robot drive by directly setting the wheel speeds.

        No distance control is proposed here.

        Parameters:
            left_vel, right_vel:
                wheel velocities (in mm/sec)
        """
        if self._debug:
            self._log.debug("drive_direct(left_vel=%s, right_vel=%s)", left_vel, right_vel)

        # be sure we will not get a pending stop during the move
        self._cancel_delayed_stop_move()

        self._send_block([OI_DRIVE_DIRECT] + int16_to_bytes(left_vel) + int16_to_bytes(right_vel))

    def stop_move(self):
        """ Stops any current move."""
        self._cancel_delayed_stop_move()
        self.drive_direct(0, 0)

    def spin(self, velocity, spin_dir=OI_SPIN_CW, angle=None):
        """ Makes the robot spin on itself at a given speed.

        If a spin angle is provided, the method will wait for the time for this angle
        to be reached (based on the requested speed) and then stops the robot and exit.
        In addition, the provided spin direction (if any) is ignored, and replaced by
        the one inferred from the angle sign (positive = CCW)

        If no angle is provided, the move is initiated and the method exits immediately,
        leaving the robot moving. It is the responsibility of the caller to handle
        what should happen then.

        See drive() method documentation for implementation note about the strategy
        used to track the angle.

        Parameters:
            velocity:
                robot's wheels velocity (in mm/sec)
            spin_dir:
                the spin direction (CW or CCW) (default: CW)
            angle:
                optional spin angle (in degrees)

        Raises:
            ValueError if spin direction is invalid
        """
        if self._debug:
            self._log.debug("spin(velocity=%s, spin_dir=%s, angle=%s)",
                            velocity, spin_dir, angle
                            )

        # be sure we will not get a pending stop during the move
        self._cancel_delayed_stop_move()

        if angle is not None:
            if angle != 0:
                spin_dir = OI_SPIN_CCW if angle > 0 else OI_SPIN_CW
            else:
                return
        elif spin_dir not in [OI_SPIN_CCW, OI_SPIN_CW]:
            raise ValueError('invalid spin direction (%d)' % spin_dir)

        self.drive(velocity, spin_dir)
        if angle:
            self._schedule_stop_move(time_for_angle(angle, velocity))

    def set_leds(self, play=None, advance=None, pwr_color=None, pwr_level=None):
        """ Changes the state of the Create LEDs.

        Parameters:
            play:
                PLAY led state (default: unchanged)
            advance:
                ADVANCE led state (default: unchanged)
            pwr_color:
                the color of the POWER LED (default: unchanged)
            pwr_level:
                the intensity of the POWER LED (default: unchanged)
        """
        if play is None:
            play = self._leds.play if self._leds else False
        if advance is None:
            advance = self._leds.advance if self._leds else False
        if pwr_color is None:
            pwr_color = self._leds.pwr_color if self._leds else OI_LED_GREEN
        else:
            pwr_color &= 0xff
        if pwr_level == None:
            pwr_level = self._leds.pwr_level if self._leds else 0
        else:
            pwr_level &= 0xff

        led_bits = 0
        if play:
            led_bits |= OI_LED_PLAY
        if advance:
            led_bits |= OI_LED_ADVANCE

        self._send_block([OI_LEDS, led_bits, pwr_color, pwr_level])
        self._leds = LedsState(play, advance, pwr_color, pwr_level)

    def set_digital_outs(self, states):
        """ Changes the state of the Create digital outputs.

        Parameters:
            states:
                a bit mask containing the state of the DOUTs
        """
        self._send_block([OI_DIGITAL_OUTPUTS,
                        states & OI_ALL_DOUTS
                        ])

    def set_low_side_drivers(self, states):
        """ Changes the ON/OFF state of the Create low-side drivers.

        Parameters:
            states:
                a bit mask containing the state of the LSDs
        """
        self._send_block([OI_LOW_SIDE_DRIVERS,
                        states & OI_ALL_DRIVERS
                        ])
        for num, mask in enumerate([OI_DRIVER_0, OI_DRIVER_1, OI_DRIVER_2]):
            if states & mask == 0:
                self._pwm[num] = 0

    def low_side_driver_pwm(self, drv0=None, drv1=None, drv2=None): #pylint: disable=W0613
        """ Sets the PWM duty cycle of low side drivers.

        Only specified settings are changed, the other ones being kept
        as is. Duty cycles are provided as a number in range [0, 100]

        Parameters:
            drv0, drv1, drv2:
                duty cycle of the corresponding driver
                (default: None => unchanged)

        Raises:
            ValueError if out of range value provided
        """
        # work on cached values to be able to rollback in case of trouble
        wrk = self._pwm

        # a bit of introspection to avoid writing repetitive code
        for drvnum in xrange(3):
            pwm = locals()['drv%d' % drvnum] # BEWARE: update this if arg names are modified
            if pwm is None:
                # if parameter is unset, use the current setting
                pwm = self._pwm[drvnum]
            else:
                if 0 <= pwm <= 100:
                    # don't update the saved sttings now, in case one of the
                    # parameters is invalid
                    wrk[drvnum] = pwm
                else:
                    raise ValueError('invalid PWM setting (%s) for driver %d' % (pwm, drvnum))

        self._send_block(
            [OI_PWM_LOW_SIDE_DRIVERS] +
            # PWMs are sent in reverse sequence
            [int(pwm * 128. / 100.) & 0xff for pwm in wrk[::-1]]
        )
        # we can commit the new settings now
        self._pwm = wrk

    def send_ir(self, data):
        """ Emits an IR byte.

        Parameters:
            data:
                the byte to be sent
        """
        self._send_block([OI_SEND_IR,
                        data & 0xff
                        ])

    def get_sensor_packet(self, packet_id):
        """ Gets a sensor packet or packets group.

        Parameters:
            packet_id:
                the id of the requested packet or group

        Returns:
            the reply returned by the Create

        Raises:
            ValueError if invalid packet id
        """
        if not OI_PACKET_MIN <= packet_id <= OI_PACKET_MAX:
            raise ValueError('invalid packet id (%d)' % packet_id)

        self._send_block([OI_READ_SENSORS,
                        packet_id & 0xff
                        ])
        return self._get_reply(OI_PACKET_SIZES[packet_id])


    def get_unpacked_sensor_packet(self, packet_id):
        """ Convenience method which returns an unpacked form for some
        of the packet groups.

        It only works for groups 1 to 5.

        Parameters:
            packet_id:
                the requested packet group id

        Returns:
            the reply unpacked as the corresponding namedtuple

        Raises:
            ValueError in packet_id is out of range
        """
        if not 1 <= packet_id <= 5:
            raise ValueError('out of range packet id (%d)' % packet_id)

        data = self.get_sensor_packet(packet_id)
        return GROUP_PACKET_CLASSES[packet_id]._make(   #pylint: disable=W0212
            struct.unpack_from(PACKET_STRUCT_FORMATS[packet_id], data)
        )

    def get_sensor_packet_list(self, packet_ids):
        """ Gets a list of sensor packets.

        Parameters:
            packed_ids:
                a list containing the ids of the requested packets

        Returns:
            a list containing the corresponding packets returned by the Create
        """
        self._send_block([OI_READ_SENSOR_LIST, len(packet_ids)] + packet_ids)
        nbytes = sum([OI_PACKET_SIZES[id_] for id_ in packet_ids if 0 <= id_ <= 42])
        return self._get_reply(nbytes)

    def get_unpacked_sensor_packet_list(self, packet_ids):
        """ Same as get_unpacked_sensor_packet, but for a list of packets.

        Same restrictions apply for the packet ids.

        Parameters:
            packet_ids:
                a list containing the ids of the requested packets
        """
        data = self.get_sensor_packet_list(packet_ids)
        res = []
        pos = 0

        for id_ in packet_ids:
            if not 1 <= id_ <= 5:
                raise ValueError('out of range packet id (%d)' % id_)

            lg = OI_PACKET_SIZES[id_]
            res.append(
                GROUP_PACKET_CLASSES[id_]._make(        #pylint: disable=W0212
                    struct.unpack_from(
                        PACKET_STRUCT_FORMATS[id_],
                        data[pos : pos + lg]
                    )
                )
            )
            pos += lg
        return res

    def get_buttons(self):
        """ Gets the current state of the buttons, as the list of pressed ones.

        The returned list contains the button ids (OI_BUTTONS_xxx) and can be
        empty if no button is currently pressed.
        """
        reply = self.get_sensor_packet(OI_PACKET_BUTTONS)
        return byte_to_buttons(ord(reply[0]))

    def get_bumpers(self):
        """ Gets the current state of the bumper, as the list of pressed parts.

        The returned list contains the bumper part ids (OI_BUMPER_xxx) and can be
        empty if the bumper is currently not pressed.
        """
        reply = self.get_sensor_packet(OI_PACKET_BUMPS_AND_WHEEL_DROPS)
        return byte_to_bumpers(ord(reply[0]))

    def get_wheel_drops(self):
        """ Gets the current state of the wheel drop sensors, as the list of
        dropped ones.

        The returned list contains the drop sensor (OI_WHEEL_DROP_xxx, OI_CASTER_DROP)
        and can be empty if all wheels are on the ground.
        """
        reply = self.get_sensor_packet(OI_PACKET_BUMPS_AND_WHEEL_DROPS)
        return byte_to_wheel_drops(ord(reply[0]))

    def define_script(self, script):
        """ Records a script to be played later.

        The Command class can be used to make easier the building of the
        script bytes sequence.

        Parameters:
            script:
                the script, as its bytes sequence
        """
        self._send_block([OI_SCRIPT_DEFINE, len(script) & 0xff] + script)

    def play_script(self):
        """ Plays the previously recorded script."""
        self._send_byte(OI_SCRIPT_PLAY)

    def _start_stream_listener(self, packet_event):
        if self._debug:
            self._log.debug('start stream listener')
        self._stream_listener = _StreamListener(self, packet_event)
        self._stream_listener.start()

    def _kill_stream_listener(self):
        if self._debug:
            self._log.debug('kill stream listener')
        self._stream_listener.stop()
        self._stream_listener.join(1)
        self._stream_listener = None

    def _is_streaming(self):
        return self._stream_listener is not None

    def stream_packets(self, packet_ids):
        """ Starts the continuous stream of sensor packets.

        The evt_packets event instance provided by this class is used to signal and
        communicate the received events. Caller can wait on it to synchronize their
        process.

        Parameters:
            packet_ids:
                the list of the ids of the packet to be streamed
        """
        if not self._stream_listener:
            self._start_stream_listener(self._evt_packets)
        self._send_block([OI_STREAM_SENSOR_LIST, len(packet_ids) & 0xff] + packet_ids)

    def stream_pause(self):
        """ Pauses the packets streaming."""
        self._send_block([OI_STREAM_PAUSE_RESUME, 0])

    def stream_resume(self):
        """ Resumes the packets streaming."""
        self._send_block([OI_STREAM_PAUSE_RESUME, 1])

    def stream_shutdown(self):
        """ Shutdowns an ongoing packet streaming.

        It is a good habit to call this method before leaving the application,
        to avoid letting orphan threads alive. Calling it if not streaming is
        active does not harm, since not done.
        """
        if self._stream_listener:
            if self._debug:
                self._log.debug('shutdown packet streaming')
            self.stream_pause()
            self._kill_stream_listener()

    def song_record(self, song_num, data):
        """ Records a song.

        Refer to iRobot Create OI documentation (p.11) for songs data encoding.

        Parameters:
            song_num:
                the song number (0 <= n <= 15)
            data:
                the song data

        Raises:
            ValueError if song number is invalid
        """
        if not 0 <= song_num <= 15:
            raise ValueError('invalid song _number (%s)' % song_num)

        self._send_block([OI_SONG_RECORD, song_num, len(data) / 2] + data)

    def song_sequence_record(self, start_num, data):
        """ Records a long song by spliting it in consecutive songs.

        Parameters:
            start_num:
                number of the first song to be recorded
            data:
                the full song data

        Returns:
            the range of the recorded songs number
        """
        sequence = [ data[i:i+32] for i in xrange(0, len(data), 32) ]
        for i, song_data in enumerate(sequence):
            self.song_record(start_num + i, song_data)
            if self._debug:
                self._log.debug('sequence %d recorded with data=%s', i, song_data)
        return [start_num + i for i in xrange(len(sequence))]

    def song_play(self, song_num):
        """ Plays a recorded song.

        Parameters:
            song_num:
                the song number (0 <= n <= 15)

        Raises:
            ValueError if song number is invalid
        """
        if not 0 <= song_num <= 15:
            raise ValueError('invalid song _number (%s)' % song_num)

        self._send_block([OI_SONG_PLAY, song_num])

    def song_sequence_play(self, song_numbers):
        """ Plays a sequence of songs.

        Parameters:
            song_numbers:
                a list of song numbers (0 <= n <= 15)

        Raises:
            ValueError if song number is invalid
            IndexError if sequence is empty
        """
        # pause an ongoing streaming, since will conflict with song end test
        # Note: we do it unconditionally, in case a streaming was started by a
        # previous run, and not stopped when exiting the process.
        self.stream_pause()

        for num in song_numbers:
            self.song_play(num)
            if self._debug:
                self._log.debug("playing song %d", num)
            # wait for the song is over before starting next one if any
            while ord(self.get_sensor_packet(OI_PACKET_SONG_PLAYING)[0]):
                time.sleep(0.01)
            if self._debug:
                self._log.debug("  --> finished")

        if self._is_streaming():
            if self._debug:
                self._log.debug("resume streaming")
            self.stream_resume()

#
# Stream listener FSM states (internal use)
#

_STATE_IDLE = 0
_STATE_GOT_HEADER = 1
_STATE_AT_PACKET_START = 2
_STATE_IN_PACKET = 3
_STATE_IN_CHECKSUM = 4


class _StreamListener(threading.Thread):
    """ The packet stream listener.

    This class is internal use only and is not supposed to be used
    by the application.
    """
    def __init__(self, robot, packet_event):
        threading.Thread.__init__(self)
        self.name = 'stream_listener'
        self._stopevent = threading.Event()
        self._robot = robot
        self._packet_event = packet_event

    def run(self):
        _serial = self._robot.serial
        _debug, trace, _simulate = self._robot.debug_settings
        _serial.flushInput()
        state = _STATE_IDLE
        packets = []
        total_expected = expected_bytes = 0
        packet_id = 0
        packet_data = ''
        while not self._stopevent.isSet():
            while _serial.inWaiting():
                b = ord(_serial.read()[0])
                if trace and trace.stream_fsa:
                    self._robot.log.debug(
                        '<Rx: %d - state=%d - total_expected=%d - expected_bytes=%d',
                        b, state, total_expected, expected_bytes
                )
                if state == _STATE_IDLE:
                    if b == OI_PACKET_HEADER:
                        packets = []
                        state = _STATE_GOT_HEADER

                elif state == _STATE_GOT_HEADER:
                    total_expected = b
                    # if trace:
                    #     self._robot.log.debug('at stream frame start (expecting %d bytes total)', total_expected)
                    state = _STATE_AT_PACKET_START

                elif state == _STATE_AT_PACKET_START:
                    packet_id = b
                    packet_data = ''
                    expected_bytes = OI_PACKET_SIZES[packet_id]
                    # if trace:
                    #     self._robot.log.debug('at stream packet %d start. (expecting %d bytes)', packet_id, expected_bytes)
                    total_expected -= 1
                    state = _STATE_IN_PACKET

                elif state == _STATE_IN_PACKET:
                    packet_data += chr(b)
                    total_expected -= 1
                    expected_bytes -= 1
                    if expected_bytes == 0:
                        if trace and trace.stream_fsa:
                            self._robot.log.debug('packet %d available', packet_id)
                        packets.append((packet_id, packet_data))
                        if total_expected == 0:
                            state = _STATE_IN_CHECKSUM
                        else:
                            state = _STATE_AT_PACKET_START

                elif state == _STATE_IN_CHECKSUM:
                    # don't care checking for the moment

                    # notify a set of packets is available
                    self._packet_event.packets = packets
                    self._packet_event.set()
                    if trace and trace.stream_fsa:
                        self._robot.log.debug('packets availability signaled')

                    state = _STATE_IDLE

            # check if someone requested us to stop
            self._stopevent.wait(0.01)

        # avoid some listener keep waiting forever
        self._packet_event.packets = None
        self._packet_event.set()

    def stop(self):
        self._stopevent.set()
