#!/usr/bin/env python
# -*- coding: utf-8 -*-
#pylint: disable=W0401,W0614

""" iRobote Create robot interface module.

This module provides a class modeling the robot and handling the communication
with the Create via its serial link. It also includes various additional
definitions and helpers.

Refer to the iRobot Create Open Interface reference document available on
iRobot Web site (http://www.irobot.com/filelibrary/pdfs/hrd/create/Create%20Open%20Interface_v2.pdf)
"""

__author__ = "Eric Pascual"
__email__ = "eric@pobot.org"
__copyright__ = "Copyright 2013, POBOT"
__version__ = "1.0"
__date__ = "June. 2013"
__status__ = "Development"
__license__ = "LGPL"

import math

from .defs import *


def inverse(b):
    """ Two-complement inverse value computation, use to wait
    for the negation of a given event in the above list."""
    return (~b & 0xff) + 1


def int16_to_bytes(v):
    """ Convenience function to convert a 16 bits int into the corresponding
    bytes sequence."""
    i16 = int(v)
    return [(i16 & 0xff00) >> 8, i16 & 0xff]


def byte_to_buttons(byte):
    """ Convenience function to convert a bit mask byte into a list of button ids."""
    return [b for b in [OI_BUTTONS_PLAY, OI_BUTTONS_ADVANCE] if b & byte]


def byte_to_bumpers(byte):
    """ Convenience function to convert a bit mask byte into a list of bumper parts."""
    return [b for b in [OI_BUMPER_LEFT, OI_BUMPER_RIGHT] if b & byte]


def byte_to_wheel_drops(byte):
    """ Convenience function to convert a bit mask byte into a list of wheel drop sensors."""
    return [b for b in [OI_WHEEL_DROP_LEFT, OI_WHEEL_DROP_RIGHT, OI_CASTER_DROP] if b & byte]


def byte_to_drivers(byte):
    """ Convenience function to convert a bit mask byte into a list of driver ids."""
    return [b for b in [OI_DRIVER_0, OI_DRIVER_1, OI_DRIVER_2] if b & byte]


def leds_to_byte(leds):
    """ Convenience function to convert a list of LEDs to the corresponding bit mask."""
    return reduce(lambda x, y : x | y, int(leds))


def dump_group_packets(gpackets):
    """ Helper function printing a list of packets in a friendly way.

    Works only for packets having a namedtuple associated representation.
    """
    if type(gpackets) is not list:
        gpackets = [gpackets]
    for pkt in gpackets:
        print("%s : " % type(pkt).__name__)
        for k, v in pkt._asdict().iteritems():          #pylint: disable=W0212
            print(" - %s : %s" % (k, v))


def time_for_distance(dist_mm, velocity):
    """ Returns the theoretical time required to travel a given distance
    at a given speed."""
    return abs(float(dist_mm) / float(velocity))


def time_for_angle(angle_deg, velocity):
    """ Returns the theoretical time required to spin  a given angle
    with a given wheel speed."""
    return time_for_distance(
        math.radians(angle_deg) * WHEEL_TO_WHEEL_DIST / 2,
        velocity
    )


def make_script(*commands):
    """ Returns the bytes sequence of a script composed of the given commands."""
    return reduce(lambda x, y: x + y, commands)



class Song(object):
    """ Song helper
    """
    _MUSIC_SCALE = ('C', 'C#', 'D', 'D#', 'E', 'F', 'F#', 'G', 'G#', 'A', 'A#', 'B')
    _REST = 0

    def __init__(self):
        self._score = []

    @staticmethod
    def note_number(note, octave=0):
        note = note.upper()
        if note not in Song._MUSIC_SCALE:
            raise ValueError('invalid note')
        if not 0 <= octave <= 8:
            raise ValueError('invalid octave')

        res = 24 + octave * len(Song._MUSIC_SCALE) + Song._MUSIC_SCALE.index(note)
        if 31 <= res <= 127:
            return res
        else:
            raise ValueError('out of range note/octave')

    @property
    def score(self):
        return self._score

    def add_note(self, note, octave, duration):
        self._score.append(Song.note_number(note, octave) if note else Song._REST)
        self._score.append(duration)
        return self

    def clear(self):
        self._score = []

    def encode(self, notes):
        self._score = []
        for note, octave, duration in notes:
            self.add_note(note, octave, duration)

    def split(self):
        lg = 2 * OI_SONG_MAX_LEN
        return [
            self._score[i:i+lg] for i in xrange(0, len(self._score), lg)
        ]


    def change_tempo(self, ratio):
        if not self._score:
            return

        for i in xrange(0, len(self._score), 2):
            self._score[i] = int(self._score[i] * ratio) & 0xff
