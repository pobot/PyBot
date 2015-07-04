**WARNING** : This repository is being reworked to split it into its independant components, for a better modularity. 

*It thus must be considered as deprecated and no more maintained as is.*

A collection of Python packages for interfacing hardwares with a RasPi (but not
only).

These packages are primarily intended for robotics activities but can be used
for any other purpose of course. 

All this stuff is not intended to be used by blindly installing it, although
it can be, but more as a source of inspiration and documentation.

The repository contains 3 parts :
- pybot/ 

    Contains the Python package, and should be deployed in dist-packages
    directory, or in any other location in the Python search path. 

    Note that you can just extract the scripts you need, since some of them
    are more or less stand-alone

- bin/
    
    Some command line scripts (for instance, Robotis Dynamixel servos tools) 
    and basic demos

- apps/
    
    Some applications we wrote, based on PyBot library


Enjoy.
