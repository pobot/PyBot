A demonstrator of goniometric location computation by using
range finders based scanning beacons.

Developed for illustrating the application of simple
trigonometric functions for young students.

The application is composed of :

- a hardware part, made of two scanning beacons built
around AX12 Dynamixel servos (Robotis) and infra-red
range finders. 

- a Python controlling sofware, managing servos and sensors,
running on a headless RasPi

- a PC application, featuring a real-time animated
graphical representation of the contraption and displaying
the tracked object location, computed on the basis of 
echos detected by the sensors

PC software and RasPi embedded code communicate over the network.

The Raspberry Wifi is configured in AP mode, and Avahi is used
so that the GUI application does not need to know any fixed
IP address.

See contraption details on POBOT robotics association web site
(http://www.pobot.org/Balises-goniometriques-2013-2eme.html)
