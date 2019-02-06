# aspi
Library of useful data collection utilities to be used with AstroPi computers for any other scientific experiments.

################################################################################

AsPi 

################################################################################

AsPi is a small library that aims to provide the simplest possible data collection
interface to the sensors available in the AstroPi computer, taking into account 
the most strict requirements of AstroPi based experiments running in the ISS.
Allowing the scientists to concentrate on the science experiment aspects and in
the respective data analysis

The main objective is to allow scientists of all ages with little or no coding 
background to harness all the data the AstroPi can provide in a very simple 
and through a "just works" approach. 

It works by periodically taking measurements from all the AstroPi sensors and 
storing the values in a CSV file. It can also, optionally, take photographs
using the AstroPi camera and storing them in files (functionality DISABLED
by default).

The following data is collected from AstroPi sensors:

    * Temperature
    * Humidity
    * Pressure
    * Orientation
    * Gyroscope
    * Accelerometer
    * Compass
    * ISS Position (calculated via pyephem)
    * Motion Sensor (using the AstroPi camera)

The AsPi library is designed to allow the program using it, to run completely 
automatically and unnattended for a specified amount of time, requiring 
absolutely no interaction from the operator other that to start the program.
 
The AsPi library provides a flat and uniform view across all the multiple
sensors and devices available in the AstroPi.  

Usage:

    datalogger = AsPiLogger()
    datalogger.start()
