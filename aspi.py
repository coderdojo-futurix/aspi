
####################################################################################
#
# Released under MIT License
#
# Copyright (c) 2019 CoderDojo Futurix <coderdojo@futurix.pt>
#
# Permission is hereby granted, free of charge, to any person obtaining a copy of 
# this software and associated documentation files (the "Software"), to deal in 
# the Software without restriction, including without limitation the rights to use,
# copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the 
# Software, and to permit persons to whom the Software is furnished to do so, 
# subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all 
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
#  INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A 
# PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT 
# HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION 
# OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE 
# SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
#


'''
################################################################################

AsPi 

################################################################################

(fork me at: https://github.com/coderdojo-futurix/aspi)

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

'''


from gpiozero import CPUTemperature
from datetime import datetime 
from logging import Formatter
from logzero import logger
from pisense import SenseHAT
from ephem import readtle, degrees
from threading import Timer, Thread
from queue import Queue
from collections import OrderedDict
import logzero
import os
import time
import locale
import math
import sys
import signal
import picamera
import picamera.array
import numpy as np

# Global AstroPi device objects
sense_hat = SenseHAT()
cpu = CPUTemperature()
camera = picamera.PiCamera()

# Default values
MIN_LOG_PERIOD_IN_SECS = 2
MIN_IMG_PERIOD_IN_SECS = 5
SHUTDOWN_TIMEOUT_IN_SECS = 3 * 60
DEFAULT_DURATION_IN_SECS = 3 * 60 * 60 - SHUTDOWN_TIMEOUT_IN_SECS
DEFAULT_SIZE_PER_LOGFILE_IN_BYTES = 30*1024*1024
DEFAULT_LOG_PERIOD_IN_SECS = 5
DEFAULT_IMG_PERIOD_IN_SECS = 10
DEFAULT_LOGFILE_PREFIX = "sense_hat_logger"
PICAMERA_SENSOR_MODE_2_RESOLUTION = ( 2592, 1944 )
ASTROPI_ORIENTATION = 270
ENABLE_DEBUG = False
NO_READING=-1
LOG_FORMAT='%(asctime)-15s.%(msecs)03d,%(message)s'
DATE_FORMAT='%Y-%m-%d %H:%M:%S'        
TIMESTAMP_FIELD = "timestamp"
TIMESTAMP_FORMAT='{:%Y-%m-%d_%Hh%Mm%Ss}'
DEGREES_PER_RADIAN = 180.0 / math.pi
CURRENT_DIR = os.path.dirname(os.path.realpath(__file__))
LOGFILE_EXT = 'csv'
NO_TIMESTAMP_FORMATTER = Formatter()


def get_timestamp():
    '''
    Simple method to return a formatted timestamp of the current time
    '''
    return TIMESTAMP_FORMAT.format(datetime.now())

class AsPi:
    '''
    Utility class that holds constants for the sensors names and respective values units
    and manages the end of the program 
    '''
    # Flag to indicate the program has ended
    _ended = False

    # Holds the termination timer after the end is called
    shutdowntimer = None


    def isShuttingDown():
        '''
        Indicates if the program has ended and the termination timer is active
        '''
        return not AsPi.shutdowntimer is None

    def terminate():
        '''
        Self terminates by issuing a SIGTERM to itself
        '''
        os.kill(os.getpid(),signal.SIGTERM)

    def end():
        '''
        Call to gracefully end of the program. A termination timer is also started. 
        If the program cleanup actions are not done in SHUTDOWN_TIMEOUT_IN_SECS seconds, 
        self termination is issued.
        '''
        if not AsPi.hasEnded():
            AsPi._ended = True
            print("Forcing termination in " + str(SHUTDOWN_TIMEOUT_IN_SECS) + " secs")
            AsPi.shutdowntimer = Timer( SHUTDOWN_TIMEOUT_IN_SECS , AsPi.terminate )
            AsPi.shutdowntimer.start()

    def hasEnded():
        '''
        Indicates if the program ended
        '''
        return AsPi._ended

    # Sensor names constants
    SENSOR_CPU_TEMP = "cpu_temp"
    SENSOR_TEMPERATURE = "temperature"
    SENSOR_PRESSURE = "pressure"
    SENSOR_HUMIDITY = "humidity"
    SENSOR_COMPASS_X = "compass_x"
    SENSOR_COMPASS_Y = "compass_y"
    SENSOR_COMPASS_Z = "compass_z"
    SENSOR_GYRO_X = "gyro_x"
    SENSOR_GYRO_Y = "gyro_y"
    SENSOR_GYRO_Z = "gyro_z"
    SENSOR_ACCEL_X = "accel_x"
    SENSOR_ACCEL_Y = "accel_y"
    SENSOR_ACCEL_Z = "accel_z"
    SENSOR_PITCH = "pitch"
    SENSOR_ROLL = "roll"
    SENSOR_YAW = "yaw"
    SENSOR_LAT = "lat"
    SENSOR_LON = "long"
    SENSOR_ELEVATION = "elevation"
    SENSOR_ECLIPSED = "eclipsed"
    SENSOR_MOTION = "motion"
    SENSOR_USERDATA = "userdata"

    # Units constants
    UNITS_DEGREES_CELSIUS = "°C"
    UNITS_RADIANS = "rad"
    UNITS_RADIANS_PER_SEC = UNITS_RADIANS + "/sec"
    UNITS_STANDARD_GRAVITIES = "g"
    UNITS_MICRO_TESLAS = "uT"
    UNITS_MILLIBARS = "mbar"
    UNITS_PERC_RELATIVE_HUMIDITY = "%RH"
    UNITS_DEGREES = "°"
    UNITS_METERS = "m"
    UNITS_BOOL = "bool"
    UNITS_COUNT = "n"
    UNITS_STR = "str"

    # Units of the values reported by each sensor
    UNITS = OrderedDict( [
        ( SENSOR_CPU_TEMP    , UNITS_DEGREES_CELSIUS        ) ,
        ( SENSOR_TEMPERATURE , UNITS_DEGREES_CELSIUS        ) ,
        ( SENSOR_PRESSURE    , UNITS_MILLIBARS              ) ,
        ( SENSOR_HUMIDITY    , UNITS_PERC_RELATIVE_HUMIDITY ) ,
        ( SENSOR_COMPASS_X   , UNITS_MICRO_TESLAS           ) ,
        ( SENSOR_COMPASS_Y   , UNITS_MICRO_TESLAS           ) ,
        ( SENSOR_COMPASS_Z   , UNITS_MICRO_TESLAS           ) ,
        ( SENSOR_GYRO_X      , UNITS_RADIANS_PER_SEC        ) ,
        ( SENSOR_GYRO_Y      , UNITS_RADIANS_PER_SEC        ) ,
        ( SENSOR_GYRO_Z      , UNITS_RADIANS_PER_SEC        ) ,
        ( SENSOR_ACCEL_X     , UNITS_STANDARD_GRAVITIES     ) ,
        ( SENSOR_ACCEL_Y     , UNITS_STANDARD_GRAVITIES     ) ,
        ( SENSOR_ACCEL_Z     , UNITS_STANDARD_GRAVITIES     ) ,
        ( SENSOR_PITCH       , UNITS_RADIANS                ) ,
        ( SENSOR_ROLL        , UNITS_RADIANS                ) ,
        ( SENSOR_YAW         , UNITS_RADIANS                ) ,
        ( SENSOR_LAT         , UNITS_DEGREES                ) ,
        ( SENSOR_LON         , UNITS_DEGREES                ) ,
        ( SENSOR_ELEVATION   , UNITS_METERS                 ) ,
        ( SENSOR_ECLIPSED    , UNITS_BOOL                   ) ,
        ( SENSOR_MOTION      , UNITS_COUNT                  ) , 
        ( SENSOR_USERDATA    , UNITS_STR                    )
    ])

    # list with all sensor names
    ALL_SENSORS = UNITS.keys()


class AsPiResult:
    '''
    Class that stores one and only one value to safely exchanve values between threads
    '''
    def __init__(self):
        self.result = Queue(maxsize = 1)
    
    def put( self, data ):
        try:
            self.result.get_nowait()
        except:
            pass
        finally:
           self.result.put( data )

    def get( self , timeout=1):
        data = None
        try:
            data = self.result.get( timeout )
        except:
            pass
        finally:
            if AsPi.hasEnded():
                return None
            return data

class AsPiSensors:
    '''
    Class that makes takes measurements from all sensors 
    '''
    userData = AsPiResult()
    lastAsPiSensorsReading = AsPiResult()
    cpu = CPUTemperature()
    iss = readtle(
            'ISS (ZARYA)' ,
            '1 25544U 98067A   19027.92703822  .00001504  00000-0  30922-4 0  9992',
            '2 25544  51.6413 338.8011 0004969 323.5710 139.9801 15.53200917153468'
        )

    def __init__(self, hat, selected_sensors = AsPi.ALL_SENSORS ):
        self.hat = hat
        self.selected_sensors = selected_sensors        
        
    def _get_latlon():
        AsPiSensors.iss.compute()
        lat = AsPiSensors.iss.sublat * DEGREES_PER_RADIAN
        lon = AsPiSensors.iss.sublong * DEGREES_PER_RADIAN
        alt = AsPiSensors.iss.elevation
        ecl = AsPiSensors.iss.eclipsed
        return lat,lon, alt, ecl

    def read(self):
        
        envreading = self.hat.environ.read()
        imureading = self.hat.imu.read()
        latitude, longitude, elevation, eclipsed = AsPiSensors._get_latlon()

        self.readings = { 
            AsPi.SENSOR_CPU_TEMP : AsPiSensors.cpu.temperature,
            AsPi.SENSOR_TEMPERATURE : envreading.temperature, 
            AsPi.SENSOR_PRESSURE : envreading.pressure,
            AsPi.SENSOR_HUMIDITY : envreading.humidity,
            AsPi.SENSOR_COMPASS_X : imureading.compass.x ,
            AsPi.SENSOR_COMPASS_Y : imureading.compass.y ,
            AsPi.SENSOR_COMPASS_Z : imureading.compass.z ,
            AsPi.SENSOR_GYRO_X : imureading.gyro.x ,
            AsPi.SENSOR_GYRO_Y : imureading.gyro.y ,
            AsPi.SENSOR_GYRO_Z : imureading.gyro.z ,
            AsPi.SENSOR_ACCEL_X : imureading.accel.x ,
            AsPi.SENSOR_ACCEL_Y : imureading.accel.y ,
            AsPi.SENSOR_ACCEL_Z : imureading.accel.z ,
            AsPi.SENSOR_PITCH : imureading.orient.pitch ,
            AsPi.SENSOR_ROLL : imureading.orient.roll,
            AsPi.SENSOR_YAW : imureading.orient.yaw,
            AsPi.SENSOR_LAT : latitude,
            AsPi.SENSOR_LON : longitude,
            AsPi.SENSOR_ELEVATION : elevation,
            AsPi.SENSOR_ECLIPSED : eclipsed,
            AsPi.SENSOR_MOTION : MotionAnalyser.occurrences, 
            AsPi.SENSOR_USERDATA :  AsPiSensors.userData.get( timeout=0)
      
        }

        # Reset motion detection occurences count
        MotionAnalyser.occurrences = 0

        AsPiSensors.lastAsPiSensorsReading.put( self.readings )

        # Return readings from selected sensors only
        return self.__selected_sensors_only( self.readings )

    def __selected_sensors_only(self, readings ):
        return list( map( lambda sensor: readings[ sensor ],  self.selected_sensors ))


class AsPiTimer:
    '''
    Recurrent Timer. It's the same as python threading timer class but a recurring one.
    Everytime it fires, calls the callback and sets another Timer. It keeps doing that until 
    it's cancelled.   
    '''
    def __init__( self, periodInSecs=DEFAULT_LOG_PERIOD_IN_SECS, func=None ):
        self.periodInSecs = periodInSecs
        self.func = func
        self.__create_timer()

    def __create_timer(self):
        self.aspitimer = Timer( self.periodInSecs, self.func )

    def start(self):
        self.__create_timer()
        self.aspitimer.start()
    
    def reset(self):
        self.start()

    def cancel(self):
        self.aspitimer.cancel()

class AsPiMemImage:
    '''
    Class to store an image in memory. 
    To be compatible with Tensorflow classification functions.
    '''
    def __init__(self):
        self.bytes = bytearray()
    def write(self, new_bytes):
        self.bytes.extend(new_bytes)
    def get_bytes(self):
        return bytes(self.bytes)


class MotionAnalyser( picamera.array.PiMotionAnalysis ):
    '''
    Analyses frames from recording video and checks if the
    frame vectors cross the thresholds indicating that movement
    was detected or not. If it detects movement the occurrences
    variable is incremented and it can be queried for movement
    events
    '''
    occurrences=0

    def analyse(self, a):
        a = np.sqrt(
            np.square(a['x'].astype(np.float)) +
            np.square(a['y'].astype(np.float))
            ).clip(0, 255).astype(np.uint8)
        # If there're more than 10 vectors with a magnitude greater
        # than 60, then say we've detected motion
        if (a > 60).sum() > 10:
            MotionAnalyser.occurrences += 1

class AsPiMotionDetector( Thread ):
    '''
    Starts and stops camera recording to /dev/null sending
    the frames to the MotionAnalyser class to detect movement.
    '''
    def __init__(self):
        Thread.__init__(self)

    def run(self):
        AsPiMotionDetector.start_motion_detection()
        while True:
            if AsPi.hasEnded():
                self.stop()
                return
            time.sleep(1)

    def stop(self):
        AsPiMotionDetector.stop_motion_detection()

    def start_motion_detection():
        camera.resolution = (640, 480)
        camera.framerate = 30
        camera.start_recording(
            '/dev/null', format='h264',
            motion_output = MotionAnalyser( camera )
            )

    def stop_motion_detection():
        if camera.recording:
            camera.stop_recording()


class AsPiCamera( Thread ):
    '''
    If enabled (it's disabled by default), it starts a thread, periodically taking pictures 
    with the AstroPi camera and storing them in files, putting the current ISS position in 
    the image file EXIF tags. It also stores the image in the lastPictureTaken class variable.
    '''
    lastPictureTaken = AsPiResult()
    isCameraEnabled = False
    
    def __init__(self, imgPeriodInSecs ):
        Thread.__init__(self)
        self.imgtimer = AsPiTimer( periodInSecs=imgPeriodInSecs, func=self.__take_picture )

    def run(self):
        self.imgtimer.start()
        while True:
            if AsPi.hasEnded():
                self.stop()
                return
            time.sleep(1)

    def stop(self):
        self.imgtimer.cancel()

    def __take_picture(self):
        AsPiMotionDetector.stop_motion_detection()
        self.__capture_image()
        AsPiMotionDetector.start_motion_detection()

    def __save_image( img, imgfilename ):
        imgfile = open(imgfilename, 'wb')
        imgfile.write( img.get_bytes() )
        imgfile.close()

    def __capture_image(self):
        imgfileprefix = AsPiLogFile.generate_fileprefix() 
        camera.resolution = PICAMERA_SENSOR_MODE_2_RESOLUTION
        camera.exif_tags['Artist'] = imgfileprefix
        lat,lon = self.__set_latlon_in_exif()
        imgfilename = imgfileprefix + "_" + lat + "_" + lon + ".jpg"
        img = AsPiMemImage()
        camera.capture( img , "jpeg")
        AsPiCamera.lastPictureTaken.put( ( imgfilename, img ) )
        AsPiCamera.__save_image( img, imgfilename )

        if AsPi.hasEnded():
            self.imgtimer.cancel()
        else:
            self.imgtimer.reset()

    def __set_latlon_in_exif(self):
        """
        A function to write lat/long to EXIF data for photographs
        (source based in the get_latlon function available in the 2019 AstroPi Mission 
        SpaceLab Phase 2 guide in the "Recording images using the camera" section)
        """
        AsPiSensors.iss.compute() # Get the lat/long values from ephem
        long_value = [float(i) for i in str(AsPiSensors.iss.sublong).split(":")]
        if long_value[0] < 0:
            long_value[0] = abs(long_value[0])
            longitude_ref = "W"
        else:
            longitude_ref = "E"
        longitude = '%d/1,%d/1,%d/10' % (long_value[0], long_value[1], long_value[2]*10)
        lat_value = [float(i) for i in str( AsPiSensors.iss.sublat).split(":")]
        if lat_value[0] < 0:
            lat_value[0] = abs(lat_value[0])
            latitude_ref = "S"
        else:
            latitude_ref = "N"
        latitude = '%d/1,%d/1,%d/10' % (lat_value[0], lat_value[1], lat_value[2]*10)

        camera.exif_tags['GPS.GPSLatitude'] = latitude
        camera.exif_tags['GPS.GPSLongitude'] = longitude
        camera.exif_tags['GPS.GPSLongitudeRef'] = longitude_ref
        camera.exif_tags['GPS.GPSLatitudeRef'] = latitude_ref
        camera.exif_tags['GPS.GPSAltitudeRef'] = "0" 
        camera.exif_tags['GPS.GPSAltitude'] = str( AsPiSensors.iss.elevation)

        latitude_str ='%s%03dd%02dm%02d' % (latitude_ref, lat_value[0], lat_value[1], lat_value[2])
        longitude_str='%s%03dd%02dm%02d' % (longitude_ref, long_value[0], long_value[1], long_value[2])

        return latitude_str ,longitude_str 


class AsPiUserLoop( Thread):
    '''
    Thread that continuously calls the provided callback. Passing as arguments, the results of the 'getdata' function.
    The result of the provided callback is then passed as argument to a 'returndata' function call.
    '''
    def __init__(self, callback , getdata, returndata):
        Thread.__init__(self)
        self.callback = callback
        self.getdata = getdata
        self.returndata = returndata

    def run(self):
        if self.callback is None:
            return
        while True:
            data = self.getdata()
            if data is None:           
                return
            response = self.callback( data )
            self.returndata( response )
            time.sleep(0.5)

class AsPiLogFile:
    '''
    Class that initializes and manages the data log file. 

    A csv data log file, with the specified naming format, is created at the beginning and everytime
    the log file gets bigger than 'logfileMaxBytes' bytes. Each file has a header in the first line
    with the sensors names and the respective units.
    Each data row is written in the csv file as a line with the field values separated by commas with 
    the timestamp in the DATE_FORMAT format as the first field.
    '''
    filePrefix = DEFAULT_LOGFILE_PREFIX

    def __init__(self
            , filePrefix = DEFAULT_LOGFILE_PREFIX
            , logfileMaxBytes = DEFAULT_SIZE_PER_LOGFILE_IN_BYTES
            , sensorList = AsPi.ALL_SENSORS
            , logToStdErr = False ):
        AsPiLogFile.filePrefix = filePrefix 
        self.logToStdErr = logToStdErr
        self.sensorList = sensorList
        self.logfileMaxBytes = logfileMaxBytes
        self.__create_datalogfile()
        
    def __create_datalogfile(self):
        self.currentDatalogFile = AsPiLogFile.generate_fileprefix() + '.' + LOGFILE_EXT
        logzero.logfile( filename=self.currentDatalogFile , disableStderrLogger=not self.logToStdErr)
        self.formatter = Formatter(fmt=LOG_FORMAT, datefmt=DATE_FORMAT)
        logzero.formatter( self.formatter )
        self.__write_header()

    def __write_header(self):
        logzero.formatter( NO_TIMESTAMP_FORMATTER )
        logger.info( self.__generate_header_line() )
        logzero.formatter( self.formatter )

    def __generate_header_line(self):
        commasep_sensor_fields = ' , '.join( [ sensor + " (" + AsPi.UNITS[ sensor ] + ")" for sensor in self.sensorList ] )
        return TIMESTAMP_FIELD + ', ' + commasep_sensor_fields
    
    def log(self, data_array):
        logger.info(",".join( map(str, data_array ) ) )
        if os.path.getsize( self.currentDatalogFile )  > self.logfileMaxBytes:
            self.__create_datalogfile()

    def generate_fileprefix():
        return '{dir}/{prefix}-{timestamp}'.format( dir=CURRENT_DIR , prefix=AsPiLogFile.filePrefix , timestamp=get_timestamp() )    

class AsPiLogger:
    '''
    MAIN CLASS. User facing class that:
        * configures all the options with the user specified values or with the predefined defaults.
        * starts the log timer, to periodically log data from the sensors
        * starts the end timer, to end the program after the specified duration
        * starts the motion detector thread to monitor and register movements event count 
        * if the user callback is specified, it starts the user loop thread to continuously send the collected data 
        to the user provided callback and receive any result to store in the CSV file as a "pseudo" sensor (SENSOR_USERDATA) value 
        * if camera is enabled, starts the camera thread to periodically take pictures with the AstroPi camera
        * Gracefully manages the program finalization phase and abnormal interruption handling (CTRL-C)
    '''
    def __init__(self
            , cameraEnabled = False
            , logPeriodInSecs = DEFAULT_LOG_PERIOD_IN_SECS
            , imgPeriodInSecs = DEFAULT_IMG_PERIOD_IN_SECS
            , filePrefix = DEFAULT_LOGFILE_PREFIX
            , logfileMaxBytes = DEFAULT_SIZE_PER_LOGFILE_IN_BYTES 
            , sensorList = AsPi.ALL_SENSORS
            , durationInSecs = DEFAULT_DURATION_IN_SECS
            , updateCallback = None 
            , logToStdErr = False ):

        AsPiCamera.isCameraEnabled = cameraEnabled
        self.logfile = AsPiLogFile( filePrefix = filePrefix, logfileMaxBytes = logfileMaxBytes, sensorList = sensorList, logToStdErr = logToStdErr )
        self.sensors = AsPiSensors(sense_hat, sensorList)
        self.logPeriodInSecs = logPeriodInSecs if logPeriodInSecs > MIN_LOG_PERIOD_IN_SECS else MIN_LOG_PERIOD_IN_SECS
        self.imgPeriodInSecs = imgPeriodInSecs if imgPeriodInSecs > MIN_IMG_PERIOD_IN_SECS else MIN_IMG_PERIOD_IN_SECS

        self.logtimer = AsPiTimer( self.logPeriodInSecs, self.__log_sensors_reading )
        self.endtimer = AsPiTimer( durationInSecs, AsPi.end )
        self.motiondetector = AsPiMotionDetector()
        if AsPiCamera.isCameraEnabled: 
            self.camera = AsPiCamera( imgPeriodInSecs )
        self.userLoop = AsPiUserLoop( updateCallback , AsPiLogger.getdata , AsPiLogger.setdata )

    def setdata( datareturned ):
        AsPiSensors.userData.put( datareturned )

    def getdata():
        while True:
            lastPictureTaken = AsPiCamera.lastPictureTaken.get() if AsPiCamera.isCameraEnabled else None
            lastAsPiSensorsReading = AsPiSensors.lastAsPiSensorsReading.get()
            if lastPictureTaken is None and lastAsPiSensorsReading is None:
                time.sleep(0.5)
                if AsPi.hasEnded():
                    return None
            else:
                return lastPictureTaken, lastAsPiSensorsReading

    def __log_sensors_reading(self):
        self.logfile.log( self.sensors.read() )
        if not AsPi.hasEnded():
            self.logtimer.reset()

    def __run(self):
        while True:
            if AsPi.hasEnded():
                return

    def start(self): 
        self.userLoop.start()
        self.motiondetector.start()
        self.logtimer.start()
        self.endtimer.start()
        if AsPiCamera.isCameraEnabled:
            self.camera.start()
        try:
            self.__run()
        except KeyboardInterrupt:
            if AsPiCamera.isCameraEnabled: 
                self.camera.stop()
            print("CTRL-C! Exiting…")
        finally:
            # clean up
            AsPi.end()
            self.logtimer.cancel()
            self.endtimer.cancel()
            if AsPiCamera.isCameraEnabled:
                self.camera.join()
            print("Waiting for user callback to finish...")
            self.userLoop.join()
            AsPi.shutdowntimer.cancel()
            print("Program finished.")








