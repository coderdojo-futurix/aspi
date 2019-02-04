from gpiozero import CPUTemperature
from datetime import datetime 
from logging import Formatter
from logzero import logger
from pisense import SenseHAT
from ephem import readtle, degrees
from threading import Timer, Thread
from queue import Queue
import logzero
import os
import time
import locale
import math

import picamera
import picamera.array
import numpy as np

sense_hat=SenseHAT()
cpu = CPUTemperature()
camera = picamera.PiCamera()

ENABLE_DEBUG = True
MIN_LOG_PERIOD_IN_SECS = 2
MIN_IMG_PERIOD_IN_SECS = 5
SHUTDOWN_TIMEOUT_IN_SECS = 5  
DEFAULT_DURATION_IN_SECS = 3 * 60 * 60 - SHUTDOWN_TIMEOUT_IN_SECS
DEFAULT_SIZE_PER_LOGFILE_IN_BYTES = 30*1024
DEFAULT_LOG_PERIOD_IN_SECS = 5
DEFAULT_IMG_PERIOD_IN_SECS = 10
DEFAULT_LOGFILE_PREFIX = "sense_hat_logger"

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
    return TIMESTAMP_FORMAT.format(datetime.now())

class AsPi:
    _ended = False

    def endnow():
        sys.exit(0)

    def end():
        AsPi._ended = True
        Timer( SHUTDOWN_TIMEOUT_IN_SECS , AsPi.endnow )

    def hasEnded():
        return AsPi._ended

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

    UNITS = {
        SENSOR_CPU_TEMP    : UNITS_DEGREES_CELSIUS,
        SENSOR_TEMPERATURE : UNITS_DEGREES_CELSIUS,
        SENSOR_PRESSURE    : UNITS_MILLIBARS,
        SENSOR_HUMIDITY    : UNITS_PERC_RELATIVE_HUMIDITY,
        SENSOR_COMPASS_X   : UNITS_MICRO_TESLAS,
        SENSOR_COMPASS_Y   : UNITS_MICRO_TESLAS,
        SENSOR_COMPASS_Z   : UNITS_MICRO_TESLAS,
        SENSOR_GYRO_X      : UNITS_RADIANS_PER_SEC,
        SENSOR_GYRO_Y      : UNITS_RADIANS_PER_SEC,
        SENSOR_GYRO_Z      : UNITS_RADIANS_PER_SEC,
        SENSOR_ACCEL_X     : UNITS_STANDARD_GRAVITIES,
        SENSOR_ACCEL_Y     : UNITS_STANDARD_GRAVITIES,
        SENSOR_ACCEL_Z     : UNITS_STANDARD_GRAVITIES,
        SENSOR_PITCH       : UNITS_RADIANS,
        SENSOR_ROLL        : UNITS_RADIANS,
        SENSOR_YAW         : UNITS_RADIANS,
        SENSOR_LAT         : UNITS_DEGREES,
        SENSOR_LON         : UNITS_DEGREES,
        SENSOR_ELEVATION   : UNITS_METERS,
        SENSOR_ECLIPSED    : UNITS_BOOL,
        SENSOR_MOTION      : UNITS_COUNT
    }

    #ALL_SENSORS = UNITS.keys()
    ALL_SENSORS = [
        SENSOR_CPU_TEMP,
        SENSOR_TEMPERATURE,
        SENSOR_PRESSURE,
        SENSOR_HUMIDITY,
        SENSOR_COMPASS_X,
        SENSOR_COMPASS_Y,
        SENSOR_COMPASS_Z,
        SENSOR_GYRO_X,
        SENSOR_GYRO_Y,
        SENSOR_GYRO_Z,
        SENSOR_ACCEL_X,
        SENSOR_ACCEL_Y,
        SENSOR_ACCEL_Z,
        SENSOR_PITCH,
        SENSOR_ROLL,
        SENSOR_YAW,
        SENSOR_LAT,
        SENSOR_LON,
        SENSOR_ELEVATION,
        SENSOR_ECLIPSED,
        SENSOR_MOTION
    ]

class Sensors:
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
        Sensors.iss.compute()
        lat = Sensors.iss.sublat * DEGREES_PER_RADIAN
        lon = Sensors.iss.sublong * DEGREES_PER_RADIAN
        alt = Sensors.iss.elevation
        ecl = Sensors.iss.eclipsed
        return lat,lon, alt, ecl

    def read(self):
        
        envreading = self.hat.environ.read()
        imureading = self.hat.imu.read()
        latitude, longitude, elevation, eclipsed = Sensors._get_latlon()

        self.readings = { 
            AsPi.SENSOR_CPU_TEMP : Sensors.cpu.temperature,
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
            AsPi.SENSOR_MOTION : MotionAnalyser.occurrences            
        }

        # Reset motion detection occurences count
        MotionAnalyser.occurrences = 0

        # Return readings from selected sensors only
        return self.__selected_sensors_only( self.readings )

    def __selected_sensors_only(self, readings ):
        return list( map( lambda sensor: readings[ sensor ],  self.selected_sensors ))


class AsPiTimer:
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

class MotionAnalyser( picamera.array.PiMotionAnalysis ):
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

class AsPiMemImage:
    def __init__(self):
        self.bytes = bytearray()
    def write(self, new_bytes):
        self.bytes.extend(new_bytes)
    def get_bytes(self):
        return bytes(self.bytes)


class AsPiCamera( Thread ):
    lastPictureTaken = Queue(maxsize = 1)

    def __init__(self, imgPeriodInSecs ):
        Thread.__init__(self)
        self.imgtimer = AsPiTimer( periodInSecs=imgPeriodInSecs, func=self.__take_picture )

    def run(self):
        self.__start_motion_detection()
        self.imgtimer.start()
        while True:
            if AsPi.hasEnded():
                self.stop()
                return
            time.sleep(1)

    def stop(self):
        self.__stop_motion_detection()
        self.imgtimer.cancel()



    def __take_picture(self):
        self.__stop_motion_detection()
        self.__capture_image()
        self.__start_motion_detection()
                
    def __start_motion_detection(self):
        camera.resolution = (640, 480)
        camera.framerate = 30
        camera.start_recording(
            '/dev/null', format='h264',
            motion_output = MotionAnalyser( camera )
            )

    def __stop_motion_detection(self):
        if camera.recording:
            camera.stop_recording()

    def getLastPictureTaken():
        img = None
        while True:
            try:
                img = AsPiCamera.lastPictureTaken.get( timeout = 1 )
                return img
            except:
                pass
            finally:
                if AsPi.hasEnded():
                    return None

    def __replace_lastPictureTaken(pic):
        try:
            AsPiCamera.lastPictureTaken.get_nowait()
        except:
            pass
        finally:
            AsPiCamera.lastPictureTaken.put( pic )

    def __save_image( img, imgfilename ):
        imgfile = open(imgfilename, 'wb')
        imgfile.write( img.get_bytes() )
        imgfile.close()


    def __capture_image(self):
        imgfileprefix = AsPiLogFile.generate_fileprefix() 
        camera.resolution = ( 2592, 1944 )
        camera.exif_tags['Artist'] = imgfileprefix
        lat,lon = self.__set_latlon_in_exif()
        imgfilename = imgfileprefix + "_" + lat + "_" + lon + ".jpg"
        img = AsPiMemImage()
        camera.capture( img , "jpeg")
        AsPiCamera.__replace_lastPictureTaken( img )
        AsPiCamera.__save_image( img, imgfilename )

        if AsPi.hasEnded():
            self.imgtimer.cancel()
        else:
            self.imgtimer.reset()


    def __set_latlon_in_exif(self):
        """
        A function to write lat/long to EXIF data for photographs
        """
        Sensors.iss.compute() # Get the lat/long values from ephem
        long_value = [float(i) for i in str(Sensors.iss.sublong).split(":")]
        if long_value[0] < 0:
            long_value[0] = abs(long_value[0])
            longitude_ref = "W"
        else:
            longitude_ref = "E"
        longitude = '%d/1,%d/1,%d/10' % (long_value[0], long_value[1], long_value[2]*10)
        lat_value = [float(i) for i in str( Sensors.iss.sublat).split(":")]
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
        camera.exif_tags['GPS.GPSAltitude'] = str( Sensors.iss.elevation)

        latitude_str ='%s%dd%dm%d' % (latitude_ref, lat_value[0], lat_value[1], lat_value[2]*10)
        longitude_str='%s%dd%dm%d' % (longitude_ref, long_value[0], long_value[1], long_value[2]*10)

        return latitude_str ,longitude_str 


class AsPiUserLoop(Thread):
    def __init__(self, callback ):
        Thread.__init__(self)
        self.callback = callback

    def run(self):
        while True:
            last_picture = AsPiCamera.getLastPictureTaken()
            if last_picture is None:
                return
            self.callback( last_picture )
            time.sleep(1)


class AsPiLogFile:
    filePrefix = DEFAULT_LOGFILE_PREFIX
    def __init__(self
            , filePrefix = DEFAULT_LOGFILE_PREFIX
            , logfileMaxBytes = DEFAULT_SIZE_PER_LOGFILE_IN_BYTES
            , sensorList = AsPi.ALL_SENSORS
            , debug = False ):
        AsPiLogFile.filePrefix = filePrefix 
        self.debugEnabled = debug
        self.sensorList = sensorList
        self.__prepare_logfile()
        self.__write_header()

    def __prepare_logfile(self):
        logzero.logfile( filename=AsPiLogFile.generate_fileprefix() + '.' + LOGFILE_EXT , disableStderrLogger=not self.debugEnabled)
        self.formatter = Formatter(fmt=LOG_FORMAT, datefmt=DATE_FORMAT)
        logzero.formatter( self.formatter )

    def __write_header(self):
        logzero.formatter( NO_TIMESTAMP_FORMATTER )
        logger.info( self.__generate_header_line() )
        logzero.formatter( self.formatter )

    def __generate_header_line(self):
        commasep_sensor_fields = ' , '.join( [ sensor + " (" + AsPi.UNITS[ sensor ] + ")" for sensor in self.sensorList ] )
        return TIMESTAMP_FIELD + ', ' + commasep_sensor_fields
    
    def log(self, data_array):
        logger.info(",".join( map(str, data_array ) ) )
        # check data log file size and create another if very big 

    def generate_fileprefix():
        return '{dir}/{prefix}-{timestamp}'.format( dir=CURRENT_DIR , prefix=AsPiLogFile.filePrefix , timestamp=get_timestamp() )    


    
class AsPiLogger:
    def __init__(self
            , logPeriodInSecs = DEFAULT_LOG_PERIOD_IN_SECS
            , imgPeriodInSecs = DEFAULT_IMG_PERIOD_IN_SECS
            , filePrefix = DEFAULT_LOGFILE_PREFIX
            , logfileMaxBytes = DEFAULT_SIZE_PER_LOGFILE_IN_BYTES 
            , sensorList = AsPi.ALL_SENSORS
            , durationInSecs = DEFAULT_DURATION_IN_SECS
            , updateCallback = None 
            , debug = False ):

        self.logfile = AsPiLogFile( filePrefix = filePrefix, logfileMaxBytes = logfileMaxBytes, sensorList = sensorList, debug = debug )
        self.sensors = Sensors(sense_hat, sensorList)
        self.logPeriodInSecs = logPeriodInSecs if logPeriodInSecs > MIN_LOG_PERIOD_IN_SECS else MIN_LOG_PERIOD_IN_SECS
        self.imgPeriodInSecs = imgPeriodInSecs if imgPeriodInSecs > MIN_IMG_PERIOD_IN_SECS else MIN_IMG_PERIOD_IN_SECS

        self.logtimer = AsPiTimer( self.logPeriodInSecs, self.__log_sensors_reading )
        self.endtimer = AsPiTimer( durationInSecs, AsPi.end )
        self.camera = AsPiCamera( imgPeriodInSecs )
        if not updateCallback is None:
            AsPiUserLoop( updateCallback ).start()

    def __log_sensors_reading(self):
        self.logfile.log( self.sensors.read() )
        if not AsPi.hasEnded():
            self.logtimer.reset()

    def __run(self):
        while True:
            if AsPi.hasEnded():
                return

    def start(self):
        self.camera.start() 
        self.logtimer.start()
        self.endtimer.start()
        try:
            self.__run()
        except KeyboardInterrupt:
            self.camera.stop()
            print("CTRL-C! Exiting…")
        finally:
            # clean up
            AsPi.end()
            self.logtimer.cancel()
            self.endtimer.cancel()
            print("Program finished.")








