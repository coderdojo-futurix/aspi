from aspi import AsPi, AsPiLogger
import time
import random
from pathlib import Path    


SENSORS = [
    AsPi.SENSOR_MOTION,
    AsPi.SENSOR_LAT,
    AsPi.SENSOR_LON,
    AsPi.SENSOR_ECLIPSED,
    AsPi.SENSOR_USERDATA
]

class ImageProcessor:
    def __init__(self):
        self.imgcounter = 0

    def process_data(self, data ):
        photo, sensor_readings = data
        if photo is None:
            print("NO photo for YOU!")
        else:
            photo_filename, photo_data = photo 
            print("New photo available")
        if sensor_readings is None:
            print("NO sensor readings for YOU!")
        else:
            print("New sensor readings available")
        time.sleep(3)
        return Path( photo_filename ).name + "-" + str(random.random())


imgproc = ImageProcessor()

aspilogger = AsPiLogger( 
    cameraEnabled = True
    , logPeriodInSecs=1 
    , imgPeriodInSecs=2
    , filePrefix="process-data-example"
    , durationInSecs = 10
    , logToStdErr=True
    , sensorList = SENSORS
    , updateCallback=imgproc.process_data)

aspilogger.start()
