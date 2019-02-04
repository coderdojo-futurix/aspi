from aspi import AsPi, AsPiLogger
import time

SENSORS = [
    AsPi.SENSOR_MOTION,
    AsPi.SENSOR_LAT,
    AsPi.SENSOR_LON,
    AsPi.SENSOR_ECLIPSED
]

class ImageProcessor:
    def __init__(self):
        self.imgcounter = 0

    def process_image(self, img ):
        self.imgcounter = self.imgcounter + 1
        print("Started Processing image " + str(self.imgcounter) + "!!!")
        time.sleep(5)
        print("Finished Processing image " + str(self.imgcounter) + "!!!")

imgproc = ImageProcessor()

aspilogger = AsPiLogger( 
    logPeriodInSecs=1 
    , imgPeriodInSecs=2
    , filePrefix="coderdojolx-wildfire"
    , durationInSecs = 20
    , debug=True
    , sensorList = SENSORS
    , updateCallback=imgproc.process_image)

aspilogger.start()
