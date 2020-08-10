import orbslam2
import logging
from tonic import Tonic

logger = logging.getLogger(__name__)
logger.setLevel(logging.DEBUG)

class Localisator:
    def __init__(self, tonic: Tonic, vocab_path, settings_path):
        self.tonic = tonic
        self.slam = orbslam2.System(vocab_path, settings_path, orbslam2.Sensor.MONOCULAR)
        self.prevtime = None

    def initialise(self, initialising_data, my_osmap):
        self.slam.set_use_viewer(True)
        self.slam.initialize()
        self.slam.osmap_init()
        self.slam.activate_localisation_only()
        print(initialising_data)
        self.slam.process_image_mono(initialising_data[0], initialising_data[1])
        map_load_path = my_osmap.map_path/(my_osmap.map_name+".yaml")
        self.slam.map_load(str(map_load_path), False, False)
        logger.debug("map loaded")


    def localise(self, data):
        img, ts = data
        data = self.tonic.image_now()
        # if self.prevtime is not None:
                # print(ts, self.prevtime, ts-self.prevtime)
        self.slam.process_image_mono(img, ts)
        pose = self.slam.get_current_pose()
        self.prevtime = ts
        return pose
