import itertools

from .bag_file_parser import BagFileParser


class InsertionExperimentBag( BagFileParser ):
    DEFAULT_TOPICS_OF_INTEREST = {
        "camera": list(itertools.product(["/camera/"],
                                         ["left/", "right/"], 
                                         ["image_raw", "image_rect_color", "camera_info"])),
        "robot": list(itertools.product(["/stage/axis/"], 
                                        ["command/", "position/", "state/moving/", "state/on/"], 
                                        ["linear_stage", "x", "y", "z"])),
        "needle": ["/needle/sensor/raw", "/needle/sensor/processed",
                   "/needle/state/current_shape", "/needle/state/kappac","/needle/state/winit", 
                   "/needle/state/skin_entry", "/stage/state/needle_pose"],
    }

    def __init__( self, bagdir: str, bagfile: str = None, yamlfile: str = None, topics: list = None ):
        super().__init__( bagdir, bagfile=bagfile, yamlfile=yamlfile )

        self.topics_of_interest = topics if topics is not None else InsertionExperimentBag.DEFAULT_TOPICS_OF_INTEREST

        # data containers
        self.camera_data = None
        self.robot_data  = None
        self.needle_data = None

    # __init__

    def parse_data(self, camera: bool = False, robot: bool = False, needle: bool = False):
        """ Parse the data  """
        # parse camera data
        if camera:
            pass # TODO

        # if: camera

        # parse robot data
        if robot:
            pass # TODO

        # if: robot

        # parse robot data
        if needle:
            pass # TODO

        # if: needle

    # parse_data


# class: InsertionExperimentBag
