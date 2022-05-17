import itertools

from .bag_file_parser import BagFileParser


class InsertionExperimentBag( BagFileParser ):
    DEFAULT_TOPICS_OF_INTEREST = {
        "camera": [ "/".join([ns, topic, cam]) for ns, topic, cam in 
                      itertools.product(["/camera"],
                                         ["left", "right"], 
                                         ["image_raw", "image_rect_color", "camera_info"])
        ],
        "robot": [ "/".join([ns, topic, axis]) for ns, topic, axis in 
                      itertools.product(["/stage/axis"], 
                                        ["command", "position", "state/moving", "state/on"], 
                                        ["linear_stage", "x", "y", "z"])
        ],
        "needle": [ "/needle/sensor/raw", "/needle/sensor/processed",
                    "/needle/state/current_shape", "/needle/state/kappac","/needle/state/winit", 
                    "/needle/state/skin_entry", "/stage/state/needle_pose" 
        ],
    }

    def __init__( self, bagdir: str, bagfile: str = None, yamlfile: str = None, topics: list = None ):
        super().__init__( bagdir, bagfile=bagfile, yamlfile=yamlfile )

        self.topics_of_interest = topics if topics is not None else InsertionExperimentBag.DEFAULT_TOPICS_OF_INTEREST

        # data containers
        self.camera_data = []
        self.robot_data  = []
        self.needle_data = []

    # __init__

    def parse_data(self, camera: bool = False, robot: bool = False, needle: bool = False):
        """ Parse the data  """
        # parse camera data
        if camera: # TODO
            pass 

        # if: camera

        # parse robot data
        if robot: # TODO
            # get the position messages as a generator (to not overload RAM)
            robot_position_topics = list(filter(lambda t: "/position/" in t, self.topics_of_interest['robot']))
            bag_rows = self.get_messages(topic_name=robot_position_topics, generator_count=len(robot_position_topics))

            # iterate through the generator
            for i, rows in enumerate(bag_rows):
                # parse the set of rows
                
                robot_positions = {
                    topic.replace('/stage/axis/position/', '') : (ts, msg.data) for ts, topic, msg in rows
                }
                timestamp = min([ts for ts, *_ in robot_positions.values()])

                # make sure this set has all 4 messages
                if len(robot_positions) != len(robot_position_topics):
                    continue
                
                # append to robot data
                self.robot_data.append(
                    ( timestamp, 
                      robot_positions['x'],
                      robot_positions['y'],
                      robot_positions['z'],
                      robot_positions['linear_stage'] )
                )

                # debugging stuff
                print( f"[{timestamp}]:" )
                for axis, val in sorted(robot_positions.items()):
                    print(f"  {axis:15s}: {val[1]:10.6f} mm")
                print(100*"=")

                if i >= 6:
                    break

            #  for
        # if: robot

        # parse needle data
        if needle: # TODO
            pass 

        # if: needle

    # parse_data


# class: InsertionExperimentBag
