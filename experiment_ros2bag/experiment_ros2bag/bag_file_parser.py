import os
import sqlite3
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message, get_service

import yaml


class BagFileParser:
    def __init__( self, bagdir: str, bagfile: str = None, yamlfile: str = None ):

        bagfile = bagfile if bagfile is not None else os.path.split(bagdir)[-1]+ "_0.db3"
        yamlfile = yamlfile if yamlfile is not None else "metadata.yaml"

        # get metadata
        self.metadata = None
        if os.path.exists( yamlfile ):
            with open(yamlfile, 'r') as f:
                self.metadata = yaml.load(f)

            # with
        # if
        
        # connect to sql
        self.bag_db = sqlite3.connect( os.path.join( bagdir, bagfile ) )
        self.cursor = self.bag_db.cursor()

        # create a message type map (from https://github.com/ros2/rosbag2/issues/473 )
        topics_data = self.cursor.execute( "SELECT id, name, type FROM topics" ).fetchall()
        self.topic_type = { name_of: type_of for id_of, name_of, type_of in topics_data }
        self.topic_id = { name_of: id_of for id_of, name_of, type_of in topics_data }
        self.topic_name = { id_of: name_of for id_of, name_of, type_of in topics_data }
        self.topic_msg = { name_of: get_message( type_of ) for id_of, name_of, type_of in topics_data }

    # __init__

    def __del__( self ):
        self.bag_db.close()

    # __del__

    def get_messages( self, topic_name = None, ts_range: tuple = (None, None), ts_range_exclude: tuple = (None, None), generator_count: int = -1):
        """
            Get all of the messages from the topic
            :param topic_name: string or list of strings of the topic name(s) to get
            :param ts_range: tuple of timestamps (Default = (None, None)) to include
            :param ts_range_exclude: tuple of timestamps (Default = (None, None))
            :return: list of tuples(timestamp, message)

        """
        # Get from the db
        sql_cmd ="SELECT timestamp, topic_id, data FROM messages"
        conditions = []

        # process topic name
        if topic_name is not None:
            if isinstance(topic_name, str):
                topic_id = self.topic_id[ topic_name ]
                conditions.append( f"topic_id = {topic_id}" )

            elif isinstance(topic_name, list) and len(topic_name) > 0:
                topic_condition = " OR ".join([ f"topic_id = {self.topic_id[name]}" for name in topic_name])
                conditions.append( f"( {topic_condition} )" )

                # for
            # elif
            else:
                raise ValueError("'topic_name' must be a str or a non-empty List[str].")

        # if

        # process timestamp ranges
        if ts_range[0] is not None and ts_range[1] is not None:
            conditions.append( f"timestamp BETWEEN {ts_range[0]} and {ts_range[1]}" )

        elif ts_range[0] is not None: # only lower-bound
            conditions.append( f"timestamp >= {ts_range[0]}")

        elif ts_range[1] is not None: # only upper-bound
            conditions.append( f"timestamp <= {ts_range[1]}")

        # process exclusion timestamp ranges
        if ts_range_exclude[0] is not None and ts_range_exclude[1] is not None:
            conditions.append( f"timestamp NOT BETWEEN {ts_range_exclude[0]} AND {ts_range_exclude[1]}")

        elif ts_range_exclude[0] is not None: # only lower-bound
            conditions.append( f"timestamp < {ts_range_exclude[0]}" )

        elif ts_range_exclude[1] is not None: # only upper-bound
            conditions.append( f"timestamp > {ts_range_exclude[1]}" )

        # combine conditions
        if len(conditions) > 0:
            sql_cmd += " WHERE " + " AND ".join(conditions) + " ORDER BY timestamp ASC"

        rows = self.cursor.execute( sql_cmd )

        # Deserialise all and timestamp them
        if generator_count > 0:
            while True:
                yield [(ts, self.topic_name[id], self.deserialize_message(msg_srl, topic_id=id)) for ts, id, msg_srl 
                        in rows.fetchmany(generator_count)]
            # while
        # if
        else: # get all
            return [(ts, self.topic_name[id], self.deserialize_message(msg_srl, topic_id=id)) for ts, id, msg_srl
                     in rows.fetchall()]

        # else
    # get_messages

    def deserialize_message(self, msg, topic_name = None, topic_id = None):
        """ Deserialize the message """
        if topic_name is not None:
            topic_msg = self.topic_msg[topic_name]

        elif topic_id is not None:
            topic_msg = self.topic_msg[self.topic_name[topic_id]]

        else: 
            raise ValueError("'either topic_name' or 'topic_id' must be provided.")

        return deserialize_message(msg, topic_msg)

    # deserialze_message



# class: BagFileParser
