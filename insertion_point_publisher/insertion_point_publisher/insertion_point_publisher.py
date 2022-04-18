import rclpy
from rclpy.node import Node

# msgs
from geometry_msgs.msg import Point
from rcl_interfaces.msg import SetParametersResult


class InsertionPointPublisher( Node ):
    PARAM_INSPT = "insertion.point"

    PARAM_INSPT_X = f"{PARAM_INSPT}.x"
    PARAM_INSPT_Y = f"{PARAM_INSPT}.y"
    PARAM_INSPT_Z = f"{PARAM_INSPT}.z"

    PARAM_INSPT_PUBTIME = f"{PARAM_INSPT}.publish.time"

    def __init__( self, name="InsertionPointPublisher" ):
        super().__init__( name )

        # set parameters
        publish_time = self.declare_parameter(
                self.PARAM_INSPT_PUBTIME, 0.01 ).get_parameter_value().double_value

        self.insertion_x = self.declare_parameter(
                self.PARAM_INSPT_X, 0, ).get_parameter_value().double_value
        self.insertion_y = self.declare_parameter(
                self.PARAM_INSPT_Y, 0 ).get_parameter_value().double_value
        self.insertion_z = self.declare_parameter(
                self.PARAM_INSPT_Z, 0 ).get_parameter_value().double_value

        self.add_on_set_parameters_callback( self.set_parameter_callback )

        # publisher
        self.pub_insertion_point = self.create_publisher( Point, 'state/skin_entry', 10 )

        # timers
        self.timer_publish_inspoint = self.create_timer(
                publish_time, self.publish_insertion_point )

        # logging
        self.get_logger().info( "Running insertion point publisher for: "
                               f"({self.insertion_x}, {self.insertion_y}, {self.insertion_z})" )

    # __init__

    def destroy_node( self ):
        self.get_logger().info( "Shutting down needle insertion point publisher" )

    def publish_insertion_point( self ):
        """ Insertion point publisher """
        msg = Point( x=self.insertion_x, y=self.insertion_y, z=self.insertion_z )

        self.pub_insertion_point.publish( msg )

    # publish_insertion_point

    def set_parameter_callback( self, params ):
        """ Set parameter callback to update insertion point parameters """
        success = False

        # iterate through parameters
        for param in params:
            success = True
            if param.name == self.PARAM_INSPT_X:
                self.insertion_x = param.get_parameter_value().double_value

            elif param.name == self.PARAM_INSPT_Y:
                self.insertion_y = param.get_parameter_value().double_value

            elif param.name == self.PARAM_INSPT_Z:
                self.insertion_z = param.get_parameter_value().double_value

            else:
                success = False

        # for

        # log the new insertion point
        if success:
            self.get_logger().info( "Updated needle insertion point to: "
                                    f"({self.insertion_x}, {self.insertion_y}, {self.insertion_z})" )
        # if

        return SetParametersResult( successful=success )

    # set_parameter_callback


# class: InsertionPointPublisher


def main( args=None ):
    rclpy.init( args=args )

    node = InsertionPointPublisher()

    rclpy.spin( node )

    node.destroy_node()
    rclpy.shutdown( node )


# main


if __name__ == '__main__':
    main()

# if __main__
