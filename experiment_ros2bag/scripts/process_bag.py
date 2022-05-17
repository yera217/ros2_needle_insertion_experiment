import argparse

from experiment_ros2bag.experiment_bag import InsertionExperimentBag


def __get_argparser():
    """ Configure argument parser """
    parser = argparse.ArgumentParser()

    parser.add_argument( "bagdir", type=str )
    
    parser.add_argument( "--bag-file", type=str, default=None  )
    parser.add_argument( "-t", "--topics", type=str, nargs="+", default=None )

    return parser


# __get_argparser

def main( args=None ):
    parser = __get_argparser()

    ARGS = parser.parse_args( args )

    bag = InsertionExperimentBag( ARGS.bagdir )

    print("Topics in bag:")
    for topic, msg_t in sorted(bag.topic_type.items()):
        print(f"    {topic:50s}: {msg_t:30s}")
    
    print("Messages for axis stage Position commands:")
    ax_pos_topics = [f"/stage/axis/position/{axis}" for axis in ["x", "y", "z", "linear_stage"]]
    ax_pos = bag.get_messages(topic_name = ax_pos_topics, generator_count=len(ax_pos_topics))

    idx = 0
    while idx < 5:
        msgs = next(ax_pos)
        for msg in msgs:
            print(f"  {msg}")
        print(100*"=")
        idx+=1

    # while

    print("Testing parsing data")
    bag.parse_data(robot=True)
    
    
# main

if __name__ == "__main__":
    main()

# if __main__