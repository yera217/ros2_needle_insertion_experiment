from launch import LaunchDescription, actions, conditions
from launch.substitutions.launch_configuration import LaunchConfiguration
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import PythonExpression, LocalSubstitution, TextSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.substitutions import FindPackageShare

pkg_hyperion_interrogator = FindPackageShare('hyperion_interrogator')
pkg_needle_shape_publisher = FindPackageShare('needle_shape_publisher')

def determineCHsAAs(needleParamFile: str):
    """ Determine the number of channels and active areas available """
    with open(needleParamFile, 'r') as paramFile:
        params = json.load(paramFile) 

    # with
    numChs = params['# channels']
    numAAs = params['# active areas']

    return numChs, numAAs
# determineCHsAAs

def generate_launch_description():
    ld = LaunchDescription()

    # determine #chs and numAAs
    numCHs, numAAs = 3, 4
    for arg in sys.argv:
        if arg.startswith("needleParamFile:="):
            needleParamFile = arg.split(":=")[1]
            numCHs, numAAs = determineCHsAAs(needleParamFile)
        
        # if
    # for 

    # arguments
    arg_simlevel = DeclareLaunchArgument(
            "sim_level_needle_sensing",
            default_value="1",
            description="Simulation level: 1 - hyperrion demo, " +
                "2 - real sensors"
        )

    arg_params = DeclareLaunchArgument( 'needleParamFile',
                                             description="The shape-sensing needle parameter json file." )
                                             
    arg_interrIP = DeclareLaunchArgument('interrogatorIP', default_value='10.0.0.55')

    # other launch files
    ld_needlepub = IncludeLaunchDescription( # needle shape publisher
             PythonLaunchDescriptionSource(
                os.path.join(pkg_needle_shape_publisher, 'sensorized_shapesensing_needle_decomposed.launch.py')),
                condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level_needle_sensing'), " == 2"])),
                launch_arguments = {'needleParamFile': LaunchConfiguration( 'needleParamFile')}.items()
            )

    ld_hyperiondemo = IncludeLaunchDescription( # hyperion demo
            PythonLaunchDescriptionSource(
                os.path.join(pkg_hyperion_interrogator, 'hyperion_demo.launch.py')
                ),
                condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level_needle_sensing'), " == 1"])),
               launch_arguments = {'numCH': TextSubstitution(text=str(numCHs)), 'numAA': TextSubstitution(text=str(numAAs))}.items()
            )
    ld_shapedemo = IncludeLaunchDescription( # shape demo
            PythonLaunchDescriptionSource(
                os.path.join(pkg_needle_shape_publisher, 'shapesensing_needle_demo.launch.py')
                ),
                condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level_needle_sensing'), " == 1"])),
               launch_arguments = {'numCH': TextSubstitution(text=str(numCHs)), 'numAA': TextSubstitution(text=str(numAAs))}.items()
            )        
    ld_hyperionstream = IncludeLaunchDescription( # hyperion streamer
            PythonLaunchDescriptionSource(
                os.path.join(pkg_hyperion_interrogator, 'hyperion_streamer.launch.py')
                ),
                condition=conditions.IfCondition(
               PythonExpression([LaunchConfiguration('sim_level_needle_sensing'), " == 2"])),
               launch_arguments = {'ip': LaunchConfiguration('interrogatorIP')}.items()
            )

    # add to launch description
    ld.add_action(arg_simlevel)
    ld.add_action(arg_params)
    ld.add_action(arg_interrIP)
   
    ld.add_action(ld_needlepub)
    ld.add_action(ld_hyperiondemo)
    ld.add_action(ld_hyperionstream)

    return ld

# generate_launch_description