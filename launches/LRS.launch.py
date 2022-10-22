import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown

from LRS_Lulav.LRS_Bag import LRS_Bag
from LRS_Lulav.LRS_params import LRS_params

def handle_done_recording(context, *args, **kwargs):
    simulation_instance_id = LaunchConfiguration("simulation_instance_id").perform(context)  
    simulation_instance_seq = LaunchConfiguration("simulation_instance_seq").perform(context)       
    bagparser = LRS_Bag(["tmp/bag/bag_0.db3"], "mongodb://localhost:27017/", "simulations", "" + simulation_instance_id + "_" + simulation_instance_seq)
    bagparser.fill_mongo()
    
    # TODO: delete file
    print(" -------- handle_done_recording: -------- ")
    
def handle_timeout(context, *args, **kwargs):        
    # TODO: send event to DB about timeout
    print(" -------- handle_timeout: -------- ")
    
def handle_shutdown(context, *args, **kwargs):    
    # TODO: send event to DB about shutting down ? send logs to server?
    print(" -------- handle_shutdown: -------- ")
    print("-------------------------------------------------------------------------------------- ")
    
def define_params(user_id, project_id, simulation_id):
    # TODO: use LRS agent SDK. 
    # Fill param files. 
    lrs_params = LRS_params(user_id, project_id, simulation_id)
    lrs_params.init_params()    
    
def launch_setup(context, *args, **kwargs):
    user_id = LaunchConfiguration("user_id").perform(context)
    project_id = LaunchConfiguration("project_id").perform(context)
    simulation_id = LaunchConfiguration("simulation_id").perform(context)
    simulation_instance_id = LaunchConfiguration("simulation_instance_id").perform(context)
    # simulation_instance_seq = LaunchConfiguration("simulation_instance_seq").perform(context)    
    
    define_params(user_id, project_id, simulation_id)
    
    demo_elbit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory('dynamics'), 'launch'), # package name
            '/dynamics_controller.launch.py' # launch file 
        ]),
        launch_arguments={}.items(),
    )
    return [demo_elbit_launch]

    # second option. add actions for nodes. 
    # demo_elbit_launch
    listeners = []
    sub_entities = demo_elbit_launch.get_sub_entities()
    # print("sub_entities", sub_entities)
    for launchDescription in sub_entities:
        eps = launchDescription.entities         
        for ep in eps:                        
            if type(ep) in [Node, ExecuteProcess]:
                # print("adding event OnProcessStart")
                listeners.append(ep)
                listeners.append(RegisterEventHandler(
                    OnProcessStart(
                        target_action=ep,
                        on_start=[
                            # print(' +++++++++++ OnProcessStart'),
                            LogInfo(msg=" +++++++++++ OnProcessStart")                    
                        ]
                    )
                ))
                
                listeners.append(RegisterEventHandler(
                    OnProcessExit(
                        target_action=ep,
                        on_exit=[
                            LogInfo(msg=" ----------- OnProcessExit")                    
                        ]
                    )
                ))

    print(f"running {len(listeners)} entities")
    return listeners
    
def generate_launch_description():        
    print("---------------------------")
    print("LRS-launching Elbit demo...")
    print("---------------------------")
    
    record_proccess = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', 'tmp/bag'], 
        output='screen', 
        log_cmd=True
    )
    
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'user_id',
            description=(
                "User id"
            ),      
        ),
        DeclareLaunchArgument(
            'project_id',
            description=(
                "Project id"
            ),        
        ),
        DeclareLaunchArgument(
            'simulation_id',
            description=(
                "Simulation id"
            ),      
        ),
        DeclareLaunchArgument(
            'simulation_instance_id',
            description=(
                "Simulation Instance id"
            ),      
        ),      
        DeclareLaunchArgument(
            'simulation_instance_seq',
            description=(
                "Simulation sequence index, as part of [sequence]/[simulation.repeats]"
            ),      
        ),    
        DeclareLaunchArgument(
            'timeout',
            description=(
                "The timeout for the simulation [sec]"
            ),      
        ),    
          
        # RECORDING Proccess
        record_proccess,
        RegisterEventHandler(
            OnExecutionComplete(
                target_action=record_proccess,
                on_completion=[
                    LogInfo(msg='OnExecutionComplete: Done Recording event'),                    
                    OpaqueFunction(function=handle_done_recording),               
                ]
            )
        ),
        RegisterEventHandler(OnProcessExit(
            target_action=record_proccess,
            on_exit=[
                LogInfo(msg='OnProcessExit: Done Recording event'),                    
                OpaqueFunction(function=handle_done_recording),               
            ]
        )),
        # User launch file
        OpaqueFunction(function=launch_setup),
        
        # Events
        TimerAction(
            period=LaunchConfiguration("timeout"),
            actions=[LogInfo(msg="---------TIMEOUT---------"), 
                     OpaqueFunction(function=handle_timeout),
                     EmitEvent(event=Shutdown(reason='TIMEOUT'))],
        ),
        # EXIT
        RegisterEventHandler(
            OnShutdown(
                on_shutdown=[
                    OpaqueFunction(function=handle_shutdown),
                    LogInfo(msg=['Launch was asked to shutdown: ',
                        LocalSubstitution('event.reason')]
                )]
            )
        ),
    ])


# ros2 launch launches/LRS.launch.py user_id:=63502ab7865fb52ab569e90c project_id:=6351584232818a188f45fd59 simulation_id:=63523680846b4ecaf0404d00 simulation_instance_id:=635236a7846b4ecaf0404d01 simulation_instance_seq:=1 timeout:=5