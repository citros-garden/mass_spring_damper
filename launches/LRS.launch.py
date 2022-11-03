import os
from datetime import datetime

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import EmitEvent, ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler, LogInfo, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration, LocalSubstitution
from launch.event_handlers import (OnExecutionComplete, OnProcessExit,
                                OnProcessIO, OnProcessStart, OnShutdown)
from launch.events import Shutdown
from bson.objectid import ObjectId

from LRS_Lulav.LRS_Bag import LRS_Bag
from LRS_Lulav.LRS_params import LRS_params
from LRS_Lulav.LRS_util import LRS_util

import pymongo
MONGO_CONNECTION_STRING = "mongodb://localhost:27017/"
mymongo = pymongo.MongoClient(MONGO_CONNECTION_STRING)

def handle_done_recording(context, *args, **kwargs):
    test_run_id = LaunchConfiguration("test_run_id").perform(context)  
    simulation_run_id = LaunchConfiguration("simulation_run_id").perform(context)       
    bagparser = LRS_Bag(["tmp/bag/bag_0.db3"], "mongodb://localhost:27017/", "test_data", test_run_id, simulation_run_id)
    bagparser.fill_mongo()
    
    # TODO: delete file
    print(" -------- handle_done_recording: -------- ")
    
def handle_timeout(context, *args, **kwargs):  
    
    # TODO: send event to DB about timeout
    print(f"[{datetime.now().strftime('%H:%M:%S')}] -------- handle_timeout: -------- ")
    
def handle_shutdown(context, *args, **kwargs):    
    # TODO: send event to DB about shutting down ? send logs to server?
    print(" -------- handle_shutdown: -------- ")
    print("-------------------------------------------------------------------------------------- ")
    
def launch_setup(context, *args, **kwargs):
    test_run_id = LaunchConfiguration("test_run_id").perform(context)
    simulation_run_id = LaunchConfiguration("simulation_run_id").perform(context)
    
    test_run = mymongo['tests']["runs"].find_one({"_id": ObjectId(test_run_id)})
    test_id = test_run["test_id"]
    
    lrs_util = LRS_util()
    test = lrs_util.get_test(test_id)            
    
    user_id = test["user_id"]
    project_id = test["simulationSettings"]["project_id"]
    
    lrs_params = LRS_params(user_id, project_id, test_id, test_run_id, simulation_run_id)
    lrs_params.init_params()    
        
    launch_id = test["simulationSettings"]["launch_id"]
    package, launch = lrs_util.get_launch_file(user_id, project_id, launch_id);    
    
    # print("-----------package.name:", package["name"])
    # print("-----------launch.name:", launch["name"])
    # # demo_elbit_launch = IncludeLaunchDescription(
    # #     PythonLaunchDescriptionSource([
    # #         os.path.join(get_package_share_directory('dynamics'), 'launch'), # package name
    # #         '/dynamics_controller.launch.py' # launch file 
    # #     ]),
    # #     launch_arguments={}.items(),
    # # )
    demo_elbit_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(get_package_share_directory(package["name"]), 'launch'), 
            f"/{launch['name']}"
        ]),
        launch_arguments={}.items(),
    )
    print(f"----------------------------------------------------------------") 
    print(f"[{datetime.now().strftime('%H:%M:%S')}] Starting clients launch") 
    print(f"----------------------------------------------------------------") 
    return [
        demo_elbit_launch,
        TimerAction(
        period=f"{test['testSettings']['timeout']}",
        actions=[LogInfo(msg="---------TIMEOUT---------"), 
            OpaqueFunction(function=handle_timeout),
            EmitEvent(event=Shutdown(reason=f"TIMEOUT of {test['testSettings']['timeout']} is reached."))],
        )
    ]

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
    print(f"[{datetime.now().strftime('%H:%M:%S')}]")     
    
    record_proccess = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '-a', '-o', 'tmp/bag'], 
        output='screen', 
        log_cmd=True
    )

    return LaunchDescription([
        # Arguments        

        DeclareLaunchArgument(
            'test_run_id',
            description=(
                "Test Run id"
            ),      
        ),      
        DeclareLaunchArgument(
            'simulation_run_id',
            description=(
                "Simulation run id, as part of [sequence]/[simulation.repeats]"
            ),      
            default_value=str(ObjectId()),
        ),    
        DeclareLaunchArgument(
            'timeout',
            description=(
                "The timeout for the simulation [sec]"
            ),   
            default_value=str(60*60*24*7),   
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


# ros2 launch launches/LRS.launch.py user_id:=63502ab7865fb52ab569e90c project_id:=6351584232818a188f45fd59 simulation_id:=63523680846b4ecaf0404d00 simulation_run_id:=635236a7846b4ecaf0404d01 timeout:=10