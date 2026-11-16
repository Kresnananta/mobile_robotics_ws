#!/usr/bin/env python3
"""
Assignment 2 (Bug 2) - bring up one of the three worlds with a TurtleBot3.

    ros2 launch a2_worlds bug_world.launch.py world:=bug_1_circles y_pose:=0.0

Arguments
    world        bug_1_circles | bug_2_boxes | bug_3_concave  (default: bug_1_circles)
    x_pose       start x                                       (default: 0.0)
    y_pose       start y -- your y0 = 0.10*(NRP mod 5) - 0.20  (default: 0.0)
    gui          true | false  (false = headless, for the scripted runs)
    spawn_delay  seconds to wait for gzserver before spawning  (default: 10.0)

If the world appears but the robot does not, and the log says
"Service /spawn_entity unavailable", gzserver was simply slower than
spawn_entity.py was willing to wait. Raise spawn_delay:

    ros2 launch a2_worlds bug_world.launch.py world:=bug_1_circles spawn_delay:=25

Requires:  export TURTLEBOT3_MODEL=burger
           export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:\
/opt/ros/humble/share/turtlebot3_gazebo/models

Everything is resolved inside an OpaqueFunction, so paths are plain Python
strings rather than launch substitutions. That is deliberate: it keeps the file
free of substitution subtleties, and it lets us check that each file exists and
say so clearly instead of failing deep inside Gazebo.
"""

import os
import re
import socket
from urllib.parse import urlparse

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, \
    LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

WORLDS = ('bug_1_circles', 'bug_2_boxes', 'bug_3_concave')

SPAWN_DELAY = 10.0       # default seconds to let gzserver come up before spawning


def _require(path, what):
    if not os.path.isfile(path):
        here = os.path.dirname(path)
        try:
            listing = ', '.join(sorted(os.listdir(here))) or '(empty)'
        except OSError:
            listing = '(directory does not exist)'
        raise RuntimeError(
            f'\n\n  {what} not found:\n    {path}\n'
            f'  what is actually in {here}:\n    {listing}\n')
    return path


def _require_gazebo_free():
    """Refuse to start if a gzserver from a previous run is still alive.

    This is the failure that looks like a mystery: the first launch works, you
    Ctrl-C it, and the second launch brings up a window with no robot in it.
    What happened is that gzserver outlived the Ctrl-C -- Gazebo 11 often does,
    especially with sensor plugins -- so the new gzserver cannot take the
    master port, and `spawn_entity` talks to the OLD server, which already has
    a robot of that name and refuses to add another.

    Nothing in the error messages says so, so we check the port ourselves.
    """
    uri = os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345')
    parsed = urlparse(uri if '//' in uri else '//' + uri)
    host = parsed.hostname or 'localhost'
    port = parsed.port or 11345
    try:
        with socket.create_connection((host, port), timeout=0.4):
            pass
    except OSError:
        return                      # nothing listening: good
    raise RuntimeError(
        f'\n\n  A Gazebo master is already listening on {host}:{port}.\n'
        f'  A gzserver from an earlier run is still alive, so this launch\n'
        f'  would attach to the old world and the robot would not appear.\n\n'
        f'  Clean up, then launch again:\n\n'
        f'    pkill -f gzserver ; pkill -f gzclient\n'
        f'    sleep 2 ; pgrep -a gzserver || echo "clean"\n\n'
        f'  If you meant to attach to a Gazebo you started yourself, unset\n'
        f'  this check by launching gzserver and the robot separately.\n')


def _declared_args(launch_file):
    """Which launch arguments a launch file actually declares.

    Passing an argument an included launch file does not declare is a hard
    error, and gazebo_ros has changed these names between releases. So we read
    them instead of assuming.
    """
    try:
        text = open(launch_file, encoding='utf-8', errors='replace').read()
    except OSError:
        return set()
    return set(re.findall(r"DeclareLaunchArgument\(\s*['\"]([A-Za-z0-9_]+)['\"]", text))


def _gzserver_args(launch_file, world_path):
    """World, verbose, and the ROS plugins -- but only the ones it declares.

    `Service /spawn_entity unavailable. Was Gazebo started with
    GazeboRosFactory?` means gzserver came up without libgazebo_ros_factory.so
    (or did not come up at all). Where the plugin is a launch argument we ask
    for it explicitly rather than trusting its default.
    """
    declared = _declared_args(launch_file)
    args = {}
    if 'world' in declared:
        args['world'] = world_path
    if 'verbose' in declared:
        args['verbose'] = 'true'
    for plugin in ('init', 'factory', 'force_system'):
        if plugin in declared:
            args[plugin] = 'true'
    return args, declared


def launch_setup(context, *args, **kwargs):
    world = LaunchConfiguration('world').perform(context)
    x_pose = LaunchConfiguration('x_pose').perform(context)
    y_pose = LaunchConfiguration('y_pose').perform(context)
    gui = LaunchConfiguration('gui').perform(context).lower() in ('true', '1', 'yes')
    try:
        spawn_delay = float(LaunchConfiguration('spawn_delay').perform(context))
    except ValueError:
        spawn_delay = SPAWN_DELAY

    if world.endswith('.world'):          # tolerate world:=bug_1_circles.world
        world = world[:-len('.world')]
    if world not in WORLDS:
        raise RuntimeError(
            f'\n\n  unknown world "{world}".\n'
            f'  choose one of: {", ".join(WORLDS)}\n')

    _require_gazebo_free()

    pkg_share = get_package_share_directory('a2_worlds')
    gazebo_ros = get_package_share_directory('gazebo_ros')
    tb3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    world_path = _require(
        os.path.join(pkg_share, 'worlds', world + '.world'), 'world file')
    gzserver_launch = _require(
        os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py'), 'gzserver.launch.py')
    gzclient_launch = _require(
        os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py'), 'gzclient.launch.py')
    rsp_launch = _require(
        os.path.join(tb3_gazebo, 'launch', 'robot_state_publisher.launch.py'),
        'turtlebot3_gazebo robot_state_publisher.launch.py')
    spawn_launch = _require(
        os.path.join(tb3_gazebo, 'launch', 'spawn_turtlebot3.launch.py'),
        'turtlebot3_gazebo spawn_turtlebot3.launch.py')

    model = os.environ.get('TURTLEBOT3_MODEL', '')
    actions = [
        LogInfo(msg=f'world   : {world_path}'),
        LogInfo(msg=f'start   : ({x_pose}, {y_pose})   gui: {gui}'),
        LogInfo(msg=f'TURTLEBOT3_MODEL = "{model}"'
                    + ('' if model else '   <-- NOT SET, the robot will not spawn')),
    ]

    gz_args, declared = _gzserver_args(gzserver_launch, world_path)
    actions.append(LogInfo(
        msg=f'gzserver: ' + ' '.join(f'{k}:={v}' for k, v in sorted(gz_args.items()))))
    if 'factory' not in declared:
        actions.append(LogInfo(
            msg='gzserver.launch.py does not declare a "factory" argument in '
                'this gazebo_ros version; it should load '
                'libgazebo_ros_factory.so by default. If spawning fails with '
                '"Service /spawn_entity unavailable", that default is the '
                'thing to check.'))
    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(gzserver_launch),
        launch_arguments=gz_args.items()))

    if gui:
        actions.append(IncludeLaunchDescription(
            PythonLaunchDescriptionSource(gzclient_launch)))

    actions.append(IncludeLaunchDescription(
        PythonLaunchDescriptionSource(rsp_launch),
        launch_arguments={'use_sim_time': 'true'}.items()))

    # The spawn is delayed, and this is not belt-and-braces: without it the
    # launch is a race. gzserver and spawn_entity.py start together,
    # spawn_entity.py waits a fixed time for the /spawn_entity service, and
    # then gives up with
    #
    #   Service /spawn_entity unavailable. Was Gazebo started with
    #   GazeboRosFactory?
    #
    # which blames the plugin even when the plugin is loaded and merely late.
    # It is luck whether a given run wins, which is why the first launch after
    # a reboot can work and the next one not. turtlebot3_gazebo's own launch
    # files have the same race; this is not something wrong with your install.
    #
    # If the robot still does not appear, raise the delay rather than guessing:
    #     ros2 launch a2_worlds bug_world.launch.py world:=bug_1_circles \
    #         y_pose:=0.0 spawn_delay:=25
    actions.append(LogInfo(msg=f'spawn   : waiting {spawn_delay:.0f} s for '
                               f'gzserver, then spawning the robot'))
    actions.append(TimerAction(period=spawn_delay, actions=[
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(spawn_launch),
            launch_arguments={'x_pose': x_pose, 'y_pose': y_pose}.items()),
    ]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('world', default_value='bug_1_circles',
                              description=' | '.join(WORLDS)),
        DeclareLaunchArgument('x_pose', default_value='0.0'),
        DeclareLaunchArgument('y_pose', default_value='0.0',
                              description='your y0 = 0.10*(NRP mod 5) - 0.20'),
        DeclareLaunchArgument('gui', default_value='true',
                              description='false for headless scripted runs'),
        DeclareLaunchArgument('spawn_delay', default_value=str(SPAWN_DELAY),
                              description='seconds to wait for gzserver before '
                                          'spawning the robot; raise it if the '
                                          'robot does not appear'),
        OpaqueFunction(function=launch_setup),
    ])
