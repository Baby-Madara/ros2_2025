#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from launch_ros.substitutions import FindPackageShare
import xacro
import os


''' world = 
    /usr/share/ignition/ignition-gazebo6/worlds: ls
        3k_shapes.sdf                         import_mesh.sdf                    polylines.sdf
        ackermann_steering.sdf                joint_controller.sdf               pose_publisher.sdf
        actor_crowd.sdf                       joint_position_controller.sdf      quadcopter.sdf
        actor.sdf                             joint_trajectory_controller.sdf    resource_spawner.sdf
        actors_population.sdf                 kinetic_energy_monitor.sdf         rf_comms.sdf
        apply_joint_force.sdf                 levels_no_performers.sdf           rolling_shapes.sdf
        apply_link_wrench.sdf                 levels.sdf                         segmentation_camera.sdf
        auv_controls.sdf                      lift_drag_battery.sdf              sensors_demo.sdf
        boundingbox_camera.sdf                lift_drag_nested.sdf               sensors.sdf
        breadcrumbs.sdf                       lift_drag.sdf                      shader_param.sdf
        buoyancy_engine.sdf                   lightmap.sdf                       shapes_bitmask.sdf
        buoyancy.sdf                          lights.sdf                         shapes.sdf
        camera_sensor.sdf                     linear_battery_demo.sdf            skid_steer_mecanum.sdf
        camera_video_record_dbl_pendulum.sdf  logical_audio_sensor_plugin.sdf    sky.sdf
        collada_world_exporter.sdf            logical_camera_sensor.sdf          spaces.sdf
        contact_sensor.sdf                    log_record_dbl_pendulum.sdf        spherical_coordinates.sdf
        conveyor.sdf                          log_record_keyboard.sdf            thermal_camera.sdf
        debug_shapes.sdf                      log_record_resources.sdf           thumbnails
        default.sdf                           log_record_shapes.sdf              touch_plugin.sdf
        depth_camera_sensor.sdf               mecanum_drive.sdf                  track_drive.sdf
        detachable_joint.sdf                  minimal_scene.sdf                  tracked_vehicle_simple.sdf
        diff_drive.sdf                        model_photo_shoot.sdf              trajectory_follower.sdf
        diff_drive_skid.sdf                   multicopter_velocity_control.sdf   triggered_camera_sensor.sdf
        elevator.sdf                          multi_lrauv_race.sdf               triggered_publisher.sdf
        empty_gui.sdf                         nested_model.sdf                   trisphere_cycle_wheel_slip.sdf
        empty.sdf                             optical_tactile_sensor_plugin.sdf  tunnel.sdf
        follow_actor.sdf                      particle_emitter2.sdf              velocity_control.sdf
        fuel.sdf                              particle_emitter.sdf               video_record_dbl_pendulum.sdf
        fuel_textured_mesh.sdf                pendulum_links.sdf                 visibility.sdf
        gpu_lidar_retro_values_sensor.sdf     perfect_comms.sdf                  visualize_contacts.sdf
        gpu_lidar_sensor.sdf                  performer_detector.sdf             visualize_lidar.sdf
        graded_buoyancy.sdf                   physics_options.sdf                wind.sdf
        grid.sdf                              plane_propeller_demo.sdf           world_joint.sdf
        heightmap.sdf                         plot_3d.sdf
'''

def generate_launch_description():
    pkg_share = FindPackageShare(package='robot_v2').find('robot_v2')
    xacro_file = os.path.join(pkg_share, 'urdf/robot.urdf.xacro')
    rviz_config = os.path.join(pkg_share, 'rviz/robot_config.rviz')
    robot_description_raw = xacro.process_file(xacro_file).toxml()
    world = 'visualize_lidar.sdf'

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{'robot_description': robot_description_raw}]
        ),
        ExecuteProcess(
            # cmd=['ign', 'gazebo', '--verbose', '-r', 'visualize_lidar.sdf'],
            cmd=['ign', 'gazebo', '--verbose', '-r', world],
            # cmd=['ign', 'gazebo', '--verbose', '-r', 'empty.sdf'],
            output='screen'
        ),
        Node(
            package='image_tools',
            executable='showimage',
            name='show_rgb_image',
            output='screen',
            remappings=[('/image', '/camera/rgb/image_raw')]
        ),
        Node(
            package='image_tools',
            executable='showimage',
            name='show_depth_image',
            output='screen',
            remappings=[('/image', '/camera/depth/image_raw')]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config]
        )
    ])

