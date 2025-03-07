from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Return the LaunchDescription with the arguments & parameters
    return LaunchDescription([
        Node(
            package='double_sls_qsf', 
            executable='sls_qsf',        
            name='sls_qsf_controller',
            output='screen',
            parameters=[{
                'lpf_enabled_': True,
                'finite_diff_enabled_': True,
                'traj_tracking_enabled_': False,
                'Kpos_x_':24.0,
                'Kpos_y_':24.0,
                'Kpos_z_':2.0,
                'Kvel_x_':50.0,
                'Kvel_y_':50.0,
                'Kvel_z_':3.0,
                'Kacc_x_':35.0,
                'Kacc_y_':35.0,
                'Kacc_z_':0.0,
                'Kjer_x_':10.0,
                'Kjer_y_':10.0,
                'Kjer_z_':0.0,
                'mass_':1.56,
                'cable_length_':0.85,
                'load_mass_':0.25,
                'c_x':0.0,
                'c_y':0.0,
                'c_z':1.0,
                'r_x':1.0,
                'r_y':1.0,
                'r_z':0.0,
                'fr_x':1.0,
                'fr_y':1.0,
                'fr_z':0.0,
                'ph_x':1.57,
                'ph_y':0.0,
                'ph_z':0.0
            }]
        )
    ])
