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
                'drag_comp_enabled_': True, # True
                'ctrl_enabled_': True, # True
                'rate_ctrl_enabled_': False, # True
                'mission_enabled_': False,
                'mavYaw_': 0.0, # 0.0
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
                'c_x_':0.0,
                'c_y_':0.0,
                'c_z_':1.0, #1.0
                'r_x_':1.0, #1.0
                'r_y_':1.0, #1.0
                'r_z_':0.0,
                'fr_x_':1.0,
                'fr_y_':1.0,
                'fr_z_':0.0,
                'ph_x_':1.57,
                'ph_y_':0.0,
                'ph_z_':0.0,
                'max_fb_acc_': 20.0,
                'rotorDragD_x_':1.25,
                'rotorDragD_y_':1.25,
                'rotorDragD_z_':0.0,
                'norm_thrust_const_':0.034436,
                'norm_thrust_offset_':0.14344,
                'attctrl_tau_':0.1,
                'c_x_1_':0.0,
                'c_y_1_':0.0,
                'c_z_1_':1.0,
                'c_x_2_':1.5,
                'c_y_2_':0.0,
                'c_z_2_':1.0,
                'c_x_3_':0.0,
                'c_y_3_':1.5,
                'c_z_3_':1.0,
                'pos_x_0_':0.0,
                'pos_y_0_':0.0,
                'pos_z_0_':1.0, #1.0
            }]
        )
    ])