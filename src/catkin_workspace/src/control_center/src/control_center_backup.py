#!/usr/bin/env python3

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMainWindow

from motor_testbench.msg import *
# import rosbag
import time

class ControlCenter(Plugin):

    def __init__(self, context):
        # rospy.init_node('QtGUI_node', anonymous = True)
        super(ControlCenter, self).__init__(context)
        self.setObjectName('ControlCenter')

        # Create window
        self._widget = QMainWindow()
        # Load the UI resource
        ui_file = os.path.join(rospkg.RosPack().get_path('control_center'), 'resource', 'motor_testbench.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ControlCenterUi')

        # Add plugin pane name/number
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Initiate the control publishers
        self.state_channel = rospy.Publisher('/motor_state/1/', tmotor_cmd, queue_size=10)
        rospy.Subscriber('/can_response/motor_1/', tmotor_data, self.motor_response_callback)

        # Connect the slider 
        self.sl_joint_pos = self._widget.sliderj1
        # self.sl_kp = self._widget.kpSlider
        # self.sl_kd = self._widget.kdSlider
        # self.sl_joint_pos.valueChanged[int].connect(self.setJ1Slider)
        # Connect the buttons
        self._widget.enable.clicked[bool].connect(self.on_click_enable)
        self._widget.disable.clicked[bool].connect(self.on_click_disable)
        self._widget.setZero.clicked[bool].connect(self.on_click_setZero)
        self.cb_enable_control = self._widget.enableControl
        self.cb_enable_control.toggled.connect(self.on_enable_control)
        self._widget.enable.clicked[bool].connect(self.on_click_move)

        # Message config
        self.state_pub_duration = 0.4 # s
        self.command_pub_duration = 0.15 # s
        self.rate = rospy.Rate(50)

        # Setup useful variables
        self.enable_control = False

    def on_click_enable(self):
        cmd = tmotor_cmd()
        cmd.status = True
        self.send_multiple_messages(cmd, self.state_pub_duration)

    def on_click_disable(self):
        cmd = tmotor_cmd()
        cmd.status = False
        self.send_multiple_messages(cmd, self.state_pub_duration)

    def on_click_setZero(self):
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = True
        self.send_multiple_messages(cmd, self.state_pub_duration)
        # Come out of set zero by re-arming the motor again
        self.on_click_enable()

    # This is to ensure that the messages are reached even 
    # with the constant rtr pings for data
    def send_multiple_messages(self, cmd, pub_duration):
        # get current time
        now = time.time()
        # publish message for a preset duration of time
        while ((time.time()-now) < pub_duration):
            self.state_channel.publish(cmd)
            self.rate.sleep()

    # Modify sliders based on if the control mode is enabled or not
    def on_enable_control(self):
        self.enable_control = self.cb_enable_control.isChecked()
        # raise Exception('Note: can handle only position control currently')

    def motor_response_callback(self, data):
        # print('in the callback')
        # Also use the status to show color bulbs
        # The updated values pos, vel and torque can be stored here if required
        # If control is disabled
        print(data.position)
        print(self._widget.ui_pos.setText(str(data.position)))
        # self._widget.ui_pos.setText(str(data.position))
        # self._widget.ui_vel.setText(str(data.velocity))
        # self._widget.ui_tau.setText(str(data.torque))
        # if not self.enable_control:
        #     # Set slider value based on the incoming encoder value
        #     self.sl_joint_pos.setValue(data.position)


    # def setJ1Slider(self):        
    #     if not self.enable_control:
    #         # if the control is disabled, ignore slider changes
    #         pass
    #     else:
    #         raise NotImplementedError


    # def on_click_move(self):
    #     # Check if control is enabled
    #     if self.enable_control:
    #         cmd = tmotor_cmd()
    #         # Read the desired position from the slider
    #         cmd.position = self.sl_joint_pos.value()
    #         cmd.velocity = 0
    #         cmd.torque = 0
    #         cmd.kp = self.sl_kp.value()
    #         cmd.kd = self.sl_kd.value()
    #         print(cmd.position, cmd.kp, cmd.kd)
    #         # self.send_multiple_messages(cmd, self.command_pub_duration)
    #     else:
    #         raise Exception('The motor is not in control mode, please check the "Enable control" first')
    #     pass







































    # TODO implementation of necessary routines
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
                





------------------------------------------------------------------------------
#!/usr/bin/env python3

import os
import rospy
import rospkg

from qt_gui.plugin import Plugin
from python_qt_binding import loadUi
from python_qt_binding.QtWidgets import QWidget, QMainWindow

from motor_testbench.msg import *
# import rosbag
import time

class ControlCenter(Plugin):

    def __init__(self, context):
        # rospy.init_node('QtGUI_node', anonymous = True)
        super(ControlCenter, self).__init__(context)
        self.setObjectName('ControlCenter')

        # Create window
        self._widget = QMainWindow()
        # Load the UI resource
        ui_file = os.path.join(rospkg.RosPack().get_path('control_center'), 'resource', 'motor_testbench.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ControlCenterUi')

        # Add plugin pane name/number
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() + (' (%d)' % context.serial_number()))
        context.add_widget(self._widget)

        # Initiate the control publishers
        self.state_channel = rospy.Publisher('/motor_state/1/', tmotor_cmd, queue_size=10)
        rospy.Subscriber('/can_response/motor_1/', tmotor_data, self.motor_response_callback)

        # Connect the slider 
        self.sl_joint_pos = self._widget.sliderj1
        self.sl_kd_val = self._widget.kdSlider
        self.sl_kp_val = self._widget.kdSlider
        self.sl_joint_pos.setMinimum(-180)
        self.sl_joint_pos.setMaximum(180)
        self.sl_joint_pos.valueChanged[int].connect(self.setJ1Slider)
        # Connect the buttons
        self._widget.enable.clicked[bool].connect(self.on_click_enable)
        self._widget.disable.clicked[bool].connect(self.on_click_disable)
        self._widget.setZero.clicked[bool].connect(self.on_click_setZero)
        self.cb_enable_control = self._widget.enableControl
        self.cb_enable_control.toggled.connect(self.on_enable_control)

        # Message config
        self.state_pub_duration = 0.4 # s
        self.command_pub_duration = 0.15 # s
        self.rate = rospy.Rate(50)

        # Setup useful variables
        self.enable_control = False

    def on_click_enable(self):
        cmd = tmotor_cmd()
        cmd.status = True
        self.send_multiple_messages(cmd, self.state_pub_duration)

    def on_click_disable(self):
        cmd = tmotor_cmd()
        cmd.status = False
        self.send_multiple_messages(cmd, self.state_pub_duration)

    def on_click_setZero(self):
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = True
        self.send_multiple_messages(cmd, self.state_pub_duration)
        # Come out of set zero by re-arming the motor again
        self.on_click_enable()

    # This is to ensure that the messages are reached even 
    # with the constant rtr pings for data
    def send_multiple_messages(self, cmd, pub_duration):
        # get current time
        now = time.time()
        # publish message for a preset duration of time
        while ((time.time()-now) < pub_duration):
            self.state_channel.publish(cmd)
            self.rate.sleep()

    # Modify sliders based on if the control mode is enabled or not
    def on_enable_control(self):
        self.enable_control = self.cb_enable_control.isChecked()

    def motor_response_callback(self, data):
        # print('in the callback')
        # Also use the status to show color bulbs
        # The updated values pos, vel and torque can be stored here if required
        # If control is disabled
        self._widget.ui_pos.setText(str(data.position))
        self._widget.ui_vel.setText(str(data.velocity))
        self._widget.ui_tau.setText(str(data.torque))
        if not self.enable_control:
            # Set slider value based on the incoming encoder value
            self.sl_joint_pos.setValue(data.position)
                        

    def setJ1Slider(self):        
        if not self.enable_control:
            # if the control is disabled, ignore slider changes
            pass
        else:
            raise NotImplementedError







































    # TODO implementation of necessary routines
    def shutdown_plugin(self):
        # TODO unregister all publishers here
        pass

    def save_settings(self, plugin_settings, instance_settings):
        # TODO save intrinsic configuration, usually using:
        # instance_settings.set_value(k, v)
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # TODO restore intrinsic configuration, usually using:
        # v = instance_settings.value(k)
        pass
                


