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

        # Connect the slider
        self.sl_joint_pos = self._widget.sliderj1
        self.sl_joint_pos.setMinimum(-180)
        self.sl_joint_pos.setMaximum(180)

        self.sl_kd_val = self._widget.kdSlider
        self.sl_kd_val.setMinimum(0)
        self.sl_kd_val.setMaximum(5)

        self.sl_kp_val = self._widget.kpSlider
        self.sl_kp_val.setMinimum(0)
        self.sl_kp_val.setMaximum(500)

        # Connect to the motorID input box
        self.motorID_input = self._widget.motorID_input

        # Connect the buttons
        self._widget.motor_connect.clicked[bool].connect(self.on_click_connect)
        self._widget.enable.clicked[bool].connect(self.on_click_enable)
        self._widget.disable.clicked[bool].connect(self.on_click_disable)
        self._widget.setZero.clicked[bool].connect(self.on_click_setZero)
        self._widget.sendControl.clicked[bool].connect(self.on_click_move)
        self.cb_enable_control = self._widget.enableControl
        self.cb_enable_control.toggled.connect(self.on_enable_control)

        # Message config
        self.state_pub_duration = 0.4 # s
        self.command_pub_duration = 0.15 # s
        self.rate = rospy.Rate(50)

        # Setup useful variables
        self.enable_control = True


    def on_click_connect(self):
        # # Make the motor number a slot into which the plugin connects to, that way we can yse the same plugin multiple times
        # Select the motorID upon an connect button is clicked
        self.motorID = int(self.motorID_input.toPlainText())

        # Initiate the corresponding state publishers
        self.state_channel = rospy.Publisher('motor_state/'+str(self.motorID)+'/', tmotor_cmd, queue_size=10)
        # Initiate control publishers
        self.command_channel = rospy.Publisher('motor_command/'+str(self.motorID)+'/', tmotor_cmd, queue_size = 10)

        # Subscribe to the response on CAN
        rospy.Subscriber('can_response/motor_'+str(self.motorID), tmotor_data, self.motor_response_callback)


    # Modify sliders based on if the control mode is enabled or not
    def on_enable_control(self):
        self.enable_control = self.cb_enable_control.isChecked()

    def on_click_enable(self):
        cmd = tmotor_cmd()
        cmd.status = True

        self.send_multiple_messages(cmd, self.state_channel, self.state_pub_duration)
        cmd = tmotor_cmd()
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.send_multiple_messages(cmd, self.command_channel, self.command_pub_duration)

    def on_click_disable(self):
        cmd = tmotor_cmd()
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.send_multiple_messages(cmd, self.command_channel, self.command_pub_duration)
        cmd = tmotor_cmd()
        cmd.status = False
        self.send_multiple_messages(cmd, self.state_channel, self.state_pub_duration)

    def on_click_setZero(self):
        cmd = tmotor_cmd()
        cmd.status = True
        cmd.setzero = True
        self.send_multiple_messages(cmd, self.state_channel, self.state_pub_duration)
        cmd = tmotor_cmd()
        cmd.position = 0
        cmd.velocity = 0
        cmd.torque = 0
        cmd.kp = 0
        cmd.kd = 0
        self.send_multiple_messages(cmd, self.command_channel, self.command_pub_duration)
        # Come out of set zero by re-arming the motor again
        # self.on_click_enable()

    def on_click_move(self):
        # Check if control is enabled
        if self.enable_control:
            cmd = tmotor_cmd()
            # Read the values from the sliders
            cmd.position = self.sl_joint_pos.value()
            cmd.velocity = 0
            cmd.torque = 0
            cmd.kp = self.sl_kp_val.value()
            cmd.kd = self.sl_kd_val.value()
            self.send_multiple_messages(cmd, self.command_channel, self.command_pub_duration)
        else:
            raise Exception('Control not enabled')

    # This is to ensure that the messages are reached even
    # with the constant rtr pings for data
    def send_multiple_messages(self, cmd, channel, pub_duration):
        # Send only one message, since RTR is off this should work as predicted
        channel.publish(cmd)
        # For continuous publish
        # # get current time
        # now = time.time()
        # # publish message for a preset duration of time
        # while ((time.time()-now) < pub_duration):
        #     channel.publish(cmd)
        #     self.rate.sleep()

    def motor_response_callback(self, data):
        # Also use the status to show color bulbs
        # The updated values pos, vel and torque can be stored here if required
        # If control is disabled
        self._widget.ui_pos.setText(str(data.position))
        self._widget.ui_vel.setText(str(data.velocity))
        self._widget.ui_tau.setText(str(data.torque))
        if not self.enable_control:
            # Set slider value based on the incoming encoder value
            self.sl_joint_pos.setValue(data.position)






























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
