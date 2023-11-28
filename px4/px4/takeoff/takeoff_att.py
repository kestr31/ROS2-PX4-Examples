import rclpy
from rclpy.node import Node
from rclpy.qos import ReliabilityPolicy, QoSProfile, DurabilityPolicy, HistoryPolicy

from px4_msgs.msg import TimesyncStatus, VehicleCommand, OffboardControlMode
from px4_msgs.msg import EstimatorStates, VehicleStatus, VehicleAngularVelocity, VehicleLocalPosition, VehicleAttitude
from px4_msgs.msg import TrajectorySetpoint, VehicleAttitudeSetpoint

import math
import numpy as np

class TakeoffAtt(Node):

    def __init__(self) -> None:
        super().__init__('takeoff')

        # INITIALIZE SUBSCRIBERS
        self.timesync_subscriber                = self.createSubscriber("/fmu/out/timesync_status", TimesyncStatus, self.timesyncStatusCallback, 10)
        self.VehicleStatus_subscriber           = self.createSubscriber("/fmu/out/vehicle_status", VehicleStatus, self.vehicleStatusCallback, 10)
        self.estimatorStates_subscriber         = self.createSubscriber("/fmu/out/estimator_states", EstimatorStates, self.estimatorStatesCallback, 10)
        self.VehicleLocalPosition_subscriber    = self.createSubscriber("/fmu/out/vehicle_local_position", VehicleLocalPosition, self.vehicleLocalPositionCallback, 10)
        self.vehicleAttitude_subscriber         = self.createSubscriber("/fmu/out/vehicle_attitude", VehicleAttitude, self.vehicleAttitudeCallback, 10)
        self.vehicleAngularVelocity_subscriber  = self.createSubscriber("/fmu/out/vehicle_angular_velocity", VehicleAngularVelocity, self.vehicleAngularVelocityCallback, 10)

        # INITIALIZE PUBLISHERS
        self.vehicleCommand_publisher           = self.createPublisher("/fmu/in/vehicle_command", VehicleCommand, 0)
        self.offboardControlMode_publisher      = self.createPublisher("/fmu/in/offboard_control_mode", OffboardControlMode, 0)
        self.trajectorySetpoint_publisher       = self.createPublisher("/fmu/in/trajectory_setpoint", TrajectorySetpoint, 0)
        self.vehicleAttitudeSetpoint_publisher  = self.createPublisher("/fmu/in/vehicle_attitude_setpoint", VehicleAttitudeSetpoint, 0)

        # INITIALIZE VARIABLES
        self.offboardSetpointCounter = 0

        # INITIALIZE CONSTRUCTORS (PX4 TOPICS)
        self.timesyncStatus         = TimesyncStatus()
        self.vehicleStatus          = VehicleStatus()
        self.estimatorStates        = EstimatorStates()
        self.vehicleLocalPosition   = VehicleLocalPosition()
        self.vehicleAttitude        = VehicleAttitude()
        self.vehicleAngularVelocity = VehicleAngularVelocity()

        # SET PARAMETERS
        self.takeoffHeight = -5.0

        self.takeOffFinished = False
        self.inAttitudeMode = False
        self.tgtAtt = 15.0

        self.timer = self.create_timer(0.1, self.timerCallback)


    def genQosProfile(self,mode,queueDepth=0):
        switcher = {
            "sub": [DurabilityPolicy.TRANSIENT_LOCAL, ReliabilityPolicy.BEST_EFFORT, HistoryPolicy.KEEP_LAST, 0],
            "pub": [DurabilityPolicy.VOLATILE,        ReliabilityPolicy.BEST_EFFORT, HistoryPolicy.KEEP_LAST, queueDepth]
        }

        if not mode in switcher:
            raise Exception("Invalid mode. Use 'sub' or 'pub'.")

        qosProfile = QoSProfile(
            durability      = switcher.get(mode)[0],
            reliability     = switcher.get(mode)[1],
            history         = switcher.get(mode)[2],
            depth           = switcher.get(mode)[3]
        )

        return qosProfile


    def createSubscriber(self,topicName,topicType,callback,queueDepth=0):
        subscriber = self.create_subscription(
            msg_type    = topicType,
            topic       = topicName,
            callback    = callback,
            qos_profile = self.genQosProfile("sub",queueDepth)
        )

        return subscriber

    def createPublisher(self,topicName,topicType,queueDepth=0):
        publisher = self.create_publisher(
            msg_type    = topicType,
            topic       = topicName,
            qos_profile = self.genQosProfile("pub",queueDepth)
        )
        return publisher

    # SUBSCRIPTION CALLBACKS
    # -----------------------------------------------------------------------------------------------
    def timesyncStatusCallback(self,timesyncStatus):
        self.timesyncStatus = timesyncStatus

    def vehicleStatusCallback(self,vehicleStatus):
        self.vehicleStatus = vehicleStatus

    def estimatorStatesCallback(self,estimatorStates):
        self.estimatorStates = estimatorStates

    def vehicleLocalPositionCallback(self,vehicleLocalPosition):
        self.vehicleLocalPosition = vehicleLocalPosition

    def vehicleAttitudeCallback(self,vehicleAttitude):
        self.vehicleAttitude = vehicleAttitude

    def vehicleAngularVelocityCallback(self,vehicleAngularVelocity):
        self.vehicleAngularVelocity = vehicleAngularVelocity


    # PUBLISHER METHODS
    def publishOffboardControlMode(self,mode):
        switcher = {
            "position":     [True,  False, False, False, False],
            "velocity":     [False, True,  False, False, False],
            "acceleration": [False, False, True,  False, False],
            "attitude":     [False, False, False, True,  False],
            "body_rate":    [False, False, False, False, True],
        }

        if not mode in switcher:
            raise Exception("Invalid mode. Use 'position', 'velocity', 'acceleration', 'attitude' or 'body_rate'.")

        msg = OffboardControlMode()

        msg.position        = switcher.get(mode)[0]
        msg.velocity        = switcher.get(mode)[1]
        msg.acceleration    = switcher.get(mode)[2]
        msg.attitude        = switcher.get(mode)[3]
        msg.body_rate       = switcher.get(mode)[4]

        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)

        self.offboardControlMode_publisher.publish(msg)


    def publishVehicleCommand(self,mode):
        switcher = {
            "setOffboard":  [VehicleCommand.VEHICLE_CMD_DO_SET_MODE, {"param1": 1.0, "param2": 6.0} ],
            "arm":          [VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, {"param1": 1.0} ],
            "disarm":       [VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, {"param1": 0.0} ],
            "land":         [VehicleCommand.VEHICLE_CMD_NAV_LAND, {} ]
        }

        if not mode in switcher:
            raise Exception("Invalid mode. Use 'setOffboard', 'arm', 'disarm' or 'land'.")

        msg = VehicleCommand()

        msg.command = switcher[mode][0]

        msg.param1  = switcher[mode][1].get("param1",0.0)
        msg.param2  = switcher[mode][1].get("param2",0.0)
        msg.param3  = switcher[mode][1].get("param3",0.0)
        msg.param4  = switcher[mode][1].get("param4",0.0)
        msg.param5  = switcher[mode][1].get("param5",0.0)
        msg.param6  = switcher[mode][1].get("param6",0.0)
        msg.param7  = switcher[mode][1].get("param7",0.0)

        msg.target_system       = 1
        msg.target_component    = 1
        msg.source_system       = 1
        msg.source_component    = 1
        msg.from_external       = True
        msg.timestamp           = int(self.get_clock().now().nanoseconds / 1000)
        
        self.vehicleCommand_publisher.publish(msg)


    def publishTrajectorySetpoint(self,position: float):
        if len(position) != 3:
            raise Exception("Invalid position. Use [x,y,z].")
        
        msg = TrajectorySetpoint()
        
        msg.position    = position
        msg.yaw         = 0.0 # HARDCODED: YAW SET TO 0 RAD
        msg.timestamp   = int(self.get_clock().now().nanoseconds / 1000)

        self.trajectorySetpoint_publisher.publish(msg)


    def publishAttitudeSetpoint(self,attitude: float, thrustBody: float):
        if not ( len(attitude) == 3 or len(attitude) == 4 ):
            raise Exception("Invalid attitude. Use [roll,pitch,yaw] or [q0,q1,q2,q3].")
        
        if not len(thrustBody) == 3:
            raise Excption("Invalid thrustBody. Use [x,y,z].")

        if len(attitude) == 3:
            input = self.euler2Quaternion(attitude)
        else:
            input = attitude

        msg = VehicleAttitudeSetpoint()

        msg.q_d = input
        msg.thrust_body = thrustBody
        msg.yaw_sp_move_rate = 0.0 # HARDCODED: YAWRATE SET TO 0 RAD/S

        self.vehicleAttitudeSetpoint_publisher.publish(msg)


    # FUNCTIONAL METHODS
    def arm(self):
        self.publishVehicleCommand("arm")

    def disarm(self):
        self.publishVehicleCommand("disarm")

    def setOffboard(self):
        self.publishVehicleCommand("setOffboard")

    def land(self):
        self.publishVehicleCommand("land")


    # UTILITY METHODS
    def euler2Quaternion(self,euler):
        if not len(euler) == 3:
            raise Exception("Invalid euler. Use [roll,pitch,yaw].")

        eulerRad = np.radians(euler)

        cosYaw      = math.cos(eulerRad[2] / 2)
        sinYaw      = math.sin(eulerRad[2] / 2)
        cosPitch    = math.cos(eulerRad[1] / 2)
        sinPitch    = math.sin(eulerRad[1] / 2)
        cosRoll     = math.cos(eulerRad[0] / 2)
        sinRoll     = math.sin(eulerRad[0] / 2)

        w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw
        x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw
        y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw
        z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw

        return [w,x,y,z]

    # TIMER CALLBACK METHOD
    def timerCallback(self) -> None:
        self.publishOffboardControlMode("position")

        if self.offboardSetpointCounter == 10:
            self.setOffboard()
            self.arm()

        if not self.takeOffFinished:
            if (self.vehicleLocalPosition.z - self.takeoffHeight) > 0.1 and self.vehicleStatus.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD:
                self.publishOffboardControlMode('position')
                self.publishTrajectorySetpoint([0.0,0.0,self.takeoffHeight])  
                print("Deviation from takeoff height: " + str(self.vehicleLocalPosition.z - self.takeoffHeight) + " m")
            elif self.vehicleLocalPosition.z - self.takeoffHeight <= 0.1:
                self.takeOffFinished = True
                self.inAttitudeMode = True
                print("Deviation from takeoff height: " + str(self.vehicleLocalPosition.z - self.takeoffHeight) + " m")
                print("Target height reached. Switching to attitude mode.")

        if self.inAttitudeMode:
            print("Attitude mode active.")
            self.publishOffboardControlMode('attitude')
            self.publishAttitudeSetpoint([0.0,self.tgtAtt,0.0],[0.0,0.0,-(math.sqrt(2)/2)*(1/math.cos(self.tgtAtt*math.pi/180))])
            print((math.sqrt(2)/2)*(1/math.cos(15*math.pi/180)))
        if self.offboardSetpointCounter < 11:
            self.offboardSetpointCounter += 1


def main(args=None) -> None:
    rclpy.init(args=args)
    takeoff_att = TakeoffAtt()
    try:
        rclpy.spin(takeoff_att)
    except KeyboardInterrupt:
        takeoff_att = get_logger().info('Keyboard Interrupt (SIGINT)')
    finally:
        takeoff_att.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(e)