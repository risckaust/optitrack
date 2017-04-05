#!/usr/bin/env python
import rospy

#from std_msgs.msg import *
from geometry_msgs.msg import Pose, TransformStamped
from optitrack.msg import RigidBody, RigidBodyArray

class RigidBodies:
	def __init__(self):
		# define mavros fake gps msg, to publish to
		self.mavros_msg = TransformStamped()
		self.mavros_msg.transform.translation.x	=0.0
		self.mavros_msg.transform.translation.y	=0.0
		self.mavros_msg.transform.translation.z	=0.0
		self.mavros_msg.transform.rotation.x	=0.0
		self.mavros_msg.transform.rotation.y	=0.0
		self.mavros_msg.transform.rotation.z	=0.0
		self.mavros_msg.transform.rotation.w	=0.0
		
		# define mocap message to subsribe to
		self.mocap_msg				= RigidBodyArray()
	
		self.rbId				= 0
		
		# check if tracking is valid
		self.tracking_valid			= False

	# retrieves mocap topic and update the mocap_msg field
	def cb(self, msg):
		if (msg is not None):
			if ( len(msg.bodies) > 0):
				self.tracking_valid=msg.bodies[self.rbId].tracking_valid

				self.mavros_msg.header.stamp		= msg.header.stamp
				self.mavros_msg.header.frame_id		="world"

				self.mavros_msg.transform.translation.x = -msg.bodies[self.rbId].pose.position.x
				self.mavros_msg.transform.translation.y = msg.bodies[self.rbId].pose.position.z
				self.mavros_msg.transform.translation.z = msg.bodies[self.rbId].pose.position.y

				self.mavros_msg.transform.rotation.x = -msg.bodies[self.rbId].pose.orientation.x
				self.mavros_msg.transform.rotation.y = msg.bodies[self.rbId].pose.orientation.z
				self.mavros_msg.transform.rotation.z = msg.bodies[self.rbId].pose.orientation.y
				self.mavros_msg.transform.rotation.w = msg.bodies[self.rbId].pose.orientation.w

def main():
	# node name
	rospy.init_node('fake_gps', anonymous=True)
	rate = rospy.Rate(50) # Hz

	# instantiate a rigid body objecy
	rBody = RigidBodies()
	
	# subscribe to mocap topic
	rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, rBody.cb)

	# publisher for the mavros fake gps topic
	mavros_pub = rospy.Publisher('/mavros/fake_gps/fix', TransformStamped, queue_size=1)

	while not rospy.is_shutdown():
		if rBody.tracking_valid :
			#print "tracking valid..."
			mavros_pub.publish(rBody.mavros_msg)
		else:
			rospy.logwarn('Tracking of rigidbody is invalid')
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
