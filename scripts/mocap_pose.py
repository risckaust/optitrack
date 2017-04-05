#!/usr/bin/env python
import rospy

#from std_msgs.msg import *
from geometry_msgs.msg import Pose, TransformStamped, PoseStamped
from optitrack.msg import RigidBody, RigidBodyArray

class RigidBodies:
	def __init__(self):
		# define mavros fake gps msg, to publish to
		self.mavros_msg = PoseStamped()
		self.mavros_msg.pose.position.x		=0.0
		self.mavros_msg.pose.position.y		=0.0
		self.mavros_msg.pose.position.z		=0.0
		self.mavros_msg.pose.orientation.x	=0.0
		self.mavros_msg.pose.orientation.y	=0.0
		self.mavros_msg.pose.orientation.z	=0.0
		self.mavros_msg.pose.orientation.w	=0.0
		
		# define mocap message to subsribe to
		self.mocap_msg				= RigidBodyArray()
	
		self.rbId				= 0
		
		# check if tracking is valid
		self.tracking_valid			= False

	# retrieves mocap topic and update /mavros/mocap/pose msg
	def cb(self, msg):
		if (msg is not None):
			if ( len(msg.bodies) > 0):
				self.tracking_valid=msg.bodies[self.rbId].tracking_valid

				#self.mavros_msg.header.stamp		= rospy.Time.now()
				self.mavros_msg.header.frame_id		="world"
				
				self.mavros_msg.pose.position.x		=-msg.bodies[self.rbId].pose.position.x	# inverted!!!
				self.mavros_msg.pose.position.y		=msg.bodies[self.rbId].pose.position.z
				self.mavros_msg.pose.position.z		=msg.bodies[self.rbId].pose.position.y

				self.mavros_msg.pose.orientation.x	=-msg.bodies[self.rbId].pose.orientation.x
				self.mavros_msg.pose.orientation.y	=msg.bodies[self.rbId].pose.orientation.z
				self.mavros_msg.pose.orientation.z	=msg.bodies[self.rbId].pose.orientation.y
				self.mavros_msg.pose.orientation.w	=msg.bodies[self.rbId].pose.orientation.w

def main():
	# node name
	rospy.init_node('mocap_pose', anonymous=True)
	rate = rospy.Rate(100) # Hz

	# instantiate a rigid body objecy
	rBody = RigidBodies()
	
	# subscribe to mocap topic
	rospy.Subscriber('/optitrack/rigid_bodies', RigidBodyArray, rBody.cb)

	# publisher for the mavros fake gps topic
	mavros_pub = rospy.Publisher('/mavros/mocap/pose', PoseStamped, queue_size=1)

	while not rospy.is_shutdown():
		if rBody.tracking_valid :
			#print "tracking valid..."
			rBody.mavros_msg.header.stamp= rospy.Time.now()
			mavros_pub.publish(rBody.mavros_msg)
		else:
			print "tracking not valid"
		rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
