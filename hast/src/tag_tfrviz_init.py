#!/usr/bin/env python  
import roslib
# roslib.load_manifest('learning_tf')
import rospy
import tf


def broadcast_tag(tag_name, map_topic, z):
    x = 0.0
    br.sendTransform((x, 0.0, z),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),("/{}/base_footprint").format(tag_name), map_topic)
    br.sendTransform((x, 0.0, z),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),("/{}/base_link").format(tag_name), map_topic)
    br.sendTransform((x, 0.0, z),tf.transformations.quaternion_from_euler(0, 0, 0), rospy.Time.now(),("/{}/fill_link").format(tag_name), map_topic)

if __name__ == '__main__':
    rospy.init_node('tag_tfrviz_init')
    map_topic = rospy.get_param('~map_topic', 'map')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(1)
    # while not rospy.is_shutdown():
    offset = 0.0
    off_delta = 0.05
    for i in range(2):
        broadcast_tag("id00_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id01_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id02_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id03_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id04_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id05_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id06_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id07_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id08_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id09_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id10_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id11_16h5", map_topic, offset); offset+=off_delta
        broadcast_tag("id12_16h5", map_topic, offset); offset+=off_delta
        rate.sleep()
