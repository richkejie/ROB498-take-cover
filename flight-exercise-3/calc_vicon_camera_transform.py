import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry

from rclpy.qos import qos_profile_system_default


FREQ_30_HZ = 1/30 # [1/Hz]
FREQ_0_5_HZ = 2 # [1/Hz]
FREQ_10_HZ = 1/10 # [1/Hz]

LOG_LATEST_POSE = True


def quaternion_multiply(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quaternion_conjugate(q):
    x, y, z, w = q
    return (-x, -y, -z, w)

def quaternion_inverse(q):
    x, y, z, w = q
    norm_sq = x * x + y * y + z * z + w * w
    if norm_sq == 0.0:
        return (0.0, 0.0, 0.0, 1.0)
    q_conj = quaternion_conjugate(q)
    return (
        q_conj[0] / norm_sq,
        q_conj[1] / norm_sq,
        q_conj[2] / norm_sq,
        q_conj[3] / norm_sq,
    )


def rotate_vector_by_quaternion(vec, q):
    vx, vy, vz = vec
    q_vec = (vx, vy, vz, 0.0)
    q_rot = quaternion_multiply(quaternion_multiply(q, q_vec), quaternion_conjugate(q))
    return (q_rot[0], q_rot[1], q_rot[2])


def transform_pose(self, pose, transform):
        """
        Transform a PoseStamped using a TransformStamped.

        Assumes `transform` maps from `transform.child_frame_id` to
        `transform.header.frame_id`. The returned pose is in
        `transform.header.frame_id`.
        """
        if pose is None or transform is None:
            return None

        q_t = (
            transform.transform.rotation.x,
            transform.transform.rotation.y,
            transform.transform.rotation.z,
            transform.transform.rotation.w,
        )
        t_t = (
            transform.transform.translation.x,
            transform.transform.translation.y,
            transform.transform.translation.z,
        )

        p_in = (
            pose.pose.position.x,
            pose.pose.position.y,
            pose.pose.position.z,
        )
        q_in = (
            pose.pose.orientation.x,
            pose.pose.orientation.y,
            pose.pose.orientation.z,
            pose.pose.orientation.w,
        )

        p_rot = rotate_vector_by_quaternion(p_in, q_t)
        p_out = (
            p_rot[0] + t_t[0],
            p_rot[1] + t_t[1],
            p_rot[2] + t_t[2],
        )
        q_out = quaternion_multiply(q_t, q_in)

        out_pose = PoseStamped()
        out_pose.header.stamp = self.get_clock().now().to_msg()
        out_pose.header.frame_id = transform.header.frame_id
        out_pose.pose.position.x = float(p_out[0])
        out_pose.pose.position.y = float(p_out[1])
        out_pose.pose.position.z = float(p_out[2])
        out_pose.pose.orientation.x = float(q_out[0])
        out_pose.pose.orientation.y = float(q_out[1])
        out_pose.pose.orientation.z = float(q_out[2])
        out_pose.pose.orientation.w = float(q_out[3])

        return out_pose
class MavrosVisionPoseNode(Node):
    def __init__(self):
        super().__init__('rob498_drone_1_vicon_camera_transform_node')
        # Poses
        self.initial_camera_pose = None # Startup pose (first power on)
        self.initial_vicon_pose = None # Startup pose (first power on)
        self.latest_camera_pose = None
        self.latest_vicon_pose = None
        self.latest_vicon_to_camera_tf = None

        # Set up publishers
        self.ego_pub = self.create_publisher(
            PoseStamped,
            "/mavros/vision_pose/pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_30_HZ, self.publish_position) # publish vision pose at 30Hz
        # self.create_timer(FREQ_0_5_HZ, self.print_position)

        self.ego_init_pub = self.create_publisher(
            PoseStamped,
            "/team1_fe3/vision_pose/initial_pose",
            qos_profile_system_default
        )
        self.create_timer(FREQ_0_5_HZ, self.publish_initial_position)

        self.vicon_to_camera_tf_pub = self.create_publisher(
            TransformStamped,
            "/team1_fe3/vicon_to_camera_transform",
            qos_profile_system_default
        )
        self.create_timer(FREQ_10_HZ, self.publish_vicon_to_camera_transform)
        
        # Set up subscribers
        self.vicon_sub = self.create_subscription(
            PoseStamped,
            '/vicon/ROB498_Drone/ROB498_Drone',
            self.vicon_callback,
            qos_profile_system_default
        )
        self.realsense_sub = self.create_subscription(
            Odometry,
            "/camera/pose/sample",
            self.realsense_callback,
            qos_profile_system_default
        )

        self.get_logger().info("MavrosVisionPoseNode initialized; initial pose not yet received.")

    ############################################################################
    # External pose update callbacks (camera & Vicon)
    ############################################################################
    def realsense_callback(self, msg):
        """Update pose from RealSense"""    
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "camera"
        current_pose.pose = msg.pose.pose
        
        # Rotate pose from camera by 180 degrees in yaw (flip x and y axes)
        current_pose.pose.position.x *= -1
        current_pose.pose.position.y *= -1

        # Update pose(s)
        if self.initial_camera_pose is None:
            self.initial_camera_pose = current_pose
            self.get_logger().info(
                f"Realsense - Set initial camera pose: x={self.initial_camera_pose.pose.position.x}, y={self.initial_camera_pose.pose.position.y}, z={self.initial_camera_pose.pose.position.z}"
            )
            
        self.latest_camera_pose = current_pose
        
        if LOG_LATEST_POSE:
            self.get_logger().info(
                f"Realsense - Latest camera pose: x={self.latest_camera_pose.pose.position.x}, y={self.latest_camera_pose.pose.position.y}, z={self.latest_camera_pose.pose.position.z}"
            )
    
    def vicon_callback(self, msg):
        """Update pose from Vicon"""
        current_pose = PoseStamped()
        current_pose.header.stamp = self.get_clock().now().to_msg()
        current_pose.header.frame_id = "vicon"
        current_pose.pose = msg.pose
        
        # Update pose(s)
        if self.initial_vicon_pose is None:
            self.initial_vicon_pose = current_pose
            self.get_logger().info(
                f"Vicon - Set initial vicon pose: x={self.initial_vicon_pose.pose.position.x}, y={self.initial_vicon_pose.pose.position.y}, z={self.initial_vicon_pose.pose.position.z}"
            )
        
        self.latest_vicon_pose = current_pose
        
        if LOG_LATEST_POSE:
            self.get_logger().info(
                f"Vicon - Latest vicon pose: x={self.latest_vicon_pose.pose.position.x}, y={self.latest_vicon_pose.pose.position.y}, z={self.latest_vicon_pose.pose.position.z}"
            )

    def calculate_transform_between_camera_and_vicon(self):
        """Compute transform T_vicon_camera from latest poses in world/map frame."""
        if self.latest_camera_pose is None or self.latest_vicon_pose is None:
            return None

        q_v = self.latest_vicon_pose.pose.orientation
        q_c = self.latest_camera_pose.pose.orientation

        v_p = self.latest_vicon_pose.pose.position
        c_p = self.latest_camera_pose.pose.position

        q_v_tuple = (q_v.x, q_v.y, q_v.z, q_v.w)
        q_c_tuple = (q_c.x, q_c.y, q_c.z, q_c.w)

        q_v_inv = self.quaternion_inverse(q_v_tuple)
        q_v_to_c = quaternion_multiply(q_v_inv, q_c_tuple)

        delta_world = (c_p.x - v_p.x, c_p.y - v_p.y, c_p.z - v_p.z)
        t_v_to_c = rotate_vector_by_quaternion(delta_world, q_v_inv)

        tf_msg = TransformStamped()
        tf_msg.header.stamp = self.get_clock().now().to_msg()
        tf_msg.header.frame_id = "vicon"
        tf_msg.child_frame_id = "camera"
        tf_msg.transform.translation.x = float(t_v_to_c[0])
        tf_msg.transform.translation.y = float(t_v_to_c[1])
        tf_msg.transform.translation.z = float(t_v_to_c[2])
        tf_msg.transform.rotation.x = float(q_v_to_c[0])
        tf_msg.transform.rotation.y = float(q_v_to_c[1])
        tf_msg.transform.rotation.z = float(q_v_to_c[2])
        tf_msg.transform.rotation.w = float(q_v_to_c[3])

        return tf_msg

    ############################################################################
    # Publisher functions
    ############################################################################
    def publish_position(self):
        """Publishes a new desired position."""
        if self.latest_camera_pose is None:
            self.get_logger().info("Latest pose not registered, nothing to publish")
        else:
            self.latest_camera_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_pub.publish(self.latest_camera_pose)

    def publish_initial_position(self):
        if self.initial_camera_pose is None:
            self.get_logger().info("Initial pose not registered, nothing to publish")
        else:
            self.initial_camera_pose.header.stamp = self.get_clock().now().to_msg()
            self.ego_init_pub.publish(self.initial_camera_pose)


    def publish_vicon_to_camera_transform(self):
        tf_msg = self.calculate_transform_between_camera_and_vicon()
        if tf_msg is None:
            return
        self.latest_vicon_to_camera_tf = tf_msg
        self.vicon_to_camera_tf_pub.publish(tf_msg)

    ############################################################################
    # Print Functions
    ############################################################################
    def print_position(self):
        if self.latest_camera_pose is not None:
            self.get_logger().info(
                f"Latest camera pose: x={self.latest_camera_pose.pose.position.x}, y={self.latest_camera_pose.pose.position.y}, z={self.latest_camera_pose.pose.position.z}"
            )
        if self.latest_vicon_pose is not None:
            self.get_logger().info(
                f"Latest vicon pose: x={self.latest_vicon_pose.pose.position.x}, y={self.latest_vicon_pose.pose.position.y}, z={self.latest_vicon_pose.pose.position.z}"
            )

def main(args=None):
    rclpy.init(args=args)
    node = MavrosVisionPoseNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
