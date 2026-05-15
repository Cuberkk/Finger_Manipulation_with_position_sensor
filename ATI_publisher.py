import rclpy
from rclpy.time import Time
import tf2_ros
import numpy as np
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from LabjackReader import LabjackATIReader
from scipy.spatial.transform import Rotation as R

class LabjackATINode(Node):
    def __init__(self):
        super().__init__('ati_node')
        self.cal_path = 'calibration_files/FT44297.cal'
        self.publish_rate = 600.0
        self.ati_reader = LabjackATIReader(
            cal_path=self.cal_path,
            aq_rate=self.publish_rate,
            bias_time=5,
            bias_switch=True
        )
        self.ati_pub = self.create_publisher(Float64MultiArray, 'ati_data', 1)
        self.timer = self.create_timer(1.0 / 600.0, self.timer_callback)

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.rot_gl_gbase = np.array([[1., 0., 0.],
                                           [0., -0.74895572, 0.66262005],
                                           [0., -0.66262005, -0.74895572]])
        self.rot_gbase_ATI = None
        self.miss_itr = 0
        self.total_itr = 0
        self.trans_delay = 1.5
        print("Labjack ATI Node Initialized")

    def timer_callback(self):
        ft_arr = self.ati_reader.read()
        data_arr = np.asarray(ft_arr[:3], dtype=np.float64)
        timestamp = self.get_clock().now().to_msg()
        self.total_itr += 1
        try:
            now = self.get_clock().now()
            t_gbase_ATI = self.tf_buffer.lookup_transform('KWGripperBase', 'KWATI', rclpy.time.Time())
            tf_time = t_gbase_ATI.header.stamp
            tf_stamp = Time(seconds=tf_time.sec, nanoseconds=tf_time.nanosec)
            dt = abs(now.nanoseconds - tf_stamp.nanoseconds) * 1e-9
            # print(dt)
            if dt < self.trans_delay:
                self.rot_gbase_ATI = self.transform_rot_generator(t_gbase_ATI)
                data_arr = (self.rot_gl_gbase @ self.rot_gbase_ATI @ data_arr.T).T
                # print(data_arr)
                timestamp_sec = np.float64(timestamp.sec + 1e-9 * timestamp.nanosec)
                data_arr = np.append(data_arr.astype(np.float64), timestamp_sec)
                # print(data_arr)
                data_msg = Float64MultiArray(data=data_arr.flatten().tolist())
                # print(f'Ground Truth: {round(abs(data_arr[2]), 2)}N')
                self.ati_pub.publish(data_msg)
            else:
                self.rot_gbase_ATI = None
                self.miss_itr += 1
                print(f"{self.miss_itr/self.total_itr * 100:.2f}%")
        except tf2_ros.TransformException:
            print("GripperBase to ATI transform not received")
            pass


    def transform_rot_generator(self, trans_msg):
        # Extract quaternion and convert to rotation matrix
        q = trans_msg.transform.rotation
        rotation = R.from_quat([q.x, q.y, q.z, q.w]).as_matrix()  # Convert quaternion to 3×3 rotation matrix
        return rotation
    
def main(args=None):
    rclpy.init(args=args)
    node = LabjackATINode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down node...")
    finally:
        node.destroy_node()
        if rclpy.ok(): 
            rclpy.shutdown()

if __name__ == '__main__':
    main()