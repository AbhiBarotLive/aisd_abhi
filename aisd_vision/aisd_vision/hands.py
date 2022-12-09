import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import mediapipe as mp
mp_hands = mp.solutions.hands
from aisd_interfaces.msg import Hand


class ImageSubscriber(Node):

    def __init__(self):
        super().__init__('image_subscriber')
        self.br = CvBridge()
        self.hand_publisher = self.create_publisher(Hand, 'cmd_hand', 10)
        self.subscription = self.create_subscription(Image,'video_frames',self.listener_callback,10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        image = self.br.imgmsg_to_cv2(msg)
        PINKY_FINGER_TIP = 20
        INDEX_FINGER_TIP = 8
        #Analyse the image for hands
        with mp_hands.Hands(
            model_complexity=0,
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5) as myhands:

         # To improve performance, optionally mark the image as not writeable to
         # pass by reference.
         image.flags.writeable = False
         image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
         results = myhands.process(image)
         if results.multi_hand_landmarks:
             #publish the hand position in terms of index finger and pinky
             msg = Hand()
             msg.xpinky = results.multi_hand_landmarks[0].landmark[PINKY_FINGER_TIP].x
             msg.xindex = results.multi_hand_landmarks[0].landmark[INDEX_FINGER_TIP].x
             if self.hand_publisher.get_subscription_count() > 0:
                 self.hand_publisher.publish(msg)
             else:
                 self.get_logger().info('waiting for subcriber')




def main(args=None):
    rclpy.init(args=args)

    image_subscriber = ImageSubscriber()

    rclpy.spin(image_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    image_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
