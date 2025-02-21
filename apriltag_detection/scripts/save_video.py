import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import signal

# 视频保存路径
video_path = '/home/wei/Videos/apriltag_video'
if not os.path.exists(video_path):
    os.makedirs(video_path)

# 视频文件名
video_file = os.path.join(video_path, 'apriltag_video.avi')

# 定义CvBridge转换器
bridge = CvBridge()

# 设置视频编解码器
fourcc = cv2.VideoWriter_fourcc(*'XVID')
# 假设图片分辨率为640x480，如果不同需要根据实际情况修改
video_writer = cv2.VideoWriter(video_file, fourcc, 20.0, (640, 480))

# 用于标志视频录制是否需要停止
stop_recording = False

def image_callback(msg):
    global stop_recording
    if stop_recording:
        return
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
        
        # 写入视频
        video_writer.write(cv_image)
        
    except Exception as e:
        rospy.logerr("Error converting image: %s", str(e))

# 处理Ctrl+C等信号
def signal_handler(sig, frame):
    global stop_recording
    rospy.loginfo("Signal received, stopping video recording...")
    stop_recording = True

    # 释放视频写入对象
    video_writer.release()
    rospy.loginfo("Video recording stopped and file saved.")
    rospy.signal_shutdown("Shutting down")

def main():
    global stop_recording
    try:
        rospy.init_node('apriltag_video_saver', anonymous=True)

        # 订阅标签检测的图像话题
        rospy.Subscriber('/tag_detections_image', Image, image_callback)

        # 捕获信号，以便优雅地终止录制
        signal.signal(signal.SIGINT, signal_handler)

        # 保持节点运行
        rospy.spin()
    
    except rospy.ROSInterruptException:
        rospy.loginfo("ROS node interrupted")
    
    finally:
        # 确保释放视频写入对象
        if not stop_recording:
            video_writer.release()
        rospy.loginfo("Video recording stopped and file saved.")

if __name__ == '__main__':
    main()

