import math
from threading import Thread

import cv2
import rospy
import numpy as np
from numba import jit
from tqdm import tqdm

from nav_msgs.msg import OccupancyGrid

__author__ = "YueLin"


@jit(nopython=True)
def scan(image: np.ndarray, h: int, w: int) -> list:
    """Simulate laser scan"""
    laser = []
    for z in range(64):
        theta, r = z * math.pi * 0.03125, 0
        sin, cos = math.sin(theta), math.cos(theta)
        while True:
            r += 1
            x = max(min(int(round(w + r * cos)), image.shape[1] - 1), 0)
            y = max(min(int(round(h + r * sin)), image.shape[0] - 1), 0)
            if image[y, x]:
                laser.append([x, y])
                break
    return laser


class Metric:
    def __init__(self, topic: str, output: str, threads: int):
        self.laser = []
        self.output = output
        self.threads = threads
        self.q = np.empty((0, 0, 4), np.uint16)
        rospy.Subscriber(topic, OccupancyGrid, self.callback)
    
    def run(self, threads: list, end: bool = False) -> None:
        """Run multi-threads"""
        if len(threads) >= self.threads or end:
            for thread in threads:
                thread.start()
            for thread in threads:
                thread.join()
            threads.clear()
    
    def rank(self, image: np.ndarray, h: int, w: int) -> None:
        """Calculate the rank code of the Jacobian matrix"""
        if np.max(image[max(h - 1, 0):min(h + 2, image.shape[0]), 
                        max(w - 1, 0):min(w + 2, image.shape[1])]):
            self.q[h, w, :] = 0xFFFF
        else:
            for angle in range(64):
                if np.linalg.matrix_rank(np.asarray([
                    + self.laser[h, w + 1, angle, :]
                    - self.laser[h, w - 1, angle, :],
                    + self.laser[h + 1, w, angle, :]
                    - self.laser[h - 1, w, angle, :],
                    + self.laser[h, w, (angle + 1) & 0x3F, :]
                    - self.laser[h, w, (angle - 1) & 0x3F, :]
                ])) == 1:
                    self.q[h, w, 3 - (angle >> 4)] += 1
                self.q[h, w, 3 - (angle >> 4)] <<= 1

    def scan(self, image: np.ndarray, h: int, w: int) -> None:
        """Simulate laser scan"""
        self.laser[h, w, :, :] = np.array([
            [w, h] for _ in range(64)
        ]) if image[h, w] else np.array(scan(image, h, w))
    
    def callback(self, msg: OccupancyGrid) -> None:
        rospy.loginfo("[metric] Received map, converting to metric map...")

        # 1. Convert ROS message to np.ndarray
        image = np.array(msg.data).reshape((
            msg.info.height, msg.info.width
        ))
        image[image < 0] = 0
        image = cv2.flip(np.uint8(image), 0)
        self.q = np.zeros((*image.shape, 4), np.uint16)
        cv2.rectangle(image, (0, 0), image.shape[::-1], 0x64, 2)

        # 2. Simulate laser scan
        threads = []
        rospy.loginfo("[metric] Simulate laser scan")
        self.laser = np.zeros((*image.shape, 64, 2), int)
        for pixel in tqdm(range(np.prod(image.shape)), "Simulate laser scan"):
            thread = Thread(
                target=self.scan, 
                args=(image, *divmod(pixel, msg.info.width))
            )
            threads.append(thread)
            self.run(threads)
        self.run(threads, True)

        # 3. Calculate the rank of Jacobian matrixs
        rospy.loginfo("[metric] Calculate the rank of Jacobian matrixs")
        for pixel in tqdm(range(np.prod(image.shape)), "Calculate rank"):
            h, w = divmod(pixel, msg.info.width)
            if min(h, w) == 0 or w == msg.info.width - 1:
                self.q[h, w, :] = 0xFFFF
                continue
            if h == msg.info.height - 1:
                self.q[h, w, :] = 0xFFFF
                break
            thread = Thread(target=self.rank, args=(image, h, w))
            threads.append(thread)
            self.run(threads)
        self.run(threads, True)
        
        # 4. Save result
        cv2.imwrite(self.output, self.q)
        rospy.loginfo("[metric] Export success. You can shutdown all nodes now")
        rospy.signal_shutdown("Export success.")


if __name__ == "__main__":
    rospy.init_node("metric")

    _ = Metric(
        rospy.get_param("~topic"),
        rospy.get_param("~output"),
        rospy.get_param("~num_threads")
    )

    rospy.spin()
