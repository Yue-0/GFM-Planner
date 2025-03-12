import cv2
import rospy
import trimesh
import numpy as np

from nav_msgs.msg import OccupancyGrid

__author__ = "YueLin"

# Initialize
rospy.init_node("architect")

# Hyper-parameters
STL = rospy.get_param("~output")
H = np.array([0, 0, rospy.get_param("~height", 1.0)])
FACES = [
    [0, 2, 4], [4, 2, 6], [1, 2, 0], [3, 2, 1], [5, 0, 4], [1, 0, 5],
    [3, 7, 2], [7, 6, 2], [7, 4, 6], [5, 4, 7], [1, 5, 3], [7, 3, 5]
]


def callback(msg: OccupancyGrid):
    rospy.loginfo("[architect] Received map, converting to STL file...")
    
    # 1. Convert ROS message to np.ndarray
    image = np.array(msg.data).reshape((
        msg.info.height, msg.info.width
    ))
    image[image < 0] = 0

    # 2. Find contours
    contours = cv2.findContours(cv2.threshold(
        np.uint8(image), 1, 0xFF, cv2.THRESH_BINARY
    )[1], cv2.RETR_CCOMP, cv2.CHAIN_APPROX_NONE)[0]

    # 3. Make mash
    meshes = []
    for contour in contours:
        m = []
        for point in contour:
            x, y = point[0]
            vertices = [np.array([
                (x + i // 2) * msg.info.resolution + msg.info.origin.position.x, 
                (y + i % 2) * msg.info.resolution + msg.info.origin.position.y,
                0o0
            ]) for i in range(4)]
            mesh = trimesh.Trimesh(vertices + [v + H for v in vertices], FACES)
            if not mesh.is_volume:
                mesh.fix_normals()
            m.append(mesh)
        mesh = trimesh.util.concatenate(m)
        mesh.update_faces(mesh.unique_faces())
        meshes.append(mesh)

    # 4. Export as STL file
    trimesh.util.concatenate(meshes).export(STL)
    rospy.loginfo("[architect] STL file exported successfully.")
    rospy.signal_shutdown("Export success.")


# Subscriber
rospy.Subscriber(rospy.get_param("~topic"), OccupancyGrid, callback)

# Spin
rospy.spin()
