# CORRECTED – Reads real Gazebo poses

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from moveit_msgs.msg import CollisionObject
from shape_msgs.msg import SolidPrimitive

class BoxSyncNode(Node):
    def __init__(self):
        super().__init__("box_sync_node")

        self.pub = self.create_publisher(CollisionObject,
                                         "/collision_object", 10)

        self.added = {"box1": False, "box2": False, "box3": False}

        # Subscribe to real bridged topics
        self.create_subscription(PoseStamped,
            "/model/box1/pose", lambda m: self.cb("box1", m), 10)
        self.create_subscription(PoseStamped,
            "/model/box2/pose", lambda m: self.cb("box2", m), 10)
        self.create_subscription(PoseStamped,
            "/model/box3/pose", lambda m: self.cb("box3", m), 10)

    def cb(self, name, msg):
        pose = msg.pose

        if not self.added[name]:
            self.add(name, pose)
            self.added[name] = True
        else:
            self.move(name, pose)

    def add(self, name, pose):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = name

        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.125, 0.125, 0.125]

        obj.primitives.append(box)
        obj.primitive_poses.append(pose)

        obj.operation = CollisionObject.ADD
        self.pub.publish(obj)

    def move(self, name, pose):
        obj = CollisionObject()
        obj.header.frame_id = "world"
        obj.id = name

        obj.primitive_poses.append(pose)
        obj.operation = CollisionObject.MOVE
        self.pub.publish(obj)

def main():
    rclpy.init()
    rclpy.spin(BoxSyncNode())
    rclpy.shutdown()

if __name__ == "__main__":
    main()
