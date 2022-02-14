import time

import numpy as np
import pybullet
import pybullet_data
from transformation import Transformation

from pyboolet import PhysicsClient
from pyboolet.multibody import Sphere, Box, URDFBody
from pyboolet.simulation_object import JointControlMode

pc = PhysicsClient()
pc.connect_gui()

pc.time_step = 0.005
pc.gravity = np.array([0, 0, -9.81])

# Use this PhysicsClient instance as default for the following objects
with pc.as_default():
    # The objects are added to the scene as soon as they are created
    raindrops = [
        Sphere(radius=0.03, rgba_color=(0.5, 0.5, 1.0, 0.5), mass=0.05,
               pose=Transformation(np.random.normal([0, 0, 2.0], [0.3, 0.3, 0.1])))
        for i in range(100)]

    # Setting no mass causes the box to be fixed in place
    boxes = [
        Box([0.8 - 0.1 * i, 0.8 - 0.1 * i, 0.1], rgba_color=(0.7, 0.7, 0.7, 1.0),
            pose=Transformation([0.0, 0.0, 0.1 * (i + 0.5)]))
        for i in range(5)
    ]

    pc.set_additional_search_path(pybullet_data.getDataPath())
    ground_plane = URDFBody("plane.urdf", use_fixed_base=True)

    panda = URDFBody("franka_panda/panda.urdf", use_fixed_base=True, base_pose=Transformation([0.0, 0.0, 0.5]))

# pybullet functionality that is not implemented here can still be accessed directly
pc.call(
    pybullet.resetDebugVisualizerCamera, cameraDistance=1.5, cameraYaw=90.0, cameraPitch=-30.0,
    cameraTargetPosition=panda.pose.translation)

# Set all panda joints to
for j in panda.revolute_joints:
    j.control_mode = JointControlMode.VELOCITY_CONTROL

for i in range(10000):
    pc.step_simulation()

    # Move all raindrops that touch the ground above the panda again
    for b in raindrops:
        if len(b.get_contact_points(ground_plane)):
            b.reset_pose(Transformation(np.random.normal([0.0, 0.0, 2.0], [0.3, 0.3, 0.1])))

    # Let the panda dance :)
    for j in panda.revolute_joints:
        j.target_velocity = np.clip(j.joint_velocity + (np.random.random() - 0.5) * 0.1,
                                    -j.joint_info.joint_max_velocity, j.joint_info.joint_max_velocity)

    time.sleep(0.005)
