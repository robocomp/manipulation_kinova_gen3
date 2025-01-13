# Create a Python class named GS_SIM
# Create a method named __init__ that takes self as an argument


import numpy as np
from genesis import gs
from genesis.utils.geom import quat_to_R
from numpy import ndarray
from scipy.spatial.transform import Rotation as R

class GS_SIM:
    def __init__(self):

        gs.init(backend=gs.gpu, logging_level="warning")

        ########################## create a scene ##########################
        self.scene = gs.Scene(
            viewer_options=gs.options.ViewerOptions(
                camera_pos=(3, -1, 1),
                camera_lookat=(0.0, 0.0, 0.5),
                camera_fov=30,
                max_FPS=60,
            ),
            sim_options=gs.options.SimOptions(
                dt=0.01,
                substeps=4,  # for more stable grasping contact
            ),
            vis_options=gs.options.VisOptions(
                segmentation_level="entity"
            ),
            show_viewer=False,
        )

        ########################## entities ##########################
        self.plane = self.scene.add_entity(
            gs.morphs.Plane(),
        )
        self.cube_1 = self.scene.add_entity(
            gs.morphs.Box(
                size=(0.04, 0.04, 0.04),
                pos=(0.5, -0.3, 0.02),
                quat = self.rotate_quaternion_wrt_z([1, 0, 0, 0], np.pi/2)
            )
        )
        self.cube_2 = self.scene.add_entity(
            gs.morphs.Box(
                size=(0.04, 0.04, 0.04),
                pos=(0.5, 0.3, 0.02),
                quat = self.rotate_quaternion_wrt_z([1, 0, 0, 0], np.pi/2)
            ), surface=gs.surfaces.Plastic(diffuse_texture=gs.textures.ColorTexture(color=(1.0, 0.5, 0.5)))
        )
        self.cube_3 = self.scene.add_entity(
            gs.morphs.Box(
                size=(0.04, 0.04, 0.04),
                pos=(0.3, 0.0, 0.02),
                quat=self.rotate_quaternion_wrt_z([1, 0, 0, 0], np.pi / 2)
            ), surface=gs.surfaces.Plastic(diffuse_texture=gs.textures.ColorTexture(color=(0.5, 1.0, 1.0)))
        )
        self.basket = self.scene.add_entity(
            gs.morphs.Mesh(
                file="basket.stl",
                scale=(0.003, 0.003, 0.001),
                pos=(0.25, 0.5, 0.0),
                fixed=True,)
        )
        self.gen3 = self.scene.add_entity(
            gs.morphs.URDF(
                file="/home/pbustos/robocomp/components/manipulation_kinova_gen3/pybullet_controller/URDFs/kinova_with_pybullet/gen3_robotiq_2f_85-mod.urdf",
                fixed=True, )
        )
        self.camera = self.scene.add_camera(
            res    = (720, 480),
            pos    = (3, -1, 1),
            lookat = (0, 0, 0.5),
            fov    = 100,
            GUI    = False
        )

        ########################## build ##########################
        self.scene.build()

        link_names = [
            'world'
            'shoulder_link',
            'half_arm_1_link',
            'half_arm_2_link',
            'forearm_link',
            'spherical_wrist_1_link',
            'spherical_wrist_2_link',
            'bracelet_link',
            'left_outer_knuckle',
            'left_inner_knuckle',
            'right_outer_knuckle',
            'right_inner_knuckle',
            'left_inner_finger',
            'right_inner_finger'
        ]

        jnt_names = [
            'joint_1',
            'joint_2',
            'joint_3',
            'joint_4',
            'joint_5',
            'joint_6',
            'joint_7',
            'finger_joint',
            'left_inner_knuckle_joint',
            'right_outer_knuckle_joint',
            'right_inner_knuckle_joint',
            'left_inner_finger_joint',
            'right_inner_finger_joint'
        ]
        self.dofs_idx = [self.gen3.get_joint(name).dof_idx_local for name in jnt_names]
        self.motors_dof = np.arange(13)
        self.fingers_dof = np.arange(7, 13)

        # set control gains
        self.gen3.set_dofs_kp( kp = np.array([4500, 4500, 3500, 3500, 2000, 2000, 2000, 100, 100, 100, 100, 100, 100])/2,
                          dofs_idx_local = self.dofs_idx)
        self.gen3.set_dofs_kv(kv = np.array([450, 450, 350, 350, 200, 200, 200, 10, 10, 10, 10, 10, 10]),
                         dofs_idx_local = self.dofs_idx)
        self.gen3.set_dofs_force_range(
                    lower = np.array([-87, -87, -87, -87, -12, -12, -12, -100, -100, -100, -100, -100, -100]),
                    upper = np.array([87, 87, 87, 87, 12, 12, 12, 100, 100, 100, 100, 100, 100]),
                    dofs_idx_local = self.dofs_idx)

        # set end effector
        self.tip_name = "bracelet_link"

        # move to default pose
        self.move_arm_to_default_pose()

    def move_arm_to_default_pose(self):
        self.gen3.set_dofs_position(np.array([0, -0.43, -0.02, 1.96, 0.0, 1.26, -1.66, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]),
                                    self.dofs_idx)
        self.camera.set_pose(transform=self.move_camera(self.gen3))
        self.scene.step()

    def rotate_quaternion_wrt_z(self, quat:np.array, angle: float) -> np.array:
        """
        Rotate a unitary quaternion with respect to the z-axis by a given angle.

        Parameters:
        quat (np.array): The original quaternion (w, x, y, z).
        angle (float): The rotation angle in radians.

        Returns:
        np.array: The rotated quaternion.
        """
        # Create the rotation quaternion for the z-axis
        half_angle = angle / 2.0
        z_rotation = np.array([np.cos(half_angle), 0, 0, np.sin(half_angle)])

        # Perform quaternion multiplication
        w1, x1, y1, z1 = quat
        w2, x2, y2, z2 = z_rotation

        rotated_quat = np.array([
            w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
            w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
            w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
            w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2
        ])

        return rotated_quat

    def move_camera(self, robot = None):
        bracelet_pose = np.eye(4)
        rotation = R.from_euler('z', 180, degrees=True).as_matrix()  # 45 degree rotation around z-axis
        bracelet_pose[:3, :3] = quat_to_R(robot.get_link("bracelet_link").get_quat().cpu().numpy()) @ rotation
        bracelet_pose[:3, 3] = robot.get_link("bracelet_link").get_pos().cpu().numpy() + [0.06, 0.0, -0.05]
        return bracelet_pose

    def draw_debug_frame(self, pos: ndarray, quat: ndarray):
        T = np.eye(4)
        T[:3, :3] = quat_to_R(quat)
        T[:3, 3] = pos
        self.scene.draw_debug_frame(T, axis_length=0.1, axis_radius=0.002)

    def step(self):
        # update camera in hand
        self.camera.set_pose(transform=self.move_camera(self.gen3))
        # update sim
        self.scene.step()

    # # set cube1 random pose
    # new_pos = [np.random.rand() * 0.2 + 0.5, np.random.rand() * 0.6 - 0.3, 0.04]
    # cube.set_pos(new_pos)
    # cube.set_quat(rotate_quaternion_wrt_z([1, 0, 0, 0], np.pi/2))
    #
    # # set cube2 random pose
    # new_pos = [np.random.rand() * 0.2 + 0.5, np.random.rand() * 0.6 - 0.3, 0.04]
    # cube_2.set_pos(new_pos)
    # cube_2.set_quat(rotate_quaternion_wrt_z([1, 0, 0, 0], np.pi / 2))