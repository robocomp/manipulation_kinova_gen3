from pybullet_utils import bullet_client as bc
from pybullet_utils import urdfEditor as ed
import pybullet
import pybullet_data
import time

p0 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p0.setAdditionalSearchPath(pybullet_data.getDataPath())

p1 = bc.BulletClient(connection_mode=pybullet.DIRECT)
p1.setAdditionalSearchPath(pybullet_data.getDataPath())

#can also connect using different modes, GUI, SHARED_MEMORY, TCP, UDP, SHARED_MEMORY_SERVER, GUI_SERVER

robot_urdf = "/home/robolab/robocomp_ws/src/robocomp/components/manipulation_kinova_gen3/pybullet_controller/kinova/kinova_with_pybullet/gen3_robotiq_2f_85.urdf"
robot_launch_pos = [-0.3, 0.0, 0.64]
robot_launch_orien = pybullet.getQuaternionFromEuler([0, 0, 0])
home_angles = [0, -0.34, 3.14, -2.54, -6.28, -0.87, 1.57, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                                0.0,
                                0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
robot_id = p1.loadURDF(robot_urdf, robot_launch_pos, robot_launch_orien,
                                       flags=pybullet.URDF_USE_SELF_COLLISION_EXCLUDE_PARENT)

flags = pybullet.URDF_USE_INERTIA_FROM_FILE

table_id = (p0
            .loadURDF("/home/robolab/software/bullet3/data/table/table.urdf", basePosition=[0, 0, 0],
                           baseOrientation=pybullet.getQuaternionFromEuler([0, 0, 1.57]), flags=flags))

husky = p1.loadURDF("/home/robolab/software/bullet3/data/table/table.urdf", flags=p0.URDF_USE_IMPLICIT_CYLINDER)
kuka = p0.loadURDF("/home/robolab/robocomp_ws/src/robocomp/components/manipulation_kinova_gen3/pybullet_controller/kinova/kinova_with_pybullet/gen3_robotiq_2f_85.urdf")

ed0 = ed.UrdfEditor()
ed0.initializeFromBulletBody(husky, p1._client)
ed1 = ed.UrdfEditor()
ed1.initializeFromBulletBody(kuka, p0._client)
#ed1.saveUrdf("combined.urdf")

parentLinkIndex = 0

jointPivotXYZInParent = [0, 0, 0]
jointPivotRPYInParent = [0, 0, 0]

jointPivotXYZInChild = [0, 0, 0]
jointPivotRPYInChild = [0, 0, 0]

newjoint = ed0.joinUrdf(ed1, parentLinkIndex, jointPivotXYZInParent, jointPivotRPYInParent,
                        jointPivotXYZInChild, jointPivotRPYInChild, p0._client, p1._client)
newjoint.joint_type = p0.JOINT_FIXED

ed0.saveUrdf("combined.urdf")

print(p0._client)
print(p1._client)
print("p0.getNumBodies()=", p0.getNumBodies())
print("p1.getNumBodies()=", p1.getNumBodies())

pgui = bc.BulletClient(connection_mode=pybullet.GUI)
pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 0)

orn = [0, 0, 0, 1]
ed0.createMultiBody([0, 0, 0], orn, pgui._client)
pgui.setRealTimeSimulation(1)

pgui.configureDebugVisualizer(pgui.COV_ENABLE_RENDERING, 1)

while (pgui.isConnected()):
  pgui.getCameraImage(320, 200, renderer=pgui.ER_BULLET_HARDWARE_OPENGL)
  time.sleep(1. / 240.)