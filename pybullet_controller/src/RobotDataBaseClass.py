class RobotDataBase():
    __doc__ = """
    This class has the following functions:
    save_time()
        |- It save the time in the list to be able to have a record that when it happened
    """
    def __init__(self,name = "Database",time_step = (1.0/240.0)):
        """
        sets up a database with variables for storing position, orientation, and
        joint angles data, as well as time steps and counters for simulation tracking.

        Args:
            name (str): name of the database for which the instance is being
                created, which is stored as an attribute of the instance.
            time_step (float): duration of each time step in seconds, and it is
                used to determine the speed at which the joint angles are updated
                in the simulation.

        """
        self.name = name
        self.joints_angles_rad = []
        self.joint_angles_vel_rad = []
        self.joint_torques = []
        self.tcp_position = []
        self.tcp_orientation_q = []
        self.tcp_orientation_e = []
        self.time = []

        self.time_step = time_step
        self.counter = 0

    def save_time (self):
        """
        increments a counter and appends its product to a list, where the time
        step is a constant.

        """
        self.counter += 1
        self.time.append(self.counter * self.time_step)