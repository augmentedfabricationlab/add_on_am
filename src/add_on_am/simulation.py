from compas.geometry import Frame, Transformation
from compas_fab.robots import Configuration
from ur_fabrication_control.kinematics.ur_kinematics import inverse_kinematics

ur_params = {
    "ur3": [0.1519, -0.24365, -0.21325, 0.11235, 0.08535, 0.0819],
    "ur5": [0.089159, -0.425, -0.39225, 0.10915, 0.09465, 0.0823],
    "ur10": [0.1273, -0.612, -0.5723, 0.163941, 0.1157,0.0922],
    "ur10e": [0.1807, -0.6127, -0.57155, 0.17415, 0.11985, 0.11655]
}

class Simulation:
    def __init__(self, path, robot, lift, start_config, group="ur10e_and_liftkit"):
        self.path = path
        self.robot = robot
        self.robot.lift_height = lift
        self.start_config = start_config
        self.group = group

    
    def check_path(self):
        not_reachable = []
        configurations = []
        T = Transformation.from_change_of_basis(self.robot.BCF, Frame.worldXY())
        prev_config = self.start_config
        for plane in self.path:
            frame_WCS = Frame(plane.Origin, plane.XAxis, plane.YAxis)
            self.set_lift_height(plane.Origin[2])
            frame = frame_WCS.transformed(T)
            joint_configs = self.analytics_inverse(frame)
            if joint_configs:
                # get sum of differences between configs and start config
                delta_configs = [sum([abs(joint_config[i] - prev_config.joint_values[i]) for i in range(len(joint_config))]) for joint_config in joint_configs]
                idx = delta_configs.index(min(delta_configs))
                configuration = Configuration.from_prismatic_and_revolute_values([self.robot.lift_height], joint_configs[idx])
            else:
                not_reachable.append(plane)
                configuration = prev_config
                # configuration, fail = self.compute_ik(frame, prev_config)
                # if fail:
                #     not_reachable.append(plane)
            
            configurations.append(configuration)
            # print(configuration.joint_values)
            prev_config = configuration
        return configurations, not_reachable
            
    def set_lift_height(self, height):
        if 1.049 < height < 1.049 + 0.72:
            self.robot.lift_height = height - 1.049
        elif height >= 1.049 + 0.72:
            self.robot.lift_height = 0.72
        else:
            self.robot.lift_height = 0


    def analytics_inverse(self, frame_WCS):
        # transform frame to robot coordinate system
        frame_BCS = frame_WCS.transformed(self.robot.transformation_WCF_BCF())
        frame_RCS = frame_BCS.transformed(self.robot.transformation_BCF_RCF())
        
        if self.robot.attached_tool:
            tool0_RCS = self.robot.from_tcf_to_t0cf([frame_RCS])[0]
        else:
            tool0_RCS = frame_RCS
        # plane_tool0_RCS = draw_frame(tool0_RCS)
        # plane_RCS = draw_frame(frame_RCS)
        # calculate solutions to frame

        ur = ur_params["ur10e"] # get the parameters according to the robot model

        
        solutions = inverse_kinematics(tool0_RCS, ur)

        return solutions

        # if not len(solutions):
        #     joint_values = [0, 0, 0, 0, 0, 0]
        #     configuration = Configuration.from_prismatic_and_revolute_values([0], joint_values)
        # else:
        #     joint_values = solutions[idx]
        #     configuration = [Configuration.from_prismatic_and_revolute_values([lift], joint_values)]

    def compute_ik(self, frame, prev_config=None):
        configuration = None
        fail = False
        if prev_config == None:
            prev_config = self.start_config
        if self.robot and self.robot.client and self.robot.client.is_connected and frame:
            try:
                configuration = self.robot.inverse_kinematics(frame, prev_config, self.group)
            except Exception as e:
                print(e)
                configuration = prev_config
                fail = True
        return configuration, fail
