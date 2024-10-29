import pinocchio as pin
from pinocchio.robot_wrapper import RobotWrapper
import numpy as np
import matplotlib.pyplot as plt

class loadRobotModel():
    def __init__(self, urdf_path):
        # 載入機器人模型
        self.model = pin.buildModelFromUrdf(urdf_path)
        self.data = self.model.createData()
        self.pos = pin.neutral(self.model)

    def calculateCom(self, plot=False):

        # Compute forward kinematics
        pin.forwardKinematics(self.model, self.data, self.pos)
        # print('q', self.pos)
        # Update frame placements
        pin.updateFramePlacements(self.model, self.data)

        com = pin.centerOfMass(self.model, self.data)
        # plt.plot(com[0], com[2], marker = 'x')

        # J = pin.computeJointJacobians(self.model, self.data, self.pos)

        com_lenth_vector = com - self.data.oMi[-1].translation
        com_lenth = (com_lenth_vector[0]**2+com_lenth_vector[2]**2)**(1/2)

        # for oMi in self.data.oMi:
        #     plt.scatter(oMi.translation[0], oMi.translation[2]) 
        
        # if plot:
        #     plt.axis('equal')
        #     plt.show()
        
        return com, com_lenth

    # def calculateInertia(self):
    #     body_inertia = 0
    #     wheel_inertia = 0
    #     wheel_joint = self.data.oMi[-1].translation
    #     for name, inertia, oMi in zip(self.model.names, self.model.inertias, self.data.oMi):
    #         if name=='wheel_joint_right' or name=='wheel_joint_left':
    #             wheel_inertia = inertia.inertia[1,1]
    #             link_com = oMi.translation + inertia.lever
    #         else:
    #             link_com = oMi.translation + inertia.lever
    #             dis = wheel_joint - link_com
    #             dis_xz = (dis[0]**2+dis[2]**2)**(1/2)
    #             print(inertia.inertia[1,1])
    #             print(inertia.mass)
    #             body_inertia += inertia.inertia[1,1] + inertia.mass*dis_xz**2

    #         plt.scatter(oMi.translation[0], oMi.translation[2]) 
    #         plt.scatter(link_com[0], link_com[2], marker='*')
            
        
    #     plt.axis('equal')
    #     plt.show()
    #     print("I", body_inertia, wheel_inertia)
    #     return body_inertia, wheel_inertia
    
    def calculateMass(self):
        wheel_mass = 0
        body_mass = 0
        for name, inertia, oMi in zip(self.model.names, self.model.inertias, self.data.oMi):
            # if name=='wheel_joint_right' or name=='wheel_joint_left':
            #     wheel_mass += inertia.mass
            # else:
            body_mass += inertia.mass
        return body_mass
    
    def getOmi(self):
        for name, oMi in zip(self.model.names, self.data.oMi):
            if name=='wheel4' or name=='wheel04':
                return oMi.translation[0]
            elif name=='wheel_r' or name=='wheel_l':
                return oMi.translation[0]
    

if __name__=="__main__":
    # urdf_path = "base_link_description/urdf/base_link.urdf"
    urdf_path = "big_bipedal_robot_v1/urdf/big bipedal robot v1.urdf"
    robot = loadRobotModel(urdf_path=urdf_path)
    # robot.pos = np.array([0., 0., 0., 0., 0., 0., 1., -1.57, 2.6, 0., -1.57, 2.6, 0.])
    com, com_lenth = robot.calculateCom(plot=True)
    print('com', com, com_lenth)
    body_mass = robot.calculateMass()
    print('mass', body_mass)
