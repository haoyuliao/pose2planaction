#!/usr/bin/env python
#Author: Hao-yu Liao
#E-mail: haoyuliao929@gmail.com
import sys, rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from moveit_msgs.srv import GetPositionIK
from geometry_msgs.msg import PoseStamped
from moveit_msgs.srv import GetMotionPlan
from sensor_msgs.msg import JointState
from moveit_msgs.msg import JointConstraint
from moveit_msgs.msg import Constraints
from moveit_msgs.action import MoveGroup


class CIK(Node):

    def __init__(self):
        super().__init__('clik_client_async')
        self.clik = self.create_client(GetPositionIK, '/compute_ik')
        while not self.clik.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqp = PoseStamped()
        self.req = GetPositionIK.Request()

    def send_request(self, gn, x, y, z):
        self.reqp.pose.position.x = x
        self.reqp.pose.position.y = y
        self.reqp.pose.position.z = z
        self.req.ik_request.group_name = gn
        self.req.ik_request.pose_stamped = self.reqp
        
        self.future = self.clik.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        
        return self.future.result()

class MontionPlanning(Node):

    def __init__(self):
        super().__init__('clim_client_async')
        self.clim = self.create_client(GetMotionPlan, '/plan_kinematic_path')
        while not self.clim.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.reqm = GetMotionPlan.Request()
        self.con = Constraints()

    def send_request(self, gn, jointName=[],position=[], tolerance_above=0.01, tolerance_below=0.01, weight=1.0):
    	self.con.joint_constraints = [JointConstraint() for i in range(len(jointName))]
    	gn = str(gn)
    	jointName = jointName.split(',')
    	position = [float(j) for j in position.split(',')]
    	for i in range(len(jointName)):
	    	self.con.joint_constraints[i].joint_name = jointName[i]
	    	self.con.joint_constraints[i].position = position[i]
	    	self.con.joint_constraints[i].tolerance_above = tolerance_above
	    	self.con.joint_constraints[i].tolerance_below = tolerance_below
	    	self.con.joint_constraints[i].weight = weight
    	self.reqm.motion_plan_request.group_name = gn
    	self.reqm.motion_plan_request.goal_constraints = [self.con]
    	self.future = self.clim.call_async(self.reqm)
    	rclpy.spin_until_future_complete(self, self.future)
    	
    	return self.future.result()



def main():
    rclpy.init()

    CIK_client = CIK()
    response1 = CIK_client.send_request(str(sys.argv[1]), float(sys.argv[2]), float(sys.argv[3]), float(sys.argv[4]))
    jointName=''
    for i in response1.solution.joint_state.name:
    	jointName+=','+i
    jointName=jointName[1:]
    position=''
    for i in response1.solution.joint_state.position:
    	position+=','+str(i)
    position=position[1:]
    
    MontionPlanning_client = MontionPlanning()
    response2 = MontionPlanning_client.send_request(str(sys.argv[1]), jointName, position)
    

    MontionPlanning_client.get_logger().info(
        'Result of %s' %(response2))
        
    CIK_client.destroy_node()
    MontionPlanning_client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



'''
ros2 run py_pose2plan pose2plan 'arm' -0.261257 -0.202463 0.324145
[INFO] [1688819409.866982700] [clim_client_async]: Result of moveit_msgs.srv.GetMotionPlan_Response(motion_plan_response=moveit_msgs.msg.MotionPlanResponse(trajectory_start=moveit_msgs.msg.RobotState(joint_state=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), name=['Joint_arm2', 'Joint_arm3', 'Joint_arm4', 'Joint_arm5', 'Joint_arm6', 'Joint_wheel1', 'Joint_wheel2', 'Joint_wheel3', 'Joint_wheel4'], position=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], effort=[]), multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), joint_names=[], transforms=[], twist=[], wrench=[]), attached_collision_objects=[], is_diff=False), group_name='arm', trajectory=moveit_msgs.msg.RobotTrajectory(joint_trajectory=trajectory_msgs.msg.JointTrajectory(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), joint_names=['Joint_arm2', 'Joint_arm3', 'Joint_arm4', 'Joint_arm5', 'Joint_arm6'], points=[trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.0, 0.0, 0.0, 0.0, 0.0], velocities=[0.0, 0.0, 0.0, 0.0, 0.0], accelerations=[0.975600323065009, 0.0, 0.0, 0.0, 0.0], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=0)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.15660267880722656, 0.009247927000990357, 0.0008317026105942204, 0.00747537763858126, 0.00048701090078490464], velocities=[0.4773792742479787, 0.0281908886467103, 0.0025353179885534405, 0.022787543476368778, 0.001484578119214291], accelerations=[1.0081601819554802, 0.0595353274863597, 0.005354247204555636, 0.04812419645499855, 0.0031352273287355457], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=566602678)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.3132053576144531, 0.018495854001980713, 0.0016634052211884408, 0.01495075527716252, 0.0009740218015698093], velocities=[0.7785161347607357, 0.04597405637117505, 0.004134628517225253, 0.03716221299272285, 0.0024210687004411068], accelerations=[0.9792360117157464, 0.05782725571531061, 0.005200633562178091, 0.04674351065146393, 0.0030452774868158653], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=797454212)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.46980803642167973, 0.02774378100297107, 0.0024951078317826613, 0.02242613291574378, 0.001461032702354714], velocities=[0.9393312996362145, 0.05547074516825091, 0.004988703258914179, 0.044838672275611176, 0.002921179802641907], accelerations=[0.7247677288729524, 0.042800028073219504, 0.003849175612890377, 0.03459655042204938, 0.0022539191997650466], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=975682668)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.6264107152289062, 0.03699170800396143, 0.0033268104423768816, 0.02990151055432504, 0.0019480436031396185], velocities=[1.0, 0.05905344066543261, 0.005310909218979726, 0.04773467283904663, 0.0031098503837498287], accelerations=[0.0, -3.987801844184241e-16, -1.6615841017434338e-17, -4.43089093798249e-17, -2.7693068362390562e-18], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=132285347)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.7830133940361329, 0.04623963500495178, 0.004158513052971102, 0.037376888192906306, 0.0024350545039245235], velocities=[1.0, 0.059053440665432604, 0.005310909218979724, 0.047734672839046616, 0.003109850383749828], accelerations=[0.0, 2.6585345627894936e-16, 5.538613672478112e-18, -2.2154454689912447e-16, -2.769306836239056e-18], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=288888026)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.9396160728433595, 0.05548756200594214, 0.004990215663565323, 0.04485226583148756, 0.002922065404709428], velocities=[1.0, 0.059053440665432624, 0.005310909218979726, 0.0477346728390466, 0.003109850383749828], accelerations=[0.0, 0.0, 1.1077227344956223e-17, 8.861781875964979e-17, 2.769306836239056e-18], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=445490705)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.0962187516505861, 0.0647354890069325, 0.005821918274159544, 0.052327643470068824, 0.003409076305494333], velocities=[1.0, 0.059053440665432645, 0.005310909218979727, 0.04773467283904664, 0.0031098503837498287], accelerations=[0.0, 2.6585345627894956e-16, 0.0, 3.544712750385994e-16, 5.5386136724781155e-18], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=602093384)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.2528214304578125, 0.07398341600792285, 0.006653620884753763, 0.05980302110865008, 0.003896087206279237], velocities=[1.0, 0.059053440665432624, 0.005310909218979722, 0.047734672839046643, 0.0031098503837498287], accelerations=[0.0, -5.317069125578989e-16, -6.092475039725925e-17, -3.1016236565877435e-16, -2.769306836239057e-18], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=758696063)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.4094241092650392, 0.08323134300891322, 0.0074853234953479834, 0.06727839874723135, 0.0043830981070641425], velocities=[0.9393312996362144, 0.055470745168250866, 0.004988703258914176, 0.044838672275611176, 0.0029211798026419064], accelerations=[-0.7247677288729525, -0.04280002807321923, -0.0038491756128902865, -0.03459655042204911, -0.0022539191997650474], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=915298741)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.5660267880722658, 0.09247927000990357, 0.008317026105942205, 0.07475377638581261, 0.004870109007849047], velocities=[0.7785161347607357, 0.04597405637117502, 0.0041346285172252535, 0.037162212992722835, 0.0024210687004411063], accelerations=[-0.979236011715746, -0.05782725571531069, -0.005200633562178136, -0.04674351065146428, -0.0030452774868158657], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=2, nanosec=93527198)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.7226294668794924, 0.10172719701089392, 0.009148728716536424, 0.08222915402439386, 0.0053571199086339515], velocities=[0.4773792742479787, 0.028190888646710287, 0.0025353179885534392, 0.022787543476368757, 0.0014845781192142906], accelerations=[-1.0081601819554802, -0.059535327486359584, -0.005354247204555623, -0.04812419645499842, -0.0031352273287355457], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=2, nanosec=324378731)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.879232145686719, 0.11097512401188428, 0.009980431327130645, 0.08970453166297512, 0.005844130809418856], velocities=[0.0, 0.0, 0.0, 0.0, 0.0], accelerations=[-0.975600323065009, -0.057612555791296455, -0.005181324749805559, -0.046569962243176416, -0.003033971039070174], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=2, nanosec=890981410))]), multi_dof_joint_trajectory=trajectory_msgs.msg.MultiDOFJointTrajectory(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), joint_names=[], points=[])), planning_time=0.013227874, error_code=moveit_msgs.msg.MoveItErrorCodes(val=1)))

'''
