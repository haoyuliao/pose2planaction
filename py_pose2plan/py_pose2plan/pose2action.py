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
        
class MontionAction(Node):

    def __init__(self):
        super().__init__('pose2action_client')
        self._action_client = ActionClient(self, MoveGroup, '/move_action')
        self.goal_msg = MoveGroup.Goal()
        self.con = Constraints()

    def send_goal(self, gn, jointName=[],position=[], tolerance_above=0.01, tolerance_below=0.01, weight=1.0):
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
        self.goal_msg.request.group_name = gn
        self.goal_msg.request.goal_constraints = [self.con]

        self._action_client.wait_for_server()

        self._send_goal_future = self._action_client.send_goal_async(self.goal_msg, feedback_callback=self.feedback_callback)

        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: %s' %(result))
        rclpy.shutdown()


    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: %s' %(feedback))

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
   
    action_client = MontionAction()
    action_client.send_goal(str(sys.argv[1]), jointName, position)
    rclpy.spin(action_client)
    
    action_client.destroy_node()
    CIK_client.destroy_node()
    
if __name__ == '__main__':
    main()
    
'''
haoyuliao@haoyuliao-MacBookPro:~/ros2_ws_test$ ros2 run py_pose2plan pose2action 'arm' -0.261257 -0.202463 0.324145
[INFO] [1688853632.766180662] [pose2action_client]: Received feedback: moveit_msgs.action.MoveGroup_Feedback(state='PLANNING')
[INFO] [1688853632.768845896] [pose2action_client]: Goal accepted :)
[INFO] [1688853635.813303519] [pose2action_client]: Received feedback: moveit_msgs.action.MoveGroup_Feedback(state='IDLE')
[INFO] [1688853635.814459196] [pose2action_client]: Result: moveit_msgs.action.MoveGroup_Result(error_code=moveit_msgs.msg.MoveItErrorCodes(val=1), trajectory_start=moveit_msgs.msg.RobotState(joint_state=sensor_msgs.msg.JointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), name=['Joint_arm2', 'Joint_arm3', 'Joint_arm4', 'Joint_arm5', 'Joint_arm6', 'Joint_wheel1', 'Joint_wheel2', 'Joint_wheel3', 'Joint_wheel4'], position=[7.409391396213323e-05, 7.867038331460208e-05, 9.509906475432217e-06, 8.252647481858731e-05, 8.414629353210329e-05, 0.0, 0.0, 0.0, 0.0], velocity=[0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0], effort=[]), multi_dof_joint_state=sensor_msgs.msg.MultiDOFJointState(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), joint_names=[], transforms=[], twist=[], wrench=[]), attached_collision_objects=[], is_diff=False), planned_trajectory=moveit_msgs.msg.RobotTrajectory(joint_trajectory=trajectory_msgs.msg.JointTrajectory(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id='base_link'), joint_names=['Joint_arm2', 'Joint_arm3', 'Joint_arm4', 'Joint_arm5', 'Joint_arm6'], points=[trajectory_msgs.msg.JointTrajectoryPoint(positions=[7.409391396213323e-05, 7.867038331460208e-05, 9.509906475432217e-06, 8.252647481858731e-05, 8.414629353210329e-05], velocities=[0.0, 0.0, 0.0, 0.0, 0.0], accelerations=[0.9786341034859779, 0.0, 0.0, 0.0, 0.0], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=0)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.17085867106467964, 0.011180827821166932, 9.772075346501712e-05, 0.006898261870836922, 0.0006930270902558483], velocities=[0.4940029585669381, 0.03211354744248899, 0.00025515430091848523, 0.01971485661419581, 0.00176121825526832], accelerations=[0.981500233534246, 0.06380418126614337, 0.0005069483929109434, 0.03917008195064361, 0.003499242461756232], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=590784577)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.34164324821539716, 0.022282985259019263, 0.00018593160045460205, 0.013713997266855255, 0.0013019078869795934], velocities=[0.8021059521601208, 0.052142334578966984, 0.00041429060279249256, 0.03201074722731048, 0.0028596663665777654], accelerations=[0.9531607647514724, 0.06196192332118988, 0.0004923109555833475, 0.03803909972900748, 0.0033982066503318972], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=0, nanosec=835137818)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.5124278253661146, 0.033385142696871595, 0.0002741424474441869, 0.020529732662873588, 0.0019107886837033382], velocities=[0.9526434773464916, 0.06192829612658865, 0.0004920437747822969, 0.03801845562791319, 0.0033963624184197815], accelerations=[0.5270077669955421, 0.03425908415015731, 0.0002722014029155642, 0.0210320249721273, 0.0018788869252793694], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=23790241)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.6832124025168322, 0.04448730013472393, 0.00036235329443377184, 0.027345468058891924, 0.0025196694804270835], velocities=[1.0, 0.06500679173187088, 0.0005165035886802517, 0.03990837761657741, 0.0035651977882429466], accelerations=[0.0, 0.0, 0.0, 4.06295112806565e-17, -1.0157377820164125e-17], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=194574818)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[0.8539969796675497, 0.05558945757257625, 0.0004505641414233567, 0.034161203454910256, 0.0031285502771508282], velocities=[1.0, 0.06500679173187093, 0.0005165035886802521, 0.03990837761657744, 0.0035651977882429483], accelerations=[0.0, 7.313312030518176e-16, 5.713525023842325e-18, 2.8440657896459575e-16, 2.793278900545137e-17], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=365359395)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.024781556818267, 0.06669161501042858, 0.0005387749884129417, 0.04097693885092859, 0.003737431073874573], velocities=[1.0, 0.0650067917318709, 0.0005165035886802519, 0.039908377616577416, 0.0035651977882429483], accelerations=[0.0, -1.2188853384196953e-15, -8.252869478883354e-18, -5.688131579291912e-16, -2.539344455041032e-17], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=536143972)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.1955661339689847, 0.07779377244828091, 0.0006269858354025266, 0.04779267424694692, 0.0043463118705983185], velocities=[1.0, 0.06500679173187088, 0.0005165035886802517, 0.03990837761657741, 0.0035651977882429475], accelerations=[0.0, 9.751082707357557e-16, 4.443852796321803e-18, 5.281836466485343e-16, 1.2696722275205152e-17], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=706928550)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.3663507111197022, 0.08889592988613325, 0.0007151966823921115, 0.05460840964296526, 0.004955192667322064], velocities=[0.9526434773464916, 0.061928296126588706, 0.0004920437747822972, 0.038018455627913214, 0.0033963624184197828], accelerations=[-0.5270077669955422, -0.0342590841501576, -0.00027220140291556367, -0.021032024972127573, -0.0018788869252793629], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=1, nanosec=877713127)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.5371352882704197, 0.09999808732398559, 0.0008034075293816965, 0.061424145038983594, 0.005564073464045809], velocities=[0.8021059521601208, 0.05214233457896696, 0.0004142906027924926, 0.03201074722731048, 0.002859666366577765], accelerations=[-0.9531607647514724, -0.06196192332119036, -0.0004923109555833512, -0.03803909972900748, -0.0033982066503319233], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=2, nanosec=66365549)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.7079198654211372, 0.1111002447618379, 0.0008916183763712813, 0.06823988043500193, 0.006172954260769553], velocities=[0.49400295856693804, 0.03211354744248896, 0.00025515430091848507, 0.01971485661419581, 0.0017612182552683184], accelerations=[-0.9815002335342462, -0.06380418126614316, -0.0005069483929109423, -0.03917008195064361, -0.0034992424617562227], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=2, nanosec=310718791)), trajectory_msgs.msg.JointTrajectoryPoint(positions=[1.8787044425718546, 0.12220240219969024, 0.0009798292233608662, 0.07505561583102026, 0.006781835057493298], velocities=[0.0, 0.0, 0.0, 0.0, 0.0], accelerations=[-0.9786341034859777, -0.0636178633470192, -0.0005054680264553886, -0.0390556993503791, -0.003489024141247328], effort=[], time_from_start=builtin_interfaces.msg.Duration(sec=2, nanosec=901503368))]), multi_dof_joint_trajectory=trajectory_msgs.msg.MultiDOFJointTrajectory(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), joint_names=[], points=[])), executed_trajectory=moveit_msgs.msg.RobotTrajectory(joint_trajectory=trajectory_msgs.msg.JointTrajectory(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), joint_names=[], points=[]), multi_dof_joint_trajectory=trajectory_msgs.msg.MultiDOFJointTrajectory(header=std_msgs.msg.Header(stamp=builtin_interfaces.msg.Time(sec=0, nanosec=0), frame_id=''), joint_names=[], points=[])), planning_time=0.0)
'''
