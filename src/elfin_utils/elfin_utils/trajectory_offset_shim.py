#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from rclpy.action import ActionServer, ActionClient, GoalResponse, CancelResponse

class TrajectoryOffsetShim(Node):
    def __init__(self):
        super().__init__('trajectory_offset_shim')

        # mismo orden y offsets que usaste en joint_state_aligner
        self.joint_names = ['elfin_joint1','elfin_joint2','elfin_joint3','elfin_joint4','elfin_joint5','elfin_joint6']
        self.offsets = [2.363, 2.013, 1.535, 0.996, 2.343, 1.075]  # rad

        # servidor de acción “alineado” (lo que verá MoveIt)
        self.server = ActionServer(
            self,
            FollowJointTrajectory,
            'elfin_arm_controller_aligned/follow_joint_trajectory',
            execute_callback=self.execute_cb,
            goal_callback=self.goal_cb,
            cancel_callback=self.cancel_cb
        )

        # cliente hacia el controlador real
        self.client = ActionClient(
            self,
            FollowJointTrajectory,
            'elfin_arm_controller/follow_joint_trajectory'
        )

        self.get_logger().info('trajectory_offset_shim listo (escucha goals alineados y reenvía restando offsets)')

    def goal_cb(self, goal_request):
        return GoalResponse.ACCEPT

    def cancel_cb(self, goal_handle):
        return CancelResponse.ACCEPT

    def _aligned_to_raw(self, goal):
        """Devuelve una copia del goal restando offsets a todas las posiciones."""
        g = FollowJointTrajectory.Goal()
        g.trajectory = goal.trajectory
        # map nombre->offset
        off_map = dict(zip(self.joint_names, self.offsets))

        # Índices de cada joint en este goal
        idx = {name: i for i, name in enumerate(g.trajectory.joint_names)}

        # restar offset en cada punto
        new_points = []
        for p in g.trajectory.points:
            q = list(p.positions) if p.positions else []
            if q:
                for name, off in off_map.items():
                    if name in idx:
                        i = idx[name]
                        q[i] = q[i] - off
            np = JointTrajectoryPoint()
            np.positions = q
            np.velocities = p.velocities
            np.accelerations = p.accelerations
            np.effort = p.effort
            np.time_from_start = p.time_from_start
            new_points.append(np)
        g.trajectory.points = new_points
        return g

    async def execute_cb(self, goal_handle):
        goal = goal_handle.request
        # Convertir a crudo
        raw_goal = self._aligned_to_raw(goal)

        # Esperar al action server real
        if not self.client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error('No está /elfin_arm_controller/follow_joint_trajectory')
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.INVALID_GOAL)

        # Enviar y esperar resultado
        send_future = self.client.send_goal_async(raw_goal)
        real_goal_handle = await send_future
        if not real_goal_handle.accepted:
            self.get_logger().error('Controlador rechazó el goal')
            goal_handle.abort()
            return FollowJointTrajectory.Result(error_code=FollowJointTrajectory.Result.INVALID_GOAL)

        result_future = real_goal_handle.get_result_async()
        result = await result_future

        # Propagar estado final
        if result.result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            goal_handle.succeed()
        else:
            goal_handle.abort()

        return result.result

def main():
    rclpy.init()
    n = TrajectoryOffsetShim()
    rclpy.spin(n)
    n.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
