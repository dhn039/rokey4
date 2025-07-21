import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import TaskResult
from turtlebot4_navigation.turtlebot4_navigator import TurtleBot4Directions
from .waypoint_manager import WaypointManager
from .navigator import Navigator
from .battery_monitor import BatteryMonitor
from .docking_manager import DockingManager
from .delivery_queue import DeliveryQueue
import time


class DeliveryManager(Node):
    def __init__(self):
        super().__init__('delivery_manager')

        self.waypoint_manager = WaypointManager()
        self.navigator = Navigator(self)
        self.battery_monitor = BatteryMonitor(namespace='/robot8/')
        self.docking_manager = DockingManager(self.navigator)
        self.delivery_queue = DeliveryQueue(self)

        self.max_retry = 3
        self.is_docked = True  # 최초엔 도킹 상태로 시작

        # self.idle_battery_timer = self.create_timer(10.0, self.idle_battery_check)
        self.get_logger().info("🚚 배송 매니저 시작됨")

        # ✅ 도킹 상태일 경우 언도킹 후 stay로 이동
        # self.init_timer = self.create_timer(1.0, self.initial_undock_and_stay)
        if self.is_docked:
            self.get_logger().info("🚪 초기 언도킹 실행")
            self.docking_manager.undock()
            self.is_docked = False
            # self.get_logger().info("📍 stay 포인트로 초기 이동")
            # self.go_to_waypoint_by_name("stay")

        # self.main_loop_timer = self.create_timer(2.0, self.main_loop)

    def initial_undock_and_stay(self):
        if self.is_docked:
            self.get_logger().info("🚪 초기 언도킹 실행")
            self.docking_manager.undock()
            self.is_docked = False
            self.get_logger().info("📍 stay 포인트로 초기 이동")
            self.go_to_waypoint_by_name("stay")

    def idle_battery_check(self):
        if self.delivery_queue.is_empty() and not self.is_docked:
            if self.battery_monitor.is_battery_low():
                self.get_logger().info("🔋 IDLE 상태에서 배터리 부족 → 도킹 및 충전")
                self.docking_manager.dock_until_charged()
                self.is_docked = True



    def main_loop(self):
        if self.delivery_queue.is_empty():
            self.get_logger().info("🕓 배송 없음 → stay 위치로 이동 및 대기")
            self.go_to_waypoint_by_name("stay")
            return

        task = self.delivery_queue.get_next_request()
        print(task)
        if len(task) != 2:
            self.get_logger().warn("🚫 유효하지 않은 배송 명령 구조 (hub, home 필수)")
            return

        hub, home = task
        route = [hub, "cross", home, "cross"]  # ✅ 주문 수신 후 hub부터 시작
        goal_poses = self.waypoint_manager.get_waypoints_by_names(route)

        if len(goal_poses) != len(route):
            self.get_logger().warn("🚫 일부 waypoint를 찾을 수 없어 작업 생략")
            return

        # if self.battery_monitor.is_battery_low():
        #     self.get_logger().info("🔋 작업 전 배터리 부족 → 도킹 및 충전")
        #     self.docking_manager.dock_until_charged()
        #     self.is_docked = True

        if self.is_docked:
            self.get_logger().info("🚪 언도킹 시작")
            self.docking_manager.undock()
            self.is_docked = False

        for i, pose in enumerate(goal_poses):
            name = route[i]
            self.get_logger().info(f"➡ 이동: {name}")
            if not self.try_navigation(pose):
                self.get_logger().warn(f"❌ {name} 도달 실패 → stay로 이동 및 충전")
                self.go_to_waypoint_by_name("stay")
                self.docking_manager.dock_until_charged()
                self.is_docked = True
                return

        self.get_logger().info("📦 배송 완료. 충전 상태 확인")
        # if self.battery_monitor.is_battery_low():
        #     self.docking_manager.dock_until_charged()
        #     self.is_docked = True

    def try_navigation(self, pose: PoseStamped) -> bool:
        for attempt in range(1, self.max_retry + 1):
            self.navigator.go_to_pose(pose)
            self.get_logger().info(f"🕒 목표 도달 대기 중... (시도 {attempt})")
            while not self.navigator.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)

            result = self.navigator.navigator.getResult()
            if result == TaskResult.SUCCEEDED:
                self.get_logger().info("✅ 도달 성공")
                return True
            else:
                self.get_logger().warn("⏳ 도달 실패, 재시도")
                return False

    def go_to_waypoint_by_name(self, name: str):
        waypoints = self.waypoint_manager.get_waypoints_by_names([name])
        if not waypoints:
            self.get_logger().warn(f"❌ {name} waypoint 없음")
            return
        pose = self.navigator.navigator.getPoseStamped(
            [waypoints[0][1], waypoints[0][2]],
            TurtleBot4Directions[waypoints[0][3].upper()]
        )
        self.try_navigation(pose)


def main(args=None):
    rclpy.init(args=args)
    node = DeliveryManager()

    try:
        while rclpy.ok():
            node.main_loop()
            rclpy.spin_once(node, timeout_sec=0.5)
            time.sleep(1.0)
    except Exception as e:
        node.get_logger().error('망함')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
