"""Action-based P6 mission coordinator with result, pose, and safety confirmation."""

import json
import math
import os
import time
import uuid

import rclpy
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from std_msgs.msg import Bool, String

from .mission_logic import MissionModel, MissionState


class MissionCoordinator(Node):
    def __init__(self):
        super().__init__('p6_mission_coordinator')
        self.declare_parameter('arm_backend', 'synthetic')
        self.declare_parameter('scenario', 'normal')
        self.declare_parameter('auto_start', True)
        self.declare_parameter('target_x', 10.0)
        self.declare_parameter('target_y', 6.5)
        self.declare_parameter('home_x', 1.0)
        self.declare_parameter('home_y', 1.0)
        self.declare_parameter('navigation_timeout_sec', 90.0)
        self.declare_parameter('stack_timeout_sec', 60.0)
        self.declare_parameter('target_timeout_sec', 15.0)
        self.declare_parameter('arm_timeout_sec', 120.0)
        self.declare_parameter('communication_timeout_sec', 4.0)
        self.declare_parameter('arrival_tolerance_m', 0.25)
        self.declare_parameter('stationary_speed_mps', 0.04)
        self.declare_parameter('stationary_yaw_rate', 0.06)
        self.declare_parameter('stationary_confirm_sec', 0.5)
        self.declare_parameter('target_detection_count', 3)
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('target_topic', '/p2_offline/pose_estimator/object_pose')
        self.declare_parameter('arm_request_topic', '/p6/arm_task/request')
        self.declare_parameter('arm_cancel_topic', '/p6/arm_task/cancel')
        self.declare_parameter('arm_result_topic', '/p6/arm_task/result')
        self.declare_parameter('arm_safety_topic', '/serial/arm_safety/status')
        self.declare_parameter('p4e_status_required', False)
        self.declare_parameter('event_log_path', '/tmp/p6_autonomous_mission_events.jsonl')

        self.backend = str(self.get_parameter('arm_backend').value)
        if self.backend != 'synthetic':
            raise RuntimeError('P6 offline launch only accepts arm_backend=synthetic; hardware mode is disabled')
        self.scenario = str(self.get_parameter('scenario').value)
        self.model = MissionModel()
        self._start_requested = bool(self.get_parameter('auto_start').value)
        self._odom = None
        self._odom_rx = 0.0
        self._latest_pose = None
        self._pose_rx = 0.0
        self._last_pose_stamp = None
        self._detection_streak = 0
        self._stationary_since = None
        self._state_started = time.monotonic()
        self._deadline = None
        self._nav_destination = None
        self._nav_attempt = 0
        self._nav_retry_at = None
        self._nav_goal_future = None
        self._nav_result_future = None
        self._nav_goal_handle = None
        self._arm_task_id = None
        self._arm_stage = None
        self._arm_result = None
        self._arm_cancel_sent = False
        self._test_cancel_at = None
        self._safety_state = None
        self._safety_rx = 0.0
        self._p4e_status_required = bool(self.get_parameter('p4e_status_required').value)
        self._event_path = str(self.get_parameter('event_log_path').value)
        self._event_pub = self.create_publisher(String, '/p6/mission_events', 20)
        self._state_pub = self.create_publisher(String, '/p6/mission_state', 1)
        self._stop_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._arm_request_pub = self.create_publisher(
            String, str(self.get_parameter('arm_request_topic').value), 10)
        self._arm_cancel_pub = self.create_publisher(
            String, str(self.get_parameter('arm_cancel_topic').value), 10)
        self._nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self._nav_lifecycle_names = (
            'controller_server', 'planner_server', 'behavior_server', 'bt_navigator',
            'smoother_server', 'waypoint_follower', 'velocity_smoother',
        )
        self._nav_lifecycle_clients = {
            name: self.create_client(GetState, '/' + name + '/get_state')
            for name in self._nav_lifecycle_names
        }
        self._nav_state_futures = None
        self._nav_check_retry_at = 0.0
        self._nav_lifecycle_states = None
        self.create_subscription(
            Odometry, str(self.get_parameter('odom_topic').value), self._on_odom, 10)
        self.create_subscription(
            PoseStamped, str(self.get_parameter('target_topic').value), self._on_target, 10)
        self.create_subscription(String, str(self.get_parameter('arm_result_topic').value),
                                 self._on_arm_result, 10)
        self.create_subscription(String, str(self.get_parameter('arm_safety_topic').value),
                                 self._on_arm_safety, 10)
        self.create_subscription(Bool, '/p6/start', self._on_start, 10)
        self.create_timer(0.05, self._tick)
        self._record('READY', backend=self.backend, scenario=self.scenario,
                     p4e_status_required=self._p4e_status_required)
        self._publish_state()
        self.get_logger().warning(
            'P6 OFFLINE ONLY: arm_backend=synthetic; no serial driver is part of this launch. '
            'Any observed P4e FAULT_LATCHED status aborts the mission.')

    @property
    def state(self):
        return self.model.state

    def _on_start(self, msg):
        if not msg.data:
            return
        self._accept_trigger()

    def _accept_trigger(self):
        if self.model.accept_trigger():
            self._state_started = time.monotonic()
            self._deadline = self._state_started + float(self.get_parameter('stack_timeout_sec').value)
            self._record('TRIGGER_ACCEPTED')
            self._publish_state()
        else:
            self._record('TRIGGER_IGNORED', reason='mission already started or terminal')

    def _on_odom(self, msg):
        self._odom = msg
        self._odom_rx = time.monotonic()

    def _on_target(self, msg):
        self._latest_pose = msg
        self._pose_rx = time.monotonic()
        stamp = (int(msg.header.stamp.sec), int(msg.header.stamp.nanosec))
        if self.state == MissionState.DETECTING_TARGET and stamp != self._last_pose_stamp:
            valid = bool(msg.header.frame_id)
            xyz = (msg.pose.position.x, msg.pose.position.y, msg.pose.position.z)
            quat = (msg.pose.orientation.x, msg.pose.orientation.y,
                    msg.pose.orientation.z, msg.pose.orientation.w)
            valid = valid and all(math.isfinite(v) for v in xyz + quat)
            valid = valid and sum(v * v for v in quat) > 0.5
            if self.scenario != 'target_loss' and valid:
                self._detection_streak += 1
            else:
                self._detection_streak = 0
            self._last_pose_stamp = stamp

    def _on_arm_result(self, msg):
        fields = msg.data.split('|')
        if len(fields) != 3:
            self._record('ARM_RESULT_REJECTED', reason='malformed result')
            return
        task_id, stage, outcome = fields
        if task_id != self._arm_task_id or stage != self._arm_stage:
            self._record('ARM_RESULT_REJECTED', reason='task id or stage mismatch',
                         received_task_id=task_id, received_stage=stage)
            return
        if self.state not in (MissionState.ARM_PICK, MissionState.ARM_UNLOAD):
            self._record('ARM_RESULT_REJECTED', reason='result received outside arm stage')
            return
        self._arm_result = (outcome == 'success')
        self._record('ARM_RESULT_RECEIVED', stage=stage, outcome=outcome, task_id=task_id)

    def _on_arm_safety(self, msg):
        text = str(msg.data)
        self._safety_state = 'unknown'
        for token in text.split():
            if token.startswith('state='):
                self._safety_state = token.split('=', 1)[1].strip().strip(',;')
                break
        self._safety_rx = time.monotonic()
        if self._safety_state == 'FAULT_LATCHED':
            self._abort('P4e arm host safety reports FAULT_LATCHED')
        elif self._p4e_status_required and self.state in (
                MissionState.ARM_PICK, MissionState.ARM_UNLOAD) and self._safety_state == 'DISARMED':
            self._abort('P4e arm host safety became DISARMED during a task')

    def _state_timeout(self):
        state = self.state
        if state == MissionState.WAITING_FOR_STACK:
            return float(self.get_parameter('stack_timeout_sec').value)
        if state in (MissionState.NAVIGATING_TO_TARGET, MissionState.NAVIGATING_HOME):
            return float(self.get_parameter('navigation_timeout_sec').value)
        if state in (MissionState.CONFIRMING_TARGET_STOP, MissionState.CONFIRMING_HOME_STOP):
            return 10.0
        if state == MissionState.DETECTING_TARGET:
            return float(self.get_parameter('target_timeout_sec').value)
        if state in (MissionState.ARM_PICK, MissionState.ARM_UNLOAD):
            if self.scenario == 'communication_loss':
                return float(self.get_parameter('communication_timeout_sec').value)
            return float(self.get_parameter('arm_timeout_sec').value)
        return None

    def _tick(self):
        now = time.monotonic()
        if self.state == MissionState.IDLE and self._start_requested:
            self._start_requested = False
            self._accept_trigger()
        if self._is_stopping_state():
            self._publish_zero()
        if self.state in (MissionState.IDLE, MissionState.SUCCEEDED, MissionState.ABORTED):
            return
        if self._deadline is not None and now > self._deadline:
            if self.state in (MissionState.NAVIGATING_TO_TARGET, MissionState.NAVIGATING_HOME):
                if self._nav_goal_handle is not None and self._nav_goal_handle.accepted:
                    self._nav_goal_handle.cancel_goal_async()
                self._handle_navigation_failure('Nav2 action timed out')
            else:
                self._abort('stage timeout in %s' % self.state.value)
            return
        if self._p4e_status_required and self.state in (
                MissionState.ARM_PICK, MissionState.ARM_UNLOAD):
            if self._safety_state is None or now - self._safety_rx > 2.0:
                self._abort('required P4e arm safety status is missing or stale')
                return

        if self.state == MissionState.WAITING_FOR_STACK:
            if (self._odom is not None and now - self._odom_rx < 0.5
                    and self._nav_client.server_is_ready()
                    and self._navigation_lifecycle_active(now)):
                self._record('NAV2_READY', action='/navigate_to_pose', odom_topic='/odom')
                target_x = float(self.get_parameter('target_x').value)
                target_y = float(self.get_parameter('target_y').value)
                if self.scenario == 'nav_failure':
                    # Place the goal inside a mapped occupied block so real Nav2 planning fails.
                    target_x, target_y = 7.75, 2.9
                self._nav_destination = ('target', target_x, target_y)
                self._transition(MissionState.NAVIGATING_TO_TARGET, 'start target navigation')
                self._begin_navigation_attempt()
        elif self.state in (MissionState.NAVIGATING_TO_TARGET, MissionState.NAVIGATING_HOME):
            self._poll_navigation(now)
        elif self.state in (MissionState.CONFIRMING_TARGET_STOP, MissionState.CONFIRMING_HOME_STOP):
            if self._confirm_stopped(now):
                if self.state == MissionState.CONFIRMING_TARGET_STOP:
                    self._detection_streak = 0
                    self._last_pose_stamp = None
                    self._transition(MissionState.DETECTING_TARGET, 'Nav2 success, pose and stop confirmed')
                    self._deadline = now + float(self.get_parameter('target_timeout_sec').value)
                else:
                    self._transition(MissionState.ARM_UNLOAD, 'base arrival and stop confirmed')
                    self._start_arm_task('unload')
        elif self.state == MissionState.DETECTING_TARGET:
            if self._pose_rx and now - self._pose_rx < 0.5 and self._detection_streak >= int(
                    self.get_parameter('target_detection_count').value):
                self._record('TARGET_CONFIRMED', consecutive_poses=self._detection_streak,
                             frame_id=self._latest_pose.header.frame_id)
                self._transition(MissionState.ARM_PICK, 'fresh repeated synthetic target poses')
                self._start_arm_task('pick')
        elif self.state in (MissionState.ARM_PICK, MissionState.ARM_UNLOAD):
            self._poll_arm_task(now)
            if (self._test_cancel_at is not None and now >= self._test_cancel_at
                    and self.state == MissionState.ARM_PICK
                    and self._arm_stage == 'pick' and self._arm_task_id is not None):
                cancel = String()
                cancel.data = self._arm_task_id
                self._arm_cancel_pub.publish(cancel)
                self._arm_cancel_sent = True
                self._test_cancel_at = None
                self._record('ARM_CANCEL_SENT_FOR_TEST', stage=self._arm_stage,
                             task_id=cancel.data)
        if self._nav_retry_at is not None and now >= self._nav_retry_at:
            self._nav_retry_at = None
            self._dispatch_navigation()

    def _transition(self, next_state, reason):
        self.model.transition(next_state)
        self._state_started = time.monotonic()
        timeout = self._state_timeout()
        self._deadline = self._state_started + timeout if timeout else None
        self._record('STATE', state=next_state.value, reason=reason)
        self._publish_state()

    def _navigation_lifecycle_active(self, now):
        if now < self._nav_check_retry_at:
            return False
        if not all(client.service_is_ready() for client in self._nav_lifecycle_clients.values()):
            return False
        if self._nav_state_futures is None:
            self._nav_state_futures = {
                name: client.call_async(GetState.Request())
                for name, client in self._nav_lifecycle_clients.items()
            }
            return False
        if not all(future.done() for future in self._nav_state_futures.values()):
            return False
        states = {}
        for name, future in self._nav_state_futures.items():
            try:
                current = future.result().current_state
                states[name] = {'id': int(current.id), 'label': current.label}
            except Exception as exc:
                states[name] = {'id': -1, 'label': str(exc)}
        if states != self._nav_lifecycle_states:
            self._record('NAV2_LIFECYCLE_STATUS', nodes=states)
            self._nav_lifecycle_states = states
        active = all(
            state['id'] == LifecycleState.PRIMARY_STATE_ACTIVE
            for state in states.values()
        )
        self._nav_state_futures = None
        self._nav_check_retry_at = now + (0.0 if active else 0.5)
        return active

    def _begin_navigation_attempt(self):
        self._nav_attempt = 0
        self._nav_retry_at = None
        self._dispatch_navigation()

    def _dispatch_navigation(self):
        if self.state not in (MissionState.NAVIGATING_TO_TARGET, MissionState.NAVIGATING_HOME):
            return
        self._nav_attempt += 1
        destination, x, y = self._nav_destination
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = x
        goal.pose.pose.position.y = y
        goal.pose.pose.orientation.w = 1.0
        self._nav_goal_handle = None
        self._nav_result_future = None
        self._nav_goal_future = self._nav_client.send_goal_async(goal)
        self._deadline = time.monotonic() + float(self.get_parameter('navigation_timeout_sec').value)
        self._record('NAV_GOAL_SENT', destination=destination, x=x, y=y,
                     attempt=self._nav_attempt, action_result_required=True)

    def _poll_navigation(self, now):
        if self._nav_goal_future is not None and self._nav_goal_future.done():
            try:
                self._nav_goal_handle = self._nav_goal_future.result()
            except Exception as exc:
                self._handle_navigation_failure('goal request exception: %s' % exc)
                return
            self._nav_goal_future = None
            if not self._nav_goal_handle.accepted:
                self._handle_navigation_failure('Nav2 rejected the goal')
                return
            self._nav_result_future = self._nav_goal_handle.get_result_async()
        if self._nav_result_future is not None and self._nav_result_future.done():
            try:
                wrapped = self._nav_result_future.result()
            except Exception as exc:
                self._handle_navigation_failure('result exception: %s' % exc)
                return
            self._nav_result_future = None
            if wrapped.status != GoalStatus.STATUS_SUCCEEDED:
                self._handle_navigation_failure('Nav2 result status=%s' % wrapped.status)
                return
            self._nav_goal_handle = None
            self._stationary_since = None
            confirm_state = (MissionState.CONFIRMING_TARGET_STOP
                             if self.state == MissionState.NAVIGATING_TO_TARGET
                             else MissionState.CONFIRMING_HOME_STOP)
            self._transition(confirm_state, 'NavigateToPose returned SUCCEEDED')
            self._record('NAV_RESULT_CONFIRMED', status='SUCCEEDED')
            self._deadline = now + 10.0

    def _handle_navigation_failure(self, reason):
        self._record('NAVIGATION_FAILURE', reason=reason, attempt=self._nav_attempt)
        self._nav_goal_handle = None
        self._nav_goal_future = None
        self._nav_result_future = None
        if self._nav_attempt <= 1:
            self._record('NAVIGATION_RETRY', next_attempt=self._nav_attempt + 1)
            self._nav_retry_at = time.monotonic() + 0.5
            self._deadline = self._nav_retry_at + float(
                self.get_parameter('navigation_timeout_sec').value)
            return
        self._abort('navigation failed after one retry: %s' % reason)

    def _confirm_stopped(self, now):
        if self._odom is None or now - self._odom_rx > 0.5:
            self._stationary_since = None
            return False
        destination, goal_x, goal_y = self._nav_destination
        pose = self._odom.pose.pose
        twist = self._odom.twist.twist
        distance = math.hypot(pose.position.x - goal_x, pose.position.y - goal_y)
        stationary = (abs(twist.linear.x) <= float(self.get_parameter('stationary_speed_mps').value)
                      and abs(twist.angular.z) <= float(self.get_parameter('stationary_yaw_rate').value))
        if distance <= float(self.get_parameter('arrival_tolerance_m').value) and stationary:
            if self._stationary_since is None:
                self._stationary_since = now
            elif now - self._stationary_since >= float(self.get_parameter('stationary_confirm_sec').value):
                self._record('ARRIVAL_AND_STOP_CONFIRMED', destination=destination,
                             x=pose.position.x, y=pose.position.y, distance_m=distance)
                return True
        else:
            self._stationary_since = None
        return False

    def _start_arm_task(self, stage):
        if self._p4e_status_required and (
                self._safety_state != 'ACTIVE_HOST' or time.monotonic() - self._safety_rx > 2.0):
            self._abort('P4e safety must be freshly ACTIVE_HOST before arm execution')
            return
        task_id = uuid.uuid4().hex
        self._arm_task_id = task_id
        self._arm_stage = stage
        self._arm_result = None
        self._arm_cancel_sent = False
        if self.scenario == 'communication_loss':
            self._record('ARM_REQUEST_DROPPED_FOR_TEST', stage=stage, task_id=task_id)
        else:
            msg = String()
            msg.data = task_id + '|' + stage
            self._arm_request_pub.publish(msg)
            self._record('ARM_REQUEST_SENT', stage=stage, task_id=task_id,
                         result_topic='/p6/arm_task/result')
        if self.scenario == 'grasp_failure' and stage == 'pick':
            self._test_cancel_at = time.monotonic() + 0.5

    def _poll_arm_task(self, now):
        if self._arm_result is None:
            return
        success = self._arm_result
        task_id, stage = self._arm_task_id, self._arm_stage
        self._arm_task_id = None
        self._arm_stage = None
        self._arm_result = None
        if not success:
            self._abort('%s stage returned failure' % stage)
            return
        if stage == 'pick' and self.state == MissionState.ARM_PICK:
            self._record('SAMPLE_PICK_CONFIRMED', task_id=task_id,
                         evidence='P3b planner completed with synthetic joint feedback')
            self._nav_destination = (
                'home', float(self.get_parameter('home_x').value),
                float(self.get_parameter('home_y').value))
            self._transition(MissionState.NAVIGATING_HOME, 'arm pick result confirmed')
            self._begin_navigation_attempt()
        elif stage == 'unload' and self.state == MissionState.ARM_UNLOAD:
            self._record('BASE_UNLOAD_CONFIRMED', task_id=task_id,
                         evidence='P3b staged unload plan completed with synthetic joint feedback')
            self._transition(MissionState.SUCCEEDED, 'base unload result confirmed')

    def _abort(self, reason):
        if self.state in (MissionState.SUCCEEDED, MissionState.ABORTED):
            return
        if self._arm_task_id and not self._arm_cancel_sent:
            cancel = String()
            cancel.data = self._arm_task_id
            self._arm_cancel_pub.publish(cancel)
            self._arm_cancel_sent = True
            self._record('ARM_CANCEL_SENT', task_id=self._arm_task_id, reason=reason)
        if self._nav_goal_handle is not None and self._nav_goal_handle.accepted:
            self._nav_goal_handle.cancel_goal_async()
        self._nav_goal_handle = None
        self._nav_goal_future = None
        self._nav_result_future = None
        self._nav_retry_at = None
        self.model.transition(MissionState.ABORTED)
        self._deadline = None
        self._record('MISSION_ABORTED', reason=reason)
        self._publish_state()
        self.get_logger().error('P6 mission aborted: %s' % reason)

    def _is_stopping_state(self):
        return self.state in (
            MissionState.CONFIRMING_TARGET_STOP, MissionState.DETECTING_TARGET,
            MissionState.ARM_PICK, MissionState.CONFIRMING_HOME_STOP,
            MissionState.ARM_UNLOAD, MissionState.SUCCEEDED, MissionState.ABORTED)

    def _publish_zero(self):
        self._stop_pub.publish(Twist())

    def _publish_state(self):
        msg = String()
        msg.data = self.state.value
        self._state_pub.publish(msg)

    def _record(self, event, **fields):
        record = {'time_unix': time.time(), 'event': event, 'state': self.state.value}
        record.update(fields)
        line = json.dumps(record, sort_keys=True, ensure_ascii=False)
        try:
            parent = os.path.dirname(self._event_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            with open(self._event_path, 'a', encoding='utf-8') as stream:
                stream.write(line + '\n')
        except OSError as exc:
            self.get_logger().error('Could not append event log: %s' % exc)
        msg = String()
        msg.data = line
        self._event_pub.publish(msg)
        if event == 'STATE':
            self.get_logger().info('%s: %s' % (fields.get('state'), fields.get('reason', '')))
        elif event.endswith('FAILURE') or event == 'MISSION_ABORTED':
            self.get_logger().error(line)
        else:
            self.get_logger().info(line)


def main(args=None):
    rclpy.init(args=args)
    node = MissionCoordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if rclpy.ok() and node.state not in (MissionState.SUCCEEDED, MissionState.ABORTED):
            node._abort('coordinator shutdown')
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
