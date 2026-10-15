"""Real-robot YOLO/LiDAR approach owned by the control-center ROS executor.

Nav2 takes the robot to a staging pose. Final feedback is the stored LiDAR target
relative to current odometry, so leaving RGB FOV does not cancel the approach.
All velocities enter /cmd_vel and the existing 3D guard / actuator watchdog.
"""
from collections import deque
from concurrent.futures import ThreadPoolExecutor
import math
import time

CLASSES = ('green_apple', 'peach', 'tomato', 'fanta_can', 'cup', 'banana')
ALIASES = {'사과': 'green_apple', '청사과': 'green_apple', 'apple': 'green_apple', 'green apple': 'green_apple',
           '복숭아': 'peach', '토마토': 'tomato', '환타': 'fanta_can', '환타캔': 'fanta_can', '환타 캔': 'fanta_can',
           'fanta': 'fanta_can', 'fanta can': 'fanta_can', '컵': 'cup', '머그': 'cup', 'mug': 'cup', '바나나': 'banana'}


def target_label(value):
    if not isinstance(value, str) or len(value) > 48:
        raise ValueError('찾을 물체 이름을 입력하세요.')
    name = value.strip().lower()
    name = ALIASES.get(name, name)
    if name not in CLASSES:
        raise ValueError('현재 YOLO 지원 물체: 사과, 복숭아, 토마토, 환타캔, 컵, 바나나')
    return name


class DetectionApproach:
    def __init__(self, node, state, settings):
        import numpy as np
        from action_msgs.msg import GoalStatusArray
        from geometry_msgs.msg import PointStamped, Twist
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
        from std_msgs.msg import String
        from head_lidar import HeadLidar
        from grasp_approach import GraspPlanner
        self.np, self.PointStamped, self.Twist, self.String = np, PointStamped, Twist, String
        self.node, self.state = node, state
        self.lidar = HeadLidar(node, settings)
        self.planner = GraspPlanner(settings)
        self.requests = deque()
        state.detection_request = self.requests.append
        self.label = ''
        self.phase, self.message = 'idle', '물체 이름을 입력하세요.'
        self.active = False
        self.observing = True
        self.target = self.anchor = self.stage_goal = None
        self.samples = deque(maxlen=3)
        self.fusion_pool = ThreadPoolExecutor(max_workers=1, thread_name_prefix='head-lidar')
        self.fusion_future = None
        self.generation = 0
        self.last_image = self.last_lidar = self.last_fusion = 0.
        self.last_seen = self.started = self.settle_until = self.arrived_since = None
        self.last_pose = self.last_command = None
        self.previous_reference = None
        self.previous_ros_time = None
        self.progress_best, self.progress_at = math.inf, 0.
        self.edge_clearance = float(settings.get('table_clearance', 0.))
        if not 0 <= self.edge_clearance <= .1:
            raise ValueError('detection.table_clearance는 0~0.1m입니다.')
        self.distance = None
        self.plan, self.candidates, self.surface_future = None, [], None
        self.surface_requested = 0.
        self.nav_busy = False
        self.cmd = node.create_publisher(Twist, '/cmd_vel', 1)
        self.status_pub = node.create_publisher(String, '/detection/closed_approach/status', 1)
        self.point_pub = node.create_publisher(PointStamped, '/detection/closed_approach/target', 1)
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL, reliability=ReliabilityPolicy.RELIABLE)
        self.nav_subscription = node.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status',
                                                        self.navigation_status, qos)

    def navigation_status(self, msg):
        self.nav_busy = any(s.status in (1, 2, 3) for s in msg.status_list)

    def report(self, phase, message):
        if (phase, message) != (self.phase, self.message):
            self.state.log('Detection · '+message, 'warn' if phase == 'error' else 'info', 'detection')
        self.phase, self.message = phase, message

    def command(self, linear=0., angular=0.):
        msg = self.Twist()
        msg.linear.x, msg.angular.z = float(linear), float(angular)
        self.cmd.publish(msg)
        self.last_command = (linear, angular)

    def cancel(self, message='접근 취소됨', phase='canceled'):
        if self.active:
            self.command()
            if self.stage_goal is not None and self.node.nav_status.get('goal') is self.stage_goal:
                self.node.cancel_goal()
        self.active = False
        self.stage_goal = None
        self.surface_future = None
        self.last_command = None
        self.report(phase, message)

    def clear(self):
        self.generation += 1
        self.target = self.anchor = None
        self.plan, self.candidates = None, []
        self.samples.clear()
        self.last_image = self.last_lidar = 0.
        self.last_seen = None
        self.previous_reference = None
        self.plan, self.candidates = None, []

    def check_reference(self, now):
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        reset = self.previous_ros_time is not None and ros_now < self.previous_ros_time-.1
        self.previous_ros_time = ros_now
        if self.target is not None or self.samples:
            rotation, position = self.node.transform(self.node.base, target='odom')
            yaw = math.atan2(rotation[1, 0], rotation[0, 0])
            if self.previous_reference:
                then, previous, previous_yaw = self.previous_reference
                elapsed = max(.0, now-then)
                reset |= (self.np.linalg.norm(position[:2]-previous[:2]) > .15+.5*elapsed
                          or abs(math.atan2(math.sin(yaw-previous_yaw), math.cos(yaw-previous_yaw))) > .15+.8*elapsed)
            self.previous_reference = now, position, yaw
        if reset:
            self.cancel('오도메트리 / 시각 초기화 · LiDAR 목표를 다시 관측합니다.', 'error')
            self.clear()
            self.lidar.clouds.clear()
            self.lidar.surface_clouds.clear()
            self.lidar.surface_cache = self.lidar.surface_cache_point = None
            self.lidar.pending.clear()

    def request(self, request):
        action = request['action']
        if action == 'select':
            self.cancel('목표 변경 · 접근 취소') if self.active else None
            self.label = target_label(request['label'])
            self.clear()
            self.report('searching', self.label+' · YOLO와 LiDAR 위치 관측 대기')
        elif action in ('start', 'stop'):
            if action == 'stop':
                self.observing = False
                self.cancel('검출 중지 · 접근 취소')
                self.clear()
            else:
                self.observing = True
            self.state.sensor_manager.service_request('detection', action)
        elif action == 'cancel':
            self.cancel()
        elif action == 'clear':
            self.cancel('저장된 물체 위치 지움')
            self.clear()
            self.report('searching' if self.label else 'idle', '저장 좌표 삭제 · 새 관측 대기')
        elif action == 'approach':
            self.start(float(request.get('edge_clearance', 0.)))

    def observe(self, now):
        if not self.label:
            return
        ros_now = self.node.get_clock().now().nanoseconds / 1e9
        if self.fusion_future is None:
            if now-self.last_fusion < .25:
                return
            with self.state.condition:
                detection = self.state.values.get('detections') or {}
            stamp = detection.get('stamp')
            if (not isinstance(stamp, (int, float)) or not math.isfinite(stamp)
                    or stamp == self.last_image or not -.1 <= ros_now-stamp <= 1.0):
                return
            if not detection.get('valid') or not detection.get('measured'):
                return
            matching = [o for o in detection.get('objects', []) if o.get('label') == self.label]
            if not matching:
                self.last_image = stamp
                return
            self.last_fusion = now
            self.fusion_future = (self.generation, stamp, self.fusion_pool.submit(
                self.lidar.locate, dict(detection, objects=matching)))
            return  # keep TF, action callbacks and odometry off the projection worker
        generation, stamp, future = self.fusion_future
        if not future.done():
            return
        self.fusion_future = None
        if generation != self.generation or not -.1 <= ros_now-stamp <= 1.0:
            return
        objects, reason = future.result()
        if objects is None:
            if self.target is None and not self.active:
                self.report('searching', reason)
            return  # retry this frame when its TF/cloud arrives
        self.last_image = stamp
        candidates = [o for o in objects if o.get('position_valid')]
        if not candidates:
            if self.target is None and not self.active:
                self.report('searching', 'YOLO 검출됨 · 물체 박스의 LiDAR 점 3개 이상 대기')
            return
        np = self.np
        reference = self.anchor if self.active else self.target
        if reference is not None:
            point = np.array(reference['point_odom'])
            candidates = [o for o in candidates if np.linalg.norm(np.array(o['point_odom'])-point) <= .25]
            if not candidates:
                return  # never switch to a different object of the same class
            item = min(candidates, key=lambda o: np.linalg.norm(np.array(o['point_odom'])-point))
        else:
            _, robot = self.node.transform(self.node.base, target='odom')
            item = min(candidates, key=lambda o: np.linalg.norm(np.array(o['point_odom'])[:2]-robot[:2]))
        if item['lidar_stamp'] == self.last_lidar:
            return
        self.last_lidar = item['lidar_stamp']
        if self.samples and (stamp-self.samples[-1]['capture_stamp'] > 1.5 or
                             np.linalg.norm(np.array(item['point_odom'])-np.array(self.samples[-1]['point_odom'])) > .12):
            self.samples.clear()
        self.samples.append(item)
        if len(self.samples) < 3:
            if not self.active and self.target is None:
                self.report('confirming', f'{self.label} · LiDAR 위치 확인 {len(self.samples)}/3')
            return
        points = np.array([o['point_odom'] for o in self.samples])
        if np.max(np.linalg.norm(points-np.median(points, axis=0), axis=1)) > .12:
            return
        self.target = dict(item)
        self.last_seen = now-max(0., ros_now-stamp)
        if not self.active and self.phase in ('idle', 'searching', 'confirming', 'ready'):
            self.report('ready', self.label+' · LiDAR 좌표 저장됨')

    def relative(self, target):
        rotation, offset = self.node.transform('odom', target=self.node.base)
        point = rotation @ self.np.array(target['point_odom']) + offset
        if not self.np.isfinite(point).all():
            raise RuntimeError('물체 상대 좌표 오류')
        return point

    def robot(self):
        rotation, position = self.node.transform(self.node.base, target='odom')
        return (float(position[0]), float(position[1]), math.atan2(rotation[1, 0], rotation[0, 0])), float(position[2])

    def start(self, clearance):
        if self.active:
            raise RuntimeError('이미 접근 중입니다. 취소 후 다시 시작하세요.')
        if not math.isfinite(clearance) or not 0 <= clearance <= .1:
            raise ValueError('책상 추가 이격은 0~0.1m입니다.')
        now = time.monotonic()
        self.check_reference(now)
        if self.target is None or self.last_seen is None or now-self.last_seen > 30.:
            raise RuntimeError('30초 이내 YOLO + LiDAR로 확인한 물체 좌표가 필요합니다.')
        if self.nav_busy or self.node.nav_status['state'] in ('active', 'sending', 'canceling'):
            raise RuntimeError('진행 중인 Nav2 이동을 먼저 취소하세요.')
        self.state.sensor_manager.require_collision_ready()
        if not self.node.nav_client:
            raise RuntimeError('Nav2 메시지 패키지가 없습니다.')
        self.edge_clearance = clearance
        self.anchor = dict(self.target)
        self.plan, self.candidates, self.stage_goal, self.surface_future = None, [], None, None
        self.active, self.started = True, now
        self.last_pose = self.last_command = None
        self.progress_best, self.progress_at = math.inf, now
        self.arrived_since = None
        try:
            if not self.state.sensor_manager.navigation_available():
                with self.state.condition:
                    service = (self.state.values.get('service_control') or {}).get('navigation') or {}
                if not service.get('busy'):
                    self.state.sensor_manager.service_request('navigation', 'start')
            self.report('waiting_nav', 'Nav2 연결·활성화 대기 → 책상 파지 위치 계획')
        except Exception:
            self.active = False
            raise

    def begin_plan(self, plan, now):
        self.plan = plan
        self.distance = plan['target_distance']
        pose, _ = self.robot()
        dock, stage = plan['dock'], plan['stage']
        # Already inside the staging distance and on its docking corridor:
        # hand over to the precision controller without driving back out.
        c, s = math.cos(dock[2]), math.sin(dock[2])
        dx, dy = pose[0]-dock[0], pose[1]-dock[1]
        lateral = -s*dx+c*dy
        along = c*dx+s*dy
        if abs(lateral) <= .12 and -.75 <= along <= .03:
            self.stage_goal = None
            self.settle_until = now+1.2
            self.report('settling', 'Nav2 준비 영역 안 · 정지 후 물체 방향 정렬')
            return
        rotation, offset = self.node.transform('odom')
        point = rotation @ self.np.array([stage[0], stage[1], 0.])+offset
        yaw = stage[2]+math.atan2(rotation[1, 0], rotation[0, 0])
        self.stage_goal = self.node.send_goal(dict(x=float(point[0]), y=float(point[1]), yaw=yaw, owner='detection'))
        self.report('navigating', f'{self.label} · Nav2 책상 {plan["side"]+1}번 면 준비 위치 이동')

    def plan_table(self, now):
        surface = self.anchor.get('support_surface')
        if surface is None and self.target is not None:
            surface = self.target.get('support_surface')
        if self.surface_future is not None:
            if not self.surface_future.done():
                return
            surface = self.surface_future.result()
            self.surface_future = None
            if surface is not None:
                self.anchor['support_surface'] = surface
        if surface is None:
            if now-self.started > 35.:
                return self.cancel('물체 아래 책상 평면·가장자리 확인 시간 초과', 'error')
            if now-self.surface_requested > 1.:
                self.surface_requested = now
                stamp = self.node.get_clock().now().nanoseconds/1e9
                self.surface_future = self.fusion_pool.submit(self.lidar.table, self.anchor['point_odom'], stamp)
            self.report('planning', '물체 아래 책상 평면·실제 가장자리 LiDAR 관측 대기')
            return
        with self.state.condition:
            joints = self.state.values.get('joints') or {}
        positions = dict(zip(joints.get('names', []), joints.get('positions', [])))
        pose, z = self.robot()
        self.candidates = self.planner.plans(self.anchor['point_odom'], surface, pose, positions,
                                              self.edge_clearance, z)
        self.begin_plan(self.candidates.pop(0), now)

    def advance(self, now):
        if not self.active:
            return
        if now-self.started > 120.:
            return self.cancel('접근 제한 시간 120초 초과', 'error')
        if self.phase == 'waiting_nav':
            if not self.node.nav_ready():
                if now-self.started > 30.:
                    return self.cancel('Nav2 활성화 시간 초과 · NAV 실행 상태 확인', 'error')
                return
            self.report('planning', 'Nav2 연결됨 · 책상 가장자리와 실제 몸체로 파지 위치 계획')
        if self.phase == 'planning':
            self.plan_table(now)
            return
        if self.phase == 'navigating':
            nav = self.node.nav_status
            if nav.get('goal') is not self.stage_goal:
                return self.cancel('Nav2 목표가 다른 요청으로 변경됨', 'error')
            if nav['state'] == 'succeeded':
                self.settle_until = now+1.2
                self.stage_goal = None
                self.report('settling', 'Nav2 준비 위치 도착 · 물체 방향 정렬로 전환')
            elif nav['state'] not in ('active', 'sending'):
                if nav['state'] in ('aborted', 'rejected', 'error') and self.candidates:
                    self.begin_plan(self.candidates.pop(0), now)
                else:
                    return self.cancel('Nav2 책상 준비 위치 이동 실패 · '+nav.get('message', ''), 'error')
            return
        self.state.sensor_manager.require_collision_ready()
        pose, z = self.robot()
        position = self.np.array(pose[:2])
        if self.last_pose:
            then, previous, old_yaw = self.last_pose
            if (now-then > .5 or self.np.linalg.norm(position-previous) > .2
                    or abs(math.atan2(math.sin(pose[2]-old_yaw), math.cos(pose[2]-old_yaw))) > .35):
                self.cancel('오도메트리 연속성 손실 · 저장 좌표 재관측 필요', 'error')
                self.clear()
                return
        self.last_pose = now, position, pose[2]
        with self.state.condition:
            odom = self.state.values.get('odom') or {}
            meta = self.state.channels.get('odom') or {}
        ros_now = self.node.get_clock().now().nanoseconds/1e9
        received, source_stamp = meta.get('received'), meta.get('stamp')
        if (received is None or now-received > .4 or source_stamp is None
                or not -.1 <= ros_now-source_stamp <= .4):
            return self.cancel('접근용 현재 오도메트리 수신 지연', 'error')
        if self.phase == 'settling':
            if now < self.settle_until:
                return
            if self.nav_busy or abs(odom.get('linear', 1.)) > .02 or abs(odom.get('angular', 1.)) > .04:
                if now-self.settle_until > 5.:
                    return self.cancel('Nav2 정지 확인 시간 초과', 'error')
                return
            self.progress_at, self.progress_best = now, math.inf
            self.report('aligning', '물체 쪽으로 방향 정렬 · 정렬 전 전진 없음')
        if self.nav_busy:
            return self.cancel('다른 Nav2 이동 감지 · 정밀 접근 중지', 'error')
        point = self.relative(self.anchor)
        object_angle = math.atan2(point[1], point[0])
        dock = self.plan['dock']
        heading = math.atan2(math.sin(dock[2]-pose[2]), math.cos(dock[2]-pose[2]))
        xy_error = math.hypot(dock[0]-pose[0], dock[1]-pose[1])
        if self.phase == 'aligning':
            if abs(heading) > .035:
                self.command(0., max(-.25, min(.25, 1.2*heading)))
                metric = xy_error+.15*abs(heading)
                if metric < self.progress_best-.005:
                    self.progress_best, self.progress_at = metric, now
                elif now-self.progress_at > 12.:
                    return self.cancel('물체 방향 회전 불가 · 현재 3D 충돌 보호 원인 확인', 'error')
                return
            self.progress_best, self.progress_at = math.inf, now
            self.report('approaching', '책상 끝까지 몸체 기준 정밀 접근 · 저장 LiDAR 좌표 사용')
        if xy_error <= .008:
            self.report('final_align', '파지 위치 도착 · 물체 정면으로 최종 방향 정렬')
            if abs(object_angle) > .025:
                self.arrived_since = None
                self.command(0., max(-.20, min(.20, 1.2*object_angle)))
            else:
                self.command()
                self.arrived_since = self.arrived_since or now
                if now-self.arrived_since >= .6:
                    self.active = False
                    self.report('succeeded', self.label+' · 책상 최대 접근·물체 방향 정렬 완료')
                return
        else:
            self.arrived_since = None
            if self.phase == 'final_align':
                self.report('approaching', '정지 위치 보정 중')
            linear, angular, done = self.planner.command(pose, self.plan)
            if done:
                return self.cancel('파지 위치의 옆 방향 오차가 남아 있습니다 · 다른 책상 면에서 재접근 필요', 'error')
            if abs(heading) > .12:
                linear = 0.
            old_v, old_w = self.last_command or (0., 0.)
            self.command(max(old_v-.04, min(old_v+.015, linear)), max(old_w-.06, min(old_w+.06, angular)))
        metric = xy_error+.15*abs(object_angle)
        if metric < self.progress_best-.005:
            self.progress_best, self.progress_at = metric, now
        elif now-self.progress_at > 12.:
            with self.state.condition:
                collision = self.state.values.get('collision_safety') or {}
            detail = collision.get('blocking_link') or collision.get('reason') or '진행 없음'
            self.cancel('책상 정밀 접근 진행 불가 · '+detail, 'error')

    def publish(self, now):
        target = self.anchor if self.active and self.anchor is not None else self.target or self.anchor
        info = None
        if target is not None:
            info = dict(label=self.label, point_odom=target['point_odom'], capture_stamp=target['capture_stamp'],
                        lidar_stamp=target['lidar_stamp'], lidar_count=target['lidar_count'],
                        lidar_oldest_stamp=target['lidar_oldest_stamp'],
                        source_topic=target['source_topic'], point_method=target['point_method'],
                        table=target.get('support_surface'),
                        capture_age=max(0., self.node.get_clock().now().nanoseconds/1e9-target['capture_stamp']),
                        visible=self.last_seen is not None and now-self.last_seen < 1., frame='odom')
            try:
                rotation, offset = self.node.transform('odom')
                info.update(point_map=(rotation @ self.np.array(target['point_odom'])+offset).tolist(), frame=self.node.fixed,
                            lidar_points_map=(self.np.array(target['lidar_points_odom']) @ rotation.T+offset).tolist())
                relative = self.relative(target)
                info.update(point_base=relative.tolist(), range=float(math.hypot(relative[0], relative[1])))
            except Exception:
                pass
            msg = self.PointStamped()
            from rclpy.time import Time
            msg.header.frame_id = 'odom'
            msg.header.stamp = Time(nanoseconds=round(target['capture_stamp']*1e9)).to_msg()
            msg.point.x, msg.point.y, msg.point.z = map(float, target['point_odom'])
            self.point_pub.publish(msg)
        plan = None
        if self.plan is not None:
            plan = dict(self.plan)
            plan.pop('cost', None)
            try:
                pose, _ = self.robot()
                plan['position_error'] = math.hypot(pose[0]-self.plan['dock'][0], pose[1]-self.plan['dock'][1])
                rotation, offset = self.node.transform('odom')
                map_yaw = math.atan2(rotation[1, 0], rotation[0, 0])
                plan['frame'] = self.node.fixed
                for kind in ('dock', 'stage'):
                    xy = rotation @ self.np.array([*self.plan[kind][:2], 0.])+offset
                    plan[kind+'_map'] = [float(xy[0]), float(xy[1]), self.plan[kind][2]+map_yaw]
                corners = self.np.array([[*c, self.plan['surface']['z']] for c in self.plan['surface']['corners']])
                plan['table_corners_map'] = (corners @ rotation.T+offset).tolist()
            except Exception:
                pass
        value = dict(phase=self.phase, message=self.message, label=self.label, classes=list(CLASSES),
                     active=self.active, distance=self.distance, edge_clearance=self.edge_clearance,
                     target=info, plan=plan, nav_ready=self.node.nav_ready(),
                     can_approach=not self.active and self.last_seen is not None and now-self.last_seen <= 30.)
        self.state.put('detection_control', value)
        import json
        self.status_pub.publish(self.String(data=json.dumps(value, allow_nan=False)))

    def tick(self):
        now = time.monotonic()
        while self.requests:
            try:
                self.request(self.requests.popleft())
            except Exception as exc:
                self.cancel(str(exc)[:300], 'error')
        try:
            self.check_reference(now)
            if self.label and self.observing:
                self.lidar.flush()
                self.observe(now)
            self.advance(now)
        except Exception as exc:
            if self.active:
                self.cancel(str(exc)[:300], 'error')
            elif self.target is None and self.label:
                self.report('searching', str(exc)[:200])
        self.publish(now)

    def close(self):
        if self.active:
            self.cancel('관제센터 종료 · 접근 취소')
        self.fusion_pool.shutdown(wait=False, cancel_futures=True)
