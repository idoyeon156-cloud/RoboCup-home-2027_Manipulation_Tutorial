"""Associate YOLO boxes with measured MID-360 returns in odom.

The camera identifies the object; only LiDAR supplies its metric position. The
captured returns can then be retained by the approach controller outside RGB FOV.
"""
from collections import deque
import math

import cv2
import numpy as np


def seconds(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class HeadLidar:
    def __init__(self, node, settings):
        from sensor_msgs.msg import CameraInfo, PointCloud2
        from rclpy.qos import QoSProfile, ReliabilityPolicy
        self.node = node
        self.clouds, self.pending, self.infos = deque(maxlen=16), deque(maxlen=12), deque(maxlen=48)
        self.surface_clouds = deque(maxlen=20)
        self.surface_cache, self.surface_cache_point, self.surface_cache_at = None, None, 0.
        self.topic = settings.get('cloud_topic', '/mid360/points_obstacles')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.subscriptions = [
            node.create_subscription(PointCloud2, self.topic, self.receive, qos),
            node.create_subscription(PointCloud2, settings.get('surface_topic', '/mid360/points_filtered'),
                                     lambda msg: self.receive(msg, True), qos),
            node.create_subscription(CameraInfo, settings.get('camera_info_topic', '/head/calibrated/color/camera_info'),
                                     self.infos.append, qos),
        ]

    def receive(self, msg, surface=False):
        self.pending.append((msg, surface))

    def flush(self):
        now = self.node.get_clock().now().nanoseconds / 1e9
        while self.pending:
            msg, surface = self.pending[0]
            stamp = seconds(msg.header.stamp)
            if not 0 < stamp or not -.1 <= now-stamp <= 1.2:
                self.pending.popleft()
                continue
            try:
                rotation, offset = self.node.transform(msg.header.frame_id, msg.header.stamp, target='odom')
                if surface:
                    _, sensor_origin = self.node.transform('livox_frame', msg.header.stamp, target='odom')
            except Exception:
                if now-stamp < .4:
                    break  # let the exact-stamp odometry TF catch up
                self.pending.popleft()
                continue
            self.pending.popleft()
            fields = {f.name: f for f in msg.fields}
            if not all(k in fields and fields[k].datatype in (7, 8) for k in ('x', 'y', 'z')):
                continue
            endian = '>' if msg.is_bigendian else '<'
            dtype = np.dtype(dict(names=['x', 'y', 'z'],
                                  formats=[endian+('f4' if fields[k].datatype == 7 else 'f8') for k in ('x', 'y', 'z')],
                                  offsets=[fields[k].offset for k in ('x', 'y', 'z')], itemsize=msg.point_step))
            records = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                                 strides=(msg.row_step, msg.point_step))
            points = np.column_stack([records[k].ravel() for k in ('x', 'y', 'z')])
            points = points[np.isfinite(points).all(axis=1)]
            if len(points):
                destination = self.surface_clouds if surface else self.clouds
                if destination and stamp <= destination[-1][0]:
                    destination.clear()
                world = points @ rotation.T + offset
                destination.append((stamp, world, sensor_origin) if surface else (stamp, world))
        for cache in (self.clouds, self.surface_clouds):
            while cache and now-cache[0][0] > 2.0:
                cache.popleft()

    def table(self, point, stamp):
        from grasp_approach import support_surface
        if (stamp < self.surface_cache_at or stamp-self.surface_cache_at >= .8 or self.surface_cache_point is None
                or np.linalg.norm(np.asarray(point)-self.surface_cache_point) > .25):
            clouds = tuple(c for c in tuple(self.surface_clouds) if stamp-2.0 <= c[0] <= stamp+.15)
            self.surface_cache = support_surface(clouds, point)
            self.surface_cache_point, self.surface_cache_at = np.asarray(point), stamp
        return self.surface_cache

    def locate(self, detection):
        from rclpy.time import Time
        infos, clouds = tuple(self.infos), tuple(self.clouds)
        stamp, frame = detection.get('stamp'), detection.get('frame')
        if not isinstance(stamp, (int, float)) or not math.isfinite(stamp) or not frame:
            return None, 'YOLO 촬영 시각·좌표계 대기'
        if not infos or not clouds:
            return None, 'Head CameraInfo / MID-360 점군 대기'
        info = min(infos, key=lambda m: abs(seconds(m.header.stamp)-stamp))
        if (abs(seconds(info.header.stamp)-stamp) > .2 or info.header.frame_id != frame
                or (info.width, info.height) != (detection.get('width'), detection.get('height'))):
            return None, 'YOLO 영상과 CameraInfo 시각·해상도·좌표계 불일치'
        if (info.distortion_model not in ('plumb_bob', 'rational_polynomial', '')
                or not all(math.isfinite(v) for v in (*info.k, *info.d)) or min(info.k[0], info.k[4]) <= 0):
            return None, 'Head 내부 보정 형식 오류'
        recent = [(s, p) for s, p in clouds if stamp-1.0 <= s <= stamp+.1]
        if not recent or max(s for s, _ in recent) < stamp-.25:
            return None, 'YOLO 촬영 시각의 LiDAR 관측 대기'
        capture = Time(nanoseconds=round(stamp*1e9)).to_msg()
        try:
            rotation, offset = self.node.transform('odom', capture, target=frame)
        except Exception:
            return None, 'YOLO 촬영 시각의 Head ↔ odom TF 대기'
        points = np.concatenate([p for _, p in recent])
        point_stamps = np.concatenate([np.full(len(p), s) for s, p in recent])
        camera = points @ rotation.T + offset
        valid = np.isfinite(camera).all(axis=1) & (camera[:, 2] > .15) & (camera[:, 2] < 5.)
        points, camera, point_stamps = points[valid], camera[valid], point_stamps[valid]
        if not len(points):
            return None, 'Head 시야 안 LiDAR 점 없음'
        pixels, _ = cv2.projectPoints(camera, np.zeros(3), np.zeros(3), np.array(info.k).reshape(3, 3),
                                      np.array(info.d) if len(info.d) else None)
        pixels = pixels.reshape(-1, 2)
        objects = []
        for source in detection.get('objects', []):
            item = dict(source, position_valid=False, position_reason='물체 박스 안 LiDAR 점 부족')
            x, y, w, h = [source.get(k, 0.) for k in ('x', 'y', 'width', 'height')]
            if not all(isinstance(v, (int, float)) and math.isfinite(v) for v in (x, y, w, h)) or min(w, h) < 2:
                objects.append(item)
                continue
            inside = ((pixels[:, 0] > x+.22*w) & (pixels[:, 0] < x+.78*w)
                      & (pixels[:, 1] > y+.22*h) & (pixels[:, 1] < y+.78*h))
            found, ranges, stamps = points[inside], camera[inside, 2], point_stamps[inside]
            # Count distinct spatial returns, keeping the newest hit per cell.
            if len(found):
                _, unique = np.unique(np.floor(found[::-1]/.01).astype(np.int32), axis=0, return_index=True)
                found, ranges, stamps = found[::-1][unique], ranges[::-1][unique], stamps[::-1][unique]
            if len(found) >= 3:
                # Nearest compact depth cluster suppresses the background behind
                # the object. Bbox association is an estimate, not segmentation.
                nearest = float(np.percentile(ranges, 20))
                cluster = np.abs(ranges-nearest) <= .08
                found, stamps = found[cluster], stamps[cluster]
                if len(found) >= 3:
                    center = np.median(found, axis=0)
                    cluster = np.linalg.norm(found-center, axis=1) <= .18
                    found, stamps = found[cluster], stamps[cluster]
                    if len(found) >= 3:
                        point = found[np.argmin(np.linalg.norm(found-np.median(found, axis=0), axis=1))]
                        sample = found[::max(1, math.ceil(len(found)/64))]
                        item.update(position_valid=True, position_reason='', point_odom=point.tolist(),
                                    lidar_points_odom=sample.tolist(), lidar_count=len(found),
                                    lidar_stamp=float(stamps.max()), lidar_oldest_stamp=float(stamps.min()),
                                    capture_stamp=stamp, point_method='yolo_mid360_bbox_cluster', source_topic=self.topic)
                        item['support_surface'] = self.table(point, stamp)
            objects.append(item)
        return objects, ''
