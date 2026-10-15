#!/usr/bin/env python3
"""Generate Nav2's footprint from the exact URDF used by the 3D guard."""
import argparse
import copy
import math
from pathlib import Path
import sys

import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent.parent/'robot'))
from collision_geometry import RobotGeometry, rectangle


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--input', required=True)
    parser.add_argument('--output', required=True)
    parser.add_argument('--safety', required=True)
    args = parser.parse_args()
    safety = yaml.safe_load(Path(args.safety).read_text())['/**']['ros__parameters']
    profile = yaml.safe_load(Path(args.input).read_text())
    model = RobotGeometry()
    control = profile['controller_server']['ros__parameters']
    # A tabletop and the mast occupy different heights. The full 3D swept
    # geometry remains enforced by the guard; its XY envelope must not reject
    # every table-approach trajectory at the starting pose. Keep Nav2's inflated
    # center obstacle critic and path critics for coarse obstacle avoidance.
    control['GraspApproach'] = copy.deepcopy(control['FollowPath'])
    control['GraspApproach']['critics'] = [critic for critic in control['GraspApproach']['critics']
                                           if critic != 'ObstacleFootprint']
    if 'GraspApproach' not in control['controller_plugins']:
        control['controller_plugins'].append('GraspApproach')
    if 'grasp_stage_checker' not in control['goal_checker_plugins']:
        control['goal_checker_plugins'].append('grasp_stage_checker')
    control['grasp_stage_checker'] = dict(plugin='nav2_controller::SimpleGoalChecker', stateful=True,
                                           xy_goal_tolerance=.10, yaw_goal_tolerance=.08)
    profile['bt_navigator']['ros__parameters']['default_nav_to_pose_bt_xml'] = str(
        Path(__file__).resolve().with_name('navigate-to-pose-replanning.xml'))
    # The guard updates this with measured arm geometry after startup. Start
    # with the static URDF shape and fail closed until arm feedback is available.
    for kind in ('local_costmap', 'global_costmap'):
        settings = profile[kind][kind]['ros__parameters']
        # The 3D guard enforces clearance along the actual motion. Expanding
        # every side here would reject moving away from a close rear obstacle.
        settings['footprint'] = str(rectangle(model.fixed))
        settings['footprint_padding'] = 0.0
        if safety.get('body_only', False) and 'inflation_layer' in settings:
            # NavFn plans the robot center: retain the body's inscribed radius,
            # with no additional preference band outside the physical footprint.
            radius = min(abs(coordinate) for point in rectangle(model.fixed) for coordinate in point)
            # Polygon transport is float32. A micrometre rounding allowance
            # avoids a continuous "smaller than inscribed radius" error flood.
            settings['inflation_layer']['inflation_radius'] = math.ceil(radius*1e6)/1e6+1e-6
        for layer in ('voxel_layer', 'obstacle_layer', 'static_layer'):
            if layer in settings:
                settings[layer]['footprint_clearing_enabled'] = False
        for layer in ('voxel_layer', 'obstacle_layer'):
            if layer in settings:
                # MID-360 memory belongs to the cloud processor. Do not build
                # another marking/clearing memory with different input here.
                sources = settings[layer].get('observation_sources', '').split()
                settings[layer]['observation_sources'] = ' '.join(
                    source for source in sources if source not in ('mid360', 'mid360_clear'))
                for source in ('mid360', 'mid360_clear'):
                    settings[layer].pop(source, None)
        plugins = settings['plugins']
        if 'collision_scene' not in plugins:
            plugins.insert(plugins.index('inflation_layer') if 'inflation_layer' in plugins else len(plugins),
                           'collision_scene')
        settings['collision_scene'] = dict(
            plugin='robocup_collision_processing::SceneLayer', enabled=True,
            topic=safety['safety_topic'], min_height_above_floor=float(safety['floor_band']),
            max_obstacle_height=float(safety['max_obstacle_height']),
            voxel_size=float(safety['safety_voxel']), cloud_timeout=float(safety['cloud_timeout']),
            use_source_points=bool(safety.get('body_only', False)),
            maximum_cells=int(safety['maximum_cells']))
    Path(args.output).write_text(yaml.safe_dump(profile, sort_keys=False))


if __name__ == '__main__':
    main()
