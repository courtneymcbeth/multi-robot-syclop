#!/usr/bin/env python3
"""
Convert multi-robot-syclop YAML scenarios to PPL/WoDaSH format.

This script reads YAML scenario files and generates:
- .env files (environment definition)
- template.xml files (motion planning problem specification)
- .robot files (robot definitions)
- Custom .g geometry files for obstacles
"""

import argparse
import math
import os
import sys
import yaml
from pathlib import Path
from ppl_geometry_generator import create_geometry_file, get_geometry_filename
from generate_skeleton import generate_skeleton

# Robot radius for the iCreate robot model
ROBOT_RADIUS = 0.5


def generate_icreate_obj(output_path, radius=ROBOT_RADIUS, height=0.02, segments=20):
    """
    Generate a cylinder OBJ file for the iCreate robot model.

    Args:
        output_path: Path where .obj file should be written
        radius: Robot radius (default: ROBOT_RADIUS)
        height: Cylinder height (default: 0.02)
        segments: Number of segments around the circle (default: 20)
    """
    lines = []
    lines.append("#A cylinder.")
    lines.append(f"#\tRadius: {radius}")
    lines.append(f"#\tHeight: {height}")
    lines.append(f"#\tSegments: {segments}")
    lines.append(f"#\tNum vertices: {2 * (segments + 1)}")
    lines.append(f"#\tNum facets: {4 * segments}")

    # Generate vertices around the circle
    for i in range(segments):
        angle = 2 * math.pi * i / segments
        x = radius * math.cos(angle)
        y = radius * math.sin(angle)
        lines.append(f"v {x} {y} {height/2}")
        lines.append(f"v {x} {y} {-height/2}")

    # Center vertices for top and bottom caps
    lines.append(f"v 0 0 {height/2}")
    lines.append(f"v 0 0 {-height/2}")

    # Face indices (1-indexed in OBJ format)
    center_top = 2 * segments + 1
    center_bottom = 2 * segments + 2

    # Top and bottom cap faces
    for i in range(segments):
        curr_top = 2 * i + 1
        next_top = 2 * ((i + 1) % segments) + 1
        curr_bottom = 2 * i + 2
        next_bottom = 2 * ((i + 1) % segments) + 2

        lines.append(f"f {center_top} {curr_top} {next_top}")
        lines.append(f"f {center_bottom} {next_bottom} {curr_bottom}")

    # Side faces
    for i in range(segments):
        curr_top = 2 * i + 1
        curr_bottom = 2 * i + 2
        next_top = 2 * ((i + 1) % segments) + 1
        next_bottom = 2 * ((i + 1) % segments) + 2

        lines.append(f"f {curr_top} {curr_bottom} {next_bottom}")
        lines.append(f"f {curr_top} {next_bottom} {next_top}")

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Generated icreate.obj: {output_path} (radius: {radius})")


def normalize_angle_for_ppl(theta_radians):
    """
    Normalize angle from radians to PPL's expected range [-1, 1].
    PPL expects angles normalized by pi, so theta_normalized = theta_radians / pi.
    """
    return theta_radians / math.pi


# Robot type mapping from syclop to PPL
ROBOT_TYPE_MAPPING = {
    'unicycle_first_order_0_sphere': {
        'geometry': 'objs/icreate.obj',
        'movement': 'Planar Rotational',
    },
    'unicycle_first_order_0': {
        'geometry': 'objs/icreate.obj',
        'movement': 'Planar Rotational',
    },
    'unicycle_second_order_0': {
        'geometry': 'objs/icreate.obj',
        'movement': 'Planar Rotational',
    },
    'single_integrator_0': {
        'geometry': 'objs/icreate.obj',
        'movement': 'Planar Rotational',
    },
    'double_integrator_0': {
        'geometry': 'objs/icreate.obj',
        'movement': 'Planar Rotational',
    },
    'car_first_order_0': {
        'geometry': 'objs/icreate.obj',
        'movement': 'Planar Rotational',
    },
}

# Robot colors for visual distinction (RGBA format)
ROBOT_COLORS = [
    '1 0 0 1',  # Red
    '0 0 1 1',  # Blue
    '0 1 0 1',  # Green
    '1 1 0 1',  # Yellow
    '1 0 1 1',  # Magenta
    '0 1 1 1',  # Cyan
    '1 0.5 0 1',  # Orange
    '0.5 0 1 1',  # Purple
]


def load_syclop_yaml(yaml_path):
    """
    Load and parse a syclop YAML scenario file.

    Args:
        yaml_path: Path to YAML file

    Returns:
        dict: Parsed scenario data with 'environment' and 'robots' keys
    """
    with open(yaml_path, 'r') as f:
        data = yaml.safe_load(f)

    if 'environment' not in data:
        raise ValueError(f"YAML file missing 'environment' section: {yaml_path}")

    return data


def rad_to_deg(radians):
    """Convert radians to degrees."""
    return radians * 180.0 / math.pi


def convert_boundary(min_coords, max_coords):
    """
    Convert syclop boundary to PPL Box2D format.

    Args:
        min_coords: [min_x, min_y]
        max_coords: [max_x, max_y]

    Returns:
        str: Boundary line in PPL format
    """
    return f"Boundary Box2D [ {min_coords[0]}:{max_coords[0]} ; {min_coords[1]}:{max_coords[1]}]"


def generate_env_file(data, output_path, num_robots, scenario_name, include_robots=True):
    """
    Generate a PPL .env file from syclop data.

    Args:
        data: Parsed YAML data
        output_path: Path where .env file should be written
        num_robots: Number of robots to include
        scenario_name: Name of the scenario
        include_robots: If True, generate -vizmo.env with robots; if False, generate plain .env
    """
    env = data['environment']
    robots = data.get('robots', [])

    if include_robots and num_robots > len(robots):
        raise ValueError(
            f"Requested {num_robots} robots but only {len(robots)} defined in YAML")

    # Extract boundary
    min_coords = env['min']
    max_coords = env['max']
    obstacles = env.get('obstacles', [])

    # Count multibodies
    # Active robots count as 1 multibody collectively, not individually
    if include_robots:
        num_multibodies = len(obstacles) + 1  # obstacles + 1 (active group)
    else:
        num_multibodies = len(obstacles)

    lines = []

    # Boundary
    lines.append(convert_boundary(min_coords, max_coords))
    lines.append("")

    # MultiBodies
    lines.append("MultiBodies")
    lines.append(str(num_multibodies))
    lines.append("")

    # Active robots (only for vizmo version)
    if include_robots:
        lines.append("Active")
        lines.append(str(num_robots))

        for i in range(num_robots):
            robot = robots[i]
            robot_type = robot['type']

            # Get robot mapping or use default
            robot_info = ROBOT_TYPE_MAPPING.get(robot_type, ROBOT_TYPE_MAPPING['unicycle_first_order_0_sphere'])

            # Assign color (cycle through available colors)
            color = ROBOT_COLORS[i % len(ROBOT_COLORS)]

            # Write robot line
            geometry = robot_info['geometry']
            movement = robot_info['movement']
            lines.append(f"{geometry} -c({color}) {movement}")

        # Connections
        lines.append("Connections")
        lines.append("0")
        lines.append("")

    # Passive obstacles
    for obs in obstacles:
        if obs['type'] != 'box':
            print(f"Warning: Skipping non-box obstacle type: {obs['type']}")
            continue

        center = obs['center']
        size = obs['size']

        # Generate geometry filename
        width = size[0]
        height = size[1]
        geom_filename = get_geometry_filename(width, height)

        # 6D transform: x y z rx ry rz (planar: z=0, rx=ry=rz=0)
        transform = f"{center[0]} {center[1]} 0 0 0 0"

        lines.append("Passive")
        lines.append(f"objs/{geom_filename} {transform}")
        lines.append("")

    # Write file
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Generated .env file: {output_path}")


def generate_robot_file(output_path):
    """
    Generate a simple .robot file for PPL.

    Args:
        output_path: Path where .robot file should be written
    """
    lines = [
        "1",
        "objs/icreate.obj Planar Rotational",
        "Connections",
        "0",
        ""
    ]

    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Generated .robot file: {output_path}")


def generate_template_xml(data, output_path, num_robots, scenario_name, envs_relative_path,
                          kinodynamic=False):
    """
    Generate a PPL template.xml file for WoDaSH from syclop data.

    Args:
        data: Parsed YAML data
        output_path: Path where template.xml should be written
        num_robots: Number of robots to include
        scenario_name: Name of the scenario
        envs_relative_path: Relative path from output directory to envs directory
        kinodynamic: If True, generate kinodynamic K-WoDaSH template with car3d dynamics
    """
    robots = data.get('robots', [])

    if num_robots > len(robots):
        raise ValueError(
            f"Requested {num_robots} robots but only {len(robots)} defined in YAML")

    lines = []

    # XML header
    lines.append("<?xml version='1.0' encoding='UTF-8'?>")
    lines.append("<MotionPlanning warnings=\"true\" warningsAsErrors=\"true\" print=\"false\">")
    lines.append("")

    # Problem section
    lines.append("  <Problem baseFilename=\"@@base@@\">")
    lines.append("")

    # Environment
    env_path = f"@@envdir@@/{scenario_name}/{scenario_name}.env"
    lines.append(f"    <Environment filename=\"{env_path}\" />")
    lines.append("")

    # Robot definitions
    robot_labels = []
    robot_path = f"@@envdir@@/{scenario_name}/robot.robot"
    for i in range(num_robots):
        robot_label = f"robot{i+1}"
        robot_labels.append(robot_label)

        lines.append(f"    <Robot label=\"{robot_label}\" filename=\"{robot_path}\">")
        if kinodynamic:
            lines.append("      <Dynamics type=\"car3d\">")
            lines.append("        <ControlLimits>")
            lines.append("            <Range min=\"0\" max=\"1\"/>")
            lines.append("            <Range min=\"-1\" max=\"1\"/>")
            lines.append("        </ControlLimits>")
            lines.append("        <StateLimits>")
            lines.append("            <Range/>")
            lines.append("            <Range/>")
            lines.append("            <Range/>")
            lines.append("        </StateLimits>")
            lines.append("      </Dynamics>")
            lines.append("      <Agent type=\"child\" debug=\"true\" />")
        else:
            lines.append("      <Agent type=\"planning\" />")
        lines.append("    </Robot>")
        lines.append("")

    # RobotGroup
    group_label = f"{num_robots}-robotGroup"
    robot_labels_str = " ".join(robot_labels)
    lines.append(f"    <RobotGroup label=\"{group_label}\"")
    lines.append(f"      robotLabels=\"{robot_labels_str}\" />")
    lines.append("")

    # Coordinator robot (virtual) for WoDaSH
    lines.append(f"    <Robot label=\"coordinator\" virtual=\"true\" filename=\"{robot_path}\">")
    if kinodynamic:
        lines.append("      <Dynamics type=\"car3d\">")
        lines.append("        <ControlLimits>")
        lines.append("            <Range min=\"0\" max=\"1\"/>")
        lines.append("            <Range min=\"-1\" max=\"1\"/>")
        lines.append("        </ControlLimits>")
        lines.append("        <StateLimits>")
        lines.append("            <Range/>")
        lines.append("            <Range/>")
        lines.append("            <Range/>")
        lines.append("        </StateLimits>")
        lines.append("      </Dynamics>")
        lines.append("      <Agent type=\"coordinator\" dmLabel=\"minkowski\" debug=\"true\">")
        for label in robot_labels:
            lines.append(f"        <Member label=\"{label}\"/>")
        lines.append("        <StepFunction type=\"plangrouppath\" debug=\"true\"/>")
        lines.append("      </Agent>")
    else:
        lines.append("      <Agent type=\"coordinator\" dmLabel=\"euclidean\">")
        for label in robot_labels:
            lines.append(f"        <Member label=\"{label}\"/>")
        lines.append("      </Agent>")
    lines.append("    </Robot>")
    lines.append("")

    # GroupTask - first instance
    group_task_label = f"{num_robots}-groupQuery"
    lines.append(f"    <GroupTask label=\"{group_task_label}\" group=\"{group_label}\">")

    for i in range(num_robots):
        robot = robots[i]
        robot_label = robot_labels[i]
        start = robot['start']  # [x, y, theta_radians]
        goal = robot['goal']    # [x, y, theta_radians]

        # CSpaceConstraint uses 3D for 2D planar robots: x y theta
        # PPL expects theta normalized to [-1, 1] (divided by pi)
        start_theta = normalize_angle_for_ppl(start[2])
        goal_theta = normalize_angle_for_ppl(goal[2])
        start_str = f"{start[0]} {start[1]} {start_theta}"
        goal_str = f"{goal[0]} {goal[1]} {goal_theta}"

        lines.append(f"      <Task label=\"query-{i+1}\" robot=\"{robot_label}\">")
        lines.append("        <StartConstraints>")
        lines.append(f"          <CSpaceConstraint point=\"{start_str}\" />")
        lines.append("        </StartConstraints>")
        lines.append("        <GoalConstraints>")
        lines.append(f"          <CSpaceConstraint point=\"{goal_str}\" />")
        lines.append("        </GoalConstraints>")
        lines.append("      </Task>")
    lines.append("    </GroupTask>")
    lines.append("")

    # Decomposition section for WoDaSH
    lines.append("    <Decomposition label=\"main\" taskLabel=\"groupTask\" coordinator=\"coordinator\">")
    lines.append("      <SemanticTask label=\"query0\" decomposable=\"false\" fixedAllocation=\"true\">")
    lines.append(f"        <GroupTask label=\"{group_task_label}\" group=\"{group_label}\">")

    for i in range(num_robots):
        robot = robots[i]
        robot_label = robot_labels[i]
        start = robot['start']
        goal = robot['goal']

        # CSpaceConstraint uses 3D for 2D planar robots: x y theta
        # PPL expects theta normalized to [-1, 1] (divided by pi)
        start_theta = normalize_angle_for_ppl(start[2])
        goal_theta = normalize_angle_for_ppl(goal[2])
        start_str = f"{start[0]} {start[1]} {start_theta}"
        goal_str = f"{goal[0]} {goal[1]} {goal_theta}"

        lines.append(f"          <Task label=\"query-{i+1}\" robot=\"{robot_label}\">")
        lines.append("            <StartConstraints>")
        lines.append(f"              <CSpaceConstraint point=\"{start_str}\" />")
        lines.append("            </StartConstraints>")
        lines.append("            <GoalConstraints>")
        lines.append(f"              <CSpaceConstraint point=\"{goal_str}\" />")
        lines.append("            </GoalConstraints>")
        lines.append("          </Task>")
    lines.append("        </GroupTask>")
    lines.append("      </SemanticTask>")
    lines.append("    </Decomposition>")
    lines.append("  </Problem>")
    lines.append("")

    # Library section
    if kinodynamic:
        lines.extend(get_kinodynamic_library_section())
    else:
        lines.extend(get_library_section())
    lines.append("")

    # TMPLibrary section for WoDaSH
    if kinodynamic:
        lines.extend(get_kinodynamic_tmp_library_section(scenario_name))
    else:
        lines.extend(get_tmp_library_section(scenario_name))

    lines.append("</MotionPlanning>")

    # Write file
    with open(output_path, 'w') as f:
        f.write('\n'.join(lines))

    print(f"Generated template: {output_path}")


def get_library_section():
    """
    Get the Library section for WoDaSH template.xml.
    Based on GridMaze/2/template-ppl.xml.

    Returns:
        list: Lines of XML for the Library section
    """
    return [
        "  <!-- Set available algorithms and parameters. -->",
        "  <Library>",
        "",
        "    <DistanceMetrics>",
        "      <Euclidean label=\"euclidean\"/>",
        "    </DistanceMetrics>",
        "",
        "    <ValidityCheckers>",
        "      <AlwaysTrueValidity label=\"alwaysTrue\"/>",
        "      <CollisionDetection label=\"rapid\" method=\"RAPID\"/>",
        "      <CollisionDetection label=\"pqp\" method=\"PQP\"/>",
        "      <CollisionDetection label=\"pqp_solid\" method=\"PQP_SOLID\" interRobotCollision=\"true\"/>",
        "      <CollisionDetection label=\"bounding_spheres\" method=\"BoundingSpheres\" interRobotCollision=\"true\"/>",
        "      <ComposeCollision label=\"bounding_pqp\" operator=\"and\">",
        "        <CollisionDetector label=\"bounding_spheres\"/>",
        "        <CollisionDetector label=\"pqp_solid\"/>",
        "      </ComposeCollision>",
        "    </ValidityCheckers>",
        "",
        "    <NeighborhoodFinders>",
        "      <BruteForceNF label=\"BFNFAll\" dmLabel=\"euclidean\" unconnected=\"false\" k=\"0\"/>",
        "      <BruteForceNF label=\"Nearest\" dmLabel=\"euclidean\" unconnected=\"false\" k=\"1\"/>",
        "      <BruteForceNF label=\"BFNF\" dmLabel=\"euclidean\" unconnected=\"false\" k=\"2\"/>",
        "      <OptimalNF label=\"OptimalK\" dmLabel=\"euclidean\" unconnected=\"false\" nfType=\"k\"/>",
        "      <RadiusNF label=\"LargeRadiusNF\" dmLabel=\"euclidean\" radius=\"5\"/>",
        "      <RadiusNF label=\"SmallRadiusNF\" dmLabel=\"euclidean\" radius=\"1\"/>",
        "      <OptimalNF label=\"OptimalRadius\" dmLabel=\"euclidean\" nfType=\"radius\"/>",
        "    </NeighborhoodFinders>",
        "",
        "    <Samplers>",
        "      <UniformRandomSampler label=\"UniformRandom\" vcLabel=\"alwaysTrue\"/>",
        "      <UniformRandomSampler label=\"UniformRandomFree\" vcLabel=\"pqp_solid\"/>",
        "    </Samplers>",
        "",
        "    <LocalPlanners>",
        "      <StraightLine label=\"sl\" binaryEvaluation=\"true\" vcLabel=\"pqp_solid\"/>",
        "      <StraightLine label=\"slAlwaysTrue\" binaryEvaluation=\"true\" vcLabel=\"alwaysTrue\"/>",
        "    </LocalPlanners>",
        "",
        "    <Extenders>",
        "      <BasicExtender label=\"BERO\" debug=\"false\" dmLabel=\"euclidean\"",
        "        vcLabel=\"pqp_solid\" maxDist=\"4.\" minDist=\".01\"/>",
        "      <BasicExtender label=\"LongBERO\" debug=\"false\" dmLabel=\"euclidean\"",
        "        vcLabel=\"pqp_solid\" maxDist=\"10.\" minDist=\".01\"/>",
        "      <DiscreteExtender label=\"DiscreteExtender\" lp=\"sl\"/>",
        "    </Extenders>",
        "",
        "    <Connectors>",
        "      <NeighborhoodConnector label=\"Closest\" nfLabel=\"BFNF\" lpLabel=\"sl\"",
        "        checkIfSameCC=\"false\" debug=\"false\"/>",
        "      <NeighborhoodConnector label=\"ClosestAll\" nfLabel=\"BFNFAll\" lpLabel=\"sl\"",
        "        checkIfSameCC=\"false\" debug=\"false\"/>",
        "      <NeighborhoodConnector label=\"ClosestAlwaysTrue\" nfLabel=\"BFNF\" lpLabel=\"slAlwaysTrue\"",
        "        checkIfSameCC=\"false\" debug=\"false\"/>",
        "      <NeighborhoodConnector label=\"OptimalClosest\" debug=\"false\"",
        "        nfLabel=\"OptimalK\" lpLabel=\"sl\" checkIfSameCC=\"false\"/>",
        "      <NeighborhoodConnector label=\"OptimalConnect\" nfLabel=\"OptimalRadius\" lpLabel=\"sl\"",
        "        checkIfSameCC=\"false\"/>",
        "    </Connectors>",
        "",
        "    <Metrics>",
        "      <NumNodesMetric label=\"NumNodes\"/>",
        "    </Metrics>",
        "",
        "    <MapEvaluators>",
        "      <TimeEvaluator label=\"SmallTimeEval\" timeout=\"2\"/>",
        "      <TimeEvaluator label=\"TimeEval\" timeout=\"10\"/>",
        "      <TimeEvaluator label=\"BigTimeEval\" timeout=\"600\"/>",
        "      <ConditionalEvaluator label=\"NodesEval\" metric_method=\"NumNodes\"",
        "        value=\"10\" operator=\">=\"/>",
        "      <ConditionalEvaluator label=\"BigNodesEval\" metric_method=\"NumNodes\"",
        "        value=\"10000\" operator=\">=\"/>",
        "      <ConditionalEvaluator label=\"EdgesEval\" metric_method=\"NumEdges\"",
        "        value=\"1000\" operator=\">\"/>",
        "",
        "      <QueryMethod label=\"SharedIndividualQuery\" debug=\"false\" safeIntervalToolLabel=\"SI\"/>",
        "      <QueryMethod label=\"SPARSQuery\" debug=\"false\" graphSearchAlg=\"dijkstras\" safeIntervalToolLabel=\"SI\"/>",
        "      <QueryMethod label=\"IndividualQuery\" debug=\"false\" safeIntervalToolLabel=\"SI\"/>",
        "      <GroupQuery label=\"CompositeQuery\" debug=\"false\"/>",
        "      <GroupDecoupledQuery label=\"DecoupledQuery\" queryLabel=\"IndividualQuery\"",
        "        debug=\"false\" ignoreOtherRobots=\"false\"/>",
        "",
        "      <ComposeEvaluator label=\"BoundedIndividualQuery\" operator=\"or\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"IndividualQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"BigBoundedIndividualQuery\" operator=\"and\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"IndividualQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"SmallBoundedCompositeQuery\" operator=\"or\">",
        "        <Evaluator label=\"SmallTimeEval\"/>",
        "        <Evaluator label=\"CompositeQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"BoundedCompositeQuery\" operator=\"or\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"CompositeQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"BoundedDecoupledQuery\" operator=\"or\">",
        "        <Evaluator label=\"TimeEval\"/>",
        "        <Evaluator label=\"DecoupledQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <SIPPMethod label=\"SIPP\"/>",
        "",
        "      <CBSQuery label=\"CBSQuery\" queryLabel=\"SIPP\" debug=\"false\" vcLabel=\"pqp_solid\" nodeLimit=\"99999999\"/>",
        "",
        "      <ComposeEvaluator label=\"BoundedCBSQuery\" operator=\"or\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"CBSQuery\"/>",
        "      </ComposeEvaluator>",
        "    </MapEvaluators>",
        "",
        "    <MPStrategies>",
        "      <!-- CompositePRM -->",
        "      <GroupPRM label=\"Composite-PRM\" debug=\"true\">",
        "        <Sampler label=\"UniformRandomFree\" number=\"10\" attempts=\"1\"/>",
        "        <Connector label=\"Closest\"/>",
        "        <Evaluator label=\"BoundedCompositeQuery\"/>",
        "      </GroupPRM>",
        "",
        "      <!-- CompositeRRT -->",
        "      <GroupRRTStrategy label=\"Composite-RRT\" debug=\"false\"",
        "        querySampler=\"UniformRandomFree\" samplerLabel=\"UniformRandom\"",
        "        nfLabel=\"Nearest\" extenderLabel=\"BERO\" restrictGrowth=\"true\"",
        "        growGoals=\"false\" growthFocus=\"0.01\" m=\"1\"",
        "        goalDmLabel=\"euclidean\" goalThreshold=\"100\">",
        "        <Evaluator label=\"SmallBoundedCompositeQuery\"/>",
        "      </GroupRRTStrategy>",
        "",
        "      <!-- Composite-DR-RRT -->",
        "      <CDRRRTLite label=\"Composite-DR-RRT\" debug=\"false\" querySampler=\"UniformRandomFree\"",
        "        samplerLabel=\"UniformRandom\" nfLabel=\"Nearest\" extenderLabel=\"BERO\"",
        "        growGoals=\"false\" regionFactor=\"2.5\" restrictGrowth=\"true\" maxSampleFails=\"500\"",
        "        penetration=\"1\" goalDmLabel=\"euclidean\" goalThreshold=\"0.00000001\">",
        "        <Evaluator label=\"BoundedCompositeQuery\"/>",
        "      </CDRRRTLite>",
        "",
        "      <EvaluateMapStrategy label=\"CompositeEval\">",
        "        <Evaluator label=\"SIPP\"/>",
        "      </EvaluateMapStrategy>",
        "    </MPStrategies>",
        "",
        "    <MPTools>",
        "      <SafeIntervalTool label=\"SI\" vcLabel=\"bounding_spheres\"/>",
        "    </MPTools>",
        "",
        "    <Solver mpStrategyLabel=\"Composite-RRT\" seed=\"@@seed@@\"",
        "      baseFilename=\"@@planner@@\" vizmoDebug=\"false\"/>",
        "",
        "  </Library>",
    ]


def get_kinodynamic_library_section():
    """
    Get the Library section for kinodynamic K-WoDaSH template.xml.
    Based on KinoWarehouse/2/template-ppl.xml.

    Returns:
        list: Lines of XML for the kinodynamic Library section
    """
    return [
        "  <!-- Set available algorithms and parameters. -->",
        "  <Library>",
        "",
        "    <DistanceMetrics>",
        "      <Euclidean label=\"euclidean\"/>",
        "      <StateDistance label=\"stateEuclidean\"/>",
        "    </DistanceMetrics>",
        "",
        "    <ValidityCheckers>",
        "      <AlwaysTrueValidity label=\"alwaysTrue\"/>",
        "      <CollisionDetection label=\"rapid\" method=\"RAPID\"/>",
        "      <CollisionDetection label=\"pqp\" method=\"PQP\"/>",
        "      <CollisionDetection label=\"pqp_solid\" method=\"PQP_SOLID\" interRobotCollision=\"true\"/>",
        "      <CollisionDetection label=\"bounding_spheres\" method=\"BoundingSpheres\" interRobotCollision=\"true\"/>",
        "      <ComposeCollision label=\"bounding_pqp\" operator=\"and\">",
        "        <CollisionDetector label=\"bounding_spheres\"/>",
        "        <CollisionDetector label=\"pqp_solid\"/>",
        "      </ComposeCollision>",
        "    </ValidityCheckers>",
        "",
        "    <NeighborhoodFinders>",
        "      <BruteForceNF label=\"BFNFAll\" dmLabel=\"euclidean\" unconnected=\"false\" k=\"0\"/>",
        "      <BruteForceNF label=\"Nearest\" dmLabel=\"stateEuclidean\" unconnected=\"false\" k=\"1\"/>",
        "      <BruteForceNF label=\"BFNF\" dmLabel=\"euclidean\" unconnected=\"false\" k=\"2\"/>",
        "      <OptimalNF label=\"OptimalK\" dmLabel=\"euclidean\" unconnected=\"false\" nfType=\"k\"/>",
        "      <RadiusNF label=\"LargeRadiusNF\" dmLabel=\"euclidean\" radius=\"5\"/>",
        "      <RadiusNF label=\"SmallRadiusNF\" dmLabel=\"euclidean\" radius=\"1\"/>",
        "      <OptimalNF label=\"OptimalRadius\" dmLabel=\"euclidean\" nfType=\"radius\"/>",
        "    </NeighborhoodFinders>",
        "",
        "    <Samplers>",
        "      <UniformRandomSampler label=\"UniformRandom\" vcLabel=\"alwaysTrue\"/>",
        "      <UniformRandomSampler label=\"UniformRandomFree\" vcLabel=\"pqp_solid\"/>",
        "      <StateSampler label=\"StateSampler\" vcLabel=\"alwaysTrue\"/>",
        "      <StateSampler label=\"StateSamplerFree\" vcLabel=\"pqp_solid\"/>",
        "    </Samplers>",
        "",
        "    <LocalPlanners>",
        "      <StraightLine label=\"sl\" binaryEvaluation=\"true\" vcLabel=\"pqp_solid\"/>",
        "      <StraightLine label=\"slAlwaysTrue\" binaryEvaluation=\"true\" vcLabel=\"alwaysTrue\"/>",
        "    </LocalPlanners>",
        "",
        "    <Extenders>",
        "      <BasicExtender label=\"BERO\" debug=\"false\" dmLabel=\"euclidean\"",
        "        vcLabel=\"pqp_solid\" maxDist=\"4.\" minDist=\".01\"/>",
        "      <BasicExtender label=\"LongBERO\" debug=\"false\" dmLabel=\"euclidean\"",
        "        vcLabel=\"pqp_solid\" maxDist=\"10.\" minDist=\".01\"/>",
        "      <DiscreteExtender label=\"DiscreteExtender\" lp=\"sl\"/>",
        "      <NewKinodynamicExtender label=\"KinoSmall\" debug=\"false\"",
        "        dmLabel=\"stateEuclidean\" vcLabel=\"pqp_solid\" integratorLabel=\"rk4\" fixed=\"false\"",
        "        numSteps=\"7\" minDist=\".01\" best=\"true\" numSampledControls=\"50\" keepLastCollision=\"false\" />",
        "      <NewKinodynamicExtender label=\"KinoExtender\" debug=\"false\"",
        "        dmLabel=\"stateEuclidean\" vcLabel=\"pqp_solid\" integratorLabel=\"rk4\" fixed=\"false\"",
        "        numSteps=\"50\" minDist=\".01\" best=\"true\" numSampledControls=\"50\" keepLastCollision=\"false\" />",
        "      <NewKinodynamicExtender label=\"KinoExtenderRandom\" debug=\"false\"",
        "        dmLabel=\"stateEuclidean\" vcLabel=\"pqp_solid\" integratorLabel=\"rk4\" fixed=\"false\"",
        "        numSteps=\"50\" minDist=\".01\" best=\"false\" numSampledControls=\"50\" keepLastCollision=\"false\"/>",
        "      <MixExtender label=\"KinoMix\">",
        "        <Extender label=\"KinoExtender\" probability=\"0.99\"/>",
        "        <Extender label=\"KinoExtenderRandom\" probability=\"0.01\"/>",
        "      </MixExtender>",
        "    </Extenders>",
        "",
        "    <Connectors>",
        "      <NeighborhoodConnector label=\"Closest\" nfLabel=\"BFNF\" lpLabel=\"sl\"",
        "        checkIfSameCC=\"false\" debug=\"false\"/>",
        "      <NeighborhoodConnector label=\"ClosestAll\" nfLabel=\"BFNFAll\" lpLabel=\"sl\"",
        "        checkIfSameCC=\"false\" debug=\"false\"/>",
        "      <NeighborhoodConnector label=\"ClosestAlwaysTrue\" nfLabel=\"BFNF\" lpLabel=\"slAlwaysTrue\"",
        "        checkIfSameCC=\"false\" debug=\"false\"/>",
        "      <NeighborhoodConnector label=\"OptimalClosest\" debug=\"false\"",
        "        nfLabel=\"OptimalK\" lpLabel=\"sl\" checkIfSameCC=\"false\"/>",
        "      <NeighborhoodConnector label=\"OptimalConnect\" nfLabel=\"OptimalRadius\" lpLabel=\"sl\"",
        "        checkIfSameCC=\"false\"/>",
        "    </Connectors>",
        "",
        "    <Metrics>",
        "      <NumNodesMetric label=\"NumNodes\"/>",
        "    </Metrics>",
        "",
        "    <MapEvaluators>",
        "      <TimeEvaluator label=\"SmallTimeEval\" timeout=\"60\"/>",
        "      <TimeEvaluator label=\"TimeEval\" timeout=\"10\"/>",
        "      <TimeEvaluator label=\"BigTimeEval\" timeout=\"600\"/>",
        "      <ConditionalEvaluator label=\"NodesEval\" metric_method=\"NumNodes\"",
        "        value=\"10\" operator=\">=\"/>",
        "      <ConditionalEvaluator label=\"BigNodesEval\" metric_method=\"NumNodes\"",
        "        value=\"10000\" operator=\">=\"/>",
        "      <ConditionalEvaluator label=\"EdgesEval\" metric_method=\"NumEdges\"",
        "        value=\"1000\" operator=\">\"/>",
        "",
        "      <QueryMethod label=\"SharedIndividualQuery\" debug=\"false\" safeIntervalToolLabel=\"SI\"/>",
        "      <QueryMethod label=\"SPARSQuery\" debug=\"false\" graphSearchAlg=\"dijkstras\" safeIntervalToolLabel=\"SI\"/>",
        "      <QueryMethod label=\"IndividualQuery\" debug=\"false\" safeIntervalToolLabel=\"SI\"/>",
        "      <GroupQuery label=\"CompositeQuery\" debug=\"false\"/>",
        "      <GroupDecoupledQuery label=\"DecoupledQuery\" queryLabel=\"IndividualQuery\"",
        "        debug=\"false\" ignoreOtherRobots=\"false\"/>",
        "",
        "      <ComposeEvaluator label=\"BoundedIndividualQuery\" operator=\"or\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"IndividualQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"BigBoundedIndividualQuery\" operator=\"and\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"IndividualQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"SmallBoundedCompositeQuery\" operator=\"or\">",
        "        <Evaluator label=\"SmallTimeEval\"/>",
        "        <Evaluator label=\"CompositeQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"BoundedCompositeQuery\" operator=\"or\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"CompositeQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <ComposeEvaluator label=\"BoundedDecoupledQuery\" operator=\"or\">",
        "        <Evaluator label=\"TimeEval\"/>",
        "        <Evaluator label=\"DecoupledQuery\"/>",
        "      </ComposeEvaluator>",
        "",
        "      <SIPPMethod label=\"SIPP\"/>",
        "",
        "      <CBSQuery label=\"CBSQuery\" queryLabel=\"SIPP\" debug=\"false\" vcLabel=\"pqp_solid\" nodeLimit=\"99999999\"/>",
        "",
        "      <ComposeEvaluator label=\"BoundedCBSQuery\" operator=\"or\">",
        "        <Evaluator label=\"BigTimeEval\"/>",
        "        <Evaluator label=\"CBSQuery\"/>",
        "      </ComposeEvaluator>",
        "    </MapEvaluators>",
        "",
        "    <MPStrategies>",
        "      <!-- CompositePRM -->",
        "      <GroupPRM label=\"Composite-PRM\" debug=\"true\">",
        "        <Sampler label=\"UniformRandomFree\" number=\"10\" attempts=\"1\"/>",
        "        <Connector label=\"Closest\"/>",
        "        <Evaluator label=\"BoundedCompositeQuery\"/>",
        "      </GroupPRM>",
        "",
        "      <!-- CompositeRRT -->",
        "      <GroupRRTStrategy label=\"Composite-RRT\" debug=\"false\"",
        "        querySampler=\"StateSamplerFree\" samplerLabel=\"StateSampler\"",
        "        nfLabel=\"Nearest\" extenderLabel=\"KinoSmall\" restrictGrowth=\"true\"",
        "        growGoals=\"false\" growthFocus=\"0.01\" m=\"1\"",
        "        goalDmLabel=\"euclidean\" goalThreshold=\"100\">",
        "        <Evaluator label=\"SmallBoundedCompositeQuery\"/>",
        "      </GroupRRTStrategy>",
        "",
        "      <!-- Composite-DR-RRT -->",
        "      <CDRRRTLite label=\"Composite-DR-RRT\" debug=\"false\" querySampler=\"StateSamplerFree\"",
        "        samplerLabel=\"StateSampler\" nfLabel=\"Nearest\" extenderLabel=\"KinoMix\"",
        "        growGoals=\"false\" regionFactor=\"4.5\" restrictGrowth=\"true\" maxSampleFails=\"8000\"",
        "        penetration=\"1\" goalDmLabel=\"euclidean\" goalThreshold=\"0.00000001\">",
        "        <Evaluator label=\"CompositeQuery\"/>",
        "      </CDRRRTLite>",
        "",
        "      <EvaluateMapStrategy label=\"CompositeEval\">",
        "        <Evaluator label=\"SIPP\"/>",
        "      </EvaluateMapStrategy>",
        "    </MPStrategies>",
        "",
        "    <MPTools>",
        "      <SafeIntervalTool label=\"SI\" vcLabel=\"bounding_spheres\"/>",
        "    </MPTools>",
        "",
        "    <Solver mpStrategyLabel=\"Composite-RRT\" seed=\"@@seed@@\"",
        "      baseFilename=\"@@planner@@\" vizmoDebug=\"false\"/>",
        "",
        "  </Library>",
    ]


def get_tmp_library_section(scenario_name):
    """
    Get the TMPLibrary section for WoDaSH template.xml.

    Args:
        scenario_name: Name of the scenario for skeleton file path

    Returns:
        list: Lines of XML for the TMPLibrary section
    """
    return [
        "  <TMPLibrary>",
        "    <TMPStrategies>",
        "      <SimpleMotionMethod label=\"Motion\" teLabel=\"simple\"/>",
        "      <WoDaSH label=\"WoDaSH\"",
        "        debug=\"false\"",
        "        teLabel=\"SubmodeQuery\"",
        "        drStrat=\"Composite-DR-RRT\"",
        "        trajStrat=\"Composite-RRT\"",
        "        sampler=\"UniformRandomFree\"",
        "        skeletonType=\"ma\"",
        "        skeletonIO=\"read\"",
        f"        skeletonFile=\"@@envdir@@/{scenario_name}/skeletons/ma_skeleton.graph\"",
        "        decompositionLabel=\"fine\"",
        "        split=\"0\"",
        "        intermediateFactor=\"10\"",
        "        regionFactor=\"2.5\"",
        "        penetration=\"1\"",
        "        edgeQuery=\"CompositeQuery\"",
        "        groundedHypergraph=\"GroundedHypergraph\"",
        "        motionEvaluator=\"ScheduledCBS\"",
        "        interleave=\"true\"",
        "        vcLabel=\"bounding_pqp\"",
        "      />",
        "    </TMPStrategies>",
        "",
        "    <PoIPlacementMethods>",
        "    </PoIPlacementMethods>",
        "",
        "    <TaskEvaluators>",
        "      <SimpleMotionEvaluator label=\"simple\"/>",
        "      <SubmodeQuery label=\"SubmodeQuery\"",
        "        mgLabel=\"ModeGraph\"",
        "        ghLabel=\"GroundedHypergraph\"",
        "        debug=\"false\"",
        "        writeHypergraph=\"false\"/>",
        "      <ScheduledCBS label=\"ScheduledCBS\"",
        "        vcLabel=\"pqp_solid\"",
        "        queryStrategy=\"CompositeEval\"",
        "        queryLabel=\"SIPP\"",
        "        sqLabel=\"SubmodeQuery\"",
        "        writeSolution=\"true\"",
        "        debug=\"false\"",
        "        bypass=\"true\"",
        "        workspace=\"true\"/>",
        "    </TaskEvaluators>",
        "",
        "    <TaskDecomposers>",
        "    </TaskDecomposers>",
        "",
        "    <TaskAllocators>",
        "    </TaskAllocators>",
        "",
        "    <StateGraphs>",
        "      <GroundedHypergraph label=\"GroundedHypergraph\" queryStrategy=\"CompositeQueryStrategy\" debug=\"false\"/>",
        "    </StateGraphs>",
        "",
        "    <Solver tmpStrategyLabel=\"@@planner@@\" baseFilename=\"@@base@@\"/>",
        "  </TMPLibrary>",
    ]


def get_kinodynamic_tmp_library_section(scenario_name):
    """
    Get the TMPLibrary section for kinodynamic K-WoDaSH template.xml.
    Based on KinoWarehouse/2/template-ppl.xml.

    Args:
        scenario_name: Name of the scenario for skeleton file path

    Returns:
        list: Lines of XML for the kinodynamic TMPLibrary section
    """
    return [
        "  <TMPLibrary>",
        "    <TMPStrategies>",
        "      <SimpleMotionMethod label=\"Motion\" teLabel=\"simple\"/>",
        "      <WoDaSH label=\"K-WoDaSH\"",
        "        debug=\"false\"",
        "        teLabel=\"SubmodeQuery\"",
        "        drStrat=\"Composite-DR-RRT\"",
        "        trajStrat=\"Composite-RRT\"",
        "        sampler=\"StateSamplerFree\"",
        "        skeletonType=\"ma\"",
        "        skeletonIO=\"read\"",
        f"        skeletonFile=\"@@envdir@@/{scenario_name}/skeletons/ma_skeleton.graph\"",
        "        decompositionLabel=\"fine\"",
        "        split=\"0\"",
        "        intermediateFactor=\"3\"",
        "        regionFactor=\"4.5\"",
        "        penetration=\"1\"",
        "        edgeQuery=\"CompositeQuery\"",
        "        groundedHypergraph=\"GroundedHypergraph\"",
        "        motionEvaluator=\"ScheduledCBS\"",
        "        interleave=\"true\"",
        "        vcLabel=\"bounding_pqp\"",
        "        goalBoundary=\"1.0\"",
        "      />",
        "    </TMPStrategies>",
        "",
        "    <PoIPlacementMethods>",
        "    </PoIPlacementMethods>",
        "",
        "    <TaskEvaluators>",
        "      <SimpleMotionEvaluator label=\"simple\"/>",
        "      <SubmodeQuery label=\"SubmodeQuery\"",
        "        mgLabel=\"ModeGraph\"",
        "        ghLabel=\"GroundedHypergraph\"",
        "        debug=\"false\"",
        "        writeHypergraph=\"false\"/>",
        "      <ScheduledCBS label=\"ScheduledCBS\"",
        "        vcLabel=\"pqp_solid\"",
        "        queryStrategy=\"CompositeEval\"",
        "        queryLabel=\"SIPP\"",
        "        sqLabel=\"SubmodeQuery\"",
        "        writeSolution=\"true\"",
        "        debug=\"false\"",
        "        bypass=\"true\"",
        "        workspace=\"true\"/>",
        "    </TaskEvaluators>",
        "",
        "    <TaskDecomposers>",
        "    </TaskDecomposers>",
        "",
        "    <TaskAllocators>",
        "    </TaskAllocators>",
        "",
        "    <StateGraphs>",
        "      <GroundedHypergraph label=\"GroundedHypergraph\" queryStrategy=\"CompositeQueryStrategy\" debug=\"false\"/>",
        "    </StateGraphs>",
        "",
        "    <Solver tmpStrategyLabel=\"@@planner@@\" baseFilename=\"@@base@@\"/>",
        "  </TMPLibrary>",
    ]


def create_directory_structure(base_output_dir, scenario_name):
    """
    Create the PPL directory structure for a scenario.

    The structure matches what RunExperiment.ipynb expects:
        base_output_dir/
        ├── envs/
        │   └── <scenario_name>/
        │       ├── <scenario_name>.env
        │       ├── robot.robot
        │       ├── objs/
        │       └── skeletons/
        └── <scenario_name>/
            └── 2/
                └── template-ppl.xml

    Args:
        base_output_dir: Base PPL experiments directory
        scenario_name: Name of the scenario

    Returns:
        dict: Paths to key directories
    """
    base_dir = Path(base_output_dir)
    scenario_dir = base_dir / scenario_name
    envs_dir = base_dir / "envs" / scenario_name  # envs at base level
    objs_dir = envs_dir / "objs"
    skeletons_dir = envs_dir / "skeletons"

    # Create directories
    scenario_dir.mkdir(parents=True, exist_ok=True)
    envs_dir.mkdir(parents=True, exist_ok=True)
    objs_dir.mkdir(parents=True, exist_ok=True)
    skeletons_dir.mkdir(parents=True, exist_ok=True)

    return {
        'scenario_dir': scenario_dir,
        'envs_dir': envs_dir,
        'objs_dir': objs_dir,
        'skeletons_dir': skeletons_dir,
    }


def create_geometry_files(data, objs_dir):
    """
    Create all needed geometry files for obstacles in the scenario.

    Args:
        data: Parsed YAML data
        objs_dir: Directory where geometry files should be created

    Returns:
        set: Set of unique geometry filenames created
    """
    env = data['environment']
    obstacles = env.get('obstacles', [])

    created_geometries = set()

    for obs in obstacles:
        if obs['type'] != 'box':
            continue

        size = obs['size']
        width = size[0]
        height = size[1]

        geom_filename = get_geometry_filename(width, height)

        if geom_filename not in created_geometries:
            output_path = objs_dir / geom_filename
            create_geometry_file(width, height, str(output_path))
            created_geometries.add(geom_filename)

    return created_geometries


def create_icreate_geometry(objs_dir, ppl_base_dir):
    """
    Create the icreate.obj robot geometry file.

    Args:
        objs_dir: Directory where icreate.obj should be created
        ppl_base_dir: Base PPL experiments directory (unused, kept for compatibility)
    """
    icreate_path = objs_dir / "icreate.obj"
    # Remove existing file/symlink to ensure we use the correct radius
    if icreate_path.exists() or icreate_path.is_symlink():
        icreate_path.unlink()
    generate_icreate_obj(str(icreate_path), radius=ROBOT_RADIUS)


def convert_scenario(yaml_path, output_dir, robot_counts, scenario_name=None, ppl_base_dir=None,
                     kinodynamic=False):
    """
    Convert a single scenario from YAML to PPL format.

    Args:
        yaml_path: Path to input YAML file
        output_dir: Base output directory for PPL experiments
        robot_counts: List of robot counts to generate (e.g., [2, 4, 6, 8])
        scenario_name: Optional scenario name (default: infer from filename)
        ppl_base_dir: Optional PPL base directory for finding icreate.obj
        kinodynamic: If True, generate kinodynamic template-ppl-k.xml, skipping existing files
    """
    print(f"\n{'='*60}")
    print(f"Converting: {yaml_path}")
    if kinodynamic:
        print(f"Mode: kinodynamic (K-WoDaSH)")
    print(f"{'='*60}")

    # Load YAML
    data = load_syclop_yaml(yaml_path)

    # Determine scenario name
    if scenario_name is None:
        scenario_name = Path(yaml_path).stem

    print(f"Scenario name: {scenario_name}")

    # Validate robot counts
    num_available_robots = len(data.get('robots', []))
    valid_counts = [count for count in robot_counts if count <= num_available_robots]

    if len(valid_counts) < len(robot_counts):
        skipped = [count for count in robot_counts if count > num_available_robots]
        print(f"Warning: Skipping robot counts {skipped} (only {num_available_robots} robots available)")

    if not valid_counts:
        print(f"Error: No valid robot counts for scenario (has {num_available_robots} robots)")
        return

    # Create directory structure
    dirs = create_directory_structure(output_dir, scenario_name)

    if kinodynamic:
        # In kinodynamic mode, only generate shared resources if they don't exist
        env_file = dirs['envs_dir'] / f"{scenario_name}.env"
        if not env_file.exists():
            print(f"\nCreating geometry files...")
            create_geometry_files(data, dirs['objs_dir'])
            if ppl_base_dir is None:
                ppl_base_dir = output_dir
            create_icreate_geometry(dirs['objs_dir'], ppl_base_dir)
            print(f"\nGenerating .env files...")
            generate_env_file(data, str(env_file), 0, scenario_name, include_robots=False)
            max_count = max(valid_counts)
            vizmo_env_file = dirs['envs_dir'] / f"{scenario_name}-vizmo.env"
            generate_env_file(data, str(vizmo_env_file), max_count, scenario_name, include_robots=True)
            robot_file = dirs['envs_dir'] / "robot.robot"
            generate_robot_file(str(robot_file))
        else:
            print(f"\nShared resources already exist, skipping generation.")

        skeleton_file = dirs['skeletons_dir'] / "ma_skeleton.graph"
        if not skeleton_file.exists():
            print(f"Generating skeleton graph...")
            skeleton_vis = dirs['skeletons_dir'] / "ma_skeleton.png"
            try:
                generate_skeleton(
                    yaml_path=yaml_path,
                    output_path=str(skeleton_file),
                    clearance=0.0,
                    prune_length=1.0,
                    simplify_tolerance=0.5,
                    visualize=False,
                    vis_output=str(skeleton_vis)
                )
            except Exception as e:
                print(f"Warning: Failed to generate skeleton: {e}")
                print(f"K-WoDaSH may not work without a skeleton file.")

        # Generate template-ppl-k.xml for each robot count
        print(f"\nGenerating template-ppl-k.xml files for robot counts: {valid_counts}")
        for count in valid_counts:
            count_dir = dirs['scenario_dir'] / str(count)
            count_dir.mkdir(exist_ok=True)

            template_file = count_dir / "template-ppl-k.xml"
            generate_template_xml(data, str(template_file), count, scenario_name, "../envs",
                                  kinodynamic=True)
    else:
        # Create geometry files for obstacles
        print(f"\nCreating geometry files...")
        create_geometry_files(data, dirs['objs_dir'])

        # Create symlink to icreate.obj
        if ppl_base_dir is None:
            ppl_base_dir = output_dir
        create_icreate_geometry(dirs['objs_dir'], ppl_base_dir)

        # Generate both .env files:
        # - {scenario_name}.env: environment only (no robots)
        # - {scenario_name}-vizmo.env: environment + robots (for visualization)
        print(f"\nGenerating .env files...")

        # Plain .env (no robots)
        env_file = dirs['envs_dir'] / f"{scenario_name}.env"
        generate_env_file(data, str(env_file), 0, scenario_name, include_robots=False)

        # Vizmo .env (with robots)
        max_count = max(valid_counts)
        vizmo_env_file = dirs['envs_dir'] / f"{scenario_name}-vizmo.env"
        generate_env_file(data, str(vizmo_env_file), max_count, scenario_name, include_robots=True)

        # Generate .robot file
        print(f"Generating .robot file...")
        robot_file = dirs['envs_dir'] / "robot.robot"
        generate_robot_file(str(robot_file))

        # Generate skeleton graph for WoDaSH
        print(f"Generating skeleton graph...")
        skeleton_file = dirs['skeletons_dir'] / "ma_skeleton.graph"
        skeleton_vis = dirs['skeletons_dir'] / "ma_skeleton.png"
        try:
            generate_skeleton(
                yaml_path=yaml_path,
                output_path=str(skeleton_file),
                clearance=0.0,
                prune_length=1.0,
                simplify_tolerance=0.5,
                visualize=False,
                vis_output=str(skeleton_vis)
            )
        except Exception as e:
            print(f"Warning: Failed to generate skeleton: {e}")
            print(f"WoDaSH may not work without a skeleton file.")

        # Generate template-ppl.xml for each robot count (WoDaSH uses template-ppl.xml)
        print(f"\nGenerating template-ppl.xml files for robot counts: {valid_counts}")
        for count in valid_counts:
            count_dir = dirs['scenario_dir'] / str(count)
            count_dir.mkdir(exist_ok=True)

            template_file = count_dir / "template-ppl.xml"
            generate_template_xml(data, str(template_file), count, scenario_name, "../envs")

    template_name = "template-ppl-k.xml" if kinodynamic else "template-ppl.xml"
    print(f"\n{'='*60}")
    print(f"Conversion complete! ({template_name})")
    print(f"Output directory: {dirs['scenario_dir']}")
    print(f"{'='*60}\n")


def convert_scenario_dir(scenario_name, experiments_dir, output_dir, ppl_base_dir=None,
                         max_robots=None, kinodynamic=False):
    """
    Convert an entire scenario directory to PPL format.

    This handles the full multi-robot-syclop directory structure:
        experiments/
        ├── scenarios/<scenario_name>.yaml    (environment only)
        └── problems/<scenario_name>/
            ├── robots_2/seed_0.yaml, seed_1.yaml, ...
            ├── robots_4/...
            └── ...

    It generates (for each seed):
        - Environment files from scenarios/<name>.yaml (shared)
        - Skeleton from scenarios/<name>.yaml (shared)
        - Templates for each (seed, robot_count) combination

    Output structure:
        <output>/
        ├── envs/<scenario>/              # Shared environment
        │   ├── <scenario>.env
        │   ├── skeletons/
        │   └── ...
        ├── <scenario>_seed0/             # Seed 0 templates
        │   ├── 2/template-ppl.xml
        │   ├── 4/template-ppl.xml
        │   └── ...
        ├── <scenario>_seed1/             # Seed 1 templates
        │   └── ...

    Args:
        scenario_name: Name of the scenario (e.g., "random5_20x20")
        experiments_dir: Path to multi-robot-syclop/experiments directory
        output_dir: Base output directory for PPL experiments
        ppl_base_dir: Optional PPL base directory for finding icreate.obj
        max_robots: Optional maximum robot count to generate
        kinodynamic: If True, generate kinodynamic template-ppl-k.xml, skipping existing files
    """
    experiments_dir = Path(experiments_dir)
    output_dir = Path(output_dir)

    print(f"\n{'='*60}")
    print(f"Converting scenario: {scenario_name}")
    if kinodynamic:
        print(f"Mode: kinodynamic (K-WoDaSH)")
    print(f"{'='*60}")

    # Find scenario file (environment only)
    scenario_file = experiments_dir / "scenarios" / f"{scenario_name}.yaml"
    if not scenario_file.exists():
        print(f"Error: Scenario file not found: {scenario_file}")
        return False

    # Find problems directory
    problems_dir = experiments_dir / "problems" / scenario_name
    if not problems_dir.exists():
        print(f"Error: Problems directory not found: {problems_dir}")
        return False

    # Auto-detect available robot counts
    robot_count_dirs = list(problems_dir.glob("robots_*"))
    available_counts = []
    for d in robot_count_dirs:
        try:
            count = int(d.name.replace("robots_", ""))
            if max_robots is None or count <= max_robots:
                available_counts.append(count)
        except ValueError:
            continue
    available_counts.sort()  # Sort numerically

    if not available_counts:
        print(f"Error: No robot count directories found in {problems_dir}")
        return False

    print(f"Found robot counts: {available_counts}")

    # Auto-detect available seeds (from smallest robot count directory)
    first_robot_dir = problems_dir / f"robots_{available_counts[0]}"
    seed_files = sorted(first_robot_dir.glob("seed_*.yaml"))
    available_seeds = []
    for sf in seed_files:
        try:
            seed_num = int(sf.stem.replace("seed_", ""))
            available_seeds.append(seed_num)
        except ValueError:
            continue
    available_seeds.sort()

    if not available_seeds:
        print(f"Error: No seed files found in {first_robot_dir}")
        return False

    print(f"Found seeds: {available_seeds}")

    # Load scenario (environment only)
    print(f"\nLoading environment from: {scenario_file}")
    env_data = load_syclop_yaml(str(scenario_file))

    # Create directory structure for shared environment
    dirs = create_directory_structure(str(output_dir), scenario_name)

    if kinodynamic:
        # In kinodynamic mode, only generate shared resources if they don't exist
        env_file = dirs['envs_dir'] / f"{scenario_name}.env"
        if not env_file.exists():
            print(f"\nCreating geometry files...")
            create_geometry_files(env_data, dirs['objs_dir'])
            if ppl_base_dir is None:
                ppl_base_dir = str(output_dir)
            create_icreate_geometry(dirs['objs_dir'], ppl_base_dir)
            print(f"\nGenerating .env files...")
            generate_env_file(env_data, str(env_file), 0, scenario_name, include_robots=False)
            max_count = max(available_counts)
            max_problem_file = problems_dir / f"robots_{max_count}" / "seed_0.yaml"
            if max_problem_file.exists():
                max_problem_data = load_syclop_yaml(str(max_problem_file))
                vizmo_env_file = dirs['envs_dir'] / f"{scenario_name}-vizmo.env"
                generate_env_file(max_problem_data, str(vizmo_env_file), max_count, scenario_name, include_robots=True)
            robot_file = dirs['envs_dir'] / "robot.robot"
            generate_robot_file(str(robot_file))
        else:
            print(f"\nShared resources already exist, skipping generation.")

        skeleton_file = dirs['skeletons_dir'] / "ma_skeleton.graph"
        if not skeleton_file.exists():
            print(f"Generating skeleton graph...")
            skeleton_vis = dirs['skeletons_dir'] / "ma_skeleton.png"
            try:
                generate_skeleton(
                    yaml_path=str(scenario_file),
                    output_path=str(skeleton_file),
                    clearance=0.0,
                    prune_length=1.0,
                    simplify_tolerance=0.5,
                    visualize=False,
                    vis_output=str(skeleton_vis)
                )
            except Exception as e:
                print(f"Warning: Failed to generate skeleton: {e}")
                print(f"K-WoDaSH may not work without a skeleton file.")
    else:
        # Create geometry files for obstacles
        print(f"\nCreating geometry files...")
        create_geometry_files(env_data, dirs['objs_dir'])

        # Create symlink to icreate.obj
        if ppl_base_dir is None:
            ppl_base_dir = str(output_dir)
        create_icreate_geometry(dirs['objs_dir'], ppl_base_dir)

        # Generate .env files (environment only, no robots)
        print(f"\nGenerating .env files...")
        env_file = dirs['envs_dir'] / f"{scenario_name}.env"
        generate_env_file(env_data, str(env_file), 0, scenario_name, include_robots=False)

        # For vizmo, we'll use the max robot count from seed_0
        max_count = max(available_counts)
        max_problem_file = problems_dir / f"robots_{max_count}" / "seed_0.yaml"
        if max_problem_file.exists():
            max_problem_data = load_syclop_yaml(str(max_problem_file))
            vizmo_env_file = dirs['envs_dir'] / f"{scenario_name}-vizmo.env"
            generate_env_file(max_problem_data, str(vizmo_env_file), max_count, scenario_name, include_robots=True)

        # Generate .robot file
        print(f"Generating .robot file...")
        robot_file = dirs['envs_dir'] / "robot.robot"
        generate_robot_file(str(robot_file))

        # Generate skeleton graph
        print(f"Generating skeleton graph...")
        skeleton_file = dirs['skeletons_dir'] / "ma_skeleton.graph"
        skeleton_vis = dirs['skeletons_dir'] / "ma_skeleton.png"
        try:
            generate_skeleton(
                yaml_path=str(scenario_file),
                output_path=str(skeleton_file),
                clearance=0.0,
                prune_length=1.0,
                simplify_tolerance=0.5,
                visualize=False,
                vis_output=str(skeleton_vis)
            )
        except Exception as e:
            print(f"Warning: Failed to generate skeleton: {e}")
            print(f"WoDaSH may not work without a skeleton file.")

    # Generate templates for each seed and robot count
    template_name = "template-ppl-k.xml" if kinodynamic else "template-ppl.xml"
    print(f"\nGenerating {template_name} files for {len(available_seeds)} seeds x {len(available_counts)} robot counts...")

    for seed in available_seeds:
        # Create seed-specific scenario directory
        seed_scenario_name = f"{scenario_name}_seed{seed}"
        seed_scenario_dir = output_dir / seed_scenario_name
        seed_scenario_dir.mkdir(parents=True, exist_ok=True)

        for count in available_counts:
            # Find the seed file for this robot count
            problem_file = problems_dir / f"robots_{count}" / f"seed_{seed}.yaml"
            if not problem_file.exists():
                print(f"  Warning: {problem_file.name} not found for robots_{count}, skipping")
                continue

            # Load problem data (has robot positions)
            problem_data = load_syclop_yaml(str(problem_file))

            count_dir = seed_scenario_dir / str(count)
            count_dir.mkdir(exist_ok=True)

            template_file = count_dir / template_name
            # Templates reference the shared environment: envs/<scenario_name>/
            generate_template_xml(problem_data, str(template_file), count, scenario_name, "../envs",
                                  kinodynamic=kinodynamic)

        print(f"  Generated {template_name} for seed {seed}")

    print(f"\n{'='*60}")
    print(f"Conversion complete! ({template_name})")
    print(f"Environment files: {dirs['envs_dir']}")
    print(f"Template directories: {scenario_name}_seed0/, {scenario_name}_seed1/, ...")
    print(f"{'='*60}\n")

    return True


def parse_robot_counts(robot_counts_arg, max_robots_arg):
    """
    Parse robot counts from command line arguments.

    Args:
        robot_counts_arg: Comma-separated robot counts (e.g., "2,4,6,8")
        max_robots_arg: Maximum robot count for automatic generation

    Returns:
        list: List of robot counts
    """
    if robot_counts_arg:
        return [int(x.strip()) for x in robot_counts_arg.split(',')]
    elif max_robots_arg:
        # Generate 2, 4, 6, ..., max_robots
        return list(range(2, max_robots_arg + 1, 2))
    else:
        # Default: just 2 robots
        return [2]


def main():
    parser = argparse.ArgumentParser(
        description='Convert multi-robot-syclop YAML scenarios to PPL/WoDaSH format',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # RECOMMENDED: Convert entire scenario (auto-detects robot counts)
  %(prog)s --scenario random5_20x20

  # Convert multiple scenarios at once
  %(prog)s --scenario random5_20x20 corridor_40x10 dense_20x20

  # Limit maximum robot count
  %(prog)s --scenario random5_20x20 --max-robots 8

  # Legacy: Convert single YAML file with specific robot counts
  %(prog)s --input problems/random5_20x20/robots_2/seed_0.yaml --robot-counts 2
        """
    )

    # Two modes: --scenario (recommended) or --input (legacy)
    parser.add_argument('--scenario', '-s', nargs='+',
                        help='Scenario name(s) to convert (e.g., "random5_20x20"). '
                             'Auto-detects robot counts from problems/ directory.')
    parser.add_argument('--input', '-i', nargs='+',
                        help='Legacy: Input YAML file(s) to convert directly')
    parser.add_argument('--output-dir', '-o', default=None,
                        help='Output directory (default: ~/PPL/multi-robot-narrow-passage-experiments)')
    parser.add_argument('--scenario-name', '-n', default=None,
                        help='Custom scenario name (only with --input)')
    parser.add_argument('--robot-counts', '-r', default=None,
                        help='Comma-separated robot counts (only with --input)')
    parser.add_argument('--max-robots', '-m', type=int, default=None,
                        help='Maximum robot count to generate')
    parser.add_argument('--ppl-base-dir', '-p', default=None,
                        help='PPL base directory for finding shared files like icreate.obj')
    parser.add_argument('--kinodynamic', '-k', action='store_true',
                        help='Generate kinodynamic K-WoDaSH template (template-ppl-k.xml) with car3d dynamics. '
                             'Existing files are not overwritten.')

    args = parser.parse_args()

    # Validate arguments
    if not args.scenario and not args.input:
        parser.error("Either --scenario or --input is required")

    # Determine output directory
    if args.output_dir:
        output_dir = Path(args.output_dir).expanduser()
    else:
        output_dir = Path.home() / "PPL" / "multi-robot-narrow-passage-experiments"

    # Ensure output directory exists
    output_dir.mkdir(parents=True, exist_ok=True)

    print(f"Output directory: {output_dir}")

    # Determine experiments directory (where scenarios/ and problems/ are)
    experiments_dir = Path(__file__).parent

    # Mode 1: Convert by scenario name (recommended)
    if args.scenario:
        print(f"Scenarios to convert: {args.scenario}")
        if args.max_robots:
            print(f"Max robots: {args.max_robots}")

        for scenario_name in args.scenario:
            try:
                convert_scenario_dir(
                    scenario_name=scenario_name,
                    experiments_dir=str(experiments_dir),
                    output_dir=str(output_dir),
                    ppl_base_dir=args.ppl_base_dir or str(output_dir),
                    max_robots=args.max_robots,
                    kinodynamic=args.kinodynamic
                )
            except Exception as e:
                print(f"Error converting scenario {scenario_name}: {e}")
                import traceback
                traceback.print_exc()

        print("\nAll conversions complete!")
        return

    # Mode 2: Legacy - convert individual YAML files
    robot_counts = parse_robot_counts(args.robot_counts, args.max_robots)
    print(f"Robot counts: {robot_counts}")

    for yaml_path in args.input:
        yaml_path = Path(yaml_path).expanduser()

        if not yaml_path.exists():
            print(f"Error: File not found: {yaml_path}")
            continue

        try:
            convert_scenario(
                str(yaml_path),
                str(output_dir),
                robot_counts,
                args.scenario_name,
                args.ppl_base_dir or str(output_dir),
                kinodynamic=args.kinodynamic
            )
        except Exception as e:
            print(f"Error converting {yaml_path}: {e}")
            import traceback
            traceback.print_exc()

    print("\nAll conversions complete!")


if __name__ == "__main__":
    main()
