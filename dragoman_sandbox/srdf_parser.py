#!/usr/bin/env python3
"""
SRDF Parser

Generic utilities for parsing SRDF (Semantic Robot Description Format) files.
This module is robot-agnostic and can be used with any SRDF file.
"""

import xml.etree.ElementTree as ET
from typing import Dict, Tuple


def parse_srdf_groups(srdf_content: str) -> Dict[str, set]:
    """
    Parse SRDF XML content and extract group definitions with their joints.

    Args:
        srdf_content: String containing processed SRDF XML

    Returns:
        Dictionary mapping group_name to set of joint names in that group
    """
    root = ET.fromstring(srdf_content)
    groups = {}

    # Find all group elements
    for group in root.findall('.//group'):
        group_name = group.get('name')
        joint_names = set()

        # Extract joints directly defined in the group
        for joint in group.findall('joint'):
            joint_name = joint.get('name')
            if joint_name:
                joint_names.add(joint_name)

        # Store in dictionary
        if joint_names:
            groups[group_name] = joint_names

    return groups


def parse_srdf_group_states(srdf_content: str) -> Dict[Tuple[str, str], Dict]:
    """
    Parse SRDF XML content and extract group states with their joint configurations.

    Args:
        srdf_content: String containing processed SRDF XML

    Returns:
        Dictionary mapping (group_name, state_name) to joint configuration
    """
    root = ET.fromstring(srdf_content)
    group_states = {}

    # Find all group_state elements
    for group_state in root.findall('.//group_state'):
        group_name = group_state.get('group')
        state_name = group_state.get('name')

        # Extract joint values
        joints = []
        values = []
        for joint in group_state.findall('joint'):
            joint_name = joint.get('name')
            joint_value = float(joint.get('value'))
            joints.append(joint_name)
            values.append(joint_value)

        # Store in dictionary
        key = (group_name, state_name)
        group_states[key] = {
            'joints': joints,
            'values': values,
            'group': group_name
        }

    return group_states
