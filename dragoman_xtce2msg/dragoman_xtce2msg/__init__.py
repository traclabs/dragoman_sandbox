"""
dragoman_xtce2msg - XTCE to ROS 2 Message Converter

This package provides tools to convert XTCE (XML Telemetric and Command Exchange)
telemetry definitions into ROS 2 message files.

Main modules:
- xtce_model: XTCE file parsing and type resolution
- generate_msg: ROS 2 message file generation
- xtce2msg: Main conversion script
"""

__version__ = '1.0.0'

from .xtce_model import (
    parse_xtce_file,
    parse_containers,
    resolve_type_definition,
    resolve_aggregate_type,
)

from .generate_msg import generate_msg

__all__ = [
    'parse_xtce_file',
    'parse_containers',
    'resolve_type_definition',
    'resolve_aggregate_type',
    'generate_msg',
]
