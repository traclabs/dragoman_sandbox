# xtce2msg.py - Main script for XTCE to ROS 2 Message Conversion
# Updated to support container-based message generation

import sys
import os
import re
import xml.etree.ElementTree as ET
from dragoman_xtce2msg.xtce_model import parse_xtce_file, parse_containers, resolve_type_definition, resolve_aggregate_type
from dragoman_xtce2msg.generate_msg import generate_msg

def to_ros_field_name(name):
    """
    Convert a parameter name to ROS 2 compliant field name.
    ROS 2 field names must match: ^(?!.*__)(?!.*_$)[a-z][a-z0-9_]*$

    This means:
    - Start with lowercase letter
    - Contain only lowercase letters, digits, and underscores
    - No double underscores
    - No trailing underscore

    Converts PascalCase/CamelCase to snake_case while preserving existing underscores.
    Examples:
        T_AbsoluteTime -> t_absolute_time
        T_EnumeratedAlarm -> t_enumerated_alarm
        T_StatusAggregate_CurrentDraw -> t_status_aggregate_current_draw
    """
    # First, insert underscores before uppercase letters that follow lowercase letters or digits
    # This handles CamelCase -> snake_case conversion
    name = re.sub(r'([a-z0-9])([A-Z])', r'\1_\2', name)

    # Convert to lowercase
    name = name.lower()

    # Replace any non-alphanumeric characters (except underscore) with underscore
    name = re.sub(r'[^a-z0-9_]', '_', name)

    # Remove double underscores
    while '__' in name:
        name = name.replace('__', '_')

    # Remove leading underscores and ensure it starts with a letter
    name = name.lstrip('_')
    if name and not name[0].isalpha():
        name = 'field_' + name

    # Remove trailing underscores
    name = name.rstrip('_')

    return name

def generate_message_for_container(container_name, container_params, parameter_metadata, type_definitions, output_dir, xtce_file_path):
    """
    Generate a single ROS message file for a container.

    Args:
        container_name: Name of the container (used as-is for message name)
        container_params: List of parameter names in this container
        parameter_metadata: Dictionary of all parameter metadata
        type_definitions: Dictionary of all type definitions
        output_dir: Directory to write the .msg file
        xtce_file_path: Source XTCE file path
    """
    # Use container name exactly as-is for the message name
    msg_name = container_name
    output_file_path = os.path.join(output_dir, f"{msg_name}.msg")

    # Resolve all parameters to final ROS types
    ros_fields = []
    ccsds_fields = []  # Separate list for CCSDS header fields

    for param_name in container_params:
        # Skip if parameter not in metadata (shouldn't happen but be safe)
        if param_name not in parameter_metadata:
            continue

        param_ref = parameter_metadata[param_name]

        # Skip CCSDS header fields (they're handled at protocol layer)
        is_ccsds_header = param_name.startswith('ccsds_packet')

        # resolve_type_definition handles all subsequent lookups without prefixes
        ros_type, is_fixed_size = resolve_type_definition(param_ref['type_ref'], type_definitions)

        # Handle aggregate types by flattening their members
        if ros_type == '__AGGREGATE__':
            type_elem = type_definitions.get(param_ref['type_ref'])
            if type_elem is not None:
                aggregate_fields = resolve_aggregate_type(type_elem, type_definitions)
                for agg_field in aggregate_fields:
                    # Prefix member names with parent parameter name and convert to ROS format
                    ros_field_name = to_ros_field_name(f"{param_name}_{agg_field['name']}")
                    field_entry = {
                        'name': ros_field_name,
                        'ros_type': agg_field['ros_type'],
                        'comments': f"Member of {param_name}: {param_ref.get('short_desc', '')}"
                    }
                    if is_ccsds_header:
                        ccsds_fields.append(field_entry)
                    else:
                        ros_fields.append(field_entry)
        else:
            # Regular field (not an aggregate) - convert to ROS format
            ros_field_name = to_ros_field_name(param_name)
            field_entry = {
                'name': ros_field_name,
                'ros_type': ros_type,
                'comments': param_ref.get('short_desc', '')
            }
            if is_ccsds_header:
                ccsds_fields.append(field_entry)
            else:
                ros_fields.append(field_entry)

    # Generate the ROS 2 message file
    print(f"  Generating ROS 2 message: {output_file_path}...")
    generate_msg(msg_name, ros_fields, output_file_path, xtce_file_path, ccsds_fields)

    return output_file_path

def main():
    if len(sys.argv) != 3:
        print(f"Usage: {sys.argv[0]} <input_xtce_file.xml> <output_directory>")
        print(f"  Note: Output is now a directory, not a single file")
        sys.exit(1)

    xtce_file_path = sys.argv[1]
    output_dir = sys.argv[2]

    if not os.path.exists(xtce_file_path):
        print(f"Error: XTCE file not found at {xtce_file_path}")
        sys.exit(1)

    # Create output directory if it doesn't exist
    os.makedirs(output_dir, exist_ok=True)

    try:
        # 1. Parse XTCE file
        print(f"Parsing XTCE file: {xtce_file_path}...")
        xtce_tree = ET.parse(xtce_file_path)
        root = xtce_tree.getroot()

        # 2. Extract ParameterTypes, Parameters, and Containers
        parameter_metadata, type_definitions = parse_xtce_file(root)
        containers = parse_containers(root)

        if not parameter_metadata:
            print("Warning: No telemetered Parameters found.")
            sys.exit(0)

        # 3. Generate messages based on containers
        if containers:
            print(f"Found {len(containers)} non-abstract containers")
            generated_files = []

            for container_name, container_info in containers.items():
                print(f"\nProcessing container: {container_name}")
                print(f"  Parameters: {container_info['parameters']}")

                output_file = generate_message_for_container(
                    container_name,
                    container_info['parameters'],
                    parameter_metadata,
                    type_definitions,
                    output_dir,
                    xtce_file_path
                )
                generated_files.append(output_file)

            print(f"\nSuccessfully created {len(generated_files)} ROS 2 message(s):")
            for f in generated_files:
                print(f"  - {f}")
        else:
            # Fallback: No containers found, generate single message with all parameters
            print("No containers found, generating single message with all parameters...")
            msg_name = os.path.splitext(os.path.basename(xtce_file_path))[0]
            output_file_path = os.path.join(output_dir, f"{msg_name}.msg")

            # Resolve all parameters to final ROS types
            ros_fields = []
            ccsds_fields = []

            for param_name, param_ref in parameter_metadata.items():
                is_ccsds_header = param_name.startswith('ccsds_packet')
                ros_type, is_fixed_size = resolve_type_definition(param_ref['type_ref'], type_definitions)

                if ros_type == '__AGGREGATE__':
                    type_elem = type_definitions.get(param_ref['type_ref'])
                    if type_elem is not None:
                        aggregate_fields = resolve_aggregate_type(type_elem, type_definitions)
                        for agg_field in aggregate_fields:
                            ros_field_name = to_ros_field_name(f"{param_name}_{agg_field['name']}")
                            field_entry = {
                                'name': ros_field_name,
                                'ros_type': agg_field['ros_type'],
                                'comments': f"Member of {param_name}: {param_ref.get('short_desc', '')}"
                            }
                            if is_ccsds_header:
                                ccsds_fields.append(field_entry)
                            else:
                                ros_fields.append(field_entry)
                else:
                    ros_field_name = to_ros_field_name(param_name)
                    field_entry = {
                        'name': ros_field_name,
                        'ros_type': ros_type,
                        'comments': param_ref.get('short_desc', '')
                    }
                    if is_ccsds_header:
                        ccsds_fields.append(field_entry)
                    else:
                        ros_fields.append(field_entry)

            generate_msg(msg_name, ros_fields, output_file_path, xtce_file_path, ccsds_fields)
            print(f"Successfully created ROS 2 message: {output_file_path}")

    except Exception as e:
        print(f"An error occurred during conversion: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == "__main__":
    main()
