# xtce_model.py - Updated to use QName lookups exclusively

import xml.etree.ElementTree as ET

# XTCE Namespace URI. Must match the xmlns="..." in the XML file.
XTCE_NS = "http://www.omg.org/spec/XTCE/20180204"

# --- QName Constants for Robust XPath Lookups ---
Q = lambda tag: f"{{{XTCE_NS}}}{tag}"

# Fully Qualified Names (QNames) for the most common elements
Q_PARAM_TYPE_SET = Q('ParameterTypeSet')
Q_TELEMETRY_META = Q('TelemetryMetaData')
Q_PARAMETER_SET = Q('ParameterSet')
Q_PARAMETER = Q('Parameter')
Q_PARAMETER_PROPS = Q('ParameterProperties')
Q_MEMBER_LIST = Q('MemberList')
Q_MEMBER = Q('Member')
Q_CONTAINER_SET = Q('ContainerSet')
Q_SEQUENCE_CONTAINER = Q('SequenceContainer')
Q_ENTRY_LIST = Q('EntryList')
Q_PARAMETER_REF_ENTRY = Q('ParameterRefEntry')
Q_BASE_CONTAINER = Q('BaseContainer')

# Data Encoding elements
Q_INT_ENC = Q('IntegerDataEncoding')
Q_FLOAT_ENC = Q('FloatDataEncoding')
Q_STR_ENC = Q('StringDataEncoding')
Q_BIN_ENC = Q('BinaryDataEncoding')
Q_DIMENSION_LIST = Q('DimensionList')
Q_DIMENSION = Q('Dimension')
Q_STARTING_INDEX = Q('StartingIndex')
Q_ENDING_INDEX = Q('EndingIndex')
Q_FIXED_VALUE = Q('FixedValue')
Q_SIZE_IN_BITS = Q('SizeInBits')
Q_FIXED = Q('Fixed')
Q_STRING_VARIABLE = Q('Variable')


# ROS 2 Fixed Type Mapping (Keep as defined previously)
ROS_INTEGER_MAP = {
    8: 'int8', 16: 'int16', 32: 'int32', 64: 'int64'
}
ROS_UINTEGER_MAP = {
    8: 'uint8', 16: 'uint16', 32: 'uint32', 64: 'uint64'
}
ROS_FLOAT_MAP = {
    32: 'float32', 64: 'float64'
}


# --- Core Parsing Functions ---

def parse_xtce_file(root): # ns_map parameter REMOVED
    """Parses the XTCE file to extract raw ParameterTypes and Parameters."""
    type_definitions = {}
    parameter_metadata = {}

    # 1. Extract all ParameterTypes (Note: uses findall with full QName format)
    # The xpath syntax "tagname/*" works for children of Q_PARAM_TYPE_SET
    param_type_set = root.find(f".//{Q_PARAM_TYPE_SET}")
    if param_type_set is not None:
        for elem in param_type_set.findall('*'):
            type_name = elem.get('name')
            if type_name:
                type_definitions[type_name] = elem

    # 2. Extract all Parameters (only telemetered ones)
    param_set = root.find(f".//{Q_TELEMETRY_META}/{Q_PARAMETER_SET}")
    if param_set is not None:
        for param_elem in param_set.findall(Q_PARAMETER):
            name = param_elem.get('name')
            type_ref = param_elem.get('parameterTypeRef')
            short_desc = param_elem.get('shortDescription', '')

            # Check data source
            data_source_elem = param_elem.find(Q_PARAMETER_PROPS)
            data_source = data_source_elem.get('dataSource') if data_source_elem is not None else 'telemetered'

            if data_source in ('telemetered', 'constant'):
                parameter_metadata[name] = {
                    'type_ref': type_ref,
                    'short_desc': short_desc,
                    'element': param_elem
                }

    return parameter_metadata, type_definitions


def parse_containers(root):
    """
    Parses the XTCE file to extract container definitions.
    Returns a dictionary mapping container names to their parameter lists.
    Only non-abstract containers are included.
    """
    containers = {}

    # Find the ContainerSet within TelemetryMetaData
    container_set = root.find(f".//{Q_TELEMETRY_META}/{Q_CONTAINER_SET}")
    if container_set is None:
        return containers

    for container_elem in container_set.findall(Q_SEQUENCE_CONTAINER):
        container_name = container_elem.get('name')
        is_abstract = container_elem.get('abstract', 'false').lower() == 'true'

        # Skip abstract containers (like ccsds_space_packet)
        if is_abstract or not container_name:
            continue

        # Extract parameters from this container
        parameters = []
        entry_list = container_elem.find(Q_ENTRY_LIST)

        if entry_list is not None:
            for param_ref_entry in entry_list.findall(Q_PARAMETER_REF_ENTRY):
                param_ref = param_ref_entry.get('parameterRef')
                if param_ref:
                    parameters.append(param_ref)

        # Store container info
        containers[container_name] = {
            'name': container_name,
            'parameters': parameters,
            'element': container_elem
        }

    return containers


# --- Type Resolution and Mapping Functions ---

def resolve_integer_type(elem, type_definitions=None):
    """Resolves Integer/Enumerated ParameterTypes."""
    bit_size = 32
    is_signed = True

    # Check ParameterType attributes
    size_in_bits_attr = elem.get('sizeInBits')
    if size_in_bits_attr:
        bit_size = int(size_in_bits_attr)

    signed_attr = elem.get('signed')
    if signed_attr == 'false':
        is_signed = False

    # Check DataEncoding details
    data_enc = elem.find(Q_INT_ENC)
    if data_enc is not None:
        # Check explicit FixedValue (chained find calls)
        size_in_bits_elem = data_enc.find(Q_SIZE_IN_BITS)
        if size_in_bits_elem is not None:
            fixed_elem = size_in_bits_elem.find(Q_FIXED)
            if fixed_elem is not None:
                fixed_val_elem = fixed_elem.find(Q_FIXED_VALUE)
                if fixed_val_elem is not None and fixed_val_elem.text and fixed_val_elem.text.isdigit():
                    bit_size = int(fixed_val_elem.text)

        # Fallback to DataEncoding attribute
        if data_enc.get('sizeInBits'):
            bit_size = int(data_enc.get('sizeInBits'))

        # Only override signedness if encoding explicitly indicates unsigned
        # The ParameterType 'signed' attribute takes precedence unless encoding is explicitly unsigned
        enc_type = data_enc.get('encoding', 'twosComplement')

        # Only override to unsigned if the signed attribute was not explicitly set to true
        # AND the encoding is unsigned/BCD/packedBCD
        if enc_type in ('unsigned', 'BCD', 'packedBCD') and signed_attr != 'true':
            is_signed = False

    # Final ROS type mapping (round up to nearest ROS primitive size)
    # The logic handles bit_size > 64 correctly from previous review.
    bit_size = next((bs for bs in [8, 16, 32, 64] if bs >= bit_size), 64)

    if is_signed:
        return ROS_INTEGER_MAP.get(bit_size, 'int32'), False
    else:
        return ROS_UINTEGER_MAP.get(bit_size, 'uint32'), False

#

def resolve_float_type(elem, type_definitions=None):
    """Resolves Float ParameterTypes."""
    bit_size = 64

    size_in_bits_attr = elem.get('sizeInBits')
    if size_in_bits_attr:
        bit_size = int(size_in_bits_attr)

    data_enc = elem.find(Q_FLOAT_ENC)
    if data_enc is not None:
        if data_enc.get('sizeInBits'):
            bit_size = int(data_enc.get('sizeInBits'))

    # Final ROS type mapping
    bit_size = next((bs for bs in [32, 64] if bs >= bit_size), 64)
    return ROS_FLOAT_MAP.get(bit_size, 'float64'), False

def resolve_string_type(elem, type_definitions=None):
    """Resolves String ParameterTypes, handling fixed size."""
    size = None
    is_variable = False
    data_enc = elem.find(Q_STR_ENC)

    if data_enc is not None:
        # Check for FixedValue inside SizeInBits (chained find calls)
        size_in_bits_elem = data_enc.find(Q_SIZE_IN_BITS)
        if size_in_bits_elem is not None:
            fixed_elem = size_in_bits_elem.find(Q_FIXED)
            if fixed_elem is not None:
                fixed_val_elem = fixed_elem.find(Q_FIXED_VALUE)
                if fixed_val_elem is not None and fixed_val_elem.text and fixed_val_elem.text.isdigit():
                    size_bits = int(fixed_val_elem.text)
                    size = size_bits // 8

        # If variable is set, check max size hint
        variable_elem = data_enc.find(Q_STRING_VARIABLE)
        if variable_elem is not None:
            is_variable = True
            max_size_bits = variable_elem.get('maxSizeInBits')
            if max_size_bits and max_size_bits.isdigit():
                size = int(max_size_bits) // 8

    # For variable-length strings with very large max sizes (>4KB), use unbounded string
    # This is more practical for ROS messages than allocating huge fixed buffers
    if size is not None and size > 0:
        if is_variable and size > 4096:
            return "string", False
        return f"string[{size}]", True

    return "string", False

def resolve_binary_type(elem, type_definitions=None):
    """Resolves Binary ParameterTypes to a fixed-size byte array."""
    size = None
    data_enc = elem.find(Q_BIN_ENC)

    if data_enc is not None:
        # Check for FixedValue inside SizeInBits (chained find calls)
        size_in_bits_elem = data_enc.find(Q_SIZE_IN_BITS)
        if size_in_bits_elem is not None:
            fixed_elem = size_in_bits_elem.find(Q_FIXED)
            if fixed_elem is not None:
                fixed_val_elem = fixed_elem.find(Q_FIXED_VALUE)
                if fixed_val_elem is not None and fixed_val_elem.text and fixed_val_elem.text.isdigit():
                    size_bits = int(fixed_val_elem.text)
                    size = size_bits // 8
            else:
                # Check if FixedValue is directly under SizeInBits
                fixed_val_elem = size_in_bits_elem.find(Q_FIXED_VALUE)
                if fixed_val_elem is not None and fixed_val_elem.text and fixed_val_elem.text.isdigit():
                    size_bits = int(fixed_val_elem.text)
                    size = size_bits // 8

    if size is not None and size > 0:
        return f"uint8[{size}]", True

    return "uint8[]", False

def resolve_array_type(elem, type_definitions):
    """Resolves Array ParameterTypes, extracting fixed dimensions and base type."""
    base_type_ref = elem.get('arrayTypeRef')

    # Recursive call to find the base type of the array
    base_ros_type, _ = resolve_type_definition(base_type_ref, type_definitions)

    dimensions = []
    dimension_list = elem.find(Q_DIMENSION_LIST)

    if dimension_list is not None:
        for dim_elem in dimension_list.findall(Q_DIMENSION):
            # Chain find calls for nested elements
            start_index_elem = dim_elem.find(Q_STARTING_INDEX)
            end_index_elem = dim_elem.find(Q_ENDING_INDEX)

            if start_index_elem is not None and end_index_elem is not None:
                start_fixed = start_index_elem.find(Q_FIXED_VALUE)
                end_fixed = end_index_elem.find(Q_FIXED_VALUE)

                if (start_fixed is not None and end_fixed is not None and
                    start_fixed.text and end_fixed.text and
                    start_fixed.text.isdigit() and end_fixed.text.isdigit()):
                    start_index = int(start_fixed.text)
                    end_index = int(end_fixed.text)
                    size = end_index - start_index + 1
                    dimensions.append(size)
                else:
                    return f"{base_ros_type}[]", False # Non-fixed array
            else:
                return f"{base_ros_type}[]", False # Non-fixed array

    if len(dimensions) == 1:
        return f"{base_ros_type}[{dimensions[0]}]", True
    elif len(dimensions) > 1:
        total_size = 1
        for s in dimensions:
            total_size *= s
        # Multi-dimensional array unrolled to a fixed single dimension in ROS msg
        return f"{base_ros_type}[{total_size}]", True

    return f"{base_ros_type}[]", False


def resolve_aggregate_type(elem, type_definitions):
    """Resolves Aggregate ParameterTypes by flattening members into individual fields."""
    fields = []
    member_list = elem.find(Q_MEMBER_LIST)

    if member_list is not None:
        for member_elem in member_list.findall(Q_MEMBER):
            member_name = member_elem.get('name')
            member_type_ref = member_elem.get('typeRef')

            if member_name and member_type_ref:
                # Recursively resolve the member's type
                member_ros_type, is_fixed = resolve_type_definition(member_type_ref, type_definitions)
                fields.append({
                    'name': member_name,
                    'ros_type': member_ros_type,
                    'is_fixed': is_fixed
                })

    return fields


def resolve_type_definition(type_name, type_definitions):
    """Public interface to resolve a ParameterTypeRef to a final ROS type string."""
    elem = type_definitions.get(type_name)
    if elem is None:
        return 'string', False

    tag = elem.tag.split('}')[-1] # Get tag name without the URI prefix

    # Handle standard primitive types
    if tag in ('IntegerParameterType', 'EnumeratedParameterType'):
        return resolve_integer_type(elem, type_definitions)
    elif tag == 'FloatParameterType':
        return resolve_float_type(elem, type_definitions)
    elif tag == 'BooleanParameterType':
        return 'bool', False
    elif tag == 'StringParameterType':
        return resolve_string_type(elem)
    elif tag == 'BinaryParameterType':
        return resolve_binary_type(elem)

    # Handle composite types
    elif tag == 'ArrayParameterType':
        # Pass type_definitions for nested resolution
        return resolve_array_type(elem, type_definitions)
    elif tag == 'AggregateParameterType':
        # Return special marker for aggregate types that need flattening
        return '__AGGREGATE__', True

    # Handle Time types
    elif tag == 'AbsoluteTimeParameterType':
        return 'builtin_interfaces/Time', False
    elif tag == 'RelativeTimeParameterType':
        return 'builtin_interfaces/Duration', False

    return 'string', False
