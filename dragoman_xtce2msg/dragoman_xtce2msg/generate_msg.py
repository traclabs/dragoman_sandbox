# generate_msg.py - Creates the ROS 2 message file (.msg)

from datetime import datetime

def generate_msg(msg_name, fields, output_path, source_file, ccsds_fields=None):
    """
    Generates a ROS 2 message file from the processed XTCE parameter fields.

    Args:
        msg_name: Name of the ROS message
        fields: List of field dictionaries to include in the message
        output_path: Path to write the .msg file
        source_file: Source XTCE file path
        ccsds_fields: Optional list of CCSDS header fields to include as comments
    """
    now = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

    with open(output_path, 'w') as f:
        # File Header
        f.write(f"# ROS 2 Message: {msg_name}.msg\n")
        f.write(f"# Generated automatically from XTCE file: {source_file}\n")
        f.write(f"# Generation Date: {now}\n")
        f.write("\n")

        # CCSDS Header Fields (commented out - typically handled at protocol layer)
        # Uncomment the section below if you need to include CCSDS header fields in the message
        # if ccsds_fields:
        #     f.write("# CCSDS Header Fields (handled at protocol layer):\n")
        #     for field in ccsds_fields:
        #         name = field['name']
        #         ros_type = field['ros_type']
        #         comments = field['comments']
        #         line = f"# {ros_type} {name}"
        #         if comments:
        #             line += f"  # {comments}"
        #         f.write(line + "\n")
        #     f.write("#\n")

        # Message Fields
        for field in fields:
            name = field['name']
            ros_type = field['ros_type']
            comments = field['comments']

            # Format the output line
            line = f"{ros_type} {name}"
            if comments:
                line += f" # {comments}"
            f.write(line + "\n")
