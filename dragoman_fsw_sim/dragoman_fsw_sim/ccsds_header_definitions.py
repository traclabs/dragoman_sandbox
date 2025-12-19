"""
Shared CCSDS Header Definition

This module provides a reusable CCSDS Primary Header BitStruct that can be
imported by all simulator construct definitions (Curiosity, IMetro, AllTypes).

The CCSDS Space Packet Primary Header is 6 bytes (48 bits) and follows the
CCSDS 133.0-B-2 standard.
"""

from construct import BitStruct, BitsInteger


CCSDSHeader = BitStruct(
    "version" / BitsInteger(3),           # Packet version (typically 0)
    "type" / BitsInteger(1),              # 0 = Telemetry, 1 = Command
    "secondary_header_flag" / BitsInteger(1),  # 0 = Not present, 1 = Present
    "apid" / BitsInteger(11),             # Application Process ID
    "sequence_flags" / BitsInteger(2),    # 3 = Unsegmented, 0 = Continuation, 1 = First, 2 = Last
    "sequence_count" / BitsInteger(14),   # Packet sequence count (wraps at 16384)
    "packet_length" / BitsInteger(16)     # Length of packet data - 1 (as per CCSDS standard)
)
