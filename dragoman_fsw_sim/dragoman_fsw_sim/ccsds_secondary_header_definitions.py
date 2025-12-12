"""
Shared CCSDS Secondary Header Definitions

This module provides reusable CCSDS Secondary Header BitStructs that can be
imported by all simulator construct definitions (Gateway, IMetro, etc.).

Secondary headers provide additional metadata beyond the primary CCSDS header:
- CommandSecondaryHeader: Contains function code and checksum for commands
- TelemetrySecondaryHeader: Contains timestamp and spare bits for telemetry
"""

from construct import BitStruct, BitsInteger


CommandSecondaryHeader = BitStruct(
    "fcn_code" / BitsInteger(8),   # Function code identifying the command
    "checksum" / BitsInteger(8)    # Checksum for command validation
)

TelemetrySecondaryHeader = BitStruct(
    "sec" / BitsInteger(48),       # Timestamp in seconds (48 bits)
    "spare" / BitsInteger(32)      # Spare bits for future use
)
