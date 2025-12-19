#!/usr/bin/env python3

import yamcs.pymdb as yp

# cFS headers defined here:
# cFS/cfe/modules/msg/option_inc/default_cfe_msg_hdr_pri.h
# struct CFE_MSG_CommandHeader
# {
#  CFE_MSG_Message_t                Msg; /**< \brief Base message */
#  CFE_MSG_CommandSecondaryHeader_t Sec; /**< \brief Secondary header */
#};
# Size command heade r: 6 bytes + function_code (1 byte) + checksum (1 byte) = 8 bytes
# struct CFE_MSG_TelemetryHeader
#{
#    CFE_MSG_Message_t                  Msg;      /**< \brief Base message */
#    CFE_MSG_TelemetrySecondaryHeader_t Sec;      /**< \brief Secondary header */
#    uint8                              Spare[4]; /**< \brief Pad to avoid compiler padding if payload
#                                                             requires 64 bit alignment */
#};
# Size telemetry header: 6 bytes + 6 bytes (secondary hdr with time info) + 4 bytes (spare) = 16 bytes

# **********************************************
# Generic cFS command
# **********************************************
def add_cfs_command_header(system: yp.System,
                           ccsds_header: yp.ccsds.CcsdsHeader,
                           name: str = "CfsPacket") -> yp.Command :

  scr_header = yp.AggregateArgument(
    name="scr_header",
    members=[
      yp.IntegerMember(
        name="fcn_code",
        signed=False,
        bits=8,
        encoding=yp.uint8_t,
        initial_value=0
      ),
      yp.IntegerMember(
        name="checksum",
        signed=False,
        bits=8,
        encoding=yp.uint8_t,
        initial_value=0
      )
    ]
  )

  return yp.Command(
     system=system,
     name=name,
     abstract = True,
     base = ccsds_header.tc_command,
     assignments = {
       ccsds_header.tc_secondary_header.name: "Present",
     },
     arguments=[
       scr_header
     ],
     entries=[
       yp.ArgumentEntry(scr_header)
     ]
  )

# *************************************
# cFS Telemetry packet
# *************************************
def add_cfs_telemetry_header(system: yp.System, ccsds_header: yp.ccsds.CcsdsHeader,
    name: str = "CfsTelemetryPacket") -> yp.Container :

  secondary_header_parameter = yp.AggregateParameter(
    system=system,
    name="scr_header",
    members=[
      yp.ArrayMember(
        name="Sec",
        data_type=yp.datatypes.IntegerDataType(encoding=yp.uint8_t, bits=8),
        length=6
      ),
      yp.ArrayMember(
        name="Spare",
        data_type=yp.datatypes.IntegerDataType(encoding=yp.uint8_t, bits=8),
        length=4
      )
    ]
  )
  return yp.Container(
    system=system,
    name=name,
    abstract=True,
    base=ccsds_header.tm_container,
    entries=[
      yp.ParameterEntry(parameter=secondary_header_parameter)
    ]
  )
