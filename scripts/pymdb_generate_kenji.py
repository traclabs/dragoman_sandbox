import yamcs.pymdb as yp
from ament_index_python.packages import get_package_share_directory
import os

spacecraft = yp.System("Spacecraft")

ccsds_header = yp.ccsds.add_ccsds_header(spacecraft)

param1 = yp.IntegerParameter(
  system=spacecraft,
  name="param1",
  signed=False,
  encoding=yp.uint8_t,
)

print(spacecraft.dumps())

# Write to an XML file
# filename = os.path.join( get_package_share_directory('dragoman_sandbox'), "xtce", "xtce_kenji.xml")
# xtce_file = open(filename, 'w')
# xtce_file.write(spacecraft.dumps())
