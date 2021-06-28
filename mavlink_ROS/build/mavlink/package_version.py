import re, sys
from xml.etree import ElementTree as ET
doc = ET.parse('/home/hai_bker96/VIAM_AUV2000_ROS/mavlink_ROS/src/mavlink/package.xml')
ver = doc.find('version').text
if re.match('\d+\.\d+\.\d+', ver):
    sys.stdout.write(ver)
else:
    sys.stderr.write('Version format error.\n')
    sys.exit(1)
