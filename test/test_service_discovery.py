from __future__ import print_function

import sys

try:
    from capabilities import service_discovery

    def test_service_discovery():
        # Todo: test this better...
        service_discovery.spec_index_from_service

except ImportError as e:
    if 'rospy' not in str(e) and 'No module named srv' not in str(e):
        raise
    print("Skipping test_discovery.py because ROS depenencies not imported: " + str(e), file=sys.stderr)
