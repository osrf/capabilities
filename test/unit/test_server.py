from __future__ import print_function

import sys

try:
    from .common import assert_raises, assert_raises_regex, redirected_stdio, environment

    from capabilities import server

    def test_create_parser():
        print()
        parser = server.create_parser()
        args = parser.parse_args([])
        assert args.package_path is None
        package_path = '/path/1:/path/2'
        args = parser.parse_args([package_path])
        assert args.package_path == package_path
        path1 = '/path/1'
        path2 = '/path/2'
        with assert_raises(SystemExit):
            with redirected_stdio(combined_io=True) as (out, err):  # Capture stderr from argparse
                args = parser.parse_args([path1, path2])
        print(out.getvalue())

    def test_main():
        with environment({}):
            with assert_raises_regex(SystemExit, 'No package paths specified'):
                server.main([])
except ImportError as e:
    if 'rospy' not in str(e) and 'No module named srv' not in str(e):
        raise
    print("Skipping test_server.py because ROS depenencies not imported: " + str(e), file=sys.stderr)
