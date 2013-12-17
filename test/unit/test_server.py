from __future__ import print_function

import sys

try:
    from mock import Mock

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

    def create_capability_instance(name):
        m = Mock()
        m.name = '{name}_pkg/{name}'.format(name=name.lower())
        m.implements = '{name_lower}_pkg/{name}'.format(name_lower=name.lower(), name=name)
        m.dependencies = []
        return server.CapabilityInstance(m, '/path/to/{name}'.format(name=name.lower()))

    def test_capability_instance():
        ci = create_capability_instance('Minimal')
        ci._CapabilityInstance__state = 'not_waiting'
        assert ci.launch() is False
        ci._CapabilityInstance__state = 'waiting'
        assert ci.cancel() is True
        ci._CapabilityInstance__state = 'not_waiting'
        assert ci.cancel() is False
        ci._CapabilityInstance__state = 'not_launching'
        assert ci.launched(-1) is False
        ci._CapabilityInstance__state = 'not_valid'
        assert ci.stopped() is False
        ci._CapabilityInstance__state = 'not_stopping'
        assert ci.terminated() is False

    def test_get_reverse_depends():
        instances = {
            'foo': create_capability_instance('Foo'),
            'bar': create_capability_instance('Bar'),
            'baz': create_capability_instance('Baz')
        }
        instances['foo'].depends_on = ['bar']
        instances['bar'].depends_on = []
        instances['baz'].depends_on = ['bar']
        result = [x.name for x in server.get_reverse_depends('bar', instances.values())]
        assert sorted(['foo_pkg/foo', 'baz_pkg/baz']) == sorted(result), sorted(result)

except ImportError as exc:
    if 'rospy' not in str(exc) and 'No module named srv' not in str(exc):
        raise
    print("Skipping test_server.py because ROS depenencies not imported: " + str(exc), file=sys.stderr)
