from common import assert_raises

from capabilities.client import CapabilitiesClient


def test_wait_for_services():
    c = CapabilitiesClient()
    with assert_raises(ValueError):
        c.wait_for_services(0.1, ['invalid_service'])
    assert c.wait_for_services(0.1) is False
    c.establish_bond(0.1)
    c._used_capabilities.add('some_pkg/SomeCap')
    c.free_capability('some_pkg/SomeCap', 0.1)
