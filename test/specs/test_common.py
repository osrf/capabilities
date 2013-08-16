from __future__ import print_function

from ..common import assert_raises_regex

from capabilities.specs.common import validate_spec_name

test_cases = [
    ['package/spec', None, None],
    ['', ValueError, "it should be of the form 'package/spec_name'"],
    ['/', ValueError, "it should be of the form 'package/spec_name'"],
    ['package/', ValueError, "it should be of the form 'package/spec_name'"],
    ['/spec', ValueError, "it should be of the form 'package/spec_name'"],
    [1, AssertionError, "must be a string"],
]


def test_validate_spec_name():
    for name, exc_type, exc_regex in test_cases:
        with assert_raises_regex(exc_type, exc_regex):
            validate_spec_name(name)
