import copy
import os
import re
import sys

from StringIO import StringIO


def assert_raises(exception_classes, callable_obj=None, *args, **kwargs):
    context = AssertRaisesContext(exception_classes)
    if callable_obj is None:
        return context
    with context:
        callable_obj(*args, **kwargs)


def assert_raises_regex(exception_classes, expected_regex, callable_obj=None, *args, **kwargs):
    context = AssertRaisesContext(exception_classes, expected_regex)
    if callable_obj is None:
        return context
    with context:
        callable_obj(*args, **kwargs)


class AssertRaisesContext(object):
    def __init__(self, expected, expected_regex=None):
        self.expected = expected
        self.expected_regex = expected_regex

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc_value, tb):
        if self.expected is None:
            if exc_type is None:
                return True
            else:
                raise
        if exc_type is None:
            try:
                exc_name = self.expected.__name__
            except AttributeError:
                exc_name = str(self.expected)
            raise AssertionError("{0} not raised".format(exc_name))
        if not issubclass(exc_type, self.expected):
            raise
        if self.expected_regex is None:
            return True
        expected_regex = self.expected_regex
        expected_regex = re.compile(expected_regex)
        if not expected_regex.search(str(exc_value)):
            raise AssertionError("'{0}' does not match '{1}'".format(expected_regex.pattern, str(exc_value)))
        return True


class redirected_stdio(object):
    def __init__(self, combined_io=False):
        self.combined_io = combined_io

    def __enter__(self):
        self.original_stdout = sys.stdout
        self.original_stderr = sys.stderr
        sys.stdout = out = StringIO()
        sys.stderr = err = out if self.combined_io else StringIO()
        return out, err

    def __exit__(self, exc_type, exc_value, traceback):
        sys.stdout = self.original_stdout
        sys.stderr = self.original_stderr


class environment(object):
    def __init__(self, seed_environment={}):
        self.seed_environment = seed_environment

    def __enter__(self):
        self.original_environ = copy.deepcopy(os.environ)
        os.environ = self.seed_environment
        return os.environ

    def __exit__(self, exc_type, exc_value, traceback):
        os.environ = self.original_environ
