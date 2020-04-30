import pycodestyle
import os


def test_pycodestyle_conformance():
    """Test source code for PEP8 conformance"""
    pycodestyle_style = pycodestyle.StyleGuide(max_line_length=120)
    pycodestyle_style.verbose = True
    report = pycodestyle_style.options.report
    report.start()
    package_dir = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', '..'))
    assert os.path.isdir(package_dir)

    pycodestyle_style.input_dir(os.path.join(package_dir, 'src', 'capabilities'))
    pycodestyle_style.options.filename = '.*'
    pycodestyle_style.input_dir(os.path.join(package_dir, 'scripts'))

    report.stop()
    assert report.total_errors == 0, "Found '{0}' code style errors (and warnings).".format(report.total_errors)
