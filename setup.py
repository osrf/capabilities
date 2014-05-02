from distutils.core import setup
from catkin_pkg.python_setup import generate_distutils_setup

d = generate_distutils_setup(
    packages=['capabilities', 'capabilities.specs'],
    package_dir={'': 'src'},
    package_data={
        'capabilities': [
            'capability_server_nodelet_manager.launch',
            'placeholder_script',
        ]
    }
)

setup(**d)
