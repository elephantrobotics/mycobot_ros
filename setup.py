import os
from glob import glob
from setuptools import setup
from setuptools import find_packages

package_name = 'myCobotROS'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[
        'scripts.control_marker',
        'scripts.control_slider',
        'scripts.display',
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='lijun zhang',
    author_email="lijun.zhang@elephantrobotics.com",
    maintainer='lijun zhang',
    maintainer_email="lijun.zhang@elephantrobotics.com",
    keywords=['ROS', 'ROS2'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='The myCobotROS package.',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'control_marker= scripts.control_marker:main',
            'control_slider= control_slider:main',
            'display= scripts.display:main',
        ],
    },
    data_files=[
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name), glob('urdf/*'))
    ]
)