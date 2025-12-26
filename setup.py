from setuptools import setup

package_name = 'assignment2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian',
    maintainer_email='you@email.com',
    description='Assignment 2 aruco detection with ROS2 + PlanSys2',
    tests_require=['pytest'],
    license='MIT',
    entry_points={
        'console_scripts': [
            'aruco_detection_2 = scripts.aruco_detection_2:main',
        ],
    },
)
