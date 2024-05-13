from setuptools import setup

package_name = 'luigi_mansion'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/luigi_mansion/launch/',['launch/sim_launch.launch.xml','launch/prep_launch.launch.xml','launch/real_launch.launch.xml']),
        ('share/luigi_mansion/config/',['config/sim_config.yaml','config/real_config.yaml']),
        ('share/luigi_mansion/lanes/',['lanes/full-lane.traj']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='racecar',
    maintainer_email='mohammedehab200218@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'plan = luigi_mansion.plan:main',
            'load_lane = luigi_mansion.load_lane:main',
            'basement_point_publisher = luigi_mansion.basement_point_publisher:main',
            'follow = luigi_mansion.follow:main',
            'stop_detector = luigi_mansion.city_driving.stop_detector.stop_detector:main',
            'traffic_light_detector = luigi_mansion.traffic_light_detector:main',
        ],
    },
)
