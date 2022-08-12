from setuptools import setup

package_name = 'webots_ros2_agv'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
    ('share/' + package_name + '/worlds', ['worlds/khoa_cn.wbt']),
    ('share/' + package_name + '/resource', ['resource/agv_webots.urdf']),
    ('share/' + package_name, ['package.xml']),
]

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=data_files,
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khanh',
    maintainer_email='khanh93vn@gmail.com',
    description='webots simulation environment and driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'agv_driver = webots_ros2_agv.agv_driver:main'
        ],
    },
)
