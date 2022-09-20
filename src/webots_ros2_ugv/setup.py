from setuptools import setup

package_name = 'webots_ros2_ugv'
data_files = [
    ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ('share/' + package_name + '/launch', ['launch/robot_launch.py']),
    ('share/' + package_name + '/launch', ['launch/simulation_launch.py']),
    ('share/' + package_name + '/worlds', ['worlds/khoa_cn.wbt']),
    ('share/' + package_name + '/resource', ['resource/ugv_webots.urdf']),
    ('share/' + package_name + '/rviz', ['rviz/urdf_config.rviz']),
    ('share/' + package_name + '/config', ['config/localization_params.yaml']),
    ('share/' + package_name + '/config', ['config/nav2_params.yaml']),
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
        ],
    },
)
