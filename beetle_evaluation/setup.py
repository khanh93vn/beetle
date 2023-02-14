from setuptools import setup

package_name = 'beetle_evaluation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='khanh',
    maintainer_email='khanh93vn@gmail.com',
    description='Nodes for Beetle evaluation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'beetle_evaluation_manager = beetle_evaluation.beetle_evaluation_manager:main'
        ],
    },
)
