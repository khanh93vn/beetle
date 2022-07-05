from setuptools import setup

package_name = 'demo_image_publisher_py'

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
    description='Demo nodes for publishing images',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = demo_image_publisher_py.image_publisher:main',
        ],
    },
)
