from setuptools import find_packages, setup

package_name = 'rgbd_tum_world_to_map_tf'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@TODO',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rgbd_tum_world_to_map_tf = rgbd_tum_world_to_map_tf.rgbd_tum_world_to_map_tf_node:main'
        ],
    },
)
