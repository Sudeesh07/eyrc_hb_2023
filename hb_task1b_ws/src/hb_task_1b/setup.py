from setuptools import find_packages, setup
from glob import glob

package_name = 'hb_task_1b'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/urdf', glob('urdf/*.xacro')),
        ('share/' + package_name + '/meshes', glob('meshes/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*'))
        ('share/' + package_name + '/scripts',glob('/scripts/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sudeesh',
    maintainer_email='sudeeshmuthum@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    
    entry_points={
        'console_scripts': [
            "controller = hb_task_1b.controller:main",
            "service_node = hb_task_1b.service_node.py:main",
        ],
    },
)
