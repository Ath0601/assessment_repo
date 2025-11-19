from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'pickplace'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.*')),
        (os.path.join('share', package_name, 'config', 'kinematics', 'default'), glob('config/kinematics/default/*.*')),
        (os.path.join('share', package_name, 'config', 'link_inertial'), glob('config/link_inertial/*.*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.*')),
        (os.path.join('share', package_name, 'urdf', 'xarm5'), glob('urdf/xarm5/*.*')),
        (os.path.join('share', package_name, 'urdf', 'common'), glob('urdf/common/*.*')),
        (os.path.join('share', package_name, 'meshes', 'xarm5', 'visual'), glob('meshes/xarm5/visual/*.*')),
        (os.path.join('share', package_name, 'meshes', 'end_tool', 'collision'), glob('meshes/end_tool/collision/*.*')),
        (os.path.join('share', package_name, 'meshes', 'gripper', 'xarm'), glob('meshes/gripper/xarm/*.*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='atharva',
    maintainer_email='atharva.abhi.kulkarni@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'box_publish_node = pickplace.box_publish:main',
            'home_pos = pickplace.home_pos:main',
        ],
    },
)
