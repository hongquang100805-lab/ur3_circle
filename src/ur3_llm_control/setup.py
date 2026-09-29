from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ur3_llm_control'
package_dir = os.path.dirname(os.path.realpath(__file__))

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.[yY][aA][mM][lL]*'))),
        (os.path.join('share', package_name, 'urdf'),
            glob(os.path.join('urdf', '*.xacro'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Le Hong Quang',
    maintainer_email='student@todo.todo',
    description='LLM control package for UR3 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_planner = ur3_llm_control.llm_planner:main',
            'skill_executor = ur3_llm_control.skill_executor:main',
            'scene_publisher = ur3_llm_control.scene_publisher:main',
        ],
    },
)
