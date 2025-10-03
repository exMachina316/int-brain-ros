# rl_policy_runner/setup.py
from setuptools import setup
import os
from glob import glob

package_name = 'rl_policy_runner'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install the models directory
        (os.path.join('share', package_name, 'models'), glob('models/*.zip')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'tf_transformations'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='A package to run a Stable Baselines3 policy on a ROS 2 robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'policy_node = rl_policy_runner.policy_node:main',
        ],
    },
)