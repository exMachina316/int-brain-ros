from setuptools import find_packages, setup

package_name = 'odom_experiments'

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
    maintainer='Eccentric Orange',
    maintainer_email='eccentric.orange2@gmail.com',
    description='TODO: Package description',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'study_wheels = odom_experiments.study_wheels:main',
            'study_imu = odom_experiments.study_imu:main',
        ],
    },
)
