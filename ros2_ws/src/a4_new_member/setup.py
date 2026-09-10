from setuptools import setup
from glob import glob

package_name = 'a4_new_member'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='Neil George',
    maintainer_email='neilgeorge03@gmail.com',
    description='MFE A4 new_member: dead-reckoning (A4.1) and 2D EKF pose estimator (A4.2).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dead_reckoning_node = a4_new_member.dead_reckoning_node:main',
            'ekf_node = a4_new_member.ekf_node:main',
        ],
    },
)
