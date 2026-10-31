from glob import glob

from setuptools import find_packages, setup

package_name = 'wsl_benchmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config/scenarios', glob('config/scenarios/*.yaml')),
    ],
    install_requires=['setuptools'],
    extras_require={'test': ['pytest']},
    zip_safe=True,
    maintainer='Azer Ben Ali',
    maintainer_email='azerbenali5017@gmail.com',
    description='WSL benchmarking: sensor statistics, perception latency, SLAM trajectory error.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'recorder = wsl_benchmark.recorder:main',
            'route_follower = wsl_benchmark.route_follower:main',
            'evaluate = wsl_benchmark.evaluate:main',
        ],
    },
)
