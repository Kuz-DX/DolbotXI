from setuptools import setup
import os
from glob import glob

package_name = 'test_depth'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # [중요 1] 런치 파일을 설치 경로로 복사하는 설정
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'slope_trigger = test_depth.slope_trigger_node:main',
            'mode_manager = test_depth.mode_manager_node:main',
        ],
    },
)
