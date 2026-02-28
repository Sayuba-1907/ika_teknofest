from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ika_main'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Burası kritik: Launch, URDF ve World dosyalarını sisteme tanıtıyoruz
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salih',
    maintainer_email='salih@todo.todo',
    description='İKA projesi ana kontrol paketi',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
   entry_points={
        'console_scripts': [
            'ana_kontrol = ika_main.ana_kontrol:main',
        ],
    },

)   