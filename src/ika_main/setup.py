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
        # Launch, URDF, World ve MESHES (3D Modeller) klasörlerini sisteme tanıtıyoruz
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.xacro')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*.world')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*.dae')), # Bu satır kritik!
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='salih',
    maintainer_email='salih@todo.todo',
    description='İKA projesi ana kontrol paketi',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ana_kontrol = ika_main.ana_kontrol:main',
            'sensor_isleme = ika_main.sensor_isleme:main',
            'otonom_surus = ika_main.otonom_surus:main',
        ],
    },
)
