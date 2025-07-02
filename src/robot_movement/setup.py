from setuptools import setup

package_name = 'robot_movement'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Tu Nombre',
    maintainer_email='tu_correo@example.com',
    description='Paquete para mover el robot a través de diferentes puntos.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'move_robot = robot_movement.movement_node:main',  # Esta es la clave
        ],
    },
)
