from setuptools import setup

package_name = 'final_project'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ramvbox',
    maintainer_email='ramvbox@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [ 'robot_control=final_project.robot_control:main',
        'robot_recognition=final_project.robot_recognition:main',
        'obstacle_avoidance=final_project.obstacle_avoidance:main',
        'image_sub_save=final_project.image_save:main',
        ],
    },
)
