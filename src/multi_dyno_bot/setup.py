from setuptools import setup, find_packages

package_name = 'multi_dyno_bot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='you@example.com',
    description='Dynamic robot coverage with obstacles',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'random_mover = multi_dyno_bot.random_mover:main',
            'go_to_goal = multi_dyno_bot.go_to_goal:main',
            'stc_controller = multi_dyno_bot.stc_controller:main',
            'coverage_visualizer = multi_dyno_bot.coverage_visualizer:main',
            'multi_stc_controller = multi_dyno_bot.multi_stc_controller:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/square_platform.world']),
        ('share/' + package_name + '/launch', ['launch/multi_dyno_bot_launch.py', 'launch/multi_dyno_bot_6bots.launch.py']),
        ('share/' + package_name + '/urdf'  , ['urdf/turtlebot3_burger.urdf']),
        ('share/' + package_name + '/models', ['models/turtlebot3_burger/model.sdf']),
    ],
)
