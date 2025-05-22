from setuptools import setup, find_packages

package_name = 'dyno_bot'

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
            'random_mover = dyno_bot.random_mover:main',
            'go_to_goal = dyno_bot.go_to_goal:main',
            'stc_controller = dyno_bot.stc_controller:main',
            'coverage_visualizer = dyno_bot.coverage_visualizer:main',
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/worlds', ['worlds/square_platform.world']),
        ('share/' + package_name + '/launch', ['launch/dyno_bot_launch.py']),
    ],
)
