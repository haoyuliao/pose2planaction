from setuptools import setup

package_name = 'py_pose2plan'

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
    maintainer='haoyuliao',
    maintainer_email='haoyuliao@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'pose2plan = py_pose2plan.pose2plan:main',
        'pose2action = py_pose2plan.pose2action:main',
        ],
    },
)
