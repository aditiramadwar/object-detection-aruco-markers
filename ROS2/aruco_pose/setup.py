from setuptools import setup

package_name = 'aruco_pose'

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
    maintainer='x_lab',
    maintainer_email='adiram@umd.edu',
    description='Pose Estimation of Aruco Markers',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'get_pose = aruco_pose.get_pose:main',
                #'talker = aruco_pose.publisher_member_function:main',
                #'listener = aruco_pose.subscriber_member_function:main',
                #'img_publisher = aruco_pose.basic_image_publisher:main',
                #'img_subscriber = aruco_pose.basic_image_subscriber:main',
        ],
    },
)
