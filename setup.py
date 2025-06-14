from setuptools import find_packages, setup

package_name = 'ros2_opencv'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='roiavira',
    maintainer_email='roiavira@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['camera_publisher_node=ros2_opencv.cameraPublisher:main',
        'subscriber_image_node=ros2_opencv.subscriberImage:main',
        'webcam_processing = ros2_opencv.webcam_processing:main',
        'image_detection = ros2_opencv.image_detection:main',
        'robot_control = ros2_opencv.robot_control:main',
        ],
    },
)
