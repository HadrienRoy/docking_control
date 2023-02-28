from setuptools import setup

package_name = 'my_apriltag'

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
    maintainer='hadrien',
    maintainer_email='hadrien@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "detect_tag = my_apriltag.detect_tag:main",
            "detect_tag_swat = my_apriltag.detect_tag_swat:main",
            "detect_tag_pupil = my_apriltag.detect_tag_pupil:main"
        ],
    },
)
