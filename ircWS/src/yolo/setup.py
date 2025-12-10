from setuptools import setup

package_name = 'yolo'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vighneshreddy',
    maintainer_email='example@email.com',
    description='YOLO inference node with ZED depth and cone color classification.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'inference = yolo.inference_node:main',
        ],
    },
)

