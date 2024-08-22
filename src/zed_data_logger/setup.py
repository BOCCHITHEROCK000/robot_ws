from setuptools import setup

package_name = 'zed_data_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='ZED velocity logger',
    license='Apache License 2.0',
    entry_points={
        'console_scripts': [
            'zed_data_logger = zed_data_logger.zed_data_logger:main',
        ],
    },
)
