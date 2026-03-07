from setuptools import find_packages, setup

setup(
    name="dmrobotics",
    version="0.1.0",
    python_requires=">=3.8, <3.11",
    packages=find_packages(),
    license="",
    author="Yipai Du",
    author_email="yipai.du@outlook.com",
    install_requires=[
        "numpy==1.24.4",
        "opencv-python==4.10.0.84",
        "opencv-contrib-python==4.10.0.84",
        "scipy==1.10.1",
        "setuptools==45.2.0",
        "cupy-cuda12x==12.3.0",
        'pyudev; platform_system=="Linux"',  # For Linux platform
    ],
    package_data={"": ["*.so"]},
    include_package_data=True,
    description="Tactile sensor interface for Daimon Robotics.",
)