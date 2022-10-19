from setuptools import setup, find_packages

setup(
    name="commonroad-dataset-converter",
    version="0.1.0",
    description="CommonRoad Dataset Converter",
    author="Cyber-Physical Systems Group, Technical University of Munich",
    author_email="commonroad@lists.lrz.de",
    url="https://commonroad.in.tum.de",
    classifiers=[
        "Programming Language :: Python",
        "Programming Language :: Python :: 3",
        "Programming Language :: Python :: 3.7",
        "Operating System :: OS Independent",
    ],
    python_requires='>=3.7',
    packages=find_packages(),
    package_data={
        "": ["*.yml", "*.yaml"],
    },
    install_requires=[
        "numpy>=1.18.2",
        "scipy>=1.4.1",
        "pandas>=0.24.2",
        "ruamel.yaml>=0.16.10",
        "commonroad-io>=2020.3",
        "typer>=0.4.0",
    ],
    entry_points={
        "console_scripts": [
            "crconvert=commonroad_dataset_converter.main:cli"
        ]
    }
)
