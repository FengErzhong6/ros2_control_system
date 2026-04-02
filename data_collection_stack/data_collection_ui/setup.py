import os

from setuptools import find_packages, setup


package_name = "data_collection_ui"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [os.path.join("resource", package_name)],
        ),
        (os.path.join("share", package_name), ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mmlab",
    maintainer_email="mmlab@example.com",
    description="Operator-facing UI skeleton for the data collection stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "ui_stub = data_collection_ui.main:main",
        ],
    },
)
