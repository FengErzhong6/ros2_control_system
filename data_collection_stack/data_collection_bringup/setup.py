from glob import glob
import os

from setuptools import find_packages, setup


package_name = "data_collection_bringup"


def package_files(root_directory: str) -> list[tuple[str, list[str]]]:
    results: list[tuple[str, list[str]]] = []
    for current_path, _, filenames in os.walk(root_directory):
        if not filenames:
            continue
        install_path = os.path.join("share", package_name, current_path)
        file_paths = [os.path.join(current_path, filename) for filename in filenames]
        results.append((install_path, file_paths))
    return results


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
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
        *package_files("config"),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mmlab",
    maintainer_email="mmlab@example.com",
    description="Top-level launch and configuration package for the data collection stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "performance_smoke = data_collection_bringup.performance_smoke:main",
            "timestamp_relay = data_collection_bringup.timestamp_relay:main",
        ],
    },
)
