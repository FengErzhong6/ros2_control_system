from glob import glob
import os

from setuptools import find_packages, setup


package_name = "policy_deployment_bringup"


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
        (os.path.join("share", package_name, "config", "recipes"), glob("config/recipes/*.yaml")),
        (os.path.join("share", package_name, "config", "policies"), glob("config/policies/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="mmlab",
    maintainer_email="mmlab@example.com",
    description="Bringup and default recipes for the policy deployment stack.",
    license="Apache-2.0",
    tests_require=["pytest"],
)
