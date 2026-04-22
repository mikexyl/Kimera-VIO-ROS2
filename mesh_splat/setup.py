from glob import glob
from setuptools import find_packages, setup


package_name = "mesh_splat"


setup(
    name=package_name,
    version="0.0.1",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/launch", glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mike Liu",
    maintainer_email="mikexyl@example.com",
    description="ROS2 Python textured mesh subscriber for Rerun visualization.",
    license="BSD",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mesh_splat_node = mesh_splat.mesh_splat_node:main",
            "dataset_recorder = mesh_splat.dataset_recorder:main",
        ],
    },
)
