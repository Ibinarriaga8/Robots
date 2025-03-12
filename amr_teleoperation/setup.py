from setuptools import find_packages, setup

package_name = "amr_teleoperation"  # Rellenar

setup(
    name=package_name,
    version="1.0.0",  # Rellenar
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Jaime Boal",  # Rellenar
    maintainer_email="jboal@comillas.edu",  # Rellenar
    description="Pub/sub tutorial for Autonomous Mobile Robots @ Comillas ICAI.",  # Rellenar
    license="Apache License 2.0",  # Rellenar
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [  # Rellenar con la función inicial de todos los nodos del paquete
            "keyboard_node = amr_teleoperation.keyboard_node:main",  # nombre_del_nodo = nombre_del_paquete.nombre_del_archivo_python:main
            "teleoperation_node = amr_teleoperation.teleoperation_node:main",
        ],
    },
)
