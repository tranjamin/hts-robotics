from setuptools import setup
import os

package_name = 'hts_anygrasp'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/' + package_name, ['package.xml']),
     ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
     ("lib/" + package_name, ["hts_anygrasp/gsnet.so", "hts_anygrasp/lib_cxx.so"]),
   ],
 install_requires=['setuptools'],
 zip_safe=False,
 entry_points={
     'console_scripts': [
             'anygrasp_node = hts_anygrasp.anygrasp_node:main'
     ],
   },
)