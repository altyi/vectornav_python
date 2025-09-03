# The MIT License (MIT)
# 
# VectorNav SDK (v0.22.0)
# Copyright (c) 2024 VectorNav Technologies, LLC
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in
# all copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
# THE SOFTWARE.

from setuptools import setup, Extension
from pybind11.setup_helpers import Pybind11Extension
from pathlib import Path
import platform 
regScan = Path('plugins/PyRegisterScan.cpp')
firUpdt = Path('plugins/PyFirmwareUpdate.cpp')
simpleLogger = Path('plugins/PySimpleLogger.cpp')
dataExp = Path('plugins/PyDataExport.cpp')
calibration = Path('plugins/PyCalibration.cpp')

# Overwrite GNSS groups to enable satInfo and rawMeas, which are disabled by default in c++
macros = [('__PYTHON__', None),('GNSS_GROUP_ENABLE', 0xFFFFFFFF),('GNSS2_GROUP_ENABLE', 0xFFFFFFFF)]
plugins = []
includes = ['../cpp/include/', '../cpp/plugins', '../cpp/libs', './include', '../cpp/plugins/Math']

if regScan.exists():
    print("Adding Register Scan Plugin")
    macros.append(('__REGSCAN__', None))
    plugins.append(str(regScan))
    plugins.extend([
        '../cpp/plugins/RegisterScan/src/RegisterScan.cpp',
        '../cpp/plugins/RegisterScan/src/ConfigReader.cpp',
        '../cpp/plugins/RegisterScan/src/ConfigWriter.cpp'
    ])
    includes.append('../cpp/plugins/RegisterScan/include')

if firUpdt.exists():
    print("Adding Firmware Update Plugin")
    macros.append(('__FIRMWARE_UPDATE__', None))
    plugins.append(str(firUpdt))
    plugins.extend([
        '../cpp/plugins/FirmwareUpdate/src/Bootloader.cpp',
        '../cpp/plugins/FirmwareUpdate/src/FirmwareUpdater.cpp',   
        '../cpp/plugins/FirmwareUpdate/src/VnXml.cpp',   
    ])
    includes.append('../cpp/plugins/FirmwareUpdate/include')

if simpleLogger.exists():
    print("Adding Simple Logger Plugin")
    macros.append(('__SIMPLE_LOGGER__', None))
    plugins.append(str(simpleLogger))
    includes.append('../cpp/plugins/SimpleLogger')

if dataExp.exists():
    print("Adding Data Exprot Plugin")
    macros.append(('__DATAEXPORT__', None))
    plugins.append(str(dataExp))
    plugins.extend([
        '../cpp/plugins/DataExport/src/ExporterCsvUtils.cpp',   
    ])
    includes.append('../cpp/plugins/DataExport/include')

if calibration.exists():
    print("Adding Calibration Plugin")
    macros.append(('__CALIBRATION__', None))
    plugins.append(str(calibration))
    plugins.extend([
        '../cpp/plugins/Calibration/src/Hsi.cpp',
        '../cpp/plugins/Calibration/src/HsiDipole.cpp',
        '../cpp/plugins/Calibration/src/HsiPatch.cpp',
        '../cpp/plugins/Calibration/src/AccBias.cpp'
    ])
    includes.append('../cpp/plugins/Calibration/include')


ext_libs = []
if platform.system() == 'Windows':
    ext_libs.append('winmm')

ext_modules = [
    Pybind11Extension(
        "vectornav",
        [
            # Pybind Files
            'src/vectornav.cpp',
            'src/PyRegisters.cpp',
            'src/PyCommands.cpp',   

            # Implemenation
            '../cpp/src/Implementation/AsciiPacketDispatcher.cpp',
            '../cpp/src/Implementation/AsciiPacketProtocol.cpp',
            '../cpp/src/Implementation/BinaryHeader.cpp',
            '../cpp/src/Implementation/CommandProcessor.cpp',
            '../cpp/src/Implementation/FaPacketDispatcher.cpp',
            '../cpp/src/Implementation/FaPacketProtocol.cpp',
            '../cpp/src/Implementation/FbPacketDispatcher.cpp',
            '../cpp/src/Implementation/FbPacketProtocol.cpp',
            '../cpp/src/Implementation/PacketSynchronizer.cpp',

            # Interface
            '../cpp/src/Interface/GenericCommand.cpp',
            '../cpp/src/Interface/Sensor.cpp',
            '../cpp/src/Interface/Registers.cpp',
            
            # plugins            
            *plugins,
        ],
        include_dirs=includes,
        language="c++",
        cxx_std=17,
        define_macros=macros,
        libraries=ext_libs
    ),
]

setup(
    name='vectornav',
    version='0.22.0',
    ext_modules=ext_modules,
)
