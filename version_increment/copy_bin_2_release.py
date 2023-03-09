""" Copy firmware.bin to release folder """
##############################################################
# Copy Binary-Files after Compilation to Release-Folder
#
# - ONLY if Target is UPLOAD
#
# - Add Version and Target to Destination-Directory 
#   e.g.: ./releases/1.0.1_OTA-Test/firmware.bin
#
# Source-Files:
#  - ./.pio/build/[$PIOENV]/firmware.bin
#  - ./.pio/build/[$PIOENV]/firmware-factory.bin
#
# Release-Folder:
#  - ./releases/[VERSION]_[$PIOENV]
#
# Files needed:
# - ./version  
#
##############################################################
# Copyright (C) 2022  Dario Carluccio
##############################################################

import os
import sys
import shutil
from genericpath import exists

Import("env")
env = DefaultEnvironment()

VERSION_FILE = 'version'

def esp32_copy_firmware_bin(source, target, env):
    print("### CF2R   Copy-Firmware.Bin-2-Release started")
    if os.path.exists(VERSION_FILE):            
        print("### CF2R   Release file found")
        # get current release
        with open(VERSION_FILE) as FILE:
            VERSION_PATCH_NUMBER = FILE.readline()            
        print('### CF2R   Current Release: {}'.format(VERSION_PATCH_NUMBER))
        # get env Name
        TARGET = env.subst("$PIOENV")
        # get Destination Dir
        PROJECT_DIR = env.subst("$PROJECT_DIR")
        RELEASE_DIR = PROJECT_DIR + os.sep + "releases"
        if not os.path.exists(RELEASE_DIR):
            os.mkdir(RELEASE_DIR)
        THIS_TARGET_DIR = RELEASE_DIR + os.sep + VERSION_PATCH_NUMBER + '_' + TARGET        
        if not os.path.exists(THIS_TARGET_DIR):
            os.mkdir(THIS_TARGET_DIR)
        # Files
        firmware_bin = env.subst("$BUILD_DIR/${PROGNAME}.bin")
        firmware_factory_bin = env.subst("$BUILD_DIR/${PROGNAME}-factory.bin")
        print('### CF2R   Source firmware.bin: {}'.format(firmware_bin))
        print('### CF2R   Source firmware-factory.bin: {}'.format(firmware_factory_bin))
        print('### CF2R   Destination dir: {}'.format(THIS_TARGET_DIR))
        if os.path.exists(firmware_bin):
            shutil.copy2(firmware_bin, THIS_TARGET_DIR)
            print('### CF2R   {} copied'.format(firmware_bin))
        if os.path.exists(firmware_factory_bin):
            shutil.copy2(firmware_factory_bin, THIS_TARGET_DIR)
            print('### CF2R   {} copied'.format(firmware_factory_bin))
    else:
        print("### CF2R: NO version file found, exiting")
    print('### CF2R   END')

# env.AddPostAction("$BUILD_DIR/${PROGNAME}.bin", esp32_copy_firmware_bin)
env.AddPostAction("upload", esp32_copy_firmware_bin)

