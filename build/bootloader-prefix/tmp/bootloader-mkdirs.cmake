# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "/Users/robert/esp/esp-idf/components/bootloader/subproject"
  "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader"
  "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix"
  "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix/tmp"
  "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix/src/bootloader-stamp"
  "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix/src"
  "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "/Users/robert/Documents/MiniQuad/i2c_simple/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
