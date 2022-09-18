This folder contais scripts needed to build libpico.a from source.

The library it builds is similar to the library shipped with Arduino for RP2040 except for the following:
1. The CYW43 wifi driver is not included.
2. The tinyusb build sets CFG_TUSB_OS to OPT_OS_FREERTOS instead of OPT_OS_PICO.

The library is built using CMake from the command line. File CMakeLists.txt in this folder is the master CMake file and is derived from the one used to build Arduino libpico.

To build the library under Windows:
1. Install MSYS2 on your Windows PC, choosing the default install folder which is C:\msys64
2. Ensure that CMake.exe is on your PATH
3. Open a command prompt and run: make-libpico.bat

The make-libpico.sh is the original one from Arduino. If you wish to build libpico under Linux, you will need to modify it to align with make-libpico.bat.