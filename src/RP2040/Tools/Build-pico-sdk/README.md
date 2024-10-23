This folder contains scripts needed to build libpico.a from source.

The libpico.a file it builds is similar to the one shipped with Arduino for RP2040 except for the following:
1. The CYW43 wifi driver is not included.
2. The tinyusb build sets CFG_TUSB_OS to OPT_OS_FREERTOS instead of OPT_OS_PICO.

The library is built using CMake from the command line. File CMakeLists.txt in this folder is the master CMake file and is derived from the one used to build Arduino libpico.

To build the library under Windows:
1. Install MSYS2 on your Windows PC, choosing the default install folder which is C:\msys64
2. In MSYS2 use pacman to install the 'make' program using command: pacman -S make
3. Ensure that CMake.exe is on your PATH
4. Ensure that 'ar' (the GNU library manager) is on your PATH. You can use the one in your ARM compiler folder or the one in MSYS2.
5. Open a command prompt and run: make-libpico.bat

Run the `make-libpico.sh` script to build the SDK

> It should run on both linux and Windows, Windows may need Visual Studio installed to complete the build.
