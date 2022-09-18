rmdir /S /Q build
mkdir build
cd build
cmake .. -DIPV6=0 -G "MSYS Makefiles"
set OLDPATH=%PATH%
PATH C:\msys64\usr\bin;%PATH%
c:\msys64\usr\bin\make -j
PATH %OLDPATH%
cd ..
