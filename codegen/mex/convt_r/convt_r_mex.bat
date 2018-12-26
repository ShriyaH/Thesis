@echo off
set MATLAB=C:\PROGRA~1\MATLAB\R2016b
set MATLAB_ARCH=win64
set MATLAB_BIN="C:\Program Files\MATLAB\R2016b\bin"
set ENTRYPOINT=mexFunction
set OUTDIR=.\
set LIB_NAME=convt_r_mex
set MEX_NAME=convt_r_mex
set MEX_EXT=.mexw64
call "C:\PROGRA~1\MATLAB\R2016b\sys\lcc64\lcc64\mex\lcc64opts.bat"
echo # Make settings for convt_r > convt_r_mex.mki
echo COMPILER=%COMPILER%>> convt_r_mex.mki
echo COMPFLAGS=%COMPFLAGS%>> convt_r_mex.mki
echo OPTIMFLAGS=%OPTIMFLAGS%>> convt_r_mex.mki
echo DEBUGFLAGS=%DEBUGFLAGS%>> convt_r_mex.mki
echo LINKER=%LINKER%>> convt_r_mex.mki
echo LINKFLAGS=%LINKFLAGS%>> convt_r_mex.mki
echo LINKOPTIMFLAGS=%LINKOPTIMFLAGS%>> convt_r_mex.mki
echo LINKDEBUGFLAGS=%LINKDEBUGFLAGS%>> convt_r_mex.mki
echo MATLAB_ARCH=%MATLAB_ARCH%>> convt_r_mex.mki
echo BORLAND=%BORLAND%>> convt_r_mex.mki
echo OMPFLAGS= >> convt_r_mex.mki
echo OMPLINKFLAGS= >> convt_r_mex.mki
echo EMC_COMPILER=lcc64>> convt_r_mex.mki
echo EMC_CONFIG=optim>> convt_r_mex.mki
"C:\Program Files\MATLAB\R2016b\bin\win64\gmake" -B -f convt_r_mex.mk
