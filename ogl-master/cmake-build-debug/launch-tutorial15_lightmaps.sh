#!/bin/sh
bindir=$(pwd)
cd /home/ivan/ivan/git/ogl-master/tutorial15_lightmaps/
export 

if test "x$1" = "x--debugger"; then
	shift
	if test "xYES" = "xYES"; then
		echo "r  " > $bindir/gdbscript
		echo "bt" >> $bindir/gdbscript
		/usr/bin/gdb -batch -command=$bindir/gdbscript --return-child-result /home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial15_lightmaps 
	else
		"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial15_lightmaps"  
	fi
else
	"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial15_lightmaps"  
fi
