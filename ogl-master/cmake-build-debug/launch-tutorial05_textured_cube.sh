#!/bin/sh
bindir=$(pwd)
cd /home/ivan/ivan/git/ogl-master/tutorial05_textured_cube/
export 

if test "x$1" = "x--debugger"; then
	shift
	if test "xYES" = "xYES"; then
		echo "r  " > $bindir/gdbscript
		echo "bt" >> $bindir/gdbscript
		/usr/bin/gdb -batch -command=$bindir/gdbscript --return-child-result /home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial05_textured_cube 
	else
		"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial05_textured_cube"  
	fi
else
	"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial05_textured_cube"  
fi
