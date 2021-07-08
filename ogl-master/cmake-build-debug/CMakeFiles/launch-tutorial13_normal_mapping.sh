#!/bin/sh
bindir=$(pwd)
cd /home/ivan/ivan/git/ogl-master/tutorial13_normal_mapping/
export 

if test "x$1" = "x--debugger"; then
	shift
	if test "xYES" = "xYES"; then
		echo "r  " > $bindir/gdbscript
		echo "bt" >> $bindir/gdbscript
		/usr/bin/gdb -batch -command=$bindir/gdbscript --return-child-result /home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial13_normal_mapping 
	else
		"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial13_normal_mapping"  
	fi
else
	"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial13_normal_mapping"  
fi
