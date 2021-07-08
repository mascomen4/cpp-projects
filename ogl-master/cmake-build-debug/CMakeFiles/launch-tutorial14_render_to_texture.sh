#!/bin/sh
bindir=$(pwd)
cd /home/ivan/ivan/git/ogl-master/tutorial14_render_to_texture/
export 

if test "x$1" = "x--debugger"; then
	shift
	if test "xYES" = "xYES"; then
		echo "r  " > $bindir/gdbscript
		echo "bt" >> $bindir/gdbscript
		/usr/bin/gdb -batch -command=$bindir/gdbscript --return-child-result /home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial14_render_to_texture 
	else
		"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial14_render_to_texture"  
	fi
else
	"/home/ivan/ivan/git/ogl-master/cmake-build-debug/tutorial14_render_to_texture"  
fi
