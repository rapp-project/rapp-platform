<?php

/** 
 * TODO: Create a RAPP by:
                            create a rapp directory
                            scan uploaded files, 
                            copy them in rapp directory, 
                            (optional) populate CMakeLists.txt
                            (optional) compile & link a C++ app, 
                            create a json descriptor file for the rapp, 
                            packaging into a tar.gz (HZ package)
                            copy to correct path (HOP Server)
 */

require_once( 'rapp_directory.php' );
require_once( 'rapp_scanner.php' );
require_once( 'rapp_cmake.php' );

// create a rapp directory 
$rapp_dir = rapp_dir_create ( "helloworld", 0.1, "i386" );

// scan a directory for C++ files
$files = rapp_cpp_scan ( "/Users/alex/projects/helloworld", true );

/** 
 * Copy C++ files into test_rapp's directory, into /src - 
 * @note if user is importing other headers, copy them too
 * @warning entry point must be defined as a file (e.g., main.cpp)
 */
rapp_dir_populate( $files, $rapp_dir, "src" );

// Populate the CMakeLists.txt for the rapp
rapp_cmake( $rapp_dir, "helloworld", 0.1, $files, $link_libs, $gcc_flags );

?>
