<?php

/** 
 * TODO: Create a RAPP by:
                            1 - scan uploaded files (if no js, cpp or python files are found, assume wrong RAPP package)
                            2 - create a rapp directory
                            3a- copy source in rapp directory 
                            3b- TODO: additional files that is not source must also be copied
                            4 - (C++) populate CMakeLists.txt
                            5 - (C++) compile & link a C++ app
                            6 - create a json descriptor file for the rapp
                            7 - packaging into a tar.gz (HZ package)
                            8 - copy to correct path (HOP Server)
 */

require_once( 'rapp_directory.php' );
require_once( 'rapp_scanner.php' );
require_once( 'rapp_cmake.php' );
require_once( 'rapp_make.php' );

/* 
    WARNING: Because `cwd` is run many times in the methods below, be EXTRA careful with CWD
    
    TODO: The below methods should run into a different PROCESS, by using the correct POSIX fork-exec.
          Otherwise, we risk messing up parallel operations between different users due to different CWD's !
*/

/// First thing, cwd into "/rapp-store/userdir" from "/rapp-store/php"
if ( cd_to_userdir() )
{
    /// scan a directory for C++ files @note 1st param does not depend on CWD
    $files = rapp_cpp_scan ( "/home/alex/projects/Helloworld", false );
    echo ( "Found C++ files: " . count( $files ) . "\r\n" );

    // create a rapp directory 
    $rapp_dir = rapp_dir_create ( "helloworld", 0.1, "i386" );
    echo ( "RAPP Dir: " . $rapp_dir . "\r\n" );

    // Copy C++ files into test_rapp's directory, into "/rapp-store/userdir/rapp-dir/src"
    rapp_dir_populate( $files, $rapp_dir, "src" );
    echo ( "RAPP C++ Files copied into " . $rapp_dir . "\r\n" );
    
    // TODO: Copy JS & Python files, into respective folders, e.g., as above ^^

    // Populate the CMakeLists.txt for the rapp - NOTE: Extra params need be infered from user information (libs, packages, flags)
    rapp_new_cmake( $rapp_dir, "helloworld", 0.1, $files, null, null, null );

    // CMake and Make output (error or otherwise)
    $cmake_result;
    $make_result;
    
    // Run CMake & Generate MakeFiles
    if ( rapp_run_cmake( $rapp_dir, $cmake_result ) )
    {
        echo( "Makefiles generated OK\r\n" );
        
        // Run `make` for compiling and linking
        if ( rapp_make( $rapp_dir, $make_result ) )
        {
            echo( "RAPP Compiled OK\r\n" );
            
            // TODO: Package the directory with JSON, and compress then move
        }
        else
            echo ( "make error(s)\r\n" . $make_result . "\r\n" );
    }
    else
        echo( "cmake error(s)\r\n" . $cmake_result . "\r\n" );
}


?>
