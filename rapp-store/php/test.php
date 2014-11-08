<?php

require_once( 'rapp_directory.php' );
require_once( 'rapp_scanner.php' );

// create a rapp directory for C++ - NOTE: Javascript and Python don't need this step - only packaging
$rapp_dir = rapp_dir_create ( "helloworld", 0.1, "i386" );

// scan a directory for C++ files - test with a HelloWorld.cpp
$files = rapp_cpp_scan ( "/Users/alex/projects/helloworld", true );

// Copy C++ files into test_rapp's directory, into /src
rapp_dir_populate( $files, $rapp_dir, "src" );

?>
