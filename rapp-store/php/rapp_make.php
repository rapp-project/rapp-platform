<?php

/**
 * @brief Execute a `make` command into the `build` directory of a RAPP project and capture all output and errors
 *
 * @version 1
 * @date 8-11-2014
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 * @copyright Ortelio Ltd
 *
 * @warning Method assumes CWD is /rapp-store/userdir
 */
function rapp_make ( $rapp_dir, &$result )
{
    if ( empty( $rapp_dir ) )
        throw new Exception ( "rapp_make: null rapp_dir param" );

    // cd into the "/rapp-dir/src/build"
    $dir = getcwd() . "/" . $rapp_dir . "/src/build";
    if ( !chdir( $dir ) )
        throw new Exception ( "rapp_make could not chdir correctly into " . $dir );
        
    $cwd = getcwd();
    echo ( "Compiling in: " . $cwd . "\r\n");
    
    // Touch the stderr.log - we will save everything in there
    touch( $cwd ."/stderr.log" );
    
    // Set descriptors
    $descr = array(
                    0 => array( "pipe", "r" ), // stdin
                    1 => array( "pipe", "w" ), // stdout
                    2 => array( "file", $cwd ."/stderr.log", "w" )  // stderr save to file
                  );
    
    // Actual Process
    $pid = proc_open( "make", $descr, $pipes );
    
    // If process failed
    if (!is_resource( $pid) ) 
        throw new Exception ( "Could not exec `make` in " . $cwd );
    
    // Process succeeded
    else if ( is_resource( $pid ) )
    {
        // Get std::out
        $stdout = stream_get_contents( $pipes[1] );
        
        // always release the pipes before killing the process - or we will get a deadlock
        fclose( $pipes[1] );
        
        // release the process & get return code
        $retval = proc_close( $pid );
    }
    
    // Capture any std::err from the file - stderr pipe doesn't work
    $stderr = file_get_contents( $cwd ."/stderr.log" );
    
    //echo ( "cmake stdout: \r\n" . $stdout . "\r\n" );
    //echo ( "cmake stderr: \r\n" . $stderr . "\r\n" );
    
    // cd back to /userdir from /userdir/rapp-dir/src/build
    chdir( "../../../" );
    
    // If stderr is empty and we have an stdout - return stdout
    if ( empty( $stderr ) && !empty( $stdout ) )
    {
        $result = $stdout;
        return true;
    }
    // else if there is text in stderr - return both stdout and stderr
    else if ( !empty( $stderr ) )
    {
        $result = $stdout . "\r\n" . $stderr;
        return false;
    }
}
 
?>