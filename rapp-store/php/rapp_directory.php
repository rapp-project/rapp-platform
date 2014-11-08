<?php

/**
 * Create a RAPP directory, and populate it with:
 *      /src dir (copy source files)
 *      /build dir (used later for compilation)
 *      /includes (additional header includes - optional)
 *
 *  @param $rapp_name is the base package name
 *  @param $version is current (integer) version
 *  @param $arch is the current cpu architecture
 *  @return void
 *
 *  @version 1
 *  @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 *  @date 7-11-2014
 *  @copyright Ortelio ltd
 */

function rapp_dir_create ( $rapp_name, $version, $arch )
{
    if ( empty($rapp_name) || empty($version) || empty($arch) )
        throw new Exception( "rapp_dir_create null param" );

    // Move up one level from cwd (/php)
    $cwd = getcwd();
    chdir( "../" );
    $ref_dir = getcwd();

    // Calculate base dir via ref_dir + rapp_name + version + arch
    $rapp_dir = $rapp_name . "-" . $version . "_" . $arch;
    $base_dir = $ref_dir . "/userdir/" . $rapp_dir;
    $dirs = array( $base_dir, $base_dir . "/src", $base_dir . "/build", $base_dir . "/includes" );

    foreach ( $dirs as &$value )
    {
        // DANGER: Take care of the umask used - it should allow only read to others
        if ( !mkdir ( $value, 0777, true ) )
        {
            error_log('Failed to create dir: ' . $value );
        }   
        //echo ( $value ."\r\n" );
    }

    // chdir into base dir
    if ( chdir( "userdir/" . $rapp_dir ) )
    {
        // Touch CMakeLists.txt
        if ( !touch( "CMakeLists.txt" ) ) error_log ( "Failed to touch CMakeLists.txt in " . getcwd() );
    }
    else
        error_log ( "Failed to chdir into " . "userdir/" . $rapp_dir . " from " . getcwd() );

    // Return local reference to the newly created rapp_dir
    return $rapp_dir;
}


/**
 * Populate @param rapp_dir and @param subdir (e.g., /src, /includes) using as source the files found in the array
 * 
 * @version 1
 * @date 6-11-2014
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 */
function rapp_dir_populate ( array $files, $rapp_dir, $subdir )
{
    if ( empty($files) || empty($rapp_dir) || empty($subdir) )
        throw new Exception( "rapp_dir_populate null param" );
   
    //echo ( $rapp_dir . "\r\n" );
    // Move up one level from cwd (/php)
    $cwd = getcwd();
    chdir( "../" );
    $ref_dir = getcwd();

    $target_dir = $ref_dir . "/" . $rapp_dir . "/" . $subdir;
    //echo ( $target_dir . "\r\n" );

    foreach ( $files as $filepath )
    {
        echo ( $filepath . "\r\n");
        // get basename from filepath
        $file = basename( $filepath );
        // calculate new target
        $target = $target_dir . "/" . $file;
        // Try to copy
        if ( !copy( $filepath, $target ) ) error_log( "failed to copy $file...\n" );
    }
    
    // TODO???
}

?>
