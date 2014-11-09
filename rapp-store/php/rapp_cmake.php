<?php
/**
 * @brief Open a CMakeLists.txt file, and populate it with correct project information prior to compilation & linking
 * 
 * @version 1
 * @date 7-11-2014
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 * @copyright Ortelio Ltd
 *
 * @warning Method assumes CWD is /rapp-store/userdir
 */
function rapp_new_cmake ( $rapp_dir, $rapp_name, $version, array $files, array $libs = NULL, array $pkgs = NULL, array $gcc_flags = NULL )
{
    // TODO: Check @params and throw if null
    
    // cd into the rapp_dir - abort if we cannot
    $dir = getcwd() . "/" . $rapp_dir;
    if ( !chdir( $dir ) )
        throw new Exception( "rapp_new_cmake could not chdir correctly into " . $dir );

    $cwd = getcwd();
    echo ( "New CMakeLists.txt in: " . $cwd . "\r\n" );
    
    if ( $cmake = fopen("src/CMakeLists.txt", "w") )
    {
        // CMake project name
        fwrite( $cmake, "PROJECT(". $rapp_name .")\r\n\r\n" );
        
        // CMake version required
        fwrite( $cmake, "cmake_minimum_required(VERSION 2.8)\r\n\r\n" );
        // Include Paths @note subject to OS
        fwrite( $cmake, "set(LIBRARY_PATH \${LIBRARY_PATH}
		      /lib
		      /usr/lib
		      /usr/lib64
		      /usr/local/lib
		      /usr/local/lib64
		      /usr/lib/x86_64-linux-gnu)\r\n\r\n" );
        // Header Include path - @note may be adjusted to include user's headers if needed?
        fwrite( $cmake, "set(INCLUDE_HEADERS \${INCLUDE_HEADERS} /usr/include/)\r\n\r\n" );
        
        // System Includes/Headers - @note everything should be there
        fwrite( $cmake, "include_directories(SYSTEM \${INCLUDE_HEADERS})\r\n\r\n" );
        
        // Iterate 1 level of subdirectories
        foreach ( new DirectoryIterator( $cwd ."/src" ) as $fileInfo )
        {
            if ( $fileInfo->isDir() )
            {
                if ( $fileInfo->getFilename() != "." &&
                     $fileInfo->getFilename() != ".." &&
                     $fileInfo->getFilename() != "build" )
                {
                    fwrite( $cmake, "add_subdirectory(" . $fileInfo->getFilename() . ")\r\n" );
                }
            }
        }
        fwrite( $cmake, "\r\n");
        
        // Iterate $libs, and search for them `find_lib`
        if ( !empty($libs) )
        {
            foreach ( $libs as $library )
            {
                $tag = strtoupper( $library );
                fwrite( $cmake, "#Find user-requested library: $library\r\n");
                fwrite( $cmake, "find_library($tag NAMES $library PATHS LIBRARY_PATH)\r\n" );
                fwrite( $cmake, "if ($tag)\r\n" );
                fwrite( $cmake, "message(STATUS \${$tag})\r\n" );
                fwrite( $cmake, "target_link_libraries($rapp_name \${$tag})\r\n" );
                fwrite( $cmake, "else (!$tag)\r\n" );
                fwrite( $cmake, "message(FATAL_ERROR \"missing $library\")\r\n" );
                fwrite( $cmake, "endif($tag)\r\n\r\n" );
            }
        }

        // Iterate $pkgs, and search for them using `find_package`
        if ( !empty($pkgs) )
        {
            foreach ( $pkgs as $pkg )
            {
                fwrite( $cmake, "#Find user-requested package: $pkg\r\n");
                fwrite( $cmake, "find_package(" . $pkg . ")\r\n\r\n" );
            }
        }

        // Find executables from all dirs and subdirs, and add them to CMake
        fwrite( $cmake, "#Set executables found in `$cwd/src`\r\n");
        fwrite( $cmake, "add_executable ($rapp_name \r\n" );
        
        $Directory = new RecursiveDirectoryIterator( $cwd . "/src" );
        $Iterator = new RecursiveIteratorIterator($Directory);
        $Regex = new RegexIterator( $Iterator, '/^.+\.(c|cpp|cxx)$/i', RecursiveRegexIterator::GET_MATCH );
        
        foreach( $Regex as $name => $object )
        {
            fwrite( $cmake, "\t\t". basename( $name ) . "\r\n" );
        }
        fwrite( $cmake, "\t\t)\r\n\r\n" );

        // Always build release - never debug
        fwrite( $cmake, "#RAPP Store always builds Release\r\n");
        fwrite( $cmake, "set(CMAKE_BUILD_TYPE Release)\r\n\r\n" );

        // Write the Compilation Flags, Compiler Test and Compiler Version Check - NOTE: Flags should be set automatically or by user?
        fwrite( $cmake, "if( CMAKE_COMPILER_IS_GNUCXX )\r\n" );
        fwrite( $cmake, "\texecute_process(COMMAND \${CMAKE_C_COMPILER} -dumpversion OUTPUT_VARIABLE GCC_VERSION) \r\n" );
        fwrite( $cmake, "if (GCC_VERSION VERSION_GREATER 4.7 OR GCC_VERSION VERSION_EQUAL 4.7) \r\n" );
        fwrite( $cmake, "\tset(CMAKE_CXX_FLAGS \${CMAKE_CXX_FLAGS} \"-std=gnu++11 -Wall\") \r\n" );
        fwrite( $cmake, "\tadd_definitions(\${CMAKE_CXX_FLAGS}) \r\n" );
        fwrite( $cmake, "\tset(CMAKE_CXX_FLAGS_DEBUG \"\${CMAKE_CXX_FLAGS_DEBUG}\") \r\n" );
        fwrite( $cmake, "\tset(CMAKE_CXX_FLAGS_RELEASE \"\${CMAKE_CXX_FLAGS_RELEASE} -O2\") \r\n" );
        fwrite( $cmake, "\tset(BUILD_SHARED_LIBS OFF) \r\n" );
        fwrite( $cmake, "elseif (GCC_VERSION VERSION_LESS 4.7) \r\n" );
        fwrite( $cmake, "\tmessage (FATAL_ERROR \"GCC Version >= 4.7 is required\") \r\n" );
        fwrite( $cmake, "endif() \r\n" );
        fwrite( $cmake, "elseif() \r\n" );
        fwrite( $cmake, "\tmessage (FATAL_ERROR \"Project has been developed only for GCC\") \r\n" );
        fwrite( $cmake, "endif()\r\n" );
    }
    
    // cd back to /userdir
    chdir( "../" );
}


/**
 * @brief Run `cmake ..` from the RAPP's build directory
 * @version 1
 * @date 8-11-2014
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 * @copyright Ortelio ltd
 *
 * @param $rapp_dir must be the root directory of a valid rapp project
 * @return string: on success stdout, or if failed, the stderr
 *
 * @warning Method assumes CWD is /rapp-store/userdir
 */
function rapp_run_cmake ( $rapp_dir, &$result )
{
    if ( empty( $rapp_dir ) )
        throw new Exception ( "rapp_run_cmake null rapp_dir param" );
    
    $dir = getcwd() . "/" . $rapp_dir . "/src/build";
    if ( !chdir( $dir ) )
        throw new Exception ( "rapp_run_cmake could not chdir correctly into " . $dir );
        
    $cwd = getcwd();
    echo ( "Running cmake in: " . $cwd . "\r\n");
    
    $stdout = "";
    $retval = "";
    
    // Touch the stderr.log - we will save everything in there
    touch( $cwd ."/stderr.log" );
    
    // Set descriptors
    $descr = array(
                    0 => array("pipe", 'r'), // stdin
                    1 => array("pipe", 'w'), // stdout
                    2 => array("file", $cwd ."/stderr.log", "w")  // stderr save to file
                  );
    
    // Actual Process
    $pid = proc_open( "cmake ..", $descr, $pipes );
    
    // If process failed
    if (!is_resource( $pid) ) 
        throw new Exception ( "Could not exec `cmake ..` in " . $cwd );
    
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
