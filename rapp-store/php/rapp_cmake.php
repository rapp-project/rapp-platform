<?php


/**
 * @brief Open a CMakeLists.txt file, and populate it with correct project information prior to compilation & linking
 * 
 * @version 1
 * @date 7-11-2014
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 * @copyright Ortelio Ltd
 */
function rapp_cmake ( $rapp_dir, $rapp_name, $version, $libs, $pkgs, $gcc_flags )
{
    $cwd = getcwd();
    chdir( "../userdir/" . $rapp_dir );
    echo ( getcwd() . "\r\n" );
    if ( $cmake = fopen("CMakeLists.txt", "w") )
    {
        // CMake project name
        fwrite( $cmake, "PROJECT(". $rapp_name .")\r\n" );
        // TODO: project version minor & major (explode version using dot)
        // CMake version required
        fwrite( $cmake, "cmake_minimum_required(VERSION 2.8)\r\n" );
        // Include Paths @note subject to OS
        fwrite( $cmake, "set(LIBRARY_PATH \${LIBRARY_PATH}
		      /lib
		      /usr/lib
		      /usr/lib64
		      /usr/local/lib
		      /usr/local/lib64
		      /usr/lib/x86_64-linux-gnu)\r\n" );
        // Header Include path - @note may be adjusted to include user's headers if needed?
        fwrite( $cmake, "set(INCLUDE_HEADERS \${INCLUDE_HEADERS} /usr/include/)\r\n" );
        // System Includes/Headers - @note everything should be there
        fwrite( $cmake, "include_directories(SYSTEM \${INCLUDE_HEADERS})\r\n" );
        
        // TODO The tricky part: iterate $rapp_dir/src for subdirs, and add them in CMakeLists.txt - @warning NON-Recursive only 1st lvl subdirs
        $Directory = new RecursiveDirectoryIterator( $rapp_dir ."/src" );
        $iter = new RecursiveIteratorIterator( $Directory );
        foreach ( $iter as $dir )
        {
            // TODO: $dir probably needs exploding from the absolute path to the relative path
            fwrite( $cmake, "add_subdirectory(" . $dir . ")\r\n" );
        }
        
        // Iterate $libs, and search for them `find_lib`
        foreach ( $libs as $library )
        {
            // TODO: Convert $library to all CAPS, and use as keyword in CMake
            $tag = convertoupper( $library );
            fwrite( $cmake, "find_library($tag NAMES $library PATHS LIBRARY_PATH)\r\n" );
            fwrite( $cmake, "if ($tag)\r\n" );
            fwrite( $cmake, "message(STATUS \${$tag})\r\n" );
            fwrite( $cmake, "target_link_libraries($rapp_name \${$tag})\r\n" );
            fwrite( $cmake, "else (!$tag)\r\n" );
            fwrite( $cmake, "message(FATAL_ERROR \"missing $library\")\r\n" );
            fwrite( $cmake, "endif($tag)\r\n" );
        }

        // Iterate $pkgs, and search for them using `find_package`
        foreach ( $pkgs as $pkg )
        {
            fwrite( $cmake, "find_package(" . $library . ")\r\n" );
        }

        // Find executables from all dirs and subdirs, and add them to CMake
        fwrite( $cmake, "add_executable ($rapp_name \r\n" );
        $Directory = new RecursiveDirectoryIterator( $rapp_dir . "/src" );
        $Iterator = new RecursiveIteratorIterator($Directory);
        $Regex = new RegexIterator( $Iterator, '/^.+\.(c|cpp|cxx)$/i', RecursiveRegexIterator::GET_MATCH );
        foreach( $Regex as $name => $object )
        {
            // TODO: Explode the absolute path, into a relative path
            fwrite( $cmake, "$name\r\n");
        }
        fwrite( $cmake, ")\r\n" );

        // Always build release - never debug
        fwrite( $cmake, "set(CMAKE_BUILD_TYPE Release)\r\n" );

        // TODO... Check compiler, add flags, and set whats left
    }
}
?>
