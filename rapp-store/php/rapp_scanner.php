<?php

/**
 * Scan a Directory for C++ files
 * Search @param path, and iterate its contents and subdirectories in order to find files that match
 * the specific REGEX: *.hpp, *.h, *.hxx and *.cpp, *.cxx, *.c files
 * @return as array
 * @param verbose used for echoing contents discovered
 * @note $path should be a /tmp directory used by Apache2 to upload user files. Those files should be copied and removed later on.
 *
 * @version 1
 * @author Alex Giokas <a.gkiokas@ortelio.co.uk>
 * @date 7-11-2014
 * @copyright Ortelio Ltd
 *
 * @warning Methods assume that CWD is ???
 */
function rapp_cpp_scan ( $path, $verbose )
{
    $result = array();
    $Directory = new RecursiveDirectoryIterator( $path );
    $Iterator = new RecursiveIteratorIterator( $Directory );
    $Regex = new RegexIterator( $Iterator, '/^.+\.(hpp|h|hxx|h++|hh|c|cpp|cxx|cc|c++)$/i', RecursiveRegexIterator::GET_MATCH );

    foreach( $Regex as $name => $object )
    {
        if ( $verbose ) echo ( $name . "\r\n" );
        array_push( $result, $name );
    }
    return $result;
}

// Scan a Directory for JavaScript files
function rapp_js_scan ( $path, $verbose )
{
    $result = array();
    $Directory = new RecursiveDirectoryIterator( $path );
    $Iterator = new RecursiveIteratorIterator( $Directory );
    $Regex = new RegexIterator( $Iterator, '/^.+\.(js|script)$/i', RecursiveRegexIterator::GET_MATCH );

    foreach( $Regex as $name => $object )
    {
        if ( $verbose ) echo ( $name . "\r\n" );
        array_push( $result, $name );
    }
    return $result;

}

// Scan a Directory for Python files
function rapp_py_scan ( $path, $verbose )
{
    $result = array();
    $Directory = new RecursiveDirectoryIterator( $path );
    $Iterator = new RecursiveIteratorIterator( $Directory );
    $Regex = new RegexIterator( $Iterator, '/^.+\.(py|python)$/i', RecursiveRegexIterator::GET_MATCH );

    foreach( $Regex as $name => $object )
    {
        if ( $verbose ) echo ( $name . "\r\n" );
        array_push( $result, $name );
    }
    return $result;

}

?>
