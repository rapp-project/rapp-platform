<?php

header('Content-Type: text/plain; charset=utf-8');
require_once( '../globals.php' );
session_start();

try
{    
    // Undefined | Multiple Files | $_FILES Corruption Attack
    // If this request falls under any of them, treat it invalid.
    if ( !isset ( $_FILES['upfile']['error']) || is_array($_FILES['upfile']['error'] ) ) 
        throw new RuntimeException('Invalid parameters.');

    // Check $_FILES['upfile']['error'] value.
    switch ( $_FILES['upfile']['error'] )
    {
        case UPLOAD_ERR_OK:
            break;
            
        case UPLOAD_ERR_NO_FILE:
            throw new RuntimeException('No file sent.');
            
        case UPLOAD_ERR_INI_SIZE:
        
        case UPLOAD_ERR_FORM_SIZE:
            throw new RuntimeException('Exceeded filesize limit.');
            
        default:
            throw new RuntimeException('Unknown errors.');
    }

    // You should also check filesize here. 
    if ( $_FILES['upfile']['size'] > 1000000 )
        throw new RuntimeException('Exceeded filesize limit.');

    // DO NOT TRUST $_FILES['upfile']['mime'] VALUE !!
    // Check MIME Type by yourself.
    $finfo = new finfo( FILEINFO_MIME_TYPE );
    
    if ( false === $ext = array_search ( $finfo->file($_FILES['upfile']['tmp_name']),
                                         array( 'jpg' => 'image/jpeg',
                                                'png' => 'image/png',
                                                'gif' => 'image/gif' ), true ) )
    {
        throw new RuntimeException('Invalid file format.');
    }
    
    if ( isset ( $_POST["name"] ) )
        error_log ( "rename thumbnail to : " . $_POST["name"] );
    
    // Destination = directory + filename - extension is already given
    $fullpath = thumbdir . $_FILES['upfile']['name'];
    
    // Try to move it
    if ( !move_uploaded_file( $_FILES['upfile']['tmp_name'], $fullpath ) )
    {
        throw new RuntimeException ('Failed to move uploaded file.');
    }
    else
    {
        $thumb = new Imagick( $fullpath );
        $thumb->resizeImage( 128, 128, Imagick::FILTER_LANCZOS, 1 );
        $thumb->writeImage( $fullpath );
        $thumb->destroy();
        echo basename( $fullpath );
        
        $_SESSION["rapp"]["thumbnail"] = $fullpath;
    }
} 
catch (RuntimeException $e) 
{
    echo $e->getMessage();
}

?> 
