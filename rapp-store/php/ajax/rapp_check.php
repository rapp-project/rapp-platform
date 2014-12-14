<?php

if ( isset( $_POST['rapp'] ) )
{
    error_log( "checking rapp name: " . $_POST['rapp'] );
    require( '../db.php' );
    $mysqli = new mysqli( db_host, db_user, db_pwd, db_name );
    
    // Always test first if mysql connection was possible
    if ( mysqli_connect_errno() ) error_log ( "Connect failed: " . mysqli_connect_error() );
    
    // Check if such a RAPP already exists
    if ( $stmt = $mysqli->prepare( "SELECT `directory`, `version`, `arch`, `owner`, `lang` FROM `tblRapp` WHERE `rapp`=?;") )
    {
        if ( !($stmt->bind_param( 's', $_POST['rapp'] ) ) )
            error_log ('bind_param() failed :' . $mysqli->error );
        
        if ( !($stmt->execute() ) )
            error_log ('execute() failed :' . $mysqli->error );
        
        $version = null;
        $arch = null;
        $owner = null;
        $dir = null;
        $lang = null;
        $json = "[";
        
        if ( !($stmt->bind_result( $dir, $version, $arch, $owner, $lang ) ) )
            error_log ('bind_result() failed :' . $mysqli->error );
        
        // Got a result, echo it back to JavaScript as JSON
        while ( $stmt->fetch() )
        {   
            $mysqli2 = new mysqli( db_host, db_user, db_pwd, db_name );
            if ( mysqli_connect_errno() ) error_log ( "Connect failed: " . mysqli_connect_error() );
            
            if( $stmt2 = $mysqli2->prepare( "SELECT `username` FROM `tblUser` WHERE `id`=?;") )
            {
                if ( !$stmt2->bind_param( "i", $owner ) )
                    error_log ('bind_param() failed :' . $mysqli->error );
                    
                if ( !$stmt2->execute() )
                    error_log ('execute() failed :' . $mysqli->error );
                    
                $result = null;
                    
                if ( !($stmt2->bind_result( $result ) ) )
                    error_log ('bind_result() failed :' . $mysqli->error );
                
                if ( $stmt2->fetch() )
                {
                    $json .= "{ \"name\" : \"".$_POST['rapp']."\", \"version\" : \"$version\", \"cpuarch\" : \"$arch\", \"owner\" : \"" 
                          . $result ."\", \"directory\" : \"$dir\", \"lang\" : \"$lang\" },";
                }
                else
                    error_log ( "stmt2 could not fetch username from owner" );
                    
                $stmt2->close();
                $mysqli2->close();
            }
            else error_log ( 'could not prepare 2nd query' );
        }
        
        if ( strlen( $json ) > 1 ) $json = substr_replace( $json ,"",-1 );
        $json .= "]";
        
        error_log ( $json );
        echo ( $json );
        
        $stmt->close();
    }
    else
        error_log('prepare() failed: ' . $mysqli->error );
        
    $mysqli->close();
}
else error_log ( 'no $_POST[rapp]' );


?>