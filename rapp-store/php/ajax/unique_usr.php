<?php

/// Check if $_POST['usr'] is unique in tblUsers
if ( isset( $_POST['usr'] ) )
{
    error_log( "checking unique username: " . $_POST['usr'] );
    require( '../db.php' );
    $mysqli = new mysqli( db_host, db_user, db_pwd, db_name );
    
    // Always test first if mysql connection was possible
    if ( mysqli_connect_errno() ) die ( "Connect failed: " . mysqli_connect_error() );
    
    // Prepare a statement to check if there is a row with such a username
    if ( $stmt = $mysqli->prepare( "SELECT EXISTS (SELECT 1 FROM `tblUser` WHERE `username`=?);") )
    {
        if ( !($stmt->bind_param( 's', $_POST['usr'] ) ) )
            die ('bind_param() failed :' . $mysqli->error );
        
        if ( !($stmt->execute() ) )
            die ('execute() failed :' . $mysqli->error );
        
        $rows = null;
        
        if ( !($stmt->bind_result( $rows ) ) )
            die ('bind_result() failed :' . $mysqli->error );
        
        if ( !($stmt->fetch() ) )
            die ('fetch() failed :' . $mysqli->error );
        
        /// Echo - ajax caller will receive the output
        echo $rows;
        $stmt->close();
    }
    else
        die('prepare() failed: ' . $mysqli->error );
        
    $mysqli->close();
}
else die ( 'no post_[usr]' );

?>
