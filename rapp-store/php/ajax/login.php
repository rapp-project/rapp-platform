<?php

if ( isset( $_POST['username'] ) && isset( $_POST['password'] ) )
{
    error_log ( "Trying Login: " . $_POST['username'] );
    
    require( '../db.php' );
    $mysqli = new mysqli( db_host, db_user, db_pwd, db_name );
    
    // Always test first if mysql connection was possible
    if ( mysqli_connect_errno() ) die ( "Connect failed: " . mysqli_connect_error() );
    
    // Prepare a statement to check if there is a row with such a username & password
    if ( $stmt = $mysqli->prepare( "SELECT EXISTS (SELECT 1 FROM `tblUser` WHERE `username`=? AND `pwd`=? AND `enabled`='1');") )
    {
        if ( !($stmt->bind_param( 'ss', $_POST['username'], $_POST['password'] ) ) )
            die ('bind_param() failed :' . $mysqli->error );
        
        if ( !($stmt->execute() ) )
            die ('execute() failed :' . $mysqli->error );
        
        $rows = null;
        
        if ( !($stmt->bind_result( $rows ) ) )
            die ('bind_result() failed :' . $mysqli->error );
        
        if ( !($stmt->fetch() ) )
            die ('fetch() failed :' . $mysqli->error );
        
        error_log( "Login Match: " . $rows );
        
        if ( $rows == 0 )
        {
            echo "Wrong Password/Username not found";
        }
        else if ( $rows == 1 )
        {
            require_once( '../user_session.php' );
            $obj = new UserSession( $_POST['username'], $_POST['password'] );
            $_SESSION['user'] = serialize( $obj );
            
            //var_dump( $obj );
            echo "OK";
        }
        
        $stmt->close();
    }
    else
        die('prepare() failed: ' . $mysqli->error );
        
    $mysqli->close();
}

else die ( "username or password fields not posted" );

?>