<?php

if ( isset( $_POST['username']) && isset($_POST['password']) )
{
    $mysqli = new mysqli( "localhost", "alex", "qwepoi", "rapp_store" );
    if ( $mysqli->connect_errno )
    {
        die("Connect failed: %s\n", $mysqli->connect_error);
    }
    // TODO: Authenticate with a Session Token
    if ( $result = $mysqli->query( "SELECT Name FROM City LIMIT 10") ) 
    {
        printf("Select returned %d rows.\n", $result->num_rows);
        $result->close();
    }
}

else die ( "username or password fields not posted" );

?>