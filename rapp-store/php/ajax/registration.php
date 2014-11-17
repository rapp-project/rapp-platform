<?php

if ( isset( $_POST['form'] ) )
{
    require( '../db.php' );
    //error_log ( $_POST['form'] );
    
    $json = json_decode( $_POST['form'], true );
    
    $error_msg = "<div class=\"ui-state-error ui-corner-all\" style=\"padding: 0 .7em;\" id=\"error_msg\">";
    $error = false;
    
    if ( empty( $json['username'] ) )
    {
        $error_msg .= "<p>Empty username - please chose a unique username!</p>";
        $error = true;
    }   
    if ( empty( $json['firstname'] ) )
    {
        $error_msg .= "<p>Empty First Name!</p>";
        $error = true;
    }   
    if ( empty( $json['surname'] ) )
    {
        $error_msg .= "<p>Empty Surname!</p>";
        $error = true;
    }   
    if ( empty( $json['password'] ) )
    {
        $error_msg .= "<p>Empty password - please chose a password</p>";
        $error = true;
    }    
    if ( empty( $json['email'] ) )
    {
        $error_msg .= "<p>Empty email - please enter your email address!</p>";
        $error = true;
    }
    if ( empty( $json['account'] ) )
    {
        $error_msg .= "<p>No Account Type chosen - please chose an account!</p>";
        $error = true;
    }
    
    if ( !$error )
    {   
        // Create an activation uuid now
        require_once( "../guid_v4.php" );
        $activation = uuidv4();
        
        // Remove hyphens
        $activation = str_replace("-","", $activation );
        
        $account = 5;      
        // No such user as type 5 - type 0 is reserved for admin
        switch ( $json['account'] )
        {
            case "Robot User":
                $account = 2;
                break;
                
            case "Developer" :
                $account = 1;
                break;
                
            case "3rd Party Access" :
                $account = 3;
                break;
        }
    
        $mysqli = new mysqli( db_host, db_user, db_pwd, db_name );
        if ( mysqli_connect_errno() ) error_log ( "Connect failed: " . mysqli_connect_error() );
        
        // Prepare the actual query - NOTE: I will need: HEX( `activation` ) to retrieve the HEX values of the binary
        if ( $stmt = $mysqli->prepare( "INSERT INTO `tblUser`( `username`, `firstname`, `lastname`, `email`, `pwd`, `usrgroup`, `activation`) VALUES (?,?,?,?,?,?, UNHEX(?) );") )
        {
            if ( !($stmt->bind_param( 'sssssis', $json['username'], $json['firstname'], $json['surname'], $json['email'], $json['password'], $account, $activation ) ) )
            {
                error_log ('bind_param() failed :' . $mysqli->error );
                die ( $error_msg . "<p>Database Error, please try again later</p></div>" );
            }
            
            if ( !($stmt->execute() ) )
            {
                error_log ('execute() failed :' . $mysqli->error );
                die ( $error_msg . "<p>Database Error, please try again later</p></div>" );
            }
            $stmt->close();
            
            // Everything went Ok
            echo ( "<p>Thank you for registering <i>" .$json['username'] ."</i>. You should soon receive an email with your activation code</p>" );
            
            // TODO: Send an activation email - user has to go to ROOT/activation?=9c4bb3dd09be4034b0b19c4b917bdb48 which then enables the account in mysql
            //require_once( "activation_mail.php" );
            //send_activation( $json['email'], $json['firstname'], $json['username'], $activation );
        }
        else
        {
            error_log('prepare() failed: ' . $mysqli->error );
            die ( $error_msg . "<p>Database Error, please try again later</p></div>" );
        }
    }
    else if ( $error )
        die ( $error_msg . "<p>Errors Encountered Processing your registration</p></div>" );
}
else
    error_log( 'ajax/registration: $_postp[form] is not set' );


?>