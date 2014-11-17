<?php

require_once( 'terms_conditions.php' );

if ( isset( $_POST['key'] ) )
{
    switch ( $_POST['key'] )
    {
        
        case "firstname":
            echo "<p>Please enter your first name only. It may be used for security validation, should you forget your password</p>";
            break;
            
        case "surname":
            echo "<p>Please enter your family name. It may be used for security validation, should you forget your password</p>";
            break;
        case "Username":
            echo "<p>Please chose a unique username that you will be able to remember. You may use alphanumeric characters only</p>";
            break;

        case "Password":
            echo "<p>Please chose a password of at least 8 characters length, using letters, numbers or symbols. <strong>Your password is case sensitive!</strong></p>
            <p>A weak password will appear red, whilst a strong password will appear green</p>";
            break;
       
        case "confirm":
            echo "<p>Please re-enter the password exactly as you entered it above. <strong>Remember</strong> passwords are case sensitive!</p>";
            break;
            
        case "Email":
            echo "<p>Please enter your email address. We will use it for validation reasons, and security checks should you forget your password.</p>
            <p>We promise we won't spam! You may, however, receive email alerts from RAPPs.</p>";
            break;
            
        case "TNC":
            echo "<p>You agree to be bound by the conditions as set hereinfafter.</p>" . TermsAndConditions_EN;
            break;
            
        case "terms_and_conditions":
            echo "<p>You must agree to be bound by the conditions in order to use the RAPP-store.</p>";
            break;
            
        case "Accounts":
            echo "<p>The following Account types exist in RAPP:
            <li>Robot User: A user that is able to download RAPPs and install them on his or her robot</li>
            <li>Developer: A user creating new RAPPs, who may submit them for distribution via the RAPP-Store</li>
            <li>3rd-Party: A user that is able remotely access a RAPP running on a robot for monitoring purposes</li></p>";
            break;
    }
}
else
    echo "delegate.php error: no post key";

?>
