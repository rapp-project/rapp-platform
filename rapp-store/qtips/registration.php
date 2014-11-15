<?php

if ( isset( $_POST['key'] ) )
{
    switch ( $_POST['key'] )
    {
        case "username":
            echo "<p>Please chose a unique username that you will be able to remember. You may use alphanumeric characters only</p>";
            break;
        
        case "firstname":
            echo "<p>Please enter your first name only. It may be used for security validation, should you forget your password</p>";
            break;
            
       case "surname":
            echo "<p>Please enter your family name. It may be used for security validation, should you forget your password</p>";
            break;

       case "pwd":
            echo "<p>Please chose a password of at least 8 characters length, using letters, numbers or symbols. Your password is case sensitive!</p>";
            break;
       
       case "confirm-password":
            echo "<p>Please re-enter the password exactly as you entered it above</p>";
            break;
            
       case "email":
            echo "<p>Please your email address. We will use it for validation reasons, and security checks should you forget your password. We promise we won't spam!</p>";
            break;
            
       case "terms-conditions":
            echo "<h2>Terms and Conditions</h2>
            <p>You agree to be bound by the conditions as set hereinfafter.</p>";
            break;
    }
}
else
    echo "delegate.php error: no post key";

?>
