<?php

/// Send activation code - NOTE: This probably needs a sendmail server running - which I do not currently have
function send_activation( $email, $firstname, $username, $uuid )
{
    if ( !$email )
        die ( "send_activation: null $email param" );
        
    if ( !$firstname )
        die ( "send_activation: null $firstname param" );

    if ( !$username )
        die ( "send_activation: null $username param" );
    
    // Forge the message
    $subject = 'Account Activation';
    $headers = 'From: no-reply@rapp-store.com' . "\r\n" . 'X-Mailer: PHP/Automatron';
    $message = 
"Dear $firstname,\r\n
\r\n
You have registered the username `$username` on www.rapp-store.com.\r\n
Please activate your account, by going to http://www.rapp-store.com/activate?=$uuid \r\n
in order to activate your account.\r\n

Yours,\r\n
The RAPP Store.";

    // Send it
    mail( $email, $subject, $message, $headers );
    
    error_log ( 'email sent to: ' . $email );
}

?>
