<?php

if ( isset( $_POST['user'] ) )
	error_log ( $_POST['user'] );

if ( isset( $_POST['pwd'] ) )
	error_log ( $_POST['pwd'] );

header('Content-Type: application/json');
echo "{\"rapp-api\":\"1\",\"service\":\"test\"}";

?>
