<?php

if ( isset( $_POST['key'] ) )
{
    echo "<p>Sidebar: qTip key: " . $_POST['key'] . "</p>";
    switch ( $_POST['key'] )
    {
    
    }
}
else
    echo "/qtips/sidebar.php error: no post key";
?>