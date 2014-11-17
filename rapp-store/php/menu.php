<?php require_once( 'user_session.php' ); ?>

<div class="rapp-logo"></div>
<div class="pure-skin-rapp pure-menu pure-menu-open pure-menu-horizontal">
    <a href="#" class="pure-menu-heading"></a>
    <ul>
    <?php
        $page =  basename( $_SERVER['PHP_SELF'] , ".php");

        // TODO: If user is already logged-in, then remove the `Sign-In` and replace it with some User-Bar
        
        $menu = 
        "
        <li><a id=\"news\" href=\"home.php\">News</a></li>
        <li><a id=\"sign-in\" href=\"#\">Sign-In</a></li>
        <li><a id=\"registration\" href=\"registration.php\">Register</a></li>
        <li><a id=\"rapps\" href=\"#\">Rapps</a></li>
        <li><a id=\"robots\" href=\"#\">Robots</a></li>
        <li><a id=\"support\"href=\"#\">Support</a></li>";
        
        switch ( $page ) 
        {
            case "home" :
            {
                $menu = str_replace( "<li><a id=\"news\" href=\"home.php\">News</a></li>",
                                     "<li class=\"pure-menu-selected\"><a id=\"news\" href=\"#\">News</a></li>", $menu );
            }
            break;
            
            case "registration" :
            {    
                // set register as selected
                $menu = str_replace( "<li><a id=\"registration\" href=\"registration.php\">Register</a></li>", 
                                     "<li class=\"pure-menu-selected\"><a id=\"registration\" href=\"registration.php\">Register</a></li>", $menu );
                
                // remove sign-in during registration: it complicates the qtip-work
                $menu = str_replace( "<li><a id=\"sign-in\" href=\"#\">Sign-In</a></li>", "", $menu );
            }
            break;
            
            // TODO...
        }

        echo $menu;
    ?>
    </ul>
</div>