<?php require_once( 'user_session.php' ); ?>

<div class="rapp-logo"></div>
<div class="pure-skin-rapp pure-menu pure-menu-open pure-menu-horizontal">
    <a href="#" class="pure-menu-heading"></a>
    <ul>

    <?php
        session_start();
        
        $user = null;
        $page =  basename( $_SERVER['PHP_SELF'] , ".php");

        if ( isset( $_SESSION['user'] ) )
            $user = unserialize( $_SESSION['user'] );
        
        $menu = 
        "
        <li><a id=\"news\" href=\"home.php\"><span class=\"oi oi-rss-alt\"></span>  News</a></li>
        <li><a id=\"sign-in\" href=\"#\"><span class=\"oi oi-account-login\"></span>  Sign-In</a></li>
        <li><a id=\"registration\" href=\"registration.php\"><span class=\"oi oi-person\"></span>  Register</a></li>
        <li><a id=\"rapps\" href=\"rapp_home.php\"><span class=\"octicon octicon-package\"></span>  Rapps</a></li>
        <li><a id=\"robots\" href=\"#\"><span class=\"octicon octicon-hubot\"></span>  Robots</a></li>
        <li><a id=\"support\"href=\"#\"><span class=\"octicon octicon-info\"></span>  Support</a></li>";
        
        //error_log ( $page );
        
        switch ( $page )
        {
            case "home" :
            {
                $menu = str_replace( "<li><a id=\"news\" href=\"home.php\"><span class=\"oi oi-rss-alt\"></span>  News</a></li>",
                                     "<li class=\"pure-menu-selected\"><a id=\"news\" href=\"home.php\"><span class=\"oi oi-rss-alt\"></span>  News</a></li>", $menu );
            }
            break;
            
            case "registration" :
            {    
                // set register as selected
                $menu = str_replace( "<li><a id=\"registration\" href=\"registration.php\"><span class=\"oi oi-person\"></span>  Register</a></li>", 
                                     "<li class=\"pure-menu-selected\"><a id=\"registration\" href=\"registration.php\"><span class=\"oi oi-person\"></span>  Register</a></li>", $menu );
                
                // remove sign-in during registration: it complicates the qtip-work
                $menu = str_replace( "<li><a id=\"sign-in\" href=\"#\"><span class=\"oi oi-account-login\"></span>  Sign-In</a></li>", "", $menu );
                
                // BUG: Login doesn't work on registration - javascript code clash
                
            }
            break;
            
            case "rapp_home" :
            {
                $menu = str_replace( "<li><a id=\"rapps\" href=\"rapp_home.php\"><span class=\"octicon octicon-package\"></span>  Rapps</a></li>",
                                     "<li class=\"pure-menu-selected\"><a id=\"rapps\" href=\"rapp_home.php\"><span class=\"octicon octicon-package\"></span>  Rapps</a></li>", $menu );
            }
            break;
            
            case "rapp_new" :
            {
                $menu = str_replace( "<li><a id=\"rapps\" href=\"rapp_home.php\"><span class=\"octicon octicon-package\"></span>  Rapps</a></li>",
                                     "<li class=\"pure-menu-selected\"><a id=\"rapps\" href=\"rapp_home.php\"><span class=\"octicon octicon-package\"></span>  Rapps</a></li>", $menu );
            }
            break;
            
            // TODO Other Options?
        }
        
        // If user is already logged-in, remove the `Sign-In` & `Register` & Add Logout Button - TODO: Create a User Bar?
        if ( $user )
        {
            $menu = str_replace( "<li><a id=\"sign-in\" href=\"#\"><span class=\"oi oi-account-login\"></span>  Sign-In</a></li>", "", $menu );
            $menu = str_replace( "<li><a id=\"registration\" href=\"registration.php\"><span class=\"oi oi-person\"></span>  Register</a></li>", "", $menu );
            $menu .= "<li><a id=\"logout\"href=\"logout.php\"><span class=\"oi oi-account-logout\"></span>  Log Out</a></li>";
        }

        echo $menu;
    ?>
    
    </ul>
</div>