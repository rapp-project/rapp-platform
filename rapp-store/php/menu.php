<?php require_once( 'user_session.php' ); ?>
<div class="pure-skin-rapp pure-menu pure-menu-open pure-menu-horizontal">
    <a href="#" class="pure-menu-heading">RAPP Store</a>
    <ul>
    <?php
        $page =  basename( $_SERVER['PHP_SELF'] , ".php");

        $menu = 
        "
        <li><a id=\"home\" href=\"#\">Home</a></li>
        <li><a id=\"sign-in\" href=\"#\">Sign-In</a></li>
        <li><a id=\"registration\" href=\"registration.php\">Register</a></li>
        <li><a id=\"rapps\" href=\"#\">Rapps</a></li>
        <li><a id=\"robots\" href=\"#\">Robots</a></li>
        <li><a id=\"support\"href=\"#\">Support</a></li>";
        
        switch ( $page ) 
        {
            case "news" :
                str_replace( "<li><a id=\"home\"", "<li class=\"pure-menu-selected\"><a id=\"home\"", $menu );
                break;
                
            case "registration" :
                str_replace( "<li><a id=\"registration\"", "<li class=\"pure-menu-selected\"><a id=\"registration\"", $menu );
                break;
            
            // TODO...
        }

        echo $menu;
    ?>
    </ul>
</div>

<?php require_once( 'sign_in.php'); ?>
<script type="text/javascript" src="/scripts/sign-in.js"></script>