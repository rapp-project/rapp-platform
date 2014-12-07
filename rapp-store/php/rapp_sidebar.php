<!-- RAPP Side-Bar -->
<script type="text/javascript" src="/scripts/rapp_sidebar.js"></script>
<div id="rapp-nav" class="sidebar pure-u-1 pure-u-md-1-24">
    <div class="pure-menu pure-menu-open rapp-side">
        <ul>
        <?php
        
        $sidebar = 
        "
            <li id=\"rapp-new\"><a href=\"rapp_new.php?step=1\"><span class=\"oi oi-cloud-upload\"></span>  New</a></li>
            <li id=\"rapp-mine\"><a href=\"#\"><span class=\"oi oi-list\"></span>  Mine</a></li>
            <li id=\"rapp-setup\"><a href=\"#\"><span class=\"octicon octicon-tools\"></span>  Setup</a></li>
            <li id=\"rapp-filter\"><a href=\"#\"><span class=\"oi oi-magnifying-glass\"></span>  Filter</a></li>";
        
        $page =  basename( $_SERVER['PHP_SELF'] , ".php");
        
        switch ( $page )
        {
            case "rapp_new" :
            {
                $sidebar = str_replace( "<li id=\"rapp-new\"><a href=\"rapp_new.php?step=1\"><span class=\"oi oi-cloud-upload\"></span>  New</a></li>",
                                        "<li id=\"rapp-new\" class=\"pure-menu-selected\"><a href=\"rapp_new.php?step=1\"><span class=\"oi oi-cloud-upload\"></span>  New</a></li>", $sidebar );
            }
            break;
        }
        
        echo $sidebar;
        ?>
        </ul>
    </div>    
</div>