<?php require_once ('head.php'); ?>
<body>

<?php
require_once( 'user_session.php' );
require_once( 'menu.php' );
require_once( 'sign_in.php');

$step = null;

if ( isset( $_GET["step"] ) )
    $step = $_GET["step"];
else 
    $step = 1;

?>
<!-- Main RAPP Box -->
<div id="layout" class="pure-g">
    
    <?php require_once( 'rapp_sidebar.php' ); ?>
    
    <?php
        // TODO: Move all echoe'd stuff into respective PHP files
            
        /// NOTE: C/C++ Submission - hide this if user does not intend to upload/submit C/C++ code.
        $step2_html =
        "<div class=\"pure-u pure-u-md-4-24\">
            <div class=\"rapp-new-menu-item\">
                <div class=\"rapp-new-menu-icon pure-u\"><h2><span class=\"oi oi-code\"</span></h2></div>
                <div class=\"pure-u-3-4\">
                    <h2>Source Code</h2>
                    <p>Select Source Code for this RAPP.
                    You may submit C++ files and Headers, C files or headers.
                    Those will be compiled or cross-compiled, and linked before being packaged.</p>
                </div>
            </div>
        </div>";
        
        /// NOTE: Dependencies - Only show this if the user has chosen to upload/submit C/C++
        $step3_html =
        "<div class=\"pure-u pure-u-md-4-24\">
            <div class=\"rapp-new-menu-item\">
                <div class=\"rapp-new-menu-icon pure-u\"><h2><span class=\"oi oi-list-rich\"></span></h2></div>
                <div class=\"pure-u-3-4\">
                    <h2>Dependencies</h2>
                    <p>Select Libraries for which your C or C++ RAPP will be linked against.
                    Currently we support a limited list of the most common libraries, so you must
                    make sure you're not using unsupported libraries which are unlikely to be present on a robot.</p>
                </div>
            </div>
         </div>";
        
        /// NOTE: Script Submission - hide this if user does not intend to upload/submit Scripts.
        $step4_html =
        "<div class=\"pure-u pure-u-md-4-24\">
            <div class=\"rapp-new-menu-item\">
                <div class=\"rapp-new-menu-icon pure-u\"><h2><span class=\"oi oi-script\"></span></h2></div>
                <div class=\"pure-u-3-4\">
                    <h2>Scripts</h2>
                    <p>Select Scripts for this RAPP.
                    You may submit Javascript files or Python files.
                    Those will be executed directly by the HOP process on the robot.</p>
                </div>
            </div>
        </div>";
        
        /// NOTE: Extra files, such as XML/JSON/Text/sqlLite, etc...
        $step5_html =
        "<div class=\"pure-u pure-u-md-4-24\">
            <div class=\"rapp-new-menu-item\">
                <div class=\"rapp-new-menu-icon pure-u\"><h2><span class=\"mega-octicon octicon-database\"></span></h2></div>
                <div class=\"pure-u-3-4\">
                    <h2>Extras</h2>
                    <p>Extra files which may be used or loaded by your RAPP internally.
                    Those files may be: images, XML, JSON, mysqlLite, text, raw serialised (non-executable), Audio files, etc.</p>
                </div>
            </div>
        </div>";
        
        /// NOTE: Building - Only show this if the user has chosen to upload/submit C/C++
        $step6_html =
        "<div class=\"pure-u pure-u-md-4-24\">
            <div class=\"rapp-new-menu-item\">
                <div class=\"rapp-new-menu-icon pure-u\"><h2><span class=\"oi oi-cog\"></span></h2></div>
                <div class=\"pure-u-3-4\">
                    <h2>Building</h2>
                    <p>Building phase for your C/C++ source code.
                    You may observe the output of the building process, and any errors or warnings produced by the compiler or linker.
                    If your RAPP does not build, it will not be packaged and distributed.</p>
                </div>
            </div>
        </div>";
        
        /// NOTE: Packaging: Compulsory for all types of RAPPs
        $step7_html =
        "<div class=\"pure-u pure-u-md-4-24\">
            <div class=\"rapp-new-menu-item\">
                <div class=\"rapp-new-menu-icon pure-u\"><h2><span class=\"oi oi-box\"></span></h2></div>
                <div class=\"pure-u-3-4\">
                    <h2>Packaging</h2>
                    <p>Packaging for the new RAPP.</p>
                </div>
            </div>
        </div>";
        
        /**
            WARNING - The question here is what happens if the user has created a Cloud/Robot Controller?
                      Do we produce two different rapps, and package them separetely?
        */
        
        switch( $step )
        {
            case 1:
                require_once ( 'rapp_step1.php' );
                break;
            
            case 2:
                echo $step2_html;
                break;
            
            case 3:
                echo $step3_html;
                break;
                
            case 4:
                echo $step4_html;
                break;
                
            case 5:
                echo $step5_html;
                break;
                
            case 6:
                echo $step6_html;
                break;
                
            case 7:
                echo $step7_html;
                break;
        }
        
        // TODO: Repeat the above switch-case for the main window html: the Forms, Upload Code, Dependencies, etc...
    ?>    
    </div>
    
</div>
</body>


<?
/*
<script>
$(function() {
    $( "#accordion" ).accordion();
});
</script>
    
<!-- RAPP Settings as a 2nd Side-bar ;-) -->
<div class="mid-box" id="rapp_new_box">

    <div id="accordion">
    
        <!-- Basic Stuff -->
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-file"></span> Basic Settings</div>
        <div>
            <div class="content pure-u-1 pure-u-md-1-1">
                <form id="basic-info" class="pure-form pure-form-stacked">
                    <label for="Name">Name</label>
                    <input id="Name" type="text" placeholder="Name">
                    <label for="Version">Version</label>
                    <input id="Version" type="text" placeholder="Version">
                </form>
            </div>
        </div>
        
        <!-- C/C++ Source Control -->
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-code"></span> Source Code</div>
        <div>
        </div>
        
        <!-- Scripts: Javascript -->
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-script"></span> Scripts</div>
        <div>
        </div>
        
        <!-- Extra Files: Images, Databases, XML, JSON - NOTE: Exclude binary, scripts, etc. -->
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-image"></span> Extras Files</span></div>
        <div>
        </div>
        
        <!-- Dependencies: Libraries and Headers -->
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-list-rich"></span> Dependencies</div>
        <div>
        </div>
        
        <!-- Build: Compile and Link -->
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-cog"></span> Build</span></div>
        <div>
        </div>
        
        <!-- RAPP Output
        <div class="rapp-new-menu-icon pure-u"><span class="oi oi-terminal"></span> Output</span></div>
        <div>
        -->
        
        <!-- Distribute -->
        <div class="rapp-new-menu-icon pure-u"><span class="octicon octicon-package"></span> Package and Distribute</span></div>
        <div>
        </div>
        
    </div>
</div>
*/
?>