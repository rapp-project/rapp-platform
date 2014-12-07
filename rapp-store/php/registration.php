<?php require_once ('head.php'); ?>

<link rel="stylesheet" href="/css/spinner.css"/>
<script type="text/javascript">
var js_enabled = $( ":root" ).find( "js" ) ? true : false;
</script></script>
<body>

    <?php require_once( 'user_session.php' ); ?>
    <?php require_once( 'menu.php' ); ?>

<style>
.big-qtip {
        max-width: 800px;
        min-width: 0px;
}
</style>
    
<div id="layout" class="pure-g">
    <div class="content pure-u-1 pure-u-md-1-1">
        <div class="l-box" id="registration_box">
            <h2 class="information-head">Registration</h2>
            <hr>
            <p>
                <form class="pure-form pure-form-aligned" id="registration" method="post" accept-charset="UTF-8">
                    <fieldset>
                    
                        <div class="pure-control-group">
                            <label for="firstname">First name</label>
                            <input id="firstname" type="text" placeholder="Bob" size="35">
                        </div>
                        
                        <div class="pure-control-group">
                            <label for="surname">Surname</label>
                            <input id="surname" type="text" placeholder="Smith" size="35">
                        </div>
                    
                        <div class="pure-control-group" usrexists>
                            <label for="Username">Username</label>
                            <input id="Username" type="text" placeholder="bsmith" size="35">
                        </div>

                        <div class="pure-control-group" emptypwd>
                            <label for="Password">Password</label>
                            <input id="Password" type="password" placeholder="Password" size="35">
                        </div>
                        
                        <div class="pure-control-group" missmatch>
                            <label for="confirm">Confirm Password</label>
                            <input id="confirm" type="password" placeholder="Confirm Password" size="35">
                        </div>

                        <div class="pure-control-group" badformat>
                            <label for="Email">Email Address</label>
                            <input id="Email" type="email" placeholder="b.smith@hotmail.com" size="35">
                        </div>

                        <div class="pure-control-group">
                            <label for="foo">Account Type</label>
                            <select id="account_type" account_type>
                                <option value="user" selected>Robot User</option>
                                <option value="dev">Developer</option>
                                <option value="audi">3rd Party access</option>
                            </select>
                        </div>

                        <hr>
                        
                        <div class="pure-controls-group">
                            <label for="terms and conditions" class="pure-checkbox">
                                <input id="terms_and_conditions" type="checkbox" mustaccept> I accept the <strong><a terms>terms and conditions</a></strong>
                            </label>
                            <button type="submit" id="process_form" class="pure-button pure-button-primary">Submit</button>
                        </div>
                        
                        
                    </fieldset>
                </form>
            </p>
        </div>
    </div>
</div>

<div id="register_dialog" style="display: none;">
    <h4>Processing your registration</h4>
    <p><div class="spinner"></div></p>
</div>

<script type="text/javascript" src="/scripts/login.js"></script>
<script type="text/javascript" src="/scripts/registration.js"></script>
<script type="text/javascript" src="/scripts/pStrength.jquery.js"></script>
<script type="text/javascript" src="/scripts/jQuery.stringify.js"></script>
<script type="text/javascript" src="/scripts/sha256.js"></script>
    
</body>
