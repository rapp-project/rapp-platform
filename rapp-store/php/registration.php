<?php require_once ('head.php'); ?>

    <script type="text/javascript" src="/scripts/pStrength.jquery.js"></script>
    
<body>

    <?php require_once( 'user_session.php' ); ?>
    <?php require_once( 'menu.php' ); ?>

<div id="layout" class="pure-g">

    <!-- Left side bar -->
    <div class="sidebar pure-u-1 pure-u-md-1-4">
        <div class="header">
            <h2 class="brand-galine">Latest News</h2>
        </div>
    </div>
    
    <div class="content pure-u-1 pure-u-md-3-4">
        <div class="pure-u-1 pure-u-md-1-2">
            <div class="l-box">
            
                        <h2 class="information-head">Registration</h2>
                        <hr>
                        <p>
                            <form class="pure-form pure-form-aligned">
                                <fieldset>
                                
                                    <div class="pure-control-group">
                                        <label for="firstname">First name</label>
                                        <input id="firstname" type="text" placeholder="Bob">
                                    </div>
                                    
                                    <div class="pure-control-group">
                                        <label for="surname">Surname</label>
                                        <input id="surname" type="text" placeholder="Smith">
                                    </div>
                                
                                    <div class="pure-control-group">
                                        <label for="username">Username</label>
                                        <input id="username" type="text" placeholder="bsmith">
                                    </div>

                                    <div class="pure-control-group">
                                        <label for="pwd">Password</label>
                                        <input id="pwd" type="password" placeholder="Password">
                                    </div>
                                    
                                    <div class="pure-control-group">
                                        <label for="confirm-password">Confirm Password</label>
                                        <input id="confirm-password" type="password" placeholder="Confirm Password">
                                    </div>

                                    <div class="pure-control-group">
                                        <label for="email">Email Address</label>
                                        <input id="email" type="email" placeholder="b.smith@hotmail.com">
                                    </div>

                                    <div class="pure-control-group">
                                        <label for="foo">Account Type</label>
                                        <select account_type>
                                            <option value="user" selected>Common User</option>
                                            <option value="dev">Developer</option>
                                            <option value="audi">3rd Party access</option>
                                        </select>
                                    </div>

                                    <hr>
                                    
                                    <div class="pure-controls">
                                        <label for="terms-conditions" class="pure-checkbox">
                                            <input id="terms-conditions" type="checkbox"> I've read the terms and conditions</label>
                                        <button type="submit" class="pure-button pure-button-primary">Submit</button>
                                    </div>
                                    
                                </fieldset>
                            </form>
                        </p>
                    </div>
                    
        </div>
    </div>
</div>

<?php require_once( 'sign_in.php'); ?>
<script type="text/javascript" src="/scripts/sign-in.js"></script>
<script type="text/javascript" src="/scripts/registration.js"></script>

<script type="text/javascript">
var js_enabled = $( ":root" ).find( "js" ) ? true : false;
</script></script>

</body>
