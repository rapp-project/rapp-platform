<script type="text/javascript" src="/scripts/login.js"></script>
<script type="text/javascript" src="/scripts/sha256.js"></script>

<div id="dialog-sign-in" title="Sign In" style="display: none;">
    <form class="pure-form pure-form-stacked" id="signin_form" method="post">
        <fieldset>
            <p>
                <label for="Username"><span class="oi oi-person"></span>  Username</label>
                <input id="Username" type="email" name="username" placeholder="user@email.com">
            </p>
            <p>
                <label for="Password"><span class="oi oi-lock-locked"></span>  Password</label>
                <input id="Password" type="password" name="password" placeholder="Password">
            </p>
            <p>
                <button type="submit" id="signin_button" class="pure-button pure-button-primary">Sign in</button>
            </p>
        </fieldset>
    </form>
</div>