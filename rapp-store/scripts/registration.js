
$( document ).ready(function() 
{
    /// Input field Tool-tips
    $("input").qtip({
        content: {
            title: function(){ return "<h2>" + $(this).attr('id') + "</h2>"},
            text: function(){ var word = $(this).attr('id');
                              return $.ajax({ url: "/qtips/registration.php", type: 'POST', data: { key : word } }); }
        },
        position: { my: 'center left', at: 'right center' },
        show: { solo: true, delay: 1000 },
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap' }
    });
    
    /// Select type tool-tips
    $("select").qtip({
        content: {
            title: function(){ return "<h2>Account Types</h2>"},
            text: function(){ return $.ajax({ url: "/qtips/registration.php", type: 'POST', data: { key : "Accounts" } }); }
        },
        position: { my: 'center left', at: 'right center' },
        show: { solo: true, delay: 1000 },
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap' }
    });
        
    /// Check password strength
    $("#Password").pStrength(
    {
        'changeBackground'          : false,
        'onPasswordStrengthChanged' : function( passwordStrength, strengthPercentage) {
            if ( $(this).val() )
                $.fn.pStrength('changeBackground', this, passwordStrength);
            else
                $.fn.pStrength('resetStyle', this);
        },
        'onValidatePassword': function( strengthPercentage ) {}
    });
    
    /// Check if Confirmed Password matches original password, when confirm input loses focus
    $( "#confirm" ).focusout(function()
    {
        var pwd = $("input[id*='Password']").val();
        var cmp = $("input[id*='confirm']").val();   
        if ( pwd.length > 0 && cmp.length > 0 )
        {
            if ( pwd != cmp )
            {
                $(this).animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
                $(this ).effect( "pulsate" );
                $("[missmatch]").qtip({
                        content: {
                            title: "<h4>Error: Password miss-match</h4>",
                            text: function(){ return "<p>Passwords do not match! Please verify you have typed the same password!</p>" ; }
                        },
                        position: { my: 'center left', at: 'right center', adjust: { x: -50, y: 0, method: 'none' } },
                        show: { solo: true, delay: 3000, when: false },
                        hide: 'mouseout',
                        style: { classes: 'qtip-rounded qtip-red error' },
                        onHide: function(){ $(this).qtip('destroy'); }
                });
                $("[missmatch").qtip("show");
            }
            else
                $(this).animate({ borderColor: "#7CFC00", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
        }
    });
    
    /// Check if username is indeed unique when username input loses focus
    $( "#Username" ).focusout(function()
    {
        var username = $(this).val();
        //console.log( "#Username focusout, checking username: " + username );
        if ( username )
        {
            $.ajax({
                type: "POST",
                data: { usr: username },
                url: "/php/ajax/unique_usr.php",
                success: function( data ){ usrUniqueTest( data, username ); }
            });
        }
    });
    
    /// Check if entered email is indeed properly formatted when input loses focus
    $( "#Email" ).focusout(function()
    {
        var email = $(this).val();
        if ( email.length> 0 )
        {
            var valid = validEmail( email );
            console.log ( email + " is: " + valid );
            if ( !valid )
            {
                $("#Email").animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
                $("#Email" ).effect( "pulsate" );
                $("[badformat]").qtip({
                        content: {
                            title: "<h4>Error: malformed email</h4>",
                            text: function(){ return "<p>email address: '" + email + "' appears to be ill-formated, please fix it<p>" ; }
                        },
                        position: { my: 'center left', at: 'right center', adjust: { x: -50, y: 0, method: 'none' } },
                        show: { solo: true, delay: 3000, when: false },
                        hide: 'mouseout',
                        style: { classes: 'qtip-rounded qtip-red error' },
                        onHide: function(){ $(this).qtip('destroy'); }
                });
                $("[badformat").qtip("show");
            }
            else
                $("#Email").animate({ borderColor: "#7CFC00", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
        }
    });
    
    // Catch Submit Button Press - Validate Form & Make Sure Terms & Conditions are Ticked
    // TODO...
    
});

// Validate email format
function validEmail( email ) 
{
    var regex = new RegExp("[a-z0-9!#$%&'*+/=?^_`{|}~-]+(?:\.[a-z0-9!#$%&'*+/=?^_`{|}~-]+)*@(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+[a-z0-9](?:[a-z0-9-]*[a-z0-9])?");
    return ( email.match(regex) == null) ? false : true;
}

/// Check if username is indeed unique
function usrUniqueTest( exists, username )
{
    if ( exists == 1 )
    {
        $("#Username").animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
        $("#Username" ).effect( "pulsate" );
        console.log ( "username exists" );
        $("[usrexists]").qtip({
                content: {
                    title: "<h4>Error: Username in use</h4>",
                    text: function(){ return "<p>Username: '" + username + "' is already in use, please chose another one!</p>" ; }
                },
                position: { my: 'center left', at: 'right center', adjust: { x: -50, y: 0, method: 'none' } },
                show: { solo: true, delay: 3000, when: false },
                hide: 'mouseout',
                style: { classes: 'qtip-rounded qtip-red error' },
                onHide: function(){ $(this).qtip('destroy'); }
        });
        $("[usrexists").qtip("show");
    }
    else
        $("#Username").animate({ borderColor: "#7CFC00", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
}

/// Validate entire form = make sure everything required is present and valid
function checkForm ( )
{
    // TODO...
}