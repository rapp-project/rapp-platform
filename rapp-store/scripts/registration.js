
/// Document Ready Functions
$( document ).ready(function()
{
    /// Input field Tool-tips
    $("input").qtip({
        content: {
            title: function(){  var title = replaceAll( "_", " ", $(this).attr('id') );
                                return "<h2>" + title + "</h2>"; },
            text: function(){ var word = $(this).attr('id');
                              return $.ajax({ url: "/qtips/registration.php", type: 'POST', data: { key : word } }); }
        },
        position: { my: 'center left', at: 'right center' },
        show: { solo: false, delay: 800 },
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap', tip: { width: 10, height: 30 } }
    });
    
    /// Select Account type tool-tips
    $("select").qtip({
        content: {
            title: function(){ return "<h2>Account Types</h2>"},
            text: function(){ return $.ajax({ url: "/qtips/registration.php", type: 'POST', data: { key : "Accounts" } }); }
        },
        position: { my: 'center left', at: 'right center' },
        show: { solo: true, delay: 800 },
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap', tip: { width: 10, height: 80 } }
    });
    
    /// Term and Conditions Label (Input already has delegate from above)
    $("[terms]").qtip({
        content: {
            title: function(){ return "<h2>Terms And Conditions</h2>"},
            text: function(){ return $.ajax({ url: "/qtips/registration.php", type: 'POST', data: { key : "TNC" } }); },
            button: true
        },
        position: { my: 'left center', at: 'center right' },
        show: { modal: false, solo: true, delay: 1000 },
        hide: false,
        style: { classes: 'qtip-bootstrap big-qtip',tip: { width: 1, height: 1 } }
    });
        
    /// Check password strength
    $("#Password").pStrength({
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
    $( "#confirm" ).focusout(function(){ confirmPwd( $(this) ); });
    
    /// Check if username is indeed unique when username input loses focus
    $( "#Username" ).focusout(function(){
        console.log ( "checking username" );
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
    $( "#Email" ).focusout(function(){
        var email = $(this).val();
        if ( email.length> 0 ) validateEmail( email );
    });
    
    // Catch Submit Button Press - Validate Form & Make Sure Terms & Conditions are Ticked
    $( "#process_form" ).on("click",function( event )
    {
        event.preventDefault();
        console.log( "Validating Form" );
        // If form is OK
        if ( checkForm() )
        {
            $("#error_msg").remove();
            
            console.log( "Form Validated OK, submitting..." );
            var registration = { firstname: $("input[id*='firstname']").val(),
                                 surname: $("input[id*='surname']").val(),
                                 username: $("input[id*='Username']").val(),
                                 password: sha256_digest( $("input[id*='Password']").val() ),
                                 email: $("input[id*='Email']").val(),
                                 account: $("#account_type").find(":selected").text() };
            
            // stringify ala-jQuery
            var json = $.stringify( registration );
            
            // pop processing dialog
            $( "#register_dialog" ).dialog({       
                open: function(event, ui) { 
                    $(this).parent().children('.ui-dialog-titlebar').hide();
                },
                autoOpen: false,
                modal: true,
                draggable: false
            });
            // Show the pop-up
            $("#register_dialog").dialog('open');
            
            // Do the Ajax call
            $.ajax({
                type: "POST",
                data: { form: json },
                url: "ajax/registration.php",
                success: function( data )
                {
                    $("form[id*='registration'").remove();
                    $("#register_dialog").dialog("close");
                    $("#registration_box").append( data );
                },
                error: function (jqXHR, textStatus, errorThrown )
                {
                    console.log ( textStatus + " " + errorThrown );
                    $("#register_dialog").dialog("close");
                    $("form[id*='registration'").append('<div class="ui-state-error ui-corner-all" style="padding: 0 .7em;" id="error_msg">\
                    <p>' + textStatus + '</p><p>' + errorThrown + '</p></div>' );
                }
            });
        }
    });

});

/// Replace all instances in a string
function replaceAll(find, replace, str) 
{
    return str.replace(new RegExp(find, 'g'), replace);
}

// Validate email format
function validateEmail( email ) 
{
    var re = /^(([^<>()[\]\\.,;:\s@\"]+(\.[^<>()[\]\\.,;:\s@\"]+)*)|(\".+\"))@((\[[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\.[0-9]{1,3}\])|(([a-zA-Z\-0-9]+\.)+[a-zA-Z]{2,}))$/;
    var valid = re.test(email);
    //console.log ( email + " is: " + valid );
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
                style: { classes: 'qtip-rounded qtip-red error', tip: { width: 10, height: 80 } },
                onHide: function(){ $(this).qtip('destroy'); }
        });
        $("[badformat").qtip("show");
    }
    else
    {
        $("#Email").animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
    }
    return valid;
}

/// Check if username is indeed unique
function usrUniqueTest( exists, username )
{
    if ( exists == 1 )
    {
        $("#Username").animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
        $("#Username" ).effect( "pulsate" );
        //console.log ( "username exists" );
        $("[usrexists]").qtip({
                content: {
                    title: "<h4>Error: Username in use</h4>",
                    text: function(){ return "<p>Username: '" + username + "' is already in use, please chose another one!</p>" ; }
                },
                position: { my: 'center left', at: 'right center', adjust: { x: -50, y: 0, method: 'none' } },
                show: { solo: true, delay: 3000, when: false },
                hide: 'mouseout',
                style: { classes: 'qtip-rounded qtip-red error', tip: { width: 10, height: 80 } },
                onHide: function(){ $(this).qtip('destroy'); }
        });
        $("[usrexists").qtip("show");
    }
    else
        $("#Username").animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
}

/// Confirm Passwords match
function confirmPwd( obj )
{
    var pwd = $("input[id*='Password']").val();
    var cmp = $("input[id*='confirm']").val();
    
    if ( pwd.length > 0 && cmp.length > 0 )
    {
        if ( pwd != cmp )
        {
            $(obj).animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
            $(obj ).effect( "pulsate" );
            $("[missmatch]").qtip({
                    content: {
                        title: "<h4>Error: Password miss-match</h4>",
                        text: function(){ return "<p>Passwords do not match! Please verify you have typed the same password!</p>" ; }
                    },
                    position: { my: 'center left', at: 'right center', adjust: { x: -50, y: 0, method: 'none' } },
                    show: { solo: true, delay: 3000, when: false },
                    hide: 'mouseout',
                    style: { classes: 'qtip-rounded qtip-red error', tip: { width: 10, height: 80 } },
                    onHide: function(){ $(this).qtip('destroy'); }
            });
            $("[missmatch").qtip("show");
            return false;
        }
        else
        {
            $(obj).animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
            return true;
        }
    }
    else return false;
}

/// Validate entire form = make sure everything required is present and valid
function checkForm ( )
{
    var usr = true, 
        fnm = true, 
        snm = true, 
        pwd = true, 
        cnf = true, 
        email = true, 
        agr = true;
    
    if ( !$("input[id*='firstname']").val() )
    {
        EmptyField( "#firstname" );
        fnm = false;
    }
    else
        $("#firstname").animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
    
    if ( !$("input[id*='surname']").val() )
    {
        EmptyField( "#surname" );
        snm = false;
    }
    else
        $("#surname").animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
    
    if ( !$("input[id*='Username']").val() )
    {
        EmptyField( "#Username" );
        usr = false;
    }
    else
        $("#Username").animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
    
    if ( !$("input[id*='Password']").val() )
    {
        EmptyField( "#Password" );
        pwd = false;
    }
    else
        $("#Password").animate({ borderColor: "#CCCDCC", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
    
    if ( !$("input[id*='confirm']").val() )
    {
        EmptyField( "#confirm" )
        cnf = false;
    }
    else
        cnf = confirmPwd( "#confirm" );
    
    if ( !$("input[id*='Email']").val() )
    {
        EmptyField( "#Email" );
        email = false;
    }
    else
        email = validateEmail( $("input[id*='Email']").val() );
    
    if ( $("input[id*='terms_and_conditions']").prop('checked') == false )
    {
        console.log ('Terms not accepted');
        $("#terms_and_conditions").qtip({
                content: {
                    text: function(){ return "<p>You must accept the Terms & Conditions in order to register</p>" ; }
                },
                position: { my: 'center right', at: 'left center', adjust: { x: -20, y: 0, method: 'none' } },
                show: { solo: true, delay: 2000, when: false },
                hide: 'mouseout',
                style: { classes: 'qtip-bootstrap', tip: { width: 10, height: 50 } },
                onHide: function(){ $(this).qtip('destroy'); }
        });
        $("#terms_and_conditions").qtip("show");
        agr = false;
    }

    return (usr && fnm && snm && pwd && cnf && email && agr);
}

/// Pop a Qtip creating an error about an empty field
function EmptyField( itemid )
{
    $(itemid).animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
    $(itemid).effect( "pulsate" );
    
    $("form[id*='registration'").find("#error_msg").remove();
    $("form[id*='registration'").append('<div class="ui-state-error ui-corner-all" style="padding: 0 .7em;" id="error_msg">\
    <p><span class="ui-icon ui-icon-alert" style="float: left; margin-right: .3em;"></span>\
    <strong>Missing Information</strong> Please complete the missing fields</p></div>');
    
    $("#error_msg").effect("pulsate");
}