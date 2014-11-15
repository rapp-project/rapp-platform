
$( document ).ready(function() 
{
    /// Input field Tool-tips
    $('input').qtip({
        content: {
            title: "Help & Tips",
            text: function(event, api){ var word = $(this).attr('id');
                                        return $.ajax({ url: "/qtips/registration.php", type: 'POST', data: { key : word } }); }
        },
        position: { my: 'center left', at: 'right center' },
        show: { solo: true, delay: 1000 },
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap' }
    });    
    
    /// General Info Tool-Tip
    $(".information-head").qtip({
        content: 
        {
            title: $(this).attr('Name'),
            text: 'blah blah blah',
        },
        show: 'mouseover',
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap' }
    });
    
    /// Check password strength
    $('#pwd').pStrength(
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
    
    $( "#confirm-password" ).focusout(function() {
        // TODO: Check if passwords match - if not animate red
        $(this).animate({ borderColor: "red", boxShadow: '0 0 5px 3px rgba(100,100,200,0.4)' }, 'slow');
        // TODO: and then produce a qtip2, with a message saying passwords do not match
    });
});

function validEmail( email ) 
{
    var regex = new RegExp("[a-z0-9!#$%&'*+/=?^_`{|}~-]+(?:\.[a-z0-9!#$%&'*+/=?^_`{|}~-]+)*@(?:[a-z0-9](?:[a-z0-9-]*[a-z0-9])?\.)+[a-z0-9](?:[a-z0-9-]*[a-z0-9])?");
    return ( email.match(regex) == null) ? false : true;
}

function checkForm ( )
{
    // TODO...
}

function pwdConfirm ( )
{
    // TODO...
}

function usrIsUnique( )
{
    // TODO...
}