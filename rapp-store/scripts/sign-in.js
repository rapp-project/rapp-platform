// Only if button Sign-In is clicked
$( "#sign-in" ).click(function()
{
    $( "#dialog-sign-in" ).dialog({ resizable: false,
                                    height: 'auto',
                                    modal: true,
                                    buttons: [{ text: "Sign-In",
                                                type: "submit",
                                                open: function() {},
                                                click: function() {
                                                    var form = $(this).dialog().find('form');
                                                    if( $("#username").val().length === 0 || $("#password").val().length === 0 )
                                                    { alert( "Username or Password field empty!" ); }
                                                    else
                                                    {
                                                        $(this).dialog().find('form').submit();
                                                    }
                                                }}
                                             ]
                                    });
});
