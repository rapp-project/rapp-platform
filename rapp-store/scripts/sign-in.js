// Only if button Sign-In is clicked

/// Document Ready Functions
$( document ).ready(function() 
{
    $( "#sign-in" ).click(function()
    {
        $( "#dialog-sign-in" ).dialog(
        { 
            resizable: false,
            height: 'auto',
            modal: true
            // TODO... Ajax call on submit OR forward somewhere else.
        });
    });
    
    $( "#signin_button").on("click",function( event )
    {
       event.preventDefault();
       
       // Check that both fields are not empty
       if ( $("#Username").val() && $("#Password").val() )
       {
           console.log( "Proceed with an ajax call to /ajax/signin.php" );
           // NOTE: Remember to sha256 the password right away!
       }
       
       if ( !$("#Username").val() )
           alert( "Empty username" );
       
       if ( !$("#Password").val() )
           alert( "Empty password" );
    });
});
