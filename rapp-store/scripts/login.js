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
           var usr = $("#Username").val();
           var pwd = sha256_digest( $("#Password").val() );
           //console.log ( pwd );
           $.ajax({
                type: "POST",
                data: { username: usr, password: pwd },
                url: "/php/ajax/login.php",
                success: function( data ){
                    //console.log( "ajax/login.php RET: " + data );
                    // NOTE: If I receive an OK, ask for a reload of the page
                    if ( data == "OK" ){ location.reload(); }
                    // Else show the error message
                    else { 
                        console.log ( data );
                        alert( data );
                    }
                }
            });
       }
       
       if ( !$("#Username").val() )
           alert( "Empty username" );
       
       if ( !$("#Password").val() )
           alert( "Empty password" );
    });
});
