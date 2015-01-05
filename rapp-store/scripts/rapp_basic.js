
/// variable existing, is the existing RAPP list in Database with a matching name.
var existing = {};
var thumbnail = null;

$(document).ready(function()
{
    // Thubmnail uploader
    $( "#fileuploader" ).uploadFile( {
        url: "/php/ajax/thumbnail.php",
        method: "POST",
        fileName: "upfile",
        formData: { "name": function(){ return $("#rappname").val(); }},
        allowedTypes: "png,gif,jpg,jpeg",
        showDone: true,
        showCancel: false,
        showAbort: false,
        onSuccess:function( files, data, xhr ){
            thumbnail = data;
            var div = $( "#thumbnail" ).find(".ajax-upload-dragdrop").html();
            if ( div )
                $(".ajax-upload-dragdrop").remove();
        }
    });
    
    // Capture focus out of RAPP Name - Check if RAPP Name already exists
    $( "#rappname" ).focusout(function() {
        //console.log ( "Checking RAPP: " + $( "#rappname" ).val() );
        if ( $( "#rappname" ).val() )
        {
            $.ajax({
                type: "POST",
                data: { rapp: $( "#rappname" ).val() },
                url: "/php/ajax/rapp_check.php",
                success: function( data ){ 
                    existing = JSON.parse( data );
                    console.log( existing );
                }
           });
        }
    });
    
    // Capture Next Button Clicks - Validate Form
    $( "#process_form" ).on('click', function(e){
        e.preventDefault();

        var name = $( "#rappname" ).val();
        var major = $('#major_version').find(":selected").text();
        var minor = $('#minor_version').find(":selected").text();
        var lang = $('#Language').find(":selected").val();
        
        var cpuarch = $('input:checkbox:checked.cpuarch').map(function () {
            return this.value;
        }).get();
        
        var robots = $('input:checkbox:checked.robots').map(function () {
            return this.value;
        }).get();
        
        console.log ( "name: " + name );
        console.log ( "version: " + major + "." + minor );
        console.log ( "language: " + lang );
        console.log ( "cpuarch: " + cpuarch );
        console.log ( "robots: " + robots );
        console.log ( "thumbnail: " + thumbnail );
        
        // if we can find previous RAPPs with the same name - do a check
        if ( existing.length > 0 )
        {
            console.log ( "WARNING: RAPPS with same name detected - Sanity Check NOW" );
            // Is The owner of the RAPP Names the same as our username ? - If not Then Do Not accept this RAPP
            // Is This CPUArch && Version different from previous ones? - If not Then Do not accept this RAPP - Ask to do an Update
        }

        // TODO: Proceed with Checks now
        console.log ( "TODO: Proceed with Checks" );

    });
});

