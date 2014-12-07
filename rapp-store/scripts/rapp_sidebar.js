$( document ).ready(function()
{
    $("#rapp-new").qtip({
        content: {
            text: function(){ var word = $(this).attr('id');
                                return $.ajax({ url: "/qtips/sidebar.php", type: 'POST', data: { key : word } }); }
        },
        position: { my: 'center left', at: 'right center', adjust: { x: 0, y: 0, method: 'none' } },
        show: { solo: false, delay: 800 },
        hide: 'mouseout',
        style: { classes: 'qtip-bootstrap', tip: { width: 10, height: 30 } }
    });
    
    // TODO: qTips for the remaining Rapp-Sidebar options
    
    // TODO: Filter should be a pop-up
    
    // TODO: Setup should be a pop-up
    
    
});
