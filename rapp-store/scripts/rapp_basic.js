/**
 * TODO   1) Get the rapp name and set the thumbnail name accordingly - Remove from ajax/thumbnail.php the SHA naming
 *        2) in ajax/thumbnail.php, resize the image accordingly: http://php.net/manual/en/imagick.resizeimage.php
 */

$(document).ready(function()
{
    $( "#fileuploader" ).uploadFile({
        url: "/php/ajax/thumbnail.php",
        fileName: "upfile", // TODO: Get RAPP Name
        allowedTypes: "png,gif,jpg,jpeg"
    });
    // TODO: On Success, remove the div, resize the thumbnail, and display it inside the <div id="thumbnail">
});