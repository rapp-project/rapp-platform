<!DOCTYPE html>
<html>
<head>    
    <link rel="stylesheet" href="/css/strength.css">
    <link rel="stylesheet" href="/css/strength-style.css">
    <script type="text/javascript" src="http://ajax.googleapis.com/ajax/libs/jquery/1.7.2/jquery.min.js"></script>
    <script type="text/javascript" src="/scripts/strength.js"></script>
</head>
<body>
    <form id="myform">
        <input id="myPassword" type="password" name="" value="" class="strength" data-password="myPassword">
     </form>
<script>
$(document).ready(function ($) {

    $("#myPassword").strength();

});
</script>
</body>
