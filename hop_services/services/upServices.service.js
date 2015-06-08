/*!
 * @file qr.service.js
 * @brief QR service running on Remote host.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/hop_services/";
var module_path = rapp_hop_path +  'utilities/js/'
/*--------------Load required modules-----------*/
var Fs = require( module_path + 'fileUtils.js' );
var Services = new Array();


service upServices ( ){
 var file = Fs.readFileSync( 'availableServices.txt' );
 var services = file.data.toString().split("\n");
 services.pop(); //Get rid of EOF character.
 console.log( services );
 return services;
}

