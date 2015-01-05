/*!
 * @file qr.service.js
 * @brief QR service running on Remote host.
 *
 */

/*---------Sets required file Paths-------------*/
var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user
  + "/rapp_platform_catkin_ws/src/rapp-platform/ric/hop_services/";
/*----------------------------------------------*/

/*--------------Load required modules-----------*/
var Fs = require( /*rapp_hop_path +*/ "../utilities/./fileUtils.js" );


var Services = new Array();



service upServices ( ){
 var data = Fs.readFileSync( 'availableServices.txt' );
 console.log ( data.toString() );
 var services = data.toString().split("\n");
 services.pop(); //Get rid of EOF character.
 console.log( services );
 return services;
}

