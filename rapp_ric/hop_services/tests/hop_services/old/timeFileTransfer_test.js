var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");

var Fs = require(rapp_hop_path + "utilities/./fileUtils.js");
var HopUtils = require(rapp_hop_path + "utilities/./HopServiceUtils.js");

import service storeFile ( _destPath, _data );

var imagePath = "~/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/test_auxiliary_files/Lenna.png";
//var non_qrImagePath = rapp_hop_path + "tests/auxiliary/Robot_Blue.jpg"
var destPath = "/home/klpanagi/hop_temps/temp.png";

var startT, endT, execT;

/*===================REMOTE SERVICE CALL============================*/
/*==================================================================*/

var params = new ServerParams(false, "155.207.19.37", 9001, "rappdev", "rappdev");


/*---------Time reading binary data and stringify them--------*/
startT = new Date();
var dataBin = Fs.readBinFileSync ( imagePath ); //typeOf dataBin == Buffer
endT = new Date();

console.log('Reading binary file Operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');
/*------------------------------------------------------------*/

/*-------------Time Call to HOP-Service-----------------------*/
startT = new Date();
var msg = storeFile( destPath, dataBin ).post(
  function( data ){
    return data;
  },
  params
);
endT = new Date();

console.log('Hop Service Call Operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');
/*------------------------------------------------------------*/


/*=====================LOCAL SERVICE CALL===========================*/
/*==================================================================*/
var params = new ServerParams(false, "localhost", 9001, "", "");


/*---------Time reading binary data and stringify them--------*/
startT = new Date();
var dataBin = Fs.readBinFileSync ( imagePath ); //typeOf dataBin == Buffer
endT = new Date();

console.log('Reading binary file + stringify operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');
/*------------------------------------------------------------*/

/*-------------Time Call to HOP-Service-----------------------*/
startT = new Date();
var msg = storeFile( destPath, dataBin ).post(
  function( data ){
    //return data;
  },
  params
);
endT = new Date();

console.log('Hop Service Call Operation took ' +
  (endT.getTime() - startT.getTime()) + ' msec');
/*------------------------------------------------------------*/

/*==================================================================*/

