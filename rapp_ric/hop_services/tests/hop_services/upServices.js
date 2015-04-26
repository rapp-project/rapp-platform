
import service upServices ();

var user = process.env.LOGNAME;
var rapp_hop_path = "/home/" + user + "/Desktop/rapp-platform-catkin-ws/src/rapp-platform/ric/hop_services/";

var ServerParams = require(rapp_hop_path + "utilities/./ServerParams.js");
//var params = new ServerParams(false, "155.207.19.37", 9001, "rappdev", "rappdev");
var params = new ServerParams(false, "localhost", 9001, "", "");
//var params = new ServerParams(false, "localhost", 9001, "", "");

var services = upServices().post(
  function (data){
    console.log( 'Received upRunning services names' );
    return data;
  },
  params
);

console.log(services);

