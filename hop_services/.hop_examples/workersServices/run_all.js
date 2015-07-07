/* initiates both services on the same port. Each service runs on a different worker 
to run
hop -v -p 9001
*/
var w = new Worker ("./slave.js");
var w1 = new Worker ("./sync_service.js");

