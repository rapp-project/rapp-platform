/*!
 * @file serve.js
 * @brief Serve prototype/class definition.
 *
 * Holds methods used for communicating with external services.
 */

/*######################-<Private Variables here>-#######################*/

//var rosConnect;

/*#######################################################################*/


/*#################-<Private Functions/Members here>-####################*/

/*#######################################################################*/

/*!
 * @brief File Streaming Class/Object definition.
 */
function RosUtils(){
  /*-----Private methods/properties goes here (var getX = function(){})*/
  var kind_ = 'RosUtils';
  var rosbridgeURL_ = '';
  var rosConnect_;
  var runTime_ = 0;
  var timeOutValue_ = 10000;

  /*--------------------------------------------------------*/

  /*---Privileged methods goes here (this.getX = function(){})---*/

  /*!
   * @brief ROS Connection Timout value get(er).
   * @return ROS connection timeout value.
   */ 
  this.getTimeoutValue = function(){
    return timeOutValue_;
  }

  /*!
   * @brief ROS Connection runtime get(er).
   * @return ROS connection runtime value.
   */ 
  this.getRunTime = function(){
    return runTime_;
  }

  /*!
   * @brief Rosbridge URL get(er).
   * @return Rosbridge URL.
   */ 
  this.getRosbridgeURL = function(){
    return rosbridgeURL_;
  }

  /*!
   * @brief Prototype kind value get(er).
   * @return RosUtils prototype kind value.
   */ 
  this.getKind = function(){
    return kind_;
  }

  /*!
   * @brief Rosbridge communication websocket object get(er).
   * @return RosConnect websocket object.
   */
  this.getRosConnect = function(){
    return rosConnect_;
  }

  /*!
   * @brief Rosbridge URL set(er).
   * @param Rosbridge URL value.
   */
  this.setRosbridgeURL = function(value){
    rosbridgeURL_ = value;
  }

  /*!
   * @brief Rosbridge connection runtime set(er).
   * @param Rosbridge runtime value.
   */
  this.setRunTime = function(value){
    runTime_ = value;
  }

  /*!
   * @brief Rosbridge connection timeout value set(er).
   * @param Rosbridge connection timeout value.
   */
  this.setTimeoutValue = function(value){
    timeOutValue_ = value;
  }

  /*!
   * @brief Rosbridge communication websocket object set(er).
   * @param Rosbridge communication websocket object (value).
   */
  this.setRosConnect = function(value){
    rosConnect_ = value;
  }
  /*-------------------------------------------------------------*/


};


/*#######################-<Prototype Methods/Members>-##########################*/


/*!
 * @brief Call this to initialize connection with ROS through rosbridge.
 * @param _rosbridgeURL Rosbridge URL, ws://localhost:9090 by default.
 */
RosUtils.prototype.init_bridge = function (_rosbridgeURL){
  if(_rosbridgeURL==''){
    this.setRosbridgeURL('ws://localhost:9090');
  }
  else{
    this.setRosbridgeURL(_rosbridgeURL);
  }
  /*<Initialize a WebSocket object to communicate with rosbridge>*/
  this.setRosConnect( new WebSocket( this.getRosbridgeURL() ) );
  this.getRosConnect().onopen = function(){
    console.log('\033[0;33mROS_bridge Connection established!\033[0;0m');
  }
  /*<Initialize runtime>*/
  this.setRunTime( new Date().getTime() );
};


/*!
 * @brief Make call to a ROS service.
 * @param _serviceName To be called service name.
 * @param _args Arguments of the specific ROS service.
 */
RosUtils.prototype.callService = function (_serviceName, _args){
  var currentTime = new Date().getTime();
  var header = {
    "seq": 1,
    "stamp": currentTime,
    "frame_id": " "
  };
  var msg = {
    "op": "call_service",
    "service": _serviceName,
    "args": _args
  };
  var flag = false;
  var retMessage = '';
  this.getRosConnect().send( JSON.stringify(msg) );
  var startT = new Date().getTime();
  var endT;
  while(flag==false)
  {
    this.getRosConnect().onmessage = function (event)
    {
      var received = event.value;
      retMessage = JSON.parse (received);
      flag = true;
    }
    endT = new Date().getTime();
    if( (endT - startT) > this.timeOutValue )
    {
      console.log("\033[01;31mROS SERVICE COMMUNICATION TIMED OUT!\033[0;0m");
      retMessage = "ROS RESPONSE TIMEOUT!";
      break;
    }
  }
  return retMessage;
}


/*########################################################################*/


//exports the class as a module
module.exports = RosUtils; 

