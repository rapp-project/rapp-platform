/*!
 * @file rosbridge.js
 *
 * @brief Rosbridge prototype that represents connection to rosbridge.
 *
 * Provides methods used to communicate with ROS (Robotic Operating System)
 * through a connection to rosbridge.
 *
 * In order to make use of this prototype, both ROS and rosbridge must
 * be installed and be up and running.
 *
 */

var RandStringGen = require ( /*rapp_hop_path +*/ "../utilities/./randStringGen.js" );


/*!
 * @brief Rosbridge connection constructor.
 */
function rosbridge( ){

  /*---<Private Members>---*/
  var url_ = undefined;

  var responseFlag_ = {};
  var serviceCalls_ = {};
  var receivedMsg_ = {};

  var timeoutValue_ = 10000; // rosbridge connection timeout value.
  var randStrGen_ = new RandStringGen( 5 );
  var this_ = this; //make use inside callback functions.

  /*-----------------------*/

  /*---<Priviledged members>---*/
  this.rosWS_ = undefined;
  this.serviceOP_ = {
    CALL_SERVICE: "call_service",
    ADVERTISE_SERVICE: "advertise_service",
    STOP_SERVICE: "stop_service"
  };

  //this.get_this = function( ){
    //return this;
  //};

  /*!
   * @brief Returns ROS Connection Timout value.
   * @return ROS connection timeout value.
   */ 
  this.get_timeoutValue = function ( ){
    return timeoutValue_;
  }

  /*!
   * @brief Returns internally set Rosbridge URL.
   * @param Rosbridge URL value.
   */
  this.get_rosbridgeURL = function ( ){
    return rosbridgeURL;
  };


  /*!
   * @brief Internally sets Rosbridge URL.
   * @param Rosbridge URL value.
   */
  this.set_rosbridgeURL = function ( _url ){
    url_ = _url;
  };


  /*!
   * @brief Check if a msg from the specific service call (_id) arrived.
   *
   * @param[in] _id Service Unique ID.
   *
   * @return True if a msg arrived. False otherwise.
   */
  this.msgReceived = function ( _callID  ){
    if ( responseFlag_[ _callID ] == true ){
      return true;
    }
    else{
      return false;
    }
  };


  /*!
   * @brief Stores received from WebSocket, message.
   *
   * @param[in] Message to store.
   */
  this.add_receivedMsg = function ( _msg, _callID ){
    receivedMsg_[ _callID ] = _msg;
    console.log( "[ROS-bridge]: Storing msg of callID: [%s]", _callID );
  };


  /*!
   * @brief Returns received from WebSocket, message.
   */
  this.get_receivedMsg = function ( _callID ){
    return receivedMsg_[ _callID ];
  };


  /*!
   * @brief Delete specific service_call_id msg from MAP.\
   */
  this.rm_receivedMsg = function ( _callID ){
    delete  receivedMsg_[ _callID ]; 
  };


  /*!
   * @brief Set the message response flag for the specific service call.
   *   Service calls are defined by a unique ID.
   *
   * @param[in] _id Service unique ID.
   */ 
  this.set_responseFlag = function ( _callID ){
    responseFlag_[ _callID ] = true;
  };


  /*!
   * @brief Set the message response flag for the specific service call.
   *   Service calls are defined by a unique ID.
   *
   * @param[in] _id Service unique ID.
   */ 
  this.clear_responseFlag = function ( _callID ){
    responseFlag_[ _callID ] = false;
  };


  /*!
   * @brief Delete specific service_call_id flag from MAP.\
   */
  this.rm_resposnseFlag = function ( _callID ){
    delete responseFlag_[ _callID ];
  };

  /*!
   * @brief Removes values addressing the specific service call id
   */
  this.rm_serviceCall_all = function ( _callID ){
    this.rm_resposnseFlag( _callID );
    this.rm_receivedMsg( _callID );
  };

  /*!
   * @brief Increment parallel calls to the specific ROS service
   * 
   * @param[in] _serviceName Name of the Service.
   */
  this.inc_serviceCalls = function ( _serviceName ){
    if ( serviceCalls_[_serviceName] == undefined ){
      serviceCalls_[_serviceName] = 1;
    }
    else{
      serviceCalls_[_serviceName] = serviceCalls_[_serviceName] + 1;
    }
  };


  /*!
   * @brief Decrement parallel calls to the specific ROS service
   * 
   * @param[in] _serviceName Name of the Service.
   */
  this.dec_serviceCalls = function ( _serviceName ){
    if ( serviceCalls_[_serviceName] !== undefined ){
      if ( serviceCalls_[_serviceName] !== 0 ){
        serviceCalls_[_serviceName] = serviceCalls_[_serviceName] - 1;
      }
    }
  };


  /*!
   * @brief Generates a unique id not allready used.
   *
   * @return Unique ID (String).
   */
  this.genUniqueID = function ( ){
    var uniqueID = randStrGen_.createUnique( );
    return uniqueID;
  };


  /*!
   * @brief Removed the unique id that adresses the service_call_id from
   *   cached unique id's
   */
  this.rm_uniqueID = function ( _callID ){
    /*--<Split the _callID to get the cached unique string id from it>--*/
    var uniqueID = _callID.split( "-" )[1];
    randStrGen_.removeCached( uniqueID );
  };


  /*!
   * @brief Sends a message to ROS-bridge using Websocket.
   *
   * @param[in] _msg Message to be sent.
   */
  this.send_message = function ( _msg, _callID ){
    var _this = this;
    /*--<Clear msg response flag of this service call>--*/
    this.clear_responseFlag( _callID );
    console.log( "[ROS-bridge]: Sending message" );
    this.rosWS_.send( JSON.stringify( _msg ) ); //sends the message.   
    this.rosWS_.onmessage = function (event){
      console.log( "[ROS-bridge]: Received message" );
      var received = event.value;
      var msg = JSON.parse( received );
      var callID = msg.id;
      _this.add_receivedMsg( msg, callID );
      _this.set_responseFlag( callID );
    };

  };

   /*---------------------------*/

  
};


//######################--PROTOTYPE MEMBERS--##########################//


/*!
 * @brief Initiates a connection with rosbridge
 * 
 * @param[int] rosbridgeURL ROS-bridge WebSocket URL.
 *
 * @return Undefined.
 */
rosbridge.prototype.connect = function( rosbridgeURL ){
  /*--<If not a rosbridgeURL is defined, set the url at localhost>--*/
  var _rosbridgeURL = rosbridgeURL || 'ws://localhost:9090';
  this.set_rosbridgeURL( _rosbridgeURL );
  
  this.rosWS_ = new WebSocket( _rosbridgeURL );
  /*--<WebSocket onopen callback handler>--*/
  this.rosWS_.onopen = function (event) {
    console.log('[ROS-bridge]: Connection opened');
  };

  /*--<WebSocket onclose callback handler>--*/
  //this.rosWS_.onclose = function (event) {
    //console.log('[ROS-bridge]: Connection closed');
  //};

  /*--<WebSocket onerror callback handler>--*/
  //this.rosWS_.onerror = function( error ){
    //console.log( '[ROS-bridge]: Websocket error: ' + error );
  //};

};


/*!
 * @beief Closes Connection with ROS-bridge.
 *
 * @return Undefined.
 */
rosbridge.prototype.close = function( ){
  this.rosWS_.close();
  //this.rosWS_ = undefined;
  //console.log('[ROS-bridge]: Connection closed');
};


/*!
 * @brief Calls the requested ROS Service Asynchronous.
 *
 * @param[in] _serviceName The name of the service to call.
 * @param[in] _args Service message arguments.
 *
 * @return ROS service returned message.
 */
rosbridge.prototype.callServiceSync = function( _serviceName, _args, headerOn ){
  /*--<Get current time>--*/
  var currentTime = new Date().getTime();

  this.inc_serviceCalls( _serviceName );
  /*--<Unique service ID for this service call>--*/
  var serviceCallID = _serviceName + "-" +
    this.genUniqueID( _serviceName );
  /*---------------------------------------------*/

  var args = {};
  if (headerOn)
  {
    /*--<Create service message header>--*/
    var header = {
      "seq": 1,
      "stamp": currentTime,
      "frame_id": " "
    };
    /*-----------------------------------*/

    /*Add service specified message arguments*/
    args[ "header" ] = header;
  }
  for ( var key in _args ){
    args[ key.toString() ] = _args[ key ];
  }

  var msg = {
    "op": this.serviceOP_.CALL_SERVICE,
    "service": _serviceName,
    "args": args,
    "id": serviceCallID 
  };
  /*---------------------------------------------------*/

  /*----Count time passed used for respose timeout----*/
  var startT = new Date( ).getTime( );
  var endT;
  /*--------------------------------------------------*/

  this.send_message( msg, serviceCallID );

  while( this.msgReceived( serviceCallID ) == false )
  {
    endT = new Date().getTime();
    if( (endT - startT) > this.get_timeoutValue( ) )
    {
      console.log("[ROS-bridge]: " + 
        "ROS SERVICE COMMUNICATION TIMED OUT!");
      this.add_receivedMsg( "ROS RESPONSE TIMEOUT!" );
      break;
    }
  }

  /*--<Get received message addressed for the specific serviceCallID>--*/
  var retMsg = this.get_receivedMsg( serviceCallID );
  /*--<Remove Cached values addressed for the specific serviceCallID>--*/
  this.rm_serviceCall_all( serviceCallID );
  /*--<Remove cached uniqueID created from randomStringGenerator>--*/
  this.rm_uniqueID( serviceCallID );
  /*--<Decrement the number of calls to the specific ROS service>--*/
  this.dec_serviceCalls( _serviceName ); 
  return retMsg;
};



//exports the class as a module
module.exports = rosbridge; 


  
