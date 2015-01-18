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
function RosUtils ( ){
  /*-----Private methods/properties goes here (var getX = function(){})*/
  var kind_ = 'RosUtils';
  var rosbridgeURL_ = '';
  var rosWebSocket_ = null;
  var runTime_ = 0;
  var timeOutValue_ = 10000;
  var resposeMessage_ = null;
  var flags_ = {
    msgResponse: false
  };

  /*--------------------------------------------------------*/

  /*---Privileged methods goes here (this.getX = function(){})---*/

  this.getFlags = function ( ){
    return flags_;
  }

  /*!
   * @brief ROS Connection Timout value get(er).
   * @return ROS connection timeout value.
   */ 
  this.getTimeoutValue = function ( ){
    return timeOutValue_;
  }

  this.msgResponse = function ( ){
    if ( flags_.msgResponse == true ){
      return true;
    }
    else{
      return false;
    }
  }

  this.getResponseMsg = function ( ){
    return resposeMessage_;
  }

  /*!
   * @brief ROS Connection runtime get(er).
   * @return ROS connection runtime value.
   */ 
  this.getRunTime = function ( ){
    return runTime_;
  }

  /*!
   * @brief Rosbridge URL get(er).
   * @return Rosbridge URL.
   */ 
  this.getRosbridgeURL = function ( ){
    return rosbridgeURL_;
  }

  /*!
   * @brief Prototype kind value get(er).
   * @return RosUtils prototype kind value.
   */ 
  this.getKind = function ( ){
    return kind_;
  }

  /*!
   * @brief Rosbridge communication websocket object get(er).
   * @return RosConnect websocket object.
   */
  var getWebSocket = function ( ){
    return rosWebSocket_;
  }

  var setMsgResponse = function ( ){
    flags_.msgResponse = true;
  }

  var clearMsgResponse = function ( ){
    flags_.msgResponse = false;
  }

  /*!
   * @brief Rosbridge URL set(er).
   * @param Rosbridge URL value.
   */
  this.setRosbridgeURL = function ( _value ){
    rosbridgeURL_ = _value;
  }

  /*!
   * @brief Rosbridge connection runtime set(er).
   * @param Rosbridge runtime value.
   */
  this.setRunTime = function ( _value ){
    runTime_ = _value;
  }

  /*!
   * @brief Rosbridge connection timeout value set(er).
   * @param Rosbridge connection timeout value.
   */
  this.setTimeoutValue = function ( _value ){
    timeOutValue_ = _value;
  }

  /*!
   * @brief Rosbridge communication websocket object set(er).
   * @param Rosbridge communication websocket object (value).
   */
  var setWebSocket = function ( _value ){
    rosWebSocket_ = _value;
  }

  /*!
   * @brief Creates and initializes a WebSocket to rosbridge.
   */
  this.createSocket = function ( ){
    if ( typeof rosWebSocket_ !== "undefined" ){
      /*Creates a new WebSocket instance if not allready exists!*/
      rosWebSocket_ = new WebSocket( rosbridgeURL_ );
      console.log( "\033[0;32mSuccesfully initialized a websocket to rosbridge\033[0;0m" );
      /*--------------------------------------------------------*/
      
      /*Implementation of websocket onopen callback*/
      rosWebSocket_.onopen = function( ){
        console.log('\033[0;33mROS_bridge Connection established!\033[0;0m');
      }
      /*-------------------------------------------*/

      /*Implementation of websocket onerror callback*/
      rosWebSocket_.onerror = function( error ){
        console.log( '\033[0;31mWebsocket error: ' + error );
      }
      /*--------------------------------------------*/


      /*Implementation of websocket onclose callback*/
      rosWebSocket_.onclose = function( ){
        console.log('\033[0;33mConnection to Rosbridge closed!\033[0;0m');
      }
      /*--------------------------------------------*/
    }
    else{
      console.log( "\033[01;31mRosbridge websocket allready exists. Invalid call!!!\033[0;0m" );
    }
  }

  this.sendMessage = function ( _msg ){
    
    clearMsgResponse( ); //clears messageReceived flag.

    console.log( "Sending message to rosbridge" );
    rosWebSocket_.send( JSON.stringify( _msg ) ); //sends the message.
    rosWebSocket_.onmessage = function (event)
    {
      console.log( "Received message from rosbridge" );
      var received = event.value;
      resposeMessage_  = JSON.parse (received);
      //flags_.msgResponse = true;
      setMsgResponse();
      this.setMsgResponse( ); //HACK TO FORCE CLOSE WEBSOCKET!!!!
    }
  }
  /*-------------------------------------------------------------*/


};


/*#######################-<Prototype Methods/Members>-##########################*/


/*!
 * @brief Call this to initialize connection with ROS through rosbridge.
 * @param _rosbridgeURL Rosbridge URL, ws://localhost:9090 by default.
 */
RosUtils.prototype.init_bridge = function (_rosbridgeURL){
  this.setRosbridgeURL( _rosbridgeURL || 'ws://localhost:9090' );
  
  /*<Initialize a WebSocket object to communicate with rosbridge>*/
  this.createSocket();  

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
    
  /*Add user specified arguments needed (except header.)*/
  var header = {
    "seq": 1,
    "stamp": currentTime,
    "frame_id": " "
  };
  
  var args = {};
  args[ "header" ] = header;

  for ( var key in _args ){
    args[ key.toString() ] = _args[ key ];
  }

  var msg = {
    "op": "call_service",
    "service": _serviceName,
    "args": args
  };
  /*---------------------------------------------------*/

  var retMessage = null;
  this.sendMessage( msg ); //Sends call to service message.
  
  /*----Count time passed used for respose timeout----*/
  var startT = new Date( ).getTime( );
  var endT;
  /*--------------------------------------------------*/
  
  while( this.msgResponse() == false )
  {
    
    endT = new Date().getTime();
    if( (endT - startT) > this.getTimeoutValue( ) )
    {
      console.log("\033[01;31mROS SERVICE COMMUNICATION TIMED OUT!\033[0;0m");
      retMessage = "ROS RESPONSE TIMEOUT!";
      break;
    }
  }
  return this.getResponseMsg( );
}


/*########################################################################*/


//exports the class as a module
module.exports = RosUtils; 

