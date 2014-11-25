/*=====================================================================*/
/*    serrano/prgm/project/hop/3.0.x/examples/wsserver/wsserver.js     */
/*    -------------------------------------------------------------    */
/*    Author      :  Manuel Serrano                                    */
/*    Creation    :  Wed May 14 17:02:10 2014                          */
/*    Last change :  Wed May 21 13:02:34 2014 (serrano)                */
/*    Copyright   :  2014 Manuel Serrano                               */
/*    -------------------------------------------------------------    */
/*    WebSocket server example                                         */
/*    -------------------------------------------------------------    */
/*    run: hop -v -g wsserver.js                                       */
/*=====================================================================*/

var wss = new WebSocketServer( "wss" );


wss.onconnection = function( event ) {
   var ws = event.value;
   

   console.log( "connection established:", ws.socket );
   //var tf= ["5","6"];
  // console.log(tf);
   var sendback="eee";
   console.log("sendback "+sendback);
   ws.onmessage = function( event ) {
      
      console.log( "received [%s]", event.value);
      var a = parseInt(event.value[0]);
      var b = parseInt(event.value[1]);
      var c =a+b;
      sendback=c.toString();
      console.log("sendback "+sendback);
      ws.send(sendback);

   };
   
};
