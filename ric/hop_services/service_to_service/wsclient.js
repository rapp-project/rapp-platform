/*=====================================================================*/
/*    serrano/prgm/project/hop/3.0.x/examples/wsclient/wsclient.js     */
/*    -------------------------------------------------------------    */
/*    Author      :  Manuel Serrano                                    */
/*    Creation    :  Wed May 14 17:02:10 2014                          */
/*    Last change :  Wed May 21 13:29:53 2014 (serrano)                */
/*    Copyright   :  2014 Manuel Serrano                               */
/*    -------------------------------------------------------------    */
/*    WebSocket client example                                         */
/*    -------------------------------------------------------------    */
/*    run: hop -v -g wsclient.js                                       */
/*    (this assumes an echo WebSocket server at localhost:9999)        */
/*=====================================================================*/

var ws = new WebSocket( "ws://localhost:9999/hop/wss" );

service fileGet( path ) {
   return hop.HTTPResponseFile( path );
}
ws.onopen = function( event ) {
   //var file = fileGet.resource( "santorini.jpg" );
   var numbers = "56";
   this.send( numbers );
};

ws.onmessage = function( event ) {
   console.log( "received [%s]", event.value );
};
