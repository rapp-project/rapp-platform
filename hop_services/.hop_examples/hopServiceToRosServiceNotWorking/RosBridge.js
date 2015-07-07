/** 
 * Represents a connection to a rosbridge.
 * @constructor 
 * @param {string} rosbridgeURL - the url of the rosbridge.
 */
function RosBridge(rosbridgeURL)
{
    this.url = rosbridgeURL;
    ws = new WebSocket(rosbridgeURL);
    ws.onopen = function (event) {
        console.log('ROSbridge connection opened');
    };
    ws.onclose = function (event)
    {
        console.log("ROSbridge connection closed");
    };
}

/**
 *  Calls a hop service asynchronously and send the result on a web socket when received.
 *  @param svc - service to call
 *  @param args - parameters of the call to the service
 *  @param message - reference on the rosbridge message causing this call to the service
 *  @param ws_reply - web socket used to transmit the result
 *  @param service_location - host and port of the service called 
 *
 *  @todo detect the current values of host and port, and use them as default values.
 */
function asyncWrapper(svc, args, message, ws_reply, service_location) {
    service_location = service_location || {host:"localhost", port:"9000"};
    
    console.log("entering asyncWrapper with message = ", message);

    var result = {};
    console.log("serviceProxyROS called", args, ws_reply);    
    svc(args).post(
        function(el) {
            el = el || {}; 
            var response = {
                op: "service_response",
                request_id: message.request_id,
                data: el
            };
            ws_reply.send(JSON.stringify(response));
        },
        service_location
    );
    console.log("after call");
}

/** 
 *   Example of string "service:/hop/increaseCounter_count:0_time:10:36:435535".
 *   
 *   @param requestId String identifying the service message.
 *   @return { protocol: "service", service: "/hop/increaseCounter" }
 */
function parseRequestId( requestId ) {
    var result = { protocol: undefined, service: undefined };
    var parser = new RegExp("^(\\w+):([\/\\w]+)_count");
    var match = requestId.match(parser);
    if (match != null) {
        result.protocol = match[1];
        result.service = match[2];
    }
    return result;
}

RosBridge.ADVERTISE_SERVICE = "advertise_service";
RosBridge.CALL_SERVICE = "call_service";
RosBridge.STOP_SERVICE = "stop_service";

var service_map = {};  // Maps a service_name to the service to be called.

RosBridge.prototype = {
    call_service: function (service_name, args, callback) {
        var msg = {
            op: RosBridge.CALL_SERVICE,
            service: service_name,
            args: args,
            id: "test"
        };
        ws.send(JSON.stringify(msg));
        service_map["test"] = callback;
    },

    /**
     *  Register a service in ROS, using rosbridge.
     *  @param service_name - the path of the service
     *  @param module_type - the module defining the service
     *  @param service_type - the type name of the service
     *  @param callback - hop service to call when this service is called
     *  @param service_location - location of the service 
     */
    advertise_service: function (service_name, module_type, service_type, callback, service_location) {
 
        var msg = {
            op: RosBridge.ADVERTISE_SERVICE,
            service_module: module_type,
            service_type: service_type,
            service_name: service_name

        };
        ws.send(JSON.stringify(msg));
        
        service_map[service_name] = callback;
        // @Bug Workaround for a bug in the implementation of .onmessage in hop: 
        // the handler is added (i.e. as using addEventHandler) instead of being overwritten.
        if (!ws.onmessage) { 
            ws.onmessage = function (event)
            {                
                console.log("event handler for ros service ", service_name);
                console.log("onmessage called, event = ", event);
               
                var eventValue = JSON.parse(event.value);
                console.log(eventValue);

                if (eventValue.request_id != undefined) {
                    var parsedMessage = parseRequestId(eventValue.request_id);
                    console.log(parsedMessage);                    
                    console.log("before callback for ", service_map[parsedMessage.service]);                      
                    asyncWrapper(service_map[parsedMessage.service], eventValue.args, eventValue, ws, service_location);
                    console.log("after callback");                    
                } else if (eventValue.id != undefined) {
                    // Response of RosBridge to a call to a CALL_SERVICE operation.
                    console.log(" in eventvalue.id" );
                    console.log(eventValue.values);
                    service_map[eventValue.id](eventValue.values);
                }
            };
        }
        console.log(ws.onmessage);
    },

    /** 
     *   Unregister the service from ROS.
     *   @param service_name - name of the service to unregister.
     */
    stop_service: function (service_name) {
        var msg = {
            op: RosBridge.STOP_SERVICE,
            service_name: service_name
        };
        ws.send(JSON.stringify(msg));
    },

    close: function () {
        ws = undefined;
    }
};
module.exports.RosBridge = RosBridge;
module.exports.parseRequestId = parseRequestId;

