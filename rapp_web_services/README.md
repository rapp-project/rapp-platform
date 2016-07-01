RAPP Platform Web Services
----------------

## Synopsis

RAPP Improvement Center (RIC) nodes, can be utilized by robots via the RAPP Platform Web Services. The service layer has been developed using HOP. That consists of a web server implementation, an http/https server, and the web services developed in Hop.js framework. Web services run within server-side workers (web workers). A worker can include more than one web service. We consider server-side workers to be forked processes, thus allowing concurrent execution.

As HOP-Web-Services act as the interface layer between the Robot Platform and the Cloud Platform (along with the RAPP API), the development of those depends on both Client (User/Developer) and RIC requirements.

Web service responses are asynchronous http responses. This way we allow robot platforms to request for cloud services while performing other tasks.

Robot platforms can access RAPP Platform services using **HTTP POST** requests, as most of the services require an arbitrary amount of data to be sent to the Cloud for processing, like image and audio data files. HOP Web Server has been configured to accept *application/x-www-urlencoded* and *multipart/form-data form submissions.

The RAPP API, used from the client-side platform, integrates calls to the HOP Server and furthermore to the HOP Web Services. A HOP Web Service delegates service calls to the RAPP Services through the [Rosbridge](https://github.com/RobotWebTools/rosbridge_suite) transport layer.
Rosbridge Server provides a WebSocket transport layer to ROS and so it allows web clients to talk ROS using the rosbridge protocol. We use the Rosbridge Websocket Server in order to interfere with AI modules developed under ROS. That WebSocket server is not exposed to the public network and it is only accessible locally.

The  communication  workflow,  from  the  client-side  platform  to  RAPP  Services  can  be  severed  into  the  following independent interface layers:

- Client-side Platform -> HOP Server.
- HOP Server -> HOP Web Services.
- HOP Web Services -> Rosbridge WebSocket Server.
- Rosbridge WebSocket Server -> RAPP Services (ROS Services).


Currently, HOP-Server is configured to act as an HTTP/HTTPS Web Server (**Does not accept proxy connections**).


## Start HOP Web Server

On a fresh clone you will first need to install required dependencies:

```bash
$ npm install
```

For initiating HOP Web Server (and HOP Web Services), a grunt task exists:

```bash
$ grunt init-hop
```

Or execute the **run.sh** script directly:

```bash
$ ./run.sh
```


We also provide [pm2](https://github.com/Unitech/pm2) configuration file for the web server to launch. If you have pm2 installed on the machine hosting the RAPP Platform, simply execute:

```bash
pm2 start server.yaml
```


**Note**: Do not change HOP Server configuration parameters, unless you know what you are doing!!


## Directories

- services/ :  Web Services implementations (Source files).
- workers/ : Web Workers implementations (Source files).
- src/ : Common source directory (Except web-service and web-worker implementations).
- config/   : This directory containes several configuration files for the HOP Server, web-workers and the web-services to use.



## Tests

Have a look at the [rapp_testing_tools](https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools) package.


## Documentation

This package's source-code is documented using the [JSDoc](https://github.com/jsdoc3/jsdoc) javascript documentation generator.

Generate documentation for HOP Web Services ONLY!:

```bash
$ grunt jsdoc:services
```

Generate documentation for service-creation-templates ONLY!:

```bash
$ grunt jsdoc:templates
```

Generate documentation for commons ONLY!:

```bash
$ grunt jsdoc:commons
```

Gererate documentation for ALL:

```bash
$ grunt jsdoc
```


Generated doc files are stored under the **doc/** directory, or under:

```bash
$ ${HOME}/rapp_platform_files/
```

if you installed the rapp-platform using the provided by the RAPP developers, setup scripts.

Furthermore, you can navigate through the rapp-platform [Wiki](https://github.com/rapp-project/rapp-platform/wiki) for more **in-depth** documentation.

## Contributors

- Konstaninos Panayiotou, [ klpanagi@gmail.com ]
- Manos Tsardoulias, [ etsardou@gmail.com ]
- Vincent Prunet, [ vincent.prunet@inria.fr ]
