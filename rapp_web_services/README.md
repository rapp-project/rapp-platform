RAPP Platform Web Services
----------------

## Synopsis

Developed algorithms located under RAPP Improvement Center (RIC), can be utilized by robots via the RAPP Platform Web Services, exposed by RIC and furthermore the RAPP Platform. We consider RAPP Platform Web Services an imperative component of the RAPP Improvement Center (RIC) as they define the client-server interface layer.


HOP Web Services run under HOP Web Broker, as a part of RIC. Communication between remote clients and the HOP Server, and furthermore HOP Web Services, is achieved via **HTTP Post requests**. HOP Web Server is exposed to the web via a public IPv4 Address. Furthermore the HOP Web Services run on a specific system port and so those are accessible from remote clients via the HOP Web Server running IPv4 Address followed by a port (**xxx.xxx.xxx.xxx:8080**).

As HOP Web Services act as the interface layer between the Robot Platform and the Cloud Platform (along with the RAPP API), the development of those depends on both Client (User/Developer) and RIC requirements.

The  communication  workflow,  from  the  client-side  platform  to  RAPP  Services  can  be  severed  into  the  following independent interface layers:

- Client-side Platform -> HOP Server.
- HOP Server -> HOP Web Services.
- HOP Web Services -> Rosbridge WebSocket Server.
- Rosbridge WebSocket Server -> RAPP Services (ROS Services).

The RAPP API, used from the client-side platform, integrates calls to the HOP Server and furthermore to the HOP Web Services. A HOP Web Service delegates service calls to the RAPP Services through the [Rosbridge](https://github.com/RobotWebTools/rosbridge_suite) interface layer.

Rosbridge Server provides a WebSocket transport layer to ROS and so it allows web clients to talk ROS using the rosbridge protocol. We use the Rosbridge Websocket Server in order to interfere with AI modules developed under ROS. That WebSocket server is not exposed to the public network and it is only accessible locally.

Furthermore, user authentication on Hop Web Services requests, is handled by Hop Web Server.
Currently we use **HTTP-Basic-Authentication** technique for enforcing access controls to the RAPP Platform web resources.

Currently, HOP Web Broker is configured to act as a remote HTTP-Server.

As we integrate HOP to be the RAPP Platform Web front-end, we reference the RAPP Platform
Web Services as **HOP Web Services**.

**Note**: Possible extension to secure communication (HTTPS) is currently under consideration.


## Initiate HOP Web Services

On a fresh clone you will first need to install devDependencies:

```shell
$ npm install
```

For initiating HOP Web Server (and HOP Web Services), execute, under this directory:

```shell
$ npm start
```

Any js file, stored under the services/ directory with a .service.js extension, is considered to be
a HOP Web Service js executable file. HOP Server is responsible for registering those services, on run-time, as workers.


**Note**: Do not change HOP Server configuration parameters, unless you know what you are doing!!

## Directories

- services/ : Front-end HOP web services running on RAPP Platform.
- module/   : Common developed modules used for developing and testing HOP services.
- config/   : This directory containes configuration files. Those files are loaded by the HOP Server on run-time.



## Tests

Have a look at the [rapp_testing_tools](https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools) package.


## Documentation

This package's source-code is documented using the [JSDoc](https://github.com/jsdoc3/jsdoc) javascript documentation generator.

Generate documentation for HOP Web Services ONLY!:

```shell
grunt jsdoc:services
```

Generate documentation for service-templates ONLY!:

```shell
grunt jsdoc:templates
```

Generate documentation for commons ONLY!:

```shell
grunt jsdoc:commons
```

Gererate documentation for ALL:

```shell
$ make doc
```

Or

```shell
$ grunt jsdoc
```

Makefile tasts integrates grunt tasks on generating source-code documentation.


Generated doc files are located under the **doc/** directory, or under:

```shell
$ ${HOME}/rapp_platform_files/
```

if you installed the rapp-platform using the provided by the RAPP developers, setup scripts.

Furthermore, you can navigate through the rapp-platform [Wiki](https://github.com/rapp-project/rapp-platform/wiki) for more **in-depth** documentation.

## Contributors

- Konstaninos Panayiotou, [ klpanagi@gmail.com ]
- Manos Tsardoulias, [ etsardou@gmail.com ]
- Vincent Prunet, [ vincent.prunet@inria.fr ]
