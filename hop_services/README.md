HOP Web Services
----------------

## Synopsis

Containes RAPP Platform web services developed using HOP web broker (Server).
These services are used in order to communicate with the RAPP Platform ecosystem
and access RIC(RAPP Improvement Center) AI modules.

Currently, HOP server is configured to act as an HTTP-Server.

**Note**: Possible extension to secure communication (HTTPS) is currently under consideration.


## Initiate HOP Web Services
In order to run HOP Web Server and the registered HOP Web Services, execute  **run.sh** script located under this directory.

```bash
./run.sh
```

Every js file defined under the services/ directory with a .service.js extension is considered to be
a HOP Web Service js executable file. HOP Server is responsible for registering those services, on initiation process.


**Note**: Do not change HOP Server configuration parameters, unless you know what you are doing!!

## Directories

- services/ : Front-end HOP web services running on RAPP Platform.
- module/   : Modules used for developing and testing HOP services.
- config/   : This directory containes configuration files. Those files are loaded by the HOP Server on run-time.



## Tests

Use the [rapp_platform_testing_tools](https://github.com/rapp-project/rapp-platform/tree/hop_services/rapp_testing_tools) package.


## Contributors

- Konstaninos Panayiotou, [ klpanagi@gmail.com ]
- Manos Tsardoulias, [ etsardou@gmail.com ]
- Vincent Prunet, [ vincent.prunet@inria.fr ]
