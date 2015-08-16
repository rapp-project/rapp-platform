## Synopsis

Containes RAPP Platform web services developed using hOP web broker.
These services are used in order to communicate with the RAPP Platform ecosystem
and access RIC(RAPP Improvement Center) AI modules.


## Initiate HOP Web Services
In order to run HOP Web Server and the registered HOP Web Services, run the **run.sh** script
located under this directory.

```bash
./run.sh
```

Every js file defined under the services/ directory with a .service.js extension is considered to be
a HOP Web Service js executable file and it is registered as a worker to HOP Web Server.


## Directories

- services/ : Front-end HOP services running on RAPP Platform.
- module/ : Module used for developing and testing HOP services.



## Tests

Developed tests and testing tools are currently located under:
```
utilities/testing_tools/
```

## Contributors

- Konstaninos Panayiotou, **[klpanagi@gmail.com]**
- Manos Tsardoulias, **[etsardou@gmail.com]**
- Vincent Prunet, **[vincent.prunet@inria.fr]**
