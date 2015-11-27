RAPP Testing Tools
------------------


## Synopsis

Testing tools used for RAPP Platform integration tests.

All tests use the RappCloud python module,[python-rapp-api](https://github.com/rapp-project/rapp-api/tree/master/python), to communicate with RAPP Platform AI modules.


## Tests Directory

By default, this package containes basic, out-of-the-box, integration tests. These tests are located
under:

```shell
$ <rapp_testing_tools_package_path>/scripts/default_tests
```


## Executing Tests

The rapp_run_test.py script is used in order to execute developed tests.

##### Execution time arguments:
- [ **-i** ] : Give as input, the test name, to execute.

```shell
$ ./rapp_run_test_.py -i face_detection_test_1.py
```

- [ **-n** ] : Give number of execution of test(s).

```shell
$ ./rapp_run_test.py -i face_detection_test_1.py -n 10
```

- [ **-t** ] : Run the test in threaded mode which means that multible invocations can be done, simultaneous. Each test execution is handled by a standalone thread.

```shell
$ ./rapp_run_test.py -i face_detection_test_1.py -n 10 -t
```


-  [ **-c** ] :  Test classes can be used to execute a spesific family of tests. Using test classes all tests, under this class, will be executed. In most cases a test class is named after the relevant RAPP Platform ROS-Package; [ **face-detection, qr-detection, speech-detection, speech-detection-sphinx4, speech-detection-google, ontology, cognitive, tts** ]

```shell
$ ./rapp_run_test.py -c face-detection
```
 

**Note:**
If no test_name is provided as argument, all tests located under tests paths
are executed!!

There are two ways of executing developed tests:
- Execute directly.
- Execute under the ROS framework.

### Using the python executable, directly.

The below example executes the qr_detection_test_1, five times in sequential
mode.

```shell
$ ./rapp_run_test.py -i qr_detection_test_1.py -n 5
```

### Using ROS framework

The below example executes all tests, once.

```shell
$ rosrun rapp_testing_tools rapp_run_test.py
```


## Developing Tests

A template.py is located under the [default_tests](https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools/scripts/default_tests) directory of the rapp_testing_tools package.

```shell
$ <path_to_rapp_testing_tools_package>/scripts/default_tests
```

Also several tests have already been developed, located under the default_tests
directory. Those can be used as a reference for developing new tests.


### Where to store developed tests

By default, developed tests are stored under the scripts/default_tests directory. 

To define external tests directories, developers have to append that path
into the params.yaml configuration file. This file is located under the config/
directory in this package.

Lets say for example that you want to store tests under:

```shell
$ ${HOME}/rapp_integration_tests_external
```

You can extend the path directories used by the test-engine to search under for python test source files. Just write down custom directory paths into the [params.yaml]([params.yaml](https://github.com/rapp-project/rapp-platform/blob/master/rapp_testing_tools/config/params.yaml)) file.

```yaml
tests_path:
  - '~/rapp_integration_tests_external'
```


## Contributors

- Konstantinos Panayiotou, [klpanagi@gmail.com]
- Manos Tsardoulias. [etsardou@gmail.com]


## Contact
- Konstantinos Panayiotou, [klpanagi@gmail.com]
