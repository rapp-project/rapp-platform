RAPP Testing Tools
------------------


## Synopsis

Testing tools used for RAPP Platform integration tests.

All tests use the RappCloud python module (python RAPP-API) in order to communicate with RAPP Platform AI modules.


## Tests Directory

By default, this package containes basic written tests. These tests are located
under:

```shell
<rapp_testing_tools_package_dir>/scripts/default_tests
```


## Executing Tests

The rapp_run_test.py script is used in order to execute developed tests.

Execution time arguments:
- -i **<test_name>** : Define test to run (use python test file full name).
- -n **<number_of_calls>** : Define number of test calls.
- -t : Run the test in threaded mode which means that multible invocations can be done, simultaneous. Each test execution is handled by a standalone thread.

**Note:**
If no test_name is provided as argument, all tests located under tests paths
are executed!!

There are two ways of executing developed tests:
- Execute directly.
- Execute using the ROS framework.

### Using the python executable, directly.

The below example executes the qr_detection_test_1, five times in sequential
mode.

```shell
./rapp_run_test.py -i qr_detection_test_1.py -n 5
```

### Using ROS framework

The below example executes all tests, once.

```shell
rosrun rapp_testing_tools rapp_run_test.py
```


## Developing Tests

A template.py is located under the default_tests directory

```shell
scripts/default_tests
```

Also several tests have already been developed, located under the default_tests
directory, and can be used as a reference for developing new tests.


### Where to store developed tests

By default, developed tests are stored under scripts/default_tests. Though
developers can define different directory paths where tests are stored.

In order to define external tests directories, developers have to append that path
into the params.yaml configuration file. This file is located under the config/
directory in this package.

Lets say for example that you want to store tests under:

```shell
~/rapp_integration_tests_external
```

In order to define to testing-engine to search into that directory for developed tests,
that path have to e defined into  **params.yaml** file:

```yaml
tests_path:
  - '~/rapp_integration_tests_external'
```


## Contributors

- Konstantinos Panayiotou, [klpanagi@gmail.com]
- Manos Tsardoulias. [etsardou@gmail.com]


## Contact
- Konstantinos Panayiotou, [klpanagi@gmail.com]
