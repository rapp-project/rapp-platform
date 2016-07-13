RAPP Testing Tools
------------------


## Synopsis

Testing tools used for RAPP Platform integration tests.

All tests use the Python implementation of the RAPP Platform API,[rapp-platform-api/python](https://github.com/rapp-project/rapp-api/tree/master/python), to request for .

Test classes inherit from Python unit testing framework, [unittest](https://docs.python.org/2.7/library/unittest.html)

## Tests Directory

By default, this package containes out-of-the-box, integration tests. These tests are located
under:

```bash
$ <rapp_testing_tools_package_path>/scripts/default_tests
```


## Executing Tests

The rapp_run_test.py script is used in order to execute developed tests.

##### Execution time arguments:
- [ **-i** ] : Test filepath, to execute. If empty, all tests under the **default_tests** directory will be executed.

```bash
$ ./rapp_run_test_.py -i default_tests/face_detection_tests.py
```

- [ **-n** ] :  Number of times of test(s) execution.

```bash
$ ./rapp_run_test.py -i default_tests/face_detection_tests.py -n 10
```

- [ **-t** ] : Run the test in threaded mode. Multible invocations can be done, simultaneous. Each test execution is handled by a single thread.

```bash
$ ./rapp_run_test.py -i default_tests/face_detection_tests.py -n 10 -t
```

**Note:**
If no test_name is provided as argument, all tests are executed!!


## Developing Tests

A template_test.py is located under the [default_tests](https://github.com/rapp-project/rapp-platform/tree/master/rapp_testing_tools/scripts/default_tests) directory of the rapp_testing_tools package.

```bash
$ <path_to_rapp_testing_tools_package>/scripts/default_tests/
```

Also, several tests have already been developed, located under the default_tests
directory. Those can be used as a reference/examples for developing new tests.


A Section in the [Wiki](https://github.com/rapp-project/rapp-platform/wiki/How-to-implement-an-integration-test%3F) exists that guides through developing integration tests for the RAPP Platform.

## Contributors

- Konstantinos Panayiotou, [klpanagi@gmail.com]
- Manos Tsardoulias. [etsardou@gmail.com]


## Contact

- Konstantinos Panayiotou, [klpanagi@gmail.com]
