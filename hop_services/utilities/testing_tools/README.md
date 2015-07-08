## Synopsis

Testing tools used for testing HOP services available on RAPP Platform and furthermore **RIC AI modules**.

## Directories

- python_tests : Developed Tests written in python are located here.
- test_data : Data used from developed python tests are located here.

## Running Tests

### run_python_tests.py 

Python script used for running developed tests.

Arguments:
- -i <test_name> : Define test to run.
- -n <number_of_calls> : Define number of test calls.
- -t : Run the test threaded which means that multible invocations can be done
simultaneous.

####Example of use

- Run the qr_detectio_test_1 Test 5 times in sequential mode:

```bash
python -i qr_detection_test_1.py -n 5
```

### stress_test_general.sh

Calls every test file located under the python_tests directory in stress testing mode.
Both threaded/parallel calls and sequential.

```bash
bash stress_test_general.sh
```

### stress_test_parallel.sh

Calls every test file located under the python_tests directory in stress testing mode and evaluates the results.
Only threaded/parallel calls.

```bash
bash stress_test_parallel.sh
```

### stress_test_sequential.sh

Calls every test file located under the python_tests directory in stress testing mode and evaluates the results.
Only sequential calls.

```bash
bash stress_test_sequential.sh
```

### test_hop_services.sh

Calls one test for each hop service available on RAPP Platform and evaluates the results

```bash
bash test_hop_services.sh
```


## Developing Tests

- All tests written must be stored into the python_tests directory.
- Test files must be stored with python extension (.py).
- Data used by developed python tests must be stored into the test_data directory.
- All tests use the RappCloud python module in order to communicate with RAPP Platform AI modules.


## Contact
- Konstantinos Panayiotou, [klpanagi@gmail.com]
