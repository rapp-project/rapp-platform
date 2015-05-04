#!/bin/bash

# Stress test script, used for hop service invocations

TEST=$1
PORT=$2
ITERATIONS=$3

for (( i=0;i<${ITERATIONS};i++)); do  
  hop -v -g -p $PORT ${TEST}
done 
