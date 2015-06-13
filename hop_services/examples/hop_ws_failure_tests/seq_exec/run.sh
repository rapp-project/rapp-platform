#!/bin/bash

RUNS=$1

for i in {1..50}
do
  hop -v -g -p 9001 test.js 
done

