#!/bin/bash

TEST=$1
ITERATIONS=$2

for (( i=0;i<${ITERATIONS};i++)); do  
  echo `hop -v -g -p 9005 ${TEST}`
done 
