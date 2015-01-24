#!/bin/bash

TEST=$1
ITERATIONS=$2

for (( i=0;i<${ITERATIONS};i++)); do  
  echo `node --harmony ${TEST}`
done 
