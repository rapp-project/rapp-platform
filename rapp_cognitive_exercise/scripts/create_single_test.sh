#!/usr/bin/sh

ROS_SERVICE="/rapp/rapp_cognitive_exercise/cognitive_test_creator"

if [ -z $1 ]; then
  echo -e "No input text file given."
  exit 1
elif [ ! -f $1 ]; then
  echo -e "Input file [$1] does not exist"
  exit 1
else
  echo -e "Creating Cognitive Test .xml from text file ${1}"
fi

rosservice call $ROS_SERVICE "inputFile: '$1'"
