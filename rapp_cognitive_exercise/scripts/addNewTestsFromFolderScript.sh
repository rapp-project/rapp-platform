for f in $(find $1 -not -name "*~" -type f); do
echo $f;
rosservice call /rapp/rapp_cognitive_exercise/cognitive_test_creator "inputFile: '$f'"; done
