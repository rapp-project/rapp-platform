Documentation about the RAPP Cognitive Exercise: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Cognitive-Exercise)

#Methodology
The RAPP Cognitive exercise system aims to provide the Robot users a means of performing basic cognitive exercises. The cognitive tests supported belong to three distinct categories, a) Arithmetic, b) Reasoning/Recall, c) Awareness. A number of subcategories exist within each category. Tests have been implemented for all subcategories in different variations and difficulty settings. The NAO robot is used in order to dictate the test questions to the user and record the answers. A user performance history is being kept in the ontology that aids in keeping track of the user’s cognitive test performance and in adjusting the difficulty of the tests he is presented with. Based on past performance the test difficulty adapts to the user’s specific needs in each test category separately in order to accurately reflect the user’s individual cognitive strengths and weaknesses. To preserve the user’s interest different tests are selected for each category using a least recently used model. To further enhance variation, even tests of the same subcategory exist in different variations.


# ROS Services

## Test Selector
This service was created in order to select the appropriate test for a user given its type. The service will load the user’s past performance from the ontology, determine the appropriate difficulty setting for the specific user and return the least recently used test of its type. In case no test type is provided then the least recently used test type category will be selected.

Service URL: ```/rapp/rapp_cognitive_exercise/cognitive_exercise_chooser```

Service type:
```bash
#Contains info about time and reference
Header header
#The username of the user
string username
#The test type requested
String testType
---
#The test’s name
string test
#The test’s type
string testType
#The test’s sub type
string testSubType
#The test’s language
string language
#The test’s questions
string[] questions
#The list of answers for each test
string[][] answers
#The correct answers for each test
string[] correct_answers
#Possible error
string error
``` 

## Record User Performance
This service was created in order to record the performance, meaning the score, of a user after completing a cognitive exercise test. The name of the test, its type and the user’s score are provided as input arguments. Within the service, a Unix timestamp is automatically generated. The timestamp reflects the time at which the test was performed. The service returns the name of the test performance entry that was created in the ontology.

Service URL: ```/rapp/rapp_cognitive_exercise/record_user_cognitive_test_performance```

Service type:
```bash
#Contains info about time and reference
Header header
#The username of the user
string username
#The type of the test
string testType
#The test which was performed
string test
#The score the user achieved on the test
String score
---
#Container for the subclasses
String user_cognitive_test_performance_entry
#Possible error
string error
``` 

## Cognitive Test creator
This service was created in order to create a cognitive test in the special xml format required and register it to the ontology. It accepts as an input a specially formatted text file containing all the information required for the test including its type, subtype, difficulty, questions, answers etc. The service formats the above information in an xml file and registers the test along with some vital information like it’s type and difficulty setting to the ontology. The service returns a bool variable that is true if xml creation and ontology registration were successful. Any possible error is contained in the error string variable also returned.

Service URL: ```/rapp/rapp_cognitive_exercise/cognitive_test_creator```

Service type:
```bash
#Contains info about time and reference
Header header
#The text file containing the test information
string inputFile
---
#True if test creation and registration to the ontology was successfull
bool success
#Possible error
string error
``` 

# Launchers

## Standard launcher

Launches the **rapp_cognitive_exercise** node and can be launched using
``` 
roslaunch rapp_cognitive_exercise cognitive_exercise.launch
``` 

# HOP services

## Test Selector RPS

The test_selector RPS is of type 3 since it contains a HOP service front-end, contacting a RAPP ROS ontology wrapper, which performs queries to the KnowRob ontology repository. The get subclass of RPS can be invoked using the following URL.

Service URL: ```localhost:9001/hop/test_selector```

### Input/Output
The test_selector RPS has two arguments, which are the username of the user and the requested test type. This is encoded in JSON format in an ASCII string representation.

The test_selector RPS the questions, the possible answers and the correct answers of the selected test. The encoding is in JSON format.

```
Input = {
  “username”: “THE_USERNAME”
  “testType”: “THE_TEST_TYPE”
}
```
```
Output = {
  “test”: “The test’s name”
  “testType”: “The test’s type”
  “testSubType”: “The test’s sub type”
  “language”: “The test’s language”
  “questions”: “The questions of the test”
  “answers”: “The possible answers for each question of the test”
  “correct_answers”: “The correct answer for each question of the test”
  “error”: “Possible error”
}

```
### Example
An example input for the test_selector RPS is
```
Input = {
  “instance_name”: “Person_1”
  “attribute_names”: “ArithmeticCts”
}
```

For this specific input, the result obtained was

```
Output = {
  “test”: “ArithmeticCts_XXXXXXX”
  “testType”: “ArithmetiCts”
  “testSubType”: “BasicArithmetic”
  “language”: “en”
  “questions”: “[How much is 2 plus 2?, How much is 5 plus 5?]”
  “answers”: “[2,3,4,5],[7,8,9,10]”
  “correct_answers”: “[4,10]"
}
```

## Record user cognitive test performance RPS

The record_user_cognitive_test_performance RPS is of type 3 since it contains a HOP service frontend, contacting a RAPP ROS ontology wrapper, which performs queries to the KnowRob ontology repository. The record user cognitive test performance of RPS can be invoked using the following URL.

Service URL: ```localhost:9001/hop/record_user_cognitive_test_performance```

### Input/Output
The record_user_cognitive_test_performance RPS has four arguments, which are the username of the user the test which was taken, the test’s type and the score achieved. This is encoded in JSON format in an ASCII string representation.

The record_user_cognitive_test_performance RPS outputs only a possible error message which is empty when the query was successful. The encoding is in JSON format.

```
Input = {
  “username”: “THE_USERNAME”
  “test”: “THE_TEST”
  “testType”: “THE_TEST_TYPE”
}
```
```
Output = {
  “error”: “Possible error”
}
```

### Example
An example input for the record_user_cognitive_test_performance RPS is
```
Input = {
  “instance_name”: “Person_1”
  “test”: “Test1”
  “testType”: “ArithmeticCts”
  “score”: “90”
}
```
For this specific input, the result obtained was

```
Output = {
  “error”: 
```

