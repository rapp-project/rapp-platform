Documentation about the RAPP Cognitive Exercise: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Cognitive-Exercise)

#Methodology
The RAPP Cognitive exercise system aims to provide the Robot users a means of performing basic cognitive exercises. The cognitive tests supported belong to three distinct categories, a) Arithmetic, b) Reasoning/Recall, c) Awareness. A number of subcategories exist within each category. Tests have been implemented for all subcategories in different variations and difficulty settings. The NAO robot is used in order to dictate the test questions to the user and record the answers. A user performance history is being kept in the ontology that aids in keeping track of the user’s cognitive test performance and in adjusting the difficulty of the tests he is presented with. Based on past performance the test difficulty adapts to the user’s specific needs in each test category separately in order to accurately reflect the user’s individual cognitive strengths and weaknesses. To preserve the user’s interest different tests are selected for each category using a least recently used model. To further enhance variation, even tests of the same subcategory exist in different variations.


# ROS Services

## Test Selector
This service was created in order to select the appropriate test for a user given its type. The service will load the user’s past performance from the ontology, determine the appropriate difficulty setting for the specific user and return the least recently used test of its type. In case no test type is provided then the least recently used test type category will be selected. By providing the testType, testSubType, testDifficulty and testIndex parameters, a test can be selected manually.

Service URL: ```/rapp/rapp_cognitive_exercise/cognitive_exercise_chooser```

Service type:
```bash
#the username of the user for which a cognitive test is requested
string username
#used to specify the selected test type
string testType
#used to specify the selected test sub type
string testSubType
#used to specify the selected test difficulty
string testDifficulty
#used to specify the selected test index (id)
string testIndex
---
#the returned test name
string test
#the returned test type
string testType
#the returned test sub type
string testSubType
#the returned test language
string language
#the questions of the returned test
string[] questions
#the possible answers of the questions
rapp_platform_ros_communications/StringArrayMsg[] answers
#the correct answers
string[] correctAnswers
#true if service call was successful
bool success
# possible error
string error
#tracen information
string[] trace
``` 

## Record User Performance
This service was created in order to record the performance, meaning the score, of a user after completing a cognitive exercise test. The name of the test, its type and the user’s score are provided as input arguments. Within the service, a Unix timestamp is automatically generated. The timestamp reflects the time at which the test was performed. The service returns the name of the test performance entry that was created in the ontology.

Service URL: ```/rapp/rapp_cognitive_exercise/record_user_cognitive_test_performance```

Service type:
```bash
#the username of the user who completed the test
string username
#the name of the completed test
string test
#the score achieved
int32 score
---
#the name of the registered cognitive test user performance entry
string userCognitiveTestPerformanceEntry
#true if service call was successful
bool success
# possible error
string error
#tracen information
string[] trace
``` 

## Cognitive Test creator
This service was created in order to create a cognitive test in the special xml format required and register it to the ontology. It accepts as an input a specially formatted text file containing all the information required for the test including its type, subtype, difficulty, questions, answers etc. The service formats the above information in an xml file and registers the test along with some vital information like it’s type and difficulty setting to the ontology. The service returns a bool variable that is true if xml creation and ontology registration were successful. Any possible error is contained in the error string variable also returned.

Service URL: ```/rapp/rapp_cognitive_exercise/cognitive_test_creator```

Service type:
```bash
#the path to the cognitive test file
string inputFile
---
#true if service call was successful
bool success
# possible error
string error
#tracen information
string[] trace
``` 

## Return tests of type subtype and diffuclty
This service will return the cognitive tests existing in the ontology for the given testType, testSubType, difficulty and language. In case any arguments are not provided, it will be ignored, as such the query can be specific or general as needed.

Service URL: ```/rapp/rapp_cognitive_exercise/return_tests_of_type_subtype_difficulty_topic```

Service type:
```bash
#the type of the requested tests
string testType
#the subtype of the requested tests
string testSubType
#the difficulty of the requested tests
string difficulty
#the language of the requested tests
string language
---
#message containing the cognitive exercises
CognitiveExercisesMsg[] cognitiveExercises
#number of cognitive exercises returned
int16 totalNumberOfTestsReturned
#true if service call was successful
bool success
# possible error
string error
#tracen information
string[] trace
``` 

## Return user cognitive test history
This service will return the history of cognitive test performance records of the user for all test categories.

Service URL: ```/rapp/rapp_cognitive_exercise/user_all_categories_history```

Service type:
```bash
#the username of the user whose score history is requsted
string username
#the test types for which the history is requested
string testType
#timestamp after which score performance entries will be included in history
int64 fromTime
#timestamp up to which score performance entries will be included in history
int64 toTime
---
#message containing a;; user performance records for each test type (category)
ArrayCognitiveExercisePerformanceRecordsMsg[] recordsPerTestType
#the test categories
string[] testCategories
#true if service call was successful
bool success
# possible error
string error
#trace information
string[] trace
``` 

## Return user cognitive test scores
This service will return the cumulative cognitive test scores of the user for all test categories.

Service URL: ```/rapp/rapp_cognitive_exercise/user_all_categories_score```

Service type:
```bash
#the username of the user whose scores are requested
string username
#the type of the test for which the scores are requested
string testType
# timestamp up to which score performance entries will be requested
int64 upToTime
---
#the test categories for which scores are returned
string[] testCategories
# the score of each test category
float64[] testScores
#true if service call was successful
bool success
# possible error
string error
#trace information
string[] trace
``` 


# Launchers

## Standard launcher

Launches the **rapp_cognitive_exercise** node and can be launched using
``` 
roslaunch rapp_cognitive_exercise cognitive_exercise.launch
``` 

# HOP services

## Cognitive-Test-Selector


### Service URL
 ```/hop/cognitive_test_chooser```

### Service request arguments
The test_selector RPS  arguments are the username of the user, the type and subtype of the requested test and the difficulty and index of the test for manual test selection. This is encoded in JSON format in an ASCII string representation.

```js
{ test_type: '', test_subtype: '', test_diff: '', test_index: '' }
```

- **test_type** (String): Cognitive Exercise test type. Can be one of
  * 'ArithmeticCts'
  * 'AwarenessCts'
  * 'ReasoningCts'
  * ''
- **test_subtype** (String): Use this to force select from this subtype. Defaults to empty string "".
- **test_diff** (String): Use this to force select from this difficulty. Defaults to empty string "".
- **test_index** (String): Use this to force select from this id. Defaults to empty string "".

#### Example
An example input for the test_selector RPS is
```js
{ test_type: 'Arithmetic', test_subtype: 'BasicArithmetic', test_diff: '1', test_index: '1' }
```

### Service response
```javascript
 { questions: [], possib_ans: [], correct_ans: [], test_instance: '', test_type: '', test_subtype: '', error: '' }
```

- **questions** (Array): The exercise set of questions.
- **possib_ans**(Array):  The set of answers for each question. vector<vector<string>>
- **correct_ans**(Array): The set of correct answers for each question. vector<string>
- **test_instance** (String): Returned test name. For example, 'ArithmeticCts_askw0Snwk'
- **test_type** (String): Cognitive exercise class/type.
- **test_subtype** (String): Cognitive exercise sub-type.
- **error** (String): Error message, if one occures.


## Cognitive-Record-Performance


### Service URL
```
/hop/cognitive_record_performance
```

### Service request arguments

```js
{ test_instance: '', score: 0 }
```

- **test_instance** (String): Cognitive Exercise test instance. The full cognitive test entry name as returned by the **cognitive_test_chooser** web service.
- **score** (Integer): User's performance score on given test entry.


### Service response


```javascript
{ performance_entry: '', error: '' }
```

- **performace_entry** (String): User's cognitive test performance entry in ontology.
- **error** (String): Error message, if one occures.


## Cognitive-Get-History

### Service Url

```
/hop/cognitive_get_history
```

### Service request arguments

```js
{ from_time: '', to_time: 0, test_type: '' }
```

- **test_type** (String): Cognitive Exercise test type. Can be one of ['ArithmeticCts', 'AwarenessCts', 'ReasoningCts'] or leave empty ("") for all.
- **from_time** (Integer): Unix timestamp.
- **to_time** (Integer):  Unix timestamp.


### Service response

application/json response.

```javascript
{ records: {}, error: '' }
```

- **records** (Object): Users history records on Cognitive Exerises
- **error** (String): Error message, if one occures.


## Cognitive-Get-Scores

### Service Url

```
/hop/cognitive_get_scores
```

### Service request arguments

```js
{ up_to_time: 0, test_type: '' }
```

- **test_type** (String): Cognitive Exercise test type. Can be one of ['ArithmeticCts', 'AwarenessCts', 'ReasoningCts'] or leave empty ("") for all.
- **up_to_time** (Integer):  Unix timestamp. Return scores that have been recorder up to this time value.


### Service response

application/json response.

```javascript
{ test_classes: [], scores: [], error: '' }
```

- **test_classes** (Array): An array of the test classes indexes.
- **scores** (Array): Array of scores. Each array index corresponds to the test class of the **test_classes** property.
- **error** (String): Error message, if one occures.
