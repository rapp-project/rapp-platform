#RAPP Cognitive Exercise Node
Contains the RAPP Cognitive Exercise Node

##Services

cognitive_exercise_chooser:
- inputs
 - string username: the RAPP MySQL username of the user
 - string testType: (contains one of the available cognitive test types or can be empty
- outputs
 - bool success: true if successful
 - string error: error information or empty if successful
 - string[] trace: info about service call and possible errors
 - string[] questions: the questions of the test
 - string[][] answers: the answers of the questions of the test organized by line
 - string[] correctAnswers: the correct answer for each question

Special case1: In case the test is of type ReasoningCts and subtype StorytellingCs
- the first question asks the user if he is ready for initiating the storytelling.
- the second question is the story itself and by the end of it the user is asked if he is ready to be asked the comprehension questions
- starting from the third question are the comprehension questions

Special case2: In case the test is of type ReasoningCts and subtype WordRememberingCts
- the first question asks the user if he is ready to listen to the words
- the second question are the words themselves and the user is asked if he heard them and if he is ready to repeat them
- the third question asks the user to repeat the words

record_user_cognitive_test_performance:
- inputs
 - string username: the RAPP MySQL username of the user
 - string test: the ontology alias name of the test
 - string score: the score achieved
- outputs
 - bool success: true if successful
 - string error: error information or empty if successful
 - string[] trace: info about service call and possible errors
 - string userCognitiveTestPerformanceEntry: the ontology name of the performance entry
