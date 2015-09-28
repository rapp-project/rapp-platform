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

