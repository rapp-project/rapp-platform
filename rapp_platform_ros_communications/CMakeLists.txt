cmake_minimum_required(VERSION 2.8.3)
project(rapp_platform_ros_communications)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs  
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files( 
  FILES 
  StringArrayMsg.msg 
) 

## Generate services in the 'srv' folder
add_service_files(
  FILES

  /CognitiveExercise/testSelectorSrv.srv
  /CognitiveExercise/recordUserCognitiveTestPerformanceSrv.srv
  /CognitiveExercise/cognitiveTestCreatorSrv.srv

  /DbWrapper/DbWrapperSrv.srv
  /DbWrapper/fetchDataSrv.srv
  /DbWrapper/updateDataSrv.srv
  /DbWrapper/writeDataSrv.srv
  /DbWrapper/deleteDataSrv.srv
  /DbWrapper/whatRappsCanRunSrv.srv
  
  /OntologyWrapper/createInstanceSrv.srv
  /OntologyWrapper/ontologySubSuperClassesOfSrv.srv
  /OntologyWrapper/ontologyIsSubSuperClassOfSrv.srv
  /OntologyWrapper/ontologyLoadDumpSrv.srv
  /OntologyWrapper/ontologyInstancesOfSrv.srv 
  /OntologyWrapper/assertRetractAttributeSrv.srv
  /OntologyWrapper/returnUserInstancesOfClassSrv.srv
  /OntologyWrapper/createOntologyAliasSrv.srv
  /OntologyWrapper/userPerformanceCognitveTestsSrv.srv
  /OntologyWrapper/createCognitiveExerciseTestSrv.srv
  /OntologyWrapper/cognitiveTestsOfTypeSrv.srv
  /OntologyWrapper/recordUserPerformanceCognitiveTestsSrv.srv
 
  /FaceDetection/FaceDetectionRosSrv.srv
  
  /QrDetection/QrDetectionRosSrv.srv
  
  /SpeechDetectionGoogleWrapper/SpeechToTextSrv.srv  
  
  /MailManagement/MailSend.srv
  
  /SpeechDetectionSphinx4Wrapper/SpeechRecognitionSphinx4Srv.srv
  /SpeechDetectionSphinx4Wrapper/SpeechRecognitionSphinx4ConfigureSrv.srv
  /SpeechDetectionSphinx4Wrapper/SpeechRecognitionSphinx4TotalSrv.srv

  /AudioProcessing/AudioProcessingDenoiseSrv.srv
  /AudioProcessing/AudioProcessingSetNoiseProfileSrv.srv
  /AudioProcessing/AudioProcessingDetectSilenceSrv.srv

  /TextToSpeechEspeak/TextToSpeechSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES rapp_platform_ros_communications
#  CATKIN_DEPENDS other_catkin_pkg
  CATKIN_DEPENDS 
    message_generation
    message_runtime
    std_msgs
    geometry_msgs
)

