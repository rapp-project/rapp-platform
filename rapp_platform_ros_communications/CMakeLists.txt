cmake_minimum_required(VERSION 2.8.3)
project(rapp_platform_ros_communications)
set(ROS_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  std_msgs
  geometry_msgs
  nav_msgs
)

################################################
## Declare ROS messages, services and actions ##
################################################

## Generate messages in the 'msg' folder
add_message_files(
  FILES
  StringArrayMsg.msg
  CognitiveExercisePerformanceRecordsMsg.msg
  MailMsg.msg
  NewsStoryMsg.msg
  WeatherForecastMsg.msg
  ArrayCognitiveExercisePerformanceRecordsMsg.msg
)

## Generate services in the 'srv' folder
add_service_files(
  FILES

  /CognitiveExercise/testSelectorSrv.srv
  /CognitiveExercise/recordUserCognitiveTestPerformanceSrv.srv
  /CognitiveExercise/cognitiveTestCreatorSrv.srv
  /CognitiveExercise/userScoresForAllCategoriesSrv.srv
  /CognitiveExercise/userScoreHistoryForAllCategoriesSrv.srv

  /HumanDetection/HumanDetectionRosSrv.srv

  /CaffeWrapper/imageClassificationSrv.srv
  /CaffeWrapper/ontologyClassBridgeSrv.srv
  /CaffeWrapper/registerImageToOntologySrv.srv

  /DbWrapper/checkIfUserExistsSrv.srv
  /DbWrapper/getUserOntologyAliasSrv.srv
  /DbWrapper/getUserLanguageSrv.srv
  /DbWrapper/registerUserOntologyAliasSrv.srv
  /DbWrapper/getUserPasswordSrv.srv
  /DbWrapper/getUsernameAssociatedWithApplicationTokenSrv.srv
  /DbWrapper/createNewPlatformUserSrv.srv
  /DbWrapper/createNewApplicationTokenSrv.srv
  /DbWrapper/checkActiveApplicationTokenSrv.srv
  /DbWrapper/checkActiveRobotSessionSrv.srv
  /DbWrapper/addStoreTokenToDeviceSrv.srv
  /DbWrapper/validateUserRoleSrv.srv
  /DbWrapper/validateExistingPlatformDeviceTokenSrv.srv
  /DbWrapper/removePlatformUserSrv.srv

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
  /OntologyWrapper/clearUserPerformanceCognitveTestsSrv.srv
  /OntologyWrapper/registerImageObjectToOntologySrv.srv
  /OntologyWrapper/retractUserOntologyAliasSrv.srv

  /CloudAgentManager/createContainerSrv.srv

  /FaceDetection/FaceDetectionRosSrv.srv

  /NewsExplorer/NewsExplorerSrv.srv
  /Geolocator/GeolocatorSrv.srv

  /WeatherReporter/WeatherReporterCurrentSrv.srv
  /WeatherReporter/WeatherReporterForecastSrv.srv

  /QrDetection/QrDetectionRosSrv.srv

  /Email/SendEmailSrv.srv
  /Email/ReceiveEmailSrv.srv

  /SpeechDetectionGoogleWrapper/SpeechToTextSrv.srv

  /SpeechDetectionSphinx4Wrapper/SpeechRecognitionSphinx4Srv.srv
  /SpeechDetectionSphinx4Wrapper/SpeechRecognitionSphinx4ConfigureSrv.srv
  /SpeechDetectionSphinx4Wrapper/SpeechRecognitionSphinx4TotalSrv.srv

  /AudioProcessing/AudioProcessingDenoiseSrv.srv
  /AudioProcessing/AudioProcessingSetNoiseProfileSrv.srv
  /AudioProcessing/AudioProcessingDetectSilenceSrv.srv
  /AudioProcessing/AudioProcessingTransformAudioSrv.srv

  /TextToSpeechEspeak/TextToSpeechSrv.srv

  /HazardDetection/LightCheckRosSrv.srv
  /HazardDetection/DoorCheckRosSrv.srv

  /PathPlanning/PathPlanningRosSrv.srv
  /Costmap2d/Costmap2dRosSrv.srv
  /PathPlanning/MapServer/MapServerGetMapRosSrv.srv
  /PathPlanning/MapServer/MapServerUploadMapRosSrv.srv

  /ApplicationAuthentication/UserTokenAuthenticationSrv.srv
  /ApplicationAuthentication/UserLoginSrv.srv
  /ApplicationAuthentication/AddNewUserFromStoreSrv.srv
  /ApplicationAuthentication/AddNewUserFromPlatformSrv.srv
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
  geometry_msgs
  nav_msgs
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

