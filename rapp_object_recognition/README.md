Recognition of known objects, implemented in RAPP Platform, is based on feature points extracted from the image. From this reason, only textured objects can be recognized and localized on the image at the moment, but in typical RAPP scenarios there are a lot of objects of this type - food boxes and cans, books, medicines and more everyday things. Object recognition module is able to recognize multiple objects at the same time, many instances od the same type of object in particular. Whole process of object learning and recognition is composed of four services.

Every user using object recognition module owns separate objects database. Recognition module can use all of the models or only selected subset during object detection, and this subset loaded at the moment is called operational memory.

## Learn new object

Recognition module is prepared to find specific object instances, learnt beforehand by the user. To learn new objects, picture consisting its view on plain (featureless) background should be used. Best recognition results are achieved, if models are learnt using the same sensor that will be used for detection (i.e. don't use high resolution, high quality object images if recognition will be done using lower quality camera). Every model should have different name. Subsequent calls to learn service with the same object name will overwrite previously learnt model. Model names are used in further calls of load service.

## Clear objects

Clears operational memory for selected user. After this operation, object recognition module will always fail to recognize anything. 

## Load models

Load one or more models to operational memory.  This operation should be done at least once before first recognition request. Subsequent calls of this service extends operational memory. To replace operational memory with new models, clear service should be called first.

## Find objects

When set of models is loaded to operational memory, user can provide query image to detect objects on. If any object of known type is recognized, its center point in query image, model name and recognition score (certainty) is returned. User can setup search results to contain only limited number of strongest object hypotheses. If more objects are found, weakest are dropped. 

# ROS Services

## Learn object
Service URL: ```/rapp/rapp_object_recognition/learn_object```

Service type:
```bash
# Path (id) of model image
string fname
# Name of the object
string name
# User name
string user
---
# Result: 0 - ok, -1 - no models, -2 - no image to analyse
int32 result
``` 

## Clear models
Service URL: ```/rapp/rapp_object_recognition/clear_models```

Service type:
```bash
# User name
string user
---
# Result: 0 - ok, -1 - no models, -2 - no image to analyse
int32 result
``` 

## Load models
Service URL: ```/rapp/rapp_object_recognition/load_models```

Service type:
```bash
# User name
string user
# Object names to load
string[] names
---
# Result: 0 - ok, -1 - no models, -2 - no image to analyse
int32 result
``` 

## Find objects
Service URL: ```/rapp/rapp_object_recognition/find_objects```

Service type:
```bash
# Path (id) of query image
string fname
# Limit search to N best matches
uint32 limit
# User name
string user
---
# List of found objects - names
string[] found_names
# List of found objects - centroids in image
geometry_msgs/Point[] found_centers
# List of found objects - scores
float64[] found_scores
# Result: 0 - ok, -1 - no models, -2 - no image to analyse
int32 result
``` 

#Launchers

##Standard launcher

Launches the **object_recognition** node and can be launched using
```
roslaunch rapp_object_recognition object_detection.launch
```

#Web services

TBD
