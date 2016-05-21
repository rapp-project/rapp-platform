Documentation about the RAPP Caffe Wrapper: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Caffe-Wrapper)

#Methodology
The RAPP Caffe Wrapper node contains the services responsible for object identification via the Caffe deep learning framework and the assistive services for translating caffe classes to ontolgoy classes and registering the object images to the ontology.


# ROS Services

  /rapp/rapp_caffe_wrapper/register_image_to_ontology

## Image classification
This service will classify an image using the Caffe deep learning framework and if requested it will translate its caffe class to ontology class and register it to the ontology.

Service URL: ```/rapp/rapp_caffe_wrapper/get_ontology_class_equivalent```

Service type:
```bash
#the url of the image
string objectFileUrl
#the username of the user calling the service
string username
#true if the image will be registered to the ontology
bool registerToOntology
---
#the value of the ontology entry if the image was registered
string ontologyNameOfImage
#the caffe object class of the image
string objectClass
bool success
# possible error
string error
#tracen information
string[] trace

``` 

## Get ontology class equivalent
This service will return the ontology class equivalent of a caffe object class.

Service URL: ```/rapp/rapp_caffe_wrapper/get_ontology_class_equivalent```

Service type:
```bash
#the caffe object class of the iamge
string caffeClass
---
#true if a corresponding object exists in the ontology
bool existsInOntology
#the ontology equivalent class of the caffe class
string ontologyClass
bool success
# possible error
string error
#tracen information
string[] trace

``` 

## Register image to ontology
This service will register an image to the ontology along with the caffe and ontology class of the depicted object.

Service URL: ```/rapp/rapp_caffe_wrapper/get_ontology_class_equivalent```

Service type:
```bash
#the caffe object class of the iamge
string caffeClass
---
#true if a corresponding object exists in the ontology
bool existsInOntology
#the ontology equivalent class of the caffe class
string ontologyClass
bool success
# possible error
string error
#tracen information
string[] trace

``` 

# Launchers

## Standard launcher

Launches the **rapp_caffe_wrapper** node and can be launched using
``` 
roslaunch rapp_caffe_wrapper rapp_caffe_wrapper_functional_tests.launch
``` 

# HOP services

## Object-Recognition-Caffe

### Service-Url

```
/hop/object_recognition_caffe
```

### Service request arguments

```js
{ file: '' }
```

- **file**: Path to the uploaded file (**image file**), stored by hop-server. This is the form-data name to attach the file to.


### Service response

application/json response.

```js
{ object_class: '', error: '' }
```

- **object_class** (String): Recognized object class.
- **error** (String): Error message, if one occures. (String)

