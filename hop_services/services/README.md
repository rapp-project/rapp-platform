## Synopsis

Containes RAPP Platform web services developed using hOP web broker.
These services are used in order to communicate with the RAPP Platform ecosystem
and access RIC(RAPP Improvement Center) AI modules.


## Services

#### [qr_detection( )](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/qr_detection.service.js)

```
qr_detection ( {file_uri: ''} )
```

  **Input parameters**

  - file_uri: Destination where the posted file data (**image file**) are saved by hop-server.
    Data are posted using a multipart/form-data post request using this field.
    e.g.

    ```
    file = {'file_uri': open(<to-send-file-path>, 'rb')}
    ```

  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode the received datia.

  > { qr_centers: [], error: '' }

  - qr_centers: Dynamic vector that containes points (x,y,z) of found QR in an image frame.
  - error: If error was encountered, an error message is pushed in this field
    and returned to the client.


  > {
  >   qr_centers: [ { y: 165, x: 165, z: 0 } ],
  >   error: '<error_mesasge>'
  > }




#### [face_detection( )](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/face_detection.service.js)

  ```
  face_detection ( {file_uri: ''} )
  ```

  **Input parameters**

  - file_uri: Destination where the posted file data (**image file**) are saved by hop-server.
    Data are posted using a multipart/form-data post request using this field.
    e.g.

    ```
    file = {'file_uri': open(<to-send-file-path>, 'rb')}
    ```

  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
  the received datia.

  > { faces: [{ up_left_point: {}, down_right_point: {} }], error: '' }

  Point coordinates are presented in Cartesian Coordinate System as:

  > {x: <value_int>, y: <value_int>, z: <value_int>}

  - faces: Dynamic vector that contains recognized faces in an image frame.
  - up_left_point: This Object literal contains the up-left point coordinates of detected face.
  - up_left_point: This Object literal contains the down-right point coordinates of detected face.
  - error: If error was encountered, an error message is pushed in this field
    and returned to the client.


  > {
  >   faces: [{
  >        up_left_point: { y: 200, x: 212, z: 0 },
  >        down_right_point: { y: 379, x: 391, z: 0 }
  >    }],
  >   error: '<error_message>'
  > }


#### [set_denoise_profile( )](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/set_denoise_profile.service.js)

  ```
  set_denoise_profile ( {file_uri: '', audio_source: '', user: ''} )
  ```

  **Input parameters**

  - **'file_uri'**: Destination where the posted file data (**audio data file**) are saved by hop-server.
    Data are posted using a multipart/form-data post request using this field.
    e.g.

    ```
    file = {'file_uri': open(<to-send-file-path>, 'rb')}
    ```

  - **'audio_source'**: A value that presents the <robot>_<encode>_<channels> information for the audio source data.
    e.g "nao_wav_1_ch".
  - **'user'**: User’s name. Used for per-user profile denoise configurations.
    e.g "klpanagi"


  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
  the received datia.

  > { error: '<error_message>' }

  - error: If error was encountered, an error message is pushed in this field
    and returned to the client.


  > {
  >  error:"RAPP Platform Failure!"
  > }


#### [speech_detection_sphinx4( )](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/speech_detection_sphinx4.service.js) 

  ```
  speech_detection_sphinx4 ( { file_uri: '', language: '', audio_source: '', words: [], sentences: [], grammar: [], user: ''})
  ``` 

  **Input parameters**

  - **'file_uri'**: Destination where the posted file data (**audio data file**) are saved by hop-server.
    Data are posted using a multipart/form-data post request using this field.
    e.g.

    ```
    file = {'file_uri': open(<to-send-file-path>, 'rb')}
    ```

  - **'language'**: Language to be used by the speech_detection_sphinx4 module.
    Currently valid language values are ‘gr’ for Greek and ‘en’ for English.
  - **'audio_source'**: A value that presents the <robot>_<encode>_<channels> information for the audio source data.
    e.g "nao_wav_1_ch".
  - **'words[]'**: A vector that carries the words to search for into the voice-audio-source.
  - **'sentences[]'**: The under consideration sentences.
  - **'grammar[]'**: Grammars to use in speech recognition.
  - **'user'**: User’s name. Used for per-user profile denoise configurations.
    e.g "klpanagi"


  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
  the received datia.

  > { words: [], error: '<error_message>' }

  - **'error'**: If error was encountered, an error message is pushed in this field
    and returned to the client.
  - **'words[]'**: A vector that contains the "words-found"



#### [ontology_subclasses_of( )](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/ontology_subclasses_of.service.js)

  ```
  ontology_subclasses_of ( { query: ''} )
  ```

  **Input parameters**

  - **'query'**: The query to the ontology database.


  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
  the received datia.

  > { results: [], trace: [], error: '<error_message>' }

  - **'results'**: Query results returned from ontology database.
  - **'trace'**:
  - **'error'**: If error was encountered, an error message is pushed in this field
    and returned to the client.


  > { [ 'http://knowrob.org/kb/knowrob.owl#Oven',
  >     'http://knowrob.org/kb/knowrob.owl#MicrowaveOven',
  >     'http://knowrob.org/kb/knowrob.owl#RegularOven',
  >     'http://knowrob.org/kb/knowrob.owl#ToasterOven'],
  >   [],
  >   ''
  > }


#### [ontology_superclasses_of( )](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/ontology_superclasses_of.service.js)

  ```
  ontology_superclasses_of ( { query: ''} )
  ```

  **Input parameters**

  - **'query'**: The query to the ontology database.


  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
  the received datia.

  > { results: [], trace: [], error: '<error_message>' }

  - **'results'**: Query results returned from ontology database.
  - **'trace'**:
  - **'error'**: If error was encountered, an error message is pushed in this field
    and returned to the client.


#### [ontology_is_subsuperclass_of( ) ](https://github.com/rapp-project/rapp-platform/blob/hop_services/hop_services/services/ontology_is_subsuperclass_of.service.js)

  ```
    ontology_is_subsuperclass_of ( { parent_class: '', child_class: '', recursive: false } )
  ```

  **Input parameters**

  - **'parent_class'**: The parent class name.
  - **'child_class'**: The child class name.
  - **'recursive'**: Defines if a recursive procedure will be used (true/false).


  **Response/Return-Data**

  The returned data are in *JSON* representation. A JSON.load() from client side must follow in order to decode
  the received datia.

  > { results: [], trace: [], error: '<error_message>' }

  - **'results'**: Query results returned from ontology database.
  - **'trace'**:
  - **'error'**: If error was encountered, an error message is pushed in this field
    and returned to the client.



## Tests

Developed tests and testing tools are currently located under:

```
../utilities/testing_tools/
```

## Contributors

- Konstaninos Panayiotou, **[klpanagi@gmail.com]**
- Manos Tsardoulias, **[etsardou@gmail.com]**
- Vincent Prunet, **[vincent.prunet@inria.fr]**
