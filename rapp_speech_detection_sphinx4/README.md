## RAPP Sphinx4-based detection

Sphinx4 speech detection is a ROS node which performs (mainly) limited vocabulary speech recognition capabilities in English and Greek languages. The node utilized the latest Sphinx4 java library, sphinxbase and cmuclmtk tools.

```rapp_speech_detection_sphinx4``` utilizes the ```rapp_audio_processing``` and ```rapp_mysql_wrapper``` modules. ```rapp_audio_processing``` is utilized to perform denoising, as the NAO captured audio is quite noisy, whereas the ```rapp_mysql_wrapper``` is used to perform authentication.

```
                                                                
 +-----------------------------+                                
 |rapp_speech_detection_sphinx4|                                
 +-----------------------------+                                
 |speech_recognition           +------------------+             
 +-----------------------------+                  |             
                |                       +---------v----------+  
                |                       | rapp_mysql_wrapper |  
                |                       +---------^----------+  
                |                                 |             
    +-----------v-----------+                     |             
    | rapp_audio_processing +---------------------+             
    +-----------------------+                                   
                                                                
```

```rapp_speech_detection_sphinx4``` exposes one ROS service, utilized for speech recognition. The input arguments are:

- **language**: The language to perform ASR (Automatic Speech Recognition). Supported values: ```en``` and ```gr```
- **words[]**: The limited vocabulary from which Sphinx4 will do the matching. Must provide individual words in the language declared in the ```language``` parameter. If left empty a generalized vocabulary will be assumed. This will be valid for English but the results are not good.
- **grammar[]**: A form of language model. Contains either words or sentences that contain the words declared in the ```words``` parameter. If grammar is declared, Sphinx4 will either return results that exist **as is** in grammar or ```<nul>``` if no matching exists.
- **sentences[]**: The second form of language model. Same functionality as ```grammar``` but Sphinx can return individual words contained in the sentences provided. This is essentially used to extract probabilities regarding the phonemes succession.
- **path**: The audio file path.
- **audio_source**: Declares the source of the audio capture in order to perform correct denoising. The different types are:
  - ```headset```: Clean sound, no denoising needed
  - ```nao_ogg```: Captured ogg file from a single microphone from NAO. Supposed to have 1 channel.
  - ```nao_wav_1_ch```: Captured wav file from one microphone of NAO. Supposed to have 1 channel, 16kHz.
  - ```nao_wav_4_ch```: Captured wav file from all 4 NAO's microphones. Supposed to have 4 channels at 48kHz.
- **user**: The user invoking the service. Must exist as ```username``` in the database to work. Also a noise profile for the declared ```user``` must exist (check ```rapp_audio_processing``` node for ```set_noise_profile``` service)

The returned parameters are:
- **error**: Possible errors
- **words[]**: The recognized words
