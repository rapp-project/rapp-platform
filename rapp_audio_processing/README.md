Documentation about the RAPP Audio Processing: [Wiki Page](https://github.com/rapp-project/rapp-platform/wiki/RAPP-Audio-Processing)

#Methodology

The audio processing node was created in order to perform necessary operations for the speech recognition modules to operate for all audio cases. Even though both Google and Sphinx4 speech recognition modules are functional when the input is captured from a headset, the same does not apply with audio captured from the NAO robot.

NAO is able to record a single audio file at a time (wav or ogg), either from all microphones (4 channels at 48kHz) or from any single microphone (1 channel, 16kHz). The RAPP Speech detection modules can operate either with ogg or with wav (1 and 4 channels) by employing the Audio processing node. Nevertheless, the one-channel audio is the most appropriate selection, since Sphinx-4 requires single channel wav files, with a 16kHz sample rate and 16 bit little-endian format. The NAO captured audio contains considerable background static noise, being probably the result of a cooling fan that also exists in the NAO head. The problem raised is that the high noise levels cause Sphinx-4, as well as Google API to fail by producing no output.

It is obvious that in order for the speech recognition modules to operate successfully, denoising operations must take place. Additionally, since each NAO robot creates its own noise with different spectral characteristics, a personalization effort must be performed, storing silence samples from each robot and extracting the NAO noise’s DFT coefficients. These denoising operations are offered as ROS services by the Audio Processing package. This node utilizes the SoX Unix audio library in order to perform spectral denoising, along with other custom made techniques.

#ROS Services

##Set noise profile service
This service was created in order to store each robot’s noise profile. The service expects three inputs: a string containing the audio file, the audio type and the user that owns the robot. The supported audio types are nao_ogg, nao_wav_1_ch and nao_wav_4_ch. 

Once the service is invoked, the audio file (ogg, wav 1 or 4 channels) is converted to wav, single channel with a sampling rate of 16kHz, employing the SoX library. Finally, the noise profile is acquired using the SoX noiseprof tool and the respective file is stored in the RAPP Platform under the user’s folder.

Service URL: ```/rapp/rapp_audio_processing/set_noise_profile```

Service type:
```bash
# The stored audio file containing silence
string noise_audio_file
# The audio type [nao_ogg, nao_wav_1_ch, nao_wav_4_ch]
string audio_file_type
# The user
string user
---
# Possible error
string error
``` 

##Denoise service

This ROS service utilizes the user’s stored noise profile in order to perform spectral subtraction against the input audio signal. For this reason the SoX library is used, and specifically the noisered plugin.

Service URL: ```/rapp/rapp_audio_processing/denoise```

Service type:
```bash
# The stored audio file containing the user’s input
string audio_file
# The audio type [nao_ogg, nao_wav_1_ch, nao_wav_4_ch]
string audio_type
# The denoised audio file
string denoised_audio_file
# The user
string user
# The denoising scale
float32 scale
---
# Possible error
string error
``` 

##Energy denoise service

The energy denoise ROS service performs hard gating in the time domain, of the signal based on the RMS metric. The hard signal gating is applied in the individual sample’s power when compared with the RMS value.

Service URL: ```/rapp/rapp_audio_processing/energy_denoise```

Service type:
```bash
# The stored audio file containing the user’s input
string audio_file
# The audio type [nao_ogg, nao_wav_1_ch, nao_wav_4_ch]
string audio_type
# The denoised audio file
string denoised_audio_file
# The user
string user
# The denoising scale
float32 scale
---
# Possible error
string error
``` 
##Detect silence service

There are cases where the captured audio files from NAO do not contain any speech. Since the recording length is limited (e.g. 3 seconds) it is possible for some cases for the actual speech to miss this critical time slot. If this happens, the detect silence service is capable of indicating this issue in order for the robot to ask again the question it was not answered.

In order to detect if the signal contains silence, we follow a statistical approach. We suppose that if the file does not contain a voice, the samples’ power levels will be homogeneous to a certain extend. Thus, we calculate the RSD (Relative Standard Deviation) of the signal’s power and compare each sample with it. If one sample has a higher value, the signal is considered to contain voice.

Service URL: ```/rapp/rapp_audio_processing/detect_silence```

Service type:
```bash
# The stored audio file containing the user’s input
string audio_file
# The silence threshold
float32 threshold
---
# The result
bool silence
# Possible error
string error
``` 

#Launchers

##Standard launcher

Launches the **rapp_audio_processing** node and can be launched using
```
roslaunch rapp_audio_processing audio_processing.launch
```

#HOP services

##Set denoise profile RPS

The only RPS Audio Processing is the set_denoise_profile. The set_denoise_profile RPS is of type 3 since it contains a HOP service frontend, contacting a RAPP ROS service, which utilizes the SoX audio library.

Service URL: ```localhost:9001/hop/set_noise_profile ```

###Input/Output
The set_noise_profile RPS has three input arguments, which are the input file, the audio file type and the user. These are encoded in JSON format in an ASCII string representation.

The set_noise_profile RPS returns the success status. The encoding is in JSON format.

```
Input = {
  “noise_audio_file”: “THE_AUDIO_FILE”
  “audio_file_type”: “nao_ogg, nao_wav_1_ch, nao_wav_4_ch”
  “user”: “THE_USERNAME”
}
```
```
Output = {
  “error”: “Possible error”
}
```
