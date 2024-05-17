# Speech Recognition

Packages for speech recognition in ros1 (Tested on 22.04/obese)

- [clf_speech_msgs](clf_speech_msgs/) message definition
- [ros_whisper](ros_whisper/) using openai-whisper for speech recognition in ros 
- [ros_rasa](ros_rasa/) using RASA for natural language understanding in ros 
- [silero_vad](silero_vad/) using silero VAD for detecting voice activity on a [https://github.com/ros-drivers/audio_common/blob/master/audio_common_msgs/msg/AudioData.msg](AudioData) topic

## Install

Either use [RDTK](https://rdtk.github.io/documentation/index.html) to setup a [speech-recognition.distribution](https://gitlab.ub.uni-bielefeld.de/rdtk/citk/-/blob/clf-jammy/distributions/tiago-jammy-one-nightly-speech.distribution?ref_type=heads) or try to read the required packages form the linked distributuion and project files.

