# ros_whisper

Use `openai-whisper` or `faster-whisper` for transcribing audio

The `_transcribe` nodes transcribe each incomming `audio_common_msgs/AudioData`. Use these with a node that records audio into a single frame e.g. the `silero_vad` node.

### Prerequisites 

package `openai-whisper` for whsiper
package `faster-whisper` for faster-whisper

To use this in a ros space make sure the packages are available e.g. by installing them into the default ROS python path

```bash
export PYTHONUSERBASE=/path/to/workspace/{devel or install}

# we link all possible python paths together so we can simply source setup.bash e.g.
cd $PYTHONUSERBASE/lib
mkdir -p python3/dist-packages
ln -s python3 python3.8
cd python3
ln -s dist-packages site-packages

# install openai-whisper requirement torch for your cuda version (cuda 11.4 -> cu114)
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu114 
pip3 install openai-whisper faster-whisper huggingface-hub==0.11.0
# packages are now useable with default setup script
source /path/to/workspace/devel/setup.bash
```
