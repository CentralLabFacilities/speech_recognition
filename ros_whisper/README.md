# ros_whisper

### Prerequisites 

package `openai-whisper`

To use this in a ros space make sure the packages are available e.g. by installing them into default ROS python path

```bash
export PYTHONUSERBASE=/path/to/workspace/devel
cd $PYTHONUSERBASE/lib
# link all possible python paths together e.g.
mkdir -p python3/dist-packages
ln -s python3 python3.8
cd python3
ln -s dist-packages site-packages
# install openai-whisper requirement torch for your cuda version (cuda 11.4 -> cu114)
pip3 install torch torchvision torchaudio --extra-index-url https://download.pytorch.org/whl/cu114 
pip3 install openai-whisper huggingface-hub==0.11.0
# packages are now useable with default setup script
source /path/to/workspace/devel/setup.bash
```
