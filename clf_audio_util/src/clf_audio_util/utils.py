import numpy as np

from audio_common_msgs.msg import AudioInfo, AudioData

def msg_to_np(msg: AudioData, type = np.float32, channels = 1):
    if channels > 1:
        dtype = np.dtype([(f"ch{i}",type) for i in range(channels)])
    else:
        dtype = type
    buffer = np.frombuffer(msg.data, dtype=dtype)
    return buffer
