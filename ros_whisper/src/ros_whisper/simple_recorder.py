import rospy
import numpy as np
from audio_common_msgs.msg import AudioData, AudioInfo


class SimpleRecorder:

    def __init__(self, timeout = 5):
        self.samplerate = 16000

        rospy.loginfo(logger_name="SimpleRecorder", msg=f"waiting for audio_info...")
        audio_info: AudioInfo = rospy.wait_for_message(
            "audio_info", AudioInfo, timeout=timeout
        )  # type: ignore

        if not audio_info.sample_rate == 16000:
            msg = f"Sample rate needs to be 16000"
            rospy.logerr(logger_name="SimpleRecorder", msg=msg)
            raise Exception(msg)

        if not audio_info.channels == 1:
            msg = f"Cant use non mono audio"
            rospy.logerr(logger_name="SimpleRecorder", msg=msg)
            raise Exception(msg)

        if audio_info.sample_format.lower() == "f32le":
            self.depth_type = np.float32
        else:
            msg = f"unhandled audio format"
            rospy.logerr(logger_name="SimpleRecorder", msg=msg)
            raise Exception(msg)
