import os.path
import rospy
import torch

from typing import Any


class SileroVAD:
    samplerate = 16000

    def __init__(
        self,
        threshold: float = 0.5,
        min_silence_duration_ms: int = 500,
        speech_pad_ms: int = 10,
    ):
        try:
            # try loading from cache
            import sys

            cache = os.path.abspath(
                os.path.expanduser("~/.cache/torch/hub/snakers4_silero-vad_master")
            )
            sys.path.append(cache)
            from utils_vad import (
                get_speech_timestamps,
                save_audio,
                read_audio,
                VADIterator,
                collect_chunks,
                OnnxWrapper,
            )

            model_dir = os.path.join(cache, "files")
            model = OnnxWrapper(os.path.join(model_dir, "silero_vad.onnx"), False)
            utils = (
                get_speech_timestamps,
                save_audio,
                read_audio,
                VADIterator,
                collect_chunks,
            )
            rospy.loginfo(logger_name="SileroVAD", msg=f"loaded model from cache")
        except Exception as e:
            rospy.loginfo(logger_name="SileroVAD", msg=f"loading from cache failed...")
            model, utils = torch.hub.load(
                repo_or_dir="snakers4/silero-vad",
                model="silero_vad",
                force_reload=False,
                onnx=True,
            )

        (_, _, _, self.VADIterator, _) = utils
        self.silero_model = model
        self.setup_vad()
        self.vad_iterator = VADIterator(
            model,
            threshold,
            self.samplerate,
            min_silence_duration_ms,
            speech_pad_ms,
        )

    def setup_vad(
        self,
        threshold: float = 0.5,
        min_silence_duration_ms: int = 300,
        speech_pad_ms: int = 500,
    ):
        self.vad_iterator = self.VADIterator(
            self.silero_model,
            threshold,
            self.samplerate,
            min_silence_duration_ms,
            speech_pad_ms,
        )

    def reset_states(self):
        self.vad_iterator.reset_states()

    def __call__(self, x):
        return self.vad_iterator(x)
