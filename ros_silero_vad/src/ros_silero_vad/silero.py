import os.path
import rospy
import torch

from typing import Any

from silero_vad.utils_vad import VADIterator, init_jit_model, OnnxWrapper
import torch
torch.set_num_threads(1)

class SileroVAD:
    samplerate = 16000

    def __init__(
        self,
        threshold: float = 0.5,
        min_silence_duration_ms: int = 500,
        speech_pad_ms: int = 10,
    ):
       
        self.VADIterator = VADIterator
        self.silero_model = load_silero_vad(True)
        self.setup_vad(threshold, min_silence_duration_ms, speech_pad_ms)

        rospy.loginfo(logger_name="SileroVAD", msg=f"loaded silero model")

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
    

def load_silero_vad(onnx=False):
    model_name = 'silero_vad.onnx' if onnx else 'silero_vad.jit'
    package_path = "silero_vad.data"
    
    try:
        import importlib_resources as impresources
        model_file_path = str(impresources.files(package_path).joinpath(model_name))
    except:
        from importlib import resources as impresources
        try:
            with impresources.path(package_path, model_name) as f:
                model_file_path = f
        except:
            model_file_path = str(impresources.files(package_path).joinpath(model_name))

    if onnx:
        model = OnnxWrapper(str(model_file_path), force_onnx_cpu=True)
    else:
        model = init_jit_model(str(model_file_path))
    
    return model


if __name__ == "__main__":
    vad = SileroVAD()