from piper import PiperVoice
import wave
from pathlib import Path

import rospy


class synthesize_args:
    def __init__(self):
        self.speaker_id = None
        self.length_scale = None
        self.noise_scale = None
        self.noise_w = None
        self.sentence_silence = 0.1


def get_model(model_dir, model_name):
    model_dir = Path(model_dir)
    onnx_path = model_dir / f"{model_name}.onnx"
    config_path = model_dir / f"{model_name}.onnx.json"

    if onnx_path.exists() and config_path.exists():
        return (onnx_path, config_path)
    else:
        if not onnx_path.exists():
            msg = f"Model not found. '{onnx_path}' does not exist"
        else:
            msg = f"Config not found. '{config_path}' does not exist"

        raise rospy.ROSException(msg)


class PiperTTS:

    def __init__(
        self, model_path, config_path, settings=synthesize_args(), use_cuda=False
    ):
        self.voice = PiperVoice.load(model_path, config_path, use_cuda=use_cuda)
        self._args = settings
        self.samplerate = self.voice.config.sample_rate
        # print(f"self.voice.config.num_speakers {self.voice.config.num_speakers}")

    def synthesize(self, text):
        return self.voice.synthesize_stream_raw(text, **vars(self._args))


if __name__ == "__main__":
    m, c = get_model(
        "/home/robocup-adm/tmp/tts/ros-piper_tts/piper_tts/models", "de_DE-karlsson-low"
    )
    s = synthesize_args()
    voice = PiperTTS(m, c, s)

    line = "Der Regenbogen ist ein atmosphärisch-optisches Phänomen, das als kreisbogenförmiges farbiges Lichtband in einer von der Sonne beschienenen Regenwand oder -wolke wahrgenommen wird."

    # with wave.open("tmp.wav", "wb") as wav_file:
    #    voice.synthesize(line, wav_file, **synthesize_args)

    audio_stream = voice.synthesize(line)

    import numpy as np
    import sounddevice as sd

    for data in audio_stream:

        npa = np.frombuffer(data, dtype=np.int16)
        # RawOutputStream does not need np
        sd.play(npa, voice.samplerate, blocking=False)

        while sd._last_callback.event.is_set() == False:
            rospy.sleep(0.1)


def make_glados():
    m, c = get_model("/home/robocup-adm/tmp/tts/karlsson", "de_DE-karlsson-low")
    voice = PiperTTS(m, c)

    line = "hallo ich bin glados. Ich finde dich doof"

    import librosa
    import psola

    frame_length_input = 2048
    fmin_input = librosa.note_to_hz("C2")
    fmax_input = librosa.note_to_hz("C7")
    pitch = 8

    def correct(f0):
        if np.isnan(f0):
            return np.nan

        # Define the degrees of the musical notes in a scale
        note_degrees = librosa.key_to_degrees("C#:min")
        note_degrees = np.concatenate((note_degrees, [note_degrees[0] + 12]))

        # Convert the fundamental frequency to MIDI note value and calculate the closest degree
        midi_note = librosa.hz_to_midi(f0)
        degree = midi_note % 12
        closest_degree_id = np.argmin(np.abs(note_degrees - degree))

        # Correct the MIDI note value based on the closest degree and convert it back to Hz
        midi_note = midi_note - (degree - note_degrees[closest_degree_id])

        return librosa.midi_to_hz(midi_note - pitch)

    def correctpitch(f0):
        corrected_f0 = np.zeros_like(f0)
        for i in range(f0.shape[0]):
            corrected_f0[i] = correct(f0[i])
        return corrected_f0

    def autotune(y, sr):
        # Estimate the fundamental frequency using the PYIN algorithm
        f0, _, _ = librosa.pyin(
            y,
            frame_length=frame_length_input,
            hop_length=(frame_length_input // 4),
            sr=sr,
            fmin=fmin_input,
            fmax=fmax_input,
        )
        # Correct the pitch of the estimated fundamental frequencies
        corrected_pitch = correctpitch(f0)
        # Perform PSOLA-based pitch shifting to match the corrected pitch
        return psola.vocode(
            y,
            sample_rate=int(sr),
            target_pitch=corrected_pitch,
            fmin=fmin_input,
            fmax=fmax_input,
        )

    with wave.open("tmp.wav", "wb") as wav_file:
        voice.voice.synthesize(line, wav_file, **vars(voice._args))
    print("voice synthesized")
    y, sr = librosa.load("tmp.wav", sr=None, mono=False)
    print("loaded in librosa...")

    if y.ndim > 1:
        y = y[0, :]

    # Perform pitch correction by shifting the pitch by 2 semitones using librosa.effects.pitch_shift
    pitch_corrected_y = librosa.effects.pitch_shift(y, sr=sr, n_steps=2)

    pitch_corrected_y = autotune(y, sr)

    filepath = Path("hello.wav")
    output_filepath = filepath.stem + "_pitch_corrected" + filepath.suffix

    import soundfile as sf

    # Save the pitch-corrected audio file using soundfile
    sf.write(str(output_filepath), pitch_corrected_y, sr)
