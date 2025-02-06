import numpy as np

from audio_common_msgs.msg import AudioInfo, AudioData


def msg_to_np(msg: AudioData, type=np.float32, channels=1):
    if channels > 1:
        dtype = np.dtype([(f"ch{i}", type) for i in range(channels)])
    else:
        dtype = type
    buffer = np.frombuffer(msg.data, dtype=dtype)
    return buffer


whisper_language_codes = {
    "en": 0,  # english
    "zh": 1,  # chinese
    "de": 2,  # german
    "es": 3,  # spanish
    "ru": 4,  # russian
    "ko": 5,  # korean
    "fr": 6,  # french
    "ja": 6,  # japanese
    "pt": 7,  # portuguese
    "tr": 8,  # turkish
    "pl": 9,  # polish
    "ca": 10,  # catalan
    "nl": 11,  # dutch
    "ar": 12,  # arabic
    "sv": 13,  # swedish
    "it": 14,  # italian
    "id": 15,  # indonesian
    "hi": 16,  # hindi
    "fi": 17,  # finnish
    "vi": 18,  # vietnamese
    "he": 19,  # hebrew
    "uk": 20,  # ukrainian
    "el": 21,  # greek
    "ms": 22,  # malay
    "cs": 23,  # czech
    "ro": 24,  # romanian
    "da": 25,  # danish
    "hu": 26,  # hungarian
    "ta": 27,  # tamil
    "no": 28,  # norwegian
    "th": 29,  # thai
    "ur": 30,  # urdu
    "hr": 31,  # croatian
    "bg": 32,  # bulgarian
    "lt": 33,  # lithuanian
    "la": 34,  # latin
    "mi": 35,  # maori
    "ml": 36,  # malayalam
    "cy": 37,  # welsh
    "sk": 38,  # slovak
    "te": 39,  # telugu
    "fa": 40,  # persian
    "lv": 41,  # latvian
    "bn": 42,  # bengali
    "sr": 43,  # serbian
    "az": 44,  # azerbaijani
    "sl": 45,  # slovenian
    "kn": 46,  # kannada
    "et": 47,  # estonian
    "mk": 48,  # macedonian
    "br": 49,  # breton
    "eu": 50,  # basque
    "is": 51,  # icelandic
    "hy": 52,  # armenian
    "ne": 53,  # nepali
    "mn": 54,  # mongolian
    "bs": 55,  # bosnian
    "kk": 56,  # kazakh
    "sq": 57,  # albanian
    "sw": 58,  # swahili
    "gl": 59,  # galician
    "mr": 60,  # marathi
    "pa": 61,  # punjabi
    "si": 62,  # sinhala
    "km": 63,  # khmer
    "sn": 64,  # shona
    "yo": 65,  # yoruba
    "so": 66,  # somali
    "af": 67,  # afrikaans
    "oc": 68,  # occitan
    "ka": 69,  # georgian
    "be": 70,  # belarusian
    "tg": 71,  # tajik
    "sd": 72,  # sindhi
    "gu": 73,  # gujarati
    "am": 74,  # amharic
    "yi": 75,  # yiddish
    "lo": 76,  # lao
    "uz": 77,  # uzbek
    "fo": 78,  # faroese
    "ht": 79,  # haitian creole
    "ps": 80,  # pashto
    "tk": 81,  # turkmen
    "nn": 82,  # nynorsk
    "mt": 83,  # maltese
    "sa": 84,  # sanskrit
    "lb": 85,  # luxembourgish
    "my": 86,  # myanmar
    "bo": 87,  # tibetan
    "tl": 88,  # tagalog
    "mg": 89,  # malagasy
    "as": 90,  # assamese
    "tt": 91,  # tatar
    "haw": 92,  # hawaiian
    "ln": 93,  # lingala
    "ha": 94,  # hausa
    "ba": 95,  # bashkir
    "jw": 96,  # javanese
    "su": 97,  # sundanese
    "yue": 98,  # cantonese
}


def lang_from_int(index):
    return list(whisper_language_codes.keys())[
        list(whisper_language_codes.values()).index(index)
    ]
