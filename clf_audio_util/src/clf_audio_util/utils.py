import numpy as np

from audio_common_msgs.msg import AudioInfo, AudioData
from clf_speech_msgs.msg import ASR


def msg_to_np(msg: AudioData, type=np.float32, channels=1):
    if channels > 1:
        dtype = np.dtype([(f"ch{i}", type) for i in range(channels)])
    else:
        dtype = type
    buffer = np.frombuffer(msg.data, dtype=dtype)
    return buffer


whisper_language_codes = {
    "en": ASR.EN,  # english
    "zh": ASR.ZH,  # chinese
    "de": ASR.DE,  # german
    "es": ASR.ES,  # spanish
    "ru": ASR.RU,  # russian
    "ko": ASR.KO,  # korean
    "fr": ASR.FR,  # french
    "ja": ASR.JA,  # japanese
    "pt": ASR.PT,  # portuguese
    "tr": ASR.TR,  # turkish
    "pl": ASR.PL,  # polish
    "ca": ASR.CA,  # catalan
    "nl": ASR.NL,  # dutch
    "ar": ASR.AR,  # arabic
    "sv": ASR.SV,  # swedish
    "it": ASR.IT,  # italian
    "id": ASR.ID,  # indonesian
    "hi": ASR.HI,  # hindi
    "fi": ASR.FI,  # finnish
    "vi": ASR.VI,  # vietnamese
    "he": ASR.HE,  # hebrew
    "uk": ASR.UK,  # ukrainian
    "el": ASR.EL,  # greek
    "ms": ASR.MS,  # malay
    "cs": ASR.CS,  # czech
    "ro": ASR.RO,  # romanian
    "da": ASR.DA,  # danish
    "hu": ASR.HU,  # hungarian
    "ta": ASR.TA,  # tamil
    "no": ASR.NO,  # norwegian
    "th": ASR.TH,  # thai
    "ur": ASR.UR,  # urdu
    "hr": ASR.HR,  # croatian
    "bg": ASR.BG,  # bulgarian
    "lt": ASR.LT,  # lithuanian
    "la": ASR.LA,  # latin
    "mi": ASR.MI,  # maori
    "ml": ASR.ML,  # malayalam
    "cy": ASR.CY,  # welsh
    "sk": ASR.SK,  # slovak
    "te": ASR.TE,  # telugu
    "fa": ASR.FA,  # persian
    "lv": ASR.LV,  # latvian
    "bn": ASR.BN,  # bengali
    "sr": ASR.SR,  # serbian
    "az": ASR.AZ,  # azerbaijani
    "sl": ASR.SL,  # slovenian
    "kn": ASR.KN,  # kannada
    "et": ASR.ET,  # estonian
    "mk": ASR.MK,  # macedonian
    "br": ASR.BR,  # breton
    "eu": ASR.EU,  # basque
    "is": ASR.IS,  # icelandic
    "hy": ASR.HY,  # armenian
    "ne": ASR.NE,  # nepali
    "mn": ASR.MN,  # mongolian
    "bs": ASR.BS,  # bosnian
    "kk": ASR.KK,  # kazakh
    "sq": ASR.SQ,  # albanian
    "sw": ASR.SW,  # swahili
    "gl": ASR.GL,  # galician
    "mr": ASR.MR,  # marathi
    "pa": ASR.PA,  # punjabi
    "si": ASR.SI,  # sinhala
    "km": ASR.KM,  # khmer
    "sn": ASR.SN,  # shona
    "yo": ASR.YO,  # yoruba
    "so": ASR.SO,  # somali
    "af": ASR.AF,  # afrikaans
    "oc": ASR.OC,  # occitan
    "ka": ASR.KA,  # georgian
    "be": ASR.BE,  # belarusian
    "tg": ASR.TG,  # tajik
    "sd": ASR.SD,  # sindhi
    "gu": ASR.GU,  # gujarati
    "am": ASR.AM,  # amharic
    "yi": ASR.YI,  # yiddish
    "lo": ASR.LO,  # lao
    "uz": ASR.UZ,  # uzbek
    "fo": ASR.FO,  # faroese
    "ht": ASR.HT,  # haitian creole
    "ps": ASR.PS,  # pashto
    "tk": ASR.TK,  # turkmen
    "nn": ASR.NN,  # nynorsk
    "mt": ASR.MT,  # maltese
    "sa": ASR.SA,  # sanskrit
    "lb": ASR.LB,  # luxembourgish
    "my": ASR.MY,  # myanmar
    "bo": ASR.BO,  # tibetan
    "tl": ASR.TL,  # tagalog
    "mg": ASR.MG,  # malagasy
    "as": ASR.AS,  # assamese
    "tt": ASR.TT,  # tatar
    "haw": ASR.HAW,  # hawaiian
    "ln": ASR.LN,  # lingala
    "ha": ASR.HA,  # hausa
    "ba": ASR.BA,  # bashkir
    "jw": ASR.JW,  # javanese
    "su": ASR.SU,  # sundanese
    "yue": ASR.YUE,  # cantonese
}

# Language | FLORES-200 code
# ---|---
# Acehnese (Arabic script) | ace_Arab
# Acehnese (Latin script) | ace_Latn
# Mesopotamian Arabic | acm_Arab
# Ta’izzi-Adeni Arabic | acq_Arab
# Tunisian Arabic | aeb_Arab
# Afrikaans | afr_Latn
# South Levantine Arabic | ajp_Arab
# Akan | aka_Latn
# Amharic | amh_Ethi
# North Levantine Arabic | apc_Arab
# Modern Standard Arabic | arb_Arab
# Modern Standard Arabic (Romanized) | arb_Latn
# Najdi Arabic | ars_Arab
# Moroccan Arabic | ary_Arab
# Egyptian Arabic | arz_Arab
# Assamese | asm_Beng
# Asturian | ast_Latn
# Awadhi | awa_Deva
# Central Aymara | ayr_Latn
# South Azerbaijani | azb_Arab
# North Azerbaijani | azj_Latn
# Bashkir | bak_Cyrl
# Bambara | bam_Latn
# Balinese | ban_Latn
# Belarusian | bel_Cyrl
# Bemba | bem_Latn
# Bengali | ben_Beng
# Bhojpuri | bho_Deva
# Banjar (Arabic script) | bjn_Arab
# Banjar (Latin script) | bjn_Latn
# Standard Tibetan | bod_Tibt
# Bosnian | bos_Latn
# Buginese | bug_Latn
# Bulgarian | bul_Cyrl
# Catalan | cat_Latn
# Cebuano | ceb_Latn
# Czech | ces_Latn
# Chokwe | cjk_Latn
# Central Kurdish | ckb_Arab
# Crimean Tatar | crh_Latn
# Welsh | cym_Latn
# Danish | dan_Latn
# German | deu_Latn
# Southwestern Dinka | dik_Latn
# Dyula | dyu_Latn
# Dzongkha | dzo_Tibt
# Greek | ell_Grek
# English | eng_Latn
# Esperanto | epo_Latn
# Estonian | est_Latn
# Basque | eus_Latn
# Ewe | ewe_Latn
# Faroese | fao_Latn
# Fijian | fij_Latn
# Finnish | fin_Latn
# Fon | fon_Latn
# French | fra_Latn
# Friulian | fur_Latn
# Nigerian Fulfulde | fuv_Latn
# Scottish Gaelic | gla_Latn
# Irish | gle_Latn
# Galician | glg_Latn
# Guarani | grn_Latn
# Gujarati | guj_Gujr
# Haitian Creole | hat_Latn
# Hausa | hau_Latn
# Hebrew | heb_Hebr
# Hindi | hin_Deva
# Chhattisgarhi | hne_Deva
# Croatian | hrv_Latn
# Hungarian | hun_Latn
# Armenian | hye_Armn
# Igbo | ibo_Latn
# Ilocano | ilo_Latn
# Indonesian | ind_Latn
# Icelandic | isl_Latn
# Italian | ita_Latn
# Javanese | jav_Latn
# Japanese | jpn_Jpan
# Kabyle | kab_Latn
# Jingpho | kac_Latn
# Kamba | kam_Latn
# Kannada | kan_Knda
# Kashmiri (Arabic script) | kas_Arab
# Kashmiri (Devanagari script) | kas_Deva
# Georgian | kat_Geor
# Central Kanuri (Arabic script) | knc_Arab
# Central Kanuri (Latin script) | knc_Latn
# Kazakh | kaz_Cyrl
# Kabiyè | kbp_Latn
# Kabuverdianu | kea_Latn
# Khmer | khm_Khmr
# Kikuyu | kik_Latn
# Kinyarwanda | kin_Latn
# Kyrgyz | kir_Cyrl
# Kimbundu | kmb_Latn
# Northern Kurdish | kmr_Latn
# Kikongo | kon_Latn
# Korean | kor_Hang
# Lao | lao_Laoo
# Ligurian | lij_Latn
# Limburgish | lim_Latn
# Lingala | lin_Latn
# Lithuanian | lit_Latn
# Lombard | lmo_Latn
# Latgalian | ltg_Latn
# Luxembourgish | ltz_Latn
# Luba-Kasai | lua_Latn
# Ganda | lug_Latn
# Luo | luo_Latn
# Mizo | lus_Latn
# Standard Latvian | lvs_Latn
# Magahi | mag_Deva
# Maithili | mai_Deva
# Malayalam | mal_Mlym
# Marathi | mar_Deva
# Minangkabau (Arabic script) | min_Arab
# Minangkabau (Latin script) | min_Latn
# Macedonian | mkd_Cyrl
# Plateau Malagasy | plt_Latn
# Maltese | mlt_Latn
# Meitei (Bengali script) | mni_Beng
# Halh Mongolian | khk_Cyrl
# Mossi | mos_Latn
# Maori | mri_Latn
# Burmese | mya_Mymr
# Dutch | nld_Latn
# Norwegian Nynorsk | nno_Latn
# Norwegian Bokmål | nob_Latn
# Nepali | npi_Deva
# Northern Sotho | nso_Latn
# Nuer | nus_Latn
# Nyanja | nya_Latn
# Occitan | oci_Latn
# West Central Oromo | gaz_Latn
# Odia | ory_Orya
# Pangasinan | pag_Latn
# Eastern Panjabi | pan_Guru
# Papiamento | pap_Latn
# Western Persian | pes_Arab
# Polish | pol_Latn
# Portuguese | por_Latn
# Dari | prs_Arab
# Southern Pashto | pbt_Arab
# Ayacucho Quechua | quy_Latn
# Romanian | ron_Latn
# Rundi | run_Latn
# Russian | rus_Cyrl
# Sango | sag_Latn
# Sanskrit | san_Deva
# Santali | sat_Olck
# Sicilian | scn_Latn
# Shan | shn_Mymr
# Sinhala | sin_Sinh
# Slovak | slk_Latn
# Slovenian | slv_Latn
# Samoan | smo_Latn
# Shona | sna_Latn
# Sindhi | snd_Arab
# Somali | som_Latn
# Southern Sotho | sot_Latn
# Spanish | spa_Latn
# Tosk Albanian | als_Latn
# Sardinian | srd_Latn
# Serbian | srp_Cyrl
# Swati | ssw_Latn
# Sundanese | sun_Latn
# Swedish | swe_Latn
# Swahili | swh_Latn
# Silesian | szl_Latn
# Tamil | tam_Taml
# Tatar | tat_Cyrl
# Telugu | tel_Telu
# Tajik | tgk_Cyrl
# Tagalog | tgl_Latn
# Thai | tha_Thai
# Tigrinya | tir_Ethi
# Tamasheq (Latin script) | taq_Latn
# Tamasheq (Tifinagh script) | taq_Tfng
# Tok Pisin | tpi_Latn
# Tswana | tsn_Latn
# Tsonga | tso_Latn
# Turkmen | tuk_Latn
# Tumbuka | tum_Latn
# Turkish | tur_Latn
# Twi | twi_Latn
# Central Atlas Tamazight | tzm_Tfng
# Uyghur | uig_Arab
# Ukrainian | ukr_Cyrl
# Umbundu | umb_Latn
# Urdu | urd_Arab
# Northern Uzbek | uzn_Latn
# Venetian | vec_Latn
# Vietnamese | vie_Latn
# Waray | war_Latn
# Wolof | wol_Latn
# Xhosa | xho_Latn
# Eastern Yiddish | ydd_Hebr
# Yoruba | yor_Latn
# Yue Chinese | yue_Hant
# Chinese (Simplified) | zho_Hans
# Chinese (Traditional) | zho_Hant
# Standard Malay | zsm_Latn
# Zulu | zul_Latn
lang_to_flores200 = {
    ASR.EN: "eng_Latn",
    ASR.ZH: "zho_Hans",
    ASR.DE: "deu_Latn",
    ASR.ES: "spa_Latn",
    ASR.RU: "rus_Cyrl",
    ASR.KO: "kor_Hang",
    ASR.FR: "fra_Latn",
    ASR.JA: "jpn_Jpan",
    ASR.MS: "zsm_Latn",
}


def lang_from_int(index):
    return list(whisper_language_codes.keys())[
        list(whisper_language_codes.values()).index(index)
    ]

