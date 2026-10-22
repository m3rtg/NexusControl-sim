"""Arayüz metinleri. Her görünen metin t("anahtar") ile alınır; diller App/locales/<dil>.json dosyalarındadır.
Yeni dil eklemek = yeni bir json dosyası. Eksik anahtar ekranda anahtarın kendisi olarak görünür."""
import json
import os

LOCALE_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), "locales")
DEFAULT_LANGUAGE = "tr"

_strings = {}
_language = DEFAULT_LANGUAGE

def available_languages():
    return sorted(f[:-5] for f in os.listdir(LOCALE_DIR) if f.endswith(".json"))

def set_language(lang):
    """Dil değişikliği yeni oluşturulan widget'lara uygulanır; uygulama yeniden başlatılınca tamamı değişir."""
    global _strings, _language
    path = os.path.join(LOCALE_DIR, f"{lang}.json")
    if not os.path.isfile(path):
        lang, path = DEFAULT_LANGUAGE, os.path.join(LOCALE_DIR, f"{DEFAULT_LANGUAGE}.json")
    with open(path, encoding="utf-8") as f:
        _strings = json.load(f)
    _language = lang

def language():
    return _language

def t(key, **values):
    text = _strings.get(key, key)
    return text.format(**values) if values else text

def num(value, digits=1):
    """Sayı biçimi dile göre (ondalık ayırıcı locale dosyasındaki "_decimal")."""
    return f"{value:.{digits}f}".replace(".", _strings.get("_decimal", "."))

set_language(DEFAULT_LANGUAGE)
