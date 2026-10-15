#!/usr/bin/env python3
"""Gezielte xrdp-Konfiguration; erhält Kommentare und wiederholte Xorg-param-Zeilen."""

import argparse
import os
from pathlib import Path
import re
import shutil
import tempfile


def update_section(text, section, settings):
    headers = list(re.finditer(r"^\s*\[([^\]\n]+)\][^\n]*$", text, re.MULTILINE))
    matches = [i for i, h in enumerate(headers) if h.group(1).lower() == section.lower()]
    if len(matches) != 1:
        raise ValueError(f"Erwartet genau einen Abschnitt [{section}]")
    i = matches[0]
    start = headers[i].end()
    end = headers[i + 1].start() if i + 1 < len(headers) else len(text)
    body = text[start:end]
    for key, value in settings.items():
        pattern = re.compile(r"^[ \t]*" + re.escape(key) + r"[ \t]*=[^\n]*$", re.MULTILINE | re.IGNORECASE)
        replacement = f"{key}={value}"
        if pattern.search(body):
            # Alle aktiven Duplikate erhalten denselben Wert (xrdp nutzt den letzten).
            body = pattern.sub(lambda _: replacement, body)
        else:
            body = body.rstrip("\n") + "\n" + replacement + "\n"
    return text[:start] + body + text[end:]


def configure(directory):
    directory = Path(directory)
    changes = {
        "xrdp.ini": {
            "Globals": {
                "port": "tcp://127.0.0.1:3389",
                "security_layer": "tls",
                "ssl_protocols": "TLSv1.2, TLSv1.3",
                "certificate": "/etc/xrdp/cert.pem",
                "key_file": "/etc/xrdp/key.pem",
                "autorun": "Xorg",
                "max_bpp": "24",
                "bitmap_cache": "true",
                "bitmap_compression": "true",
                "bulk_compression": "true",
                "tcp_nodelay": "true",
            },
        },
        "sesman.ini": {
            "Globals": {
                "ListenAddress": "127.0.0.1",
                "EnableUserWindowManager": "true",
                "UserWindowManager": ".local/bin/linobot2-rdp-session",
            },
            "Security": {
                "AllowRootLogin": "false",
                "TerminalServerUsers": "linobot2-rdp",
                "AlwaysGroupCheck": "true",
            },
        },
    }
    prepared = []
    # Erst beide Dateien prüfen; bei ungültiger Konfiguration nichts überschreiben.
    for filename, sections in changes.items():
        path = directory / filename
        if path.is_symlink() or not path.is_file():
            raise ValueError(f"Keine reguläre Konfigurationsdatei: {path}")
        original = path.read_text()
        result = original
        if filename == "xrdp.ini" and not re.search(r"^\[Xorg\]\s*$", original, re.MULTILINE | re.IGNORECASE):
            raise ValueError("Xorg-Abschnitt fehlt; xorgxrdp-Installation prüfen")
        for section, settings in sections.items():
            result = update_section(result, section, settings)
        prepared.append((path, original, result))

    for path, original, result in prepared:
        if original == result:
            continue
        backup = path.with_name(path.name + ".linobot2-backup")
        if not backup.exists():
            shutil.copy2(path, backup)
        stat = path.stat()
        fd, temporary = tempfile.mkstemp(prefix=path.name + ".", dir=directory)
        try:
            with os.fdopen(fd, "w") as stream:
                stream.write(result)
            os.chmod(temporary, stat.st_mode & 0o777)
            if os.geteuid() == 0:
                os.chown(temporary, stat.st_uid, stat.st_gid)
            os.replace(temporary, path)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)


if __name__ == "__main__":
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--config-dir", default="/etc/xrdp")
    configure(parser.parse_args().config_dir)