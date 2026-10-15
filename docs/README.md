# Projektdokumentation

Stand: 15.10.2026. Aktive Projektleitdokumente liegen zentral hier; paketbezogene Anleitungen bleiben jeweils im ROS-Paket.

## Aktive Dokumente und Zuständigkeit

| Dokument | Zuständigkeit | Verbindlichkeit |
| --- | --- | --- |
| [`DEVELOPMENT_PLAN.md`](DEVELOPMENT_PLAN.md) | Zielarchitektur, Entwicklungsreihenfolge und Gates | Autoritativer Fahrplan; ändert sich nur bei bewusster Planentscheidung |
| [`IMPLEMENTATION_STATUS.md`](IMPLEMENTATION_STATUS.md) | Nachweise, Laufberichte, Testergebnisse und bekannte Grenzen | Quelle für belegten Ist-Stand; fehlgeschlagene Läufe bleiben sichtbar |
| [`HANDOFF.md`](HANDOFF.md) | Kompakter aktueller Kontext für die nächste Sitzung | Übergabehilfe; verweist auf Plan, Status und To-dos statt sie zu duplizieren |
| [`OPEN_TASKS.md`](OPEN_TASKS.md) | Noch offene G4-/G5-Arbeit mit Reihenfolge und Abschlusskriterien | Aktuelle Aufgabenliste; erledigte Punkte hier entfernen und Nachweise im Status belassen |
| [`RDP_GUIDE.md`](RDP_GUIDE.md) | Aktueller, sicherer Zugriff auf die Linux-Simulationsdesktop-Sitzung | Betriebsnotiz für die bestehende Umgebung |
| [`G9_LIFECYCLE.md`](G9_LIFECYCLE.md) | Frischaufbau, Releaseaktivierung, Start/Stop, Recovery und Rollback | Verbindlicher lokaler Software-Lifecycle für das G9-Gate |

## Referenzen und Archiv

- [`reference/SOURCE_MANIFEST.md`](reference/SOURCE_MANIFEST.md): Quellherkunft, Schnittstellen- und Sicherungsmanifest.
- [`reference/MACOS_SETUP_PLAN.md`](reference/MACOS_SETUP_PLAN.md): historischer Plattform-/Bestandskatalog mit weiterhin relevanten Inventar- und Sicherungsdaten; kein aktueller Ablaufplan.
- [`archive/`](archive/README.md): ersetzte oder veraltete Anleitungen, nur zur Rückverfolgbarkeit. Archivierte Dokumente sind nicht als aktuelle Arbeitsanweisung zu verwenden.

## Pflege-Regeln

1. Entwicklungsreihenfolge und Gate-Definition nur im Development Plan pflegen.
2. Tatsächliche Belege und Abweichungen nur im Implementation Status ergänzen.
3. HANDOFF kurz halten und keine Laufchronik duplizieren.
4. Offene Arbeit in OPEN_TASKS bündeln; vor einem Lauf erst Vertrags-/Sicherheitslücken schließen.
5. Alte Pläne nicht löschen, wenn sie einzigartige Provenienz enthalten: nach `reference/` oder `archive/` verschieben und dort ausdrücklich einordnen.
