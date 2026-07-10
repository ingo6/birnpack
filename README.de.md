# birnpack

**Ein experimenteller verlustfreier Kompressor mit einem „Wellen-Vorhersager" — ein handgebautes Context-Mixing-Modell in einer einzigen C-Datei.**

birnpack entstand aus einem Forschungsprojekt mit einer Regel: *Erfinde deine eigene Welle — nie fremde Kompressoren aufrufen oder nachbauen (zip/lzma/zstd/flac), nie Container-Formate dekodieren.* Alles in `src/welle_fast.c` (1.626 Zeilen, ~113 selbst entwickelte Modell-Mechaniken) sagt rohe Bytes direkt vorher.

*English: see [README.md](README.md)*

## Was es gut kann

Starke Kompression auf gemischten Alltagsdateien. Auf unserem 17-Datei-Benchmark (Büro-Dokumente, CAD-Text, Bilder, Programme, Logs) erreichte birnpack eine Gesamt-Ratio von **0,539** — und schlug `gzip -9` auf **jeder einzelnen Datei**, z. B.:

| Dateityp | birnpack | gzip -9 |
|---|---|---|
| ELF-Programm (139 KB) | 46.850 | 61.924 |
| Bibliothek .so (680 KB) | 177.715 | 272.702 |
| Text-Log (293 KB) | 26.308 | 38.972 |
| C-Quelltext (109 KB) | 20.740 | 26.712 |
| STL-Mesh (2 MB) | 77.963 | (gzip weit dahinter) |

Bereits komprimierte Formate (JPEG, HEIC) schrumpfen nur minimal — das ist erwartbar und ehrlich: birnpack packt nie ein Format aus, um zu schummeln.

## Was es *nicht* ist

- **Nicht schnell.** Context-Mixing (PAQ-Familie als Idee, eigene Umsetzung): ca. 2–5 MB/s. Ziel der Forschung war die Ratio; Tempo wurde nur byte-identisch optimiert (−20 % im Projektverlauf).
- **Kein Archiver.** Eine Datei rein, eine Datei raus. Keine Archive, keine Metadaten.
- **Experimentell.** Das Format kann sich zwischen Versionen ändern. Nie als einzige Kopie von etwas verwenden.

## Bauen & benutzen

```sh
make               # baut ./birnpack  (beliebiger C-Compiler, braucht -pthread -lm)
./birnpack c eingabe.bin gepackt.bp    # packen
./birnpack d gepackt.bp ausgabe.bin    # entpacken
make test          # Roundtrip-Selbsttest (byte-genau auf 6 Testdateien)
```

## Selbst nachprüfen

Jede Behauptung hier ist testbar: `make test` packt und entpackt generierte + Repo-Dateien und vergleicht byte-genau. Probier eigene Dateien — Ergebnisse gern als Issue melden (Dateityp, Größen, Zeiten).

## Die Ehrlichkeits-Regeln des Projekts

1. **Messen, nie Demo.** Jede Verbesserung musste einen byte-genauen Voll-Korpus-Benchmark überleben.
2. **Kein Format-Dekodieren.** Rohe Bytes vorhersagen: ja. JPEG/deflate/CABAC-Ströme auspacken: schummeln.
3. **Verlustfrei ist heilig.** Ein einziges falsches Byte = ganzer Lauf ungültig.

## Lizenz

MIT — siehe [LICENSE](LICENSE). Benutzen, testen, kaputtmachen, Bescheid sagen.
