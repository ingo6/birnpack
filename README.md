# birnpack

**An experimental lossless compressor built around a "wave predictor" — a hand-written context-mixing model in a single C file.**

birnpack grew out of a research project with one rule: *invent your own wave — never call or imitate existing compressors (zip/lzma/zstd/flac), never decode container formats.* Everything in `src/welle_fast.c` (1,626 lines, ~113 hand-evolved model mechanics) predicts raw bytes directly.

*Deutsch: siehe [README.de.md](README.de.md)*

## What it's good at

Strong compression ratios on mixed real-world files. On our 17-file benchmark corpus (office documents, CAD text, images, executables, logs) birnpack reached an overall ratio of **0.539** — beating `gzip -9` on **every single file**, including:

| File type | birnpack | gzip -9 |
|---|---|---|
| ELF executable (139 KB) | 46,850 | 61,924 |
| shared library (680 KB) | 177,715 | 272,702 |
| text log (293 KB) | 26,308 | 38,972 |
| C source (109 KB) | 20,740 | 26,712 |
| STL mesh (2 MB) | 77,963 | (gzip far behind) |

Already-compressed formats (JPEG, HEIC) shrink only marginally — that is expected and honest; birnpack never unpacks a format to cheat.

## What it's *not*

- **Not fast.** This is a context-mixing model (PAQ-family idea, own implementation): roughly 2–5 MB/s. Ratio was the research goal; speed was optimized only where it stayed byte-identical (−20 % over the project).
- **Not a container.** One file in, one file out. No archives, no metadata.
- **Experimental.** Format may change between versions. Don't use it as your only copy of anything.

## Build & use

```sh
make               # builds ./birnpack  (any C compiler, needs -pthread -lm)
./birnpack c input.bin packed.bp    # compress
./birnpack d packed.bp output.bin   # decompress
make test          # roundtrip self-test (byte-exact on 6 test files)
```

## Verify it yourself

Every claim here is testable: `make test` packs and unpacks generated + repo files and byte-compares. Try your own files — we'd love to hear results (open an issue with file type, sizes, and timing).

## Honesty rules the project was built under

1. **Measure, never demo.** Every improvement had to survive a full-corpus, byte-exact benchmark.
2. **No format decoding.** Predicting raw bytes is allowed; unpacking JPEG/deflate/CABAC streams is cheating.
3. **Lossless is holy.** A single wrong byte fails the whole run.

## License

MIT — see [LICENSE](LICENSE). Use it, test it, break it, tell us.
