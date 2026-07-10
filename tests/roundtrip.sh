#!/usr/bin/env bash
# birnpack roundtrip test: pack -> unpack -> byte-exact compare.
# Uses the repo's own files plus generated test patterns — no external data needed.
set -u
cd "$(dirname "$0")/.." || exit 1
BIN=./birnpack
[ -x "$BIN" ] || { echo "build first: make"; exit 1; }
T=$(mktemp -d); trap 'rm -rf "$T"' EXIT

# --- assemble test set: repo files + generated patterns ---
cp src/welle_fast.c "$T/src.c"                 # C source (text)
cp Makefile "$T/makefile.txt"                  # small text
cp "$BIN" "$T/binary.bin"                      # ELF binary
head -c 262144 /dev/urandom > "$T/random.bin"  # incompressible
python3 - "$T" <<'EOF' 2>/dev/null || perl -e '
  open(F,">","'"$T"'/pattern.bin"); for $i (0..131071){ print F chr(($i*7+int(50*sin($i/40)))%256); } close F;
  open(F,">","'"$T"'/repeat.txt"); print F "the wave rolls on and on. " x 8000; close F;'
import sys, math
t = sys.argv[1]
with open(t+"/pattern.bin","wb") as f:      # smooth wave-like signal
    f.write(bytes(((i*7+int(50*math.sin(i/40)))%256) for i in range(131072)))
with open(t+"/repeat.txt","wb") as f:       # highly repetitive text
    f.write(b"the wave rolls on and on. "*8000)
EOF

fail=0; n=0; to=0; tc=0
printf "%-14s %10s %10s %7s %s\n" FILE ORIG PACKED RATIO OK
for f in "$T"/src.c "$T"/makefile.txt "$T"/binary.bin "$T"/random.bin "$T"/pattern.bin "$T"/repeat.txt; do
  [ -f "$f" ] || continue
  b=$(basename "$f"); n=$((n+1))
  "$BIN" c "$f" "$T/$b.bp"   >/dev/null 2>&1 || { echo "$b: ENCODE FAILED"; fail=$((fail+1)); continue; }
  "$BIN" d "$T/$b.bp" "$T/$b.out" >/dev/null 2>&1 || { echo "$b: DECODE FAILED"; fail=$((fail+1)); continue; }
  o=$(stat -c%s "$f" 2>/dev/null || stat -f%z "$f"); c=$(stat -c%s "$T/$b.bp" 2>/dev/null || stat -f%z "$T/$b.bp")
  if cmp -s "$f" "$T/$b.out"; then ok="yes"; else ok="NO — LOSSLESS BROKEN"; fail=$((fail+1)); fi
  to=$((to+o)); tc=$((tc+c))
  printf "%-14s %10s %10s %7s %s\n" "$b" "$o" "$c" "$(awk "BEGIN{printf \"%.3f\",$c/$o}")" "$ok"
done
echo "----------------------------------------------------"
printf "%-14s %10s %10s %7s\n" TOTAL "$to" "$tc" "$(awk "BEGIN{printf \"%.3f\",$tc/$to}")"
if [ "$fail" -eq 0 ]; then echo "PASS: $n/$n files byte-exact lossless."; exit 0
else echo "FAIL: $fail problem(s)."; exit 1; fi
