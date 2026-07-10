/* welle_fast.c — SCHNELLE Welle (eigener Algorithmus, C, 1 Kern).
 *
 * !!! VERBOTEN: KEINE Fremd-Kopie !!!
 * !!! VERBOTEN: KEINE Fremd-Kopie !!!
 * !!! VERBOTEN: KEINE Fremd-Kopie !!!
 * Fremd-Format-Kompressoren NIEMALS in welle_fast nachbauen.
 *
 * INGO-GESETZ: eigene Welle. KEINE Fremd-Kompressoren. Hier ist von Grund auf
 * selbst gebaut: ein logistischer Kontext-Mischer (die "Welle" = adaptive
 * Bit-Vorhersage aus mehreren Byte-Kontexten) + ein binaerer Range-Coder.
 * Gleiches Modell in encode/decode -> Lockstep -> byte-exakt.
 *
 * Ziel dieser Runde: TEMPO. Feste Integer-Tabellen, keine Floats im heissen
 * Pfad, ein Kern, schlank. (welle.py bleibt unberuehrt.)
 *
 * Bauen:  cc -O3 -pthread -o welle_fast welle_fast.c   (R6: Mehrkern-Bloecke)
 * Nutzen: welle_fast c IN OUT   (pack)
 *         welle_fast d IN OUT   (unpack)
 */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <pthread.h>
#ifdef _WIN32
#include <windows.h>
static long portable_ncpu(void) { SYSTEM_INFO si; GetSystemInfo(&si); return si.dwNumberOfProcessors > 0 ? (long)si.dwNumberOfProcessors : 1; }
#else
#include <sys/mman.h>
#include <unistd.h>
static long portable_ncpu(void) { long n = sysconf(_SC_NPROCESSORS_ONLN); return n > 0 ? n : 1; }
#endif
#ifdef __linux__
#include <sys/prctl.h>
#endif

/* ---- logistische Tabellen (Integer, wie in der klassischen CM-Welt) ---- */
static int STRETCH[4096];
static short SQUASH[4096]; /* index d+2048 in [-2048,2047] -> p in 0..4095 */

static int squash(int d) {
    if (d >= 2047) return 4095;
    if (d <= -2047) return 0;
    static const int t[33] = {
        1,2,3,6,10,16,27,45,73,120,194,310,488,747,1101,1546,2047,
        2549,2994,3348,3607,3785,3901,3975,4022,4050,4068,4079,4085,
        4089,4092,4093,4094};
    int w = d & 127;
    int i = (d >> 7) + 16;
    return (t[i] * (128 - w) + t[i + 1] * w + 64) >> 7;
}

static void init_tables(void) {
    for (int i = 0; i < 4096; i++) SQUASH[i] = (short)squash(i - 2048);
    /* stretch = inverse of squash, monoton */
    int pi = 0;
    for (int d = -2047; d <= 2047; d++) {
        int p = squash(d);
        for (int j = pi; j <= p; j++) STRETCH[j] = d;
        pi = p + 1;
    }
    for (int j = pi; j < 4096; j++) STRETCH[j] = 2047;
}

/* ---- Modell: N hashbasierte Kontexte, logistisch gemischt ---- */
#define NCTX   8   /* R297: 5 dichte Kontexte + 3 Sparse-Kontexte (Stride-Skip) */
/* R2 (ALLE-FORMATE): TBITS 22->20. 2^22*6*2 = 48 MB Tabellen liegen weit ausserhalb
 * des Caches (L3=18MB) -> jeder Zugriff RAM-langsam. 2^20 = 12 MB passt in den L3
 * -> ~2.5x schneller auf grossen Dateien. Ratio bleibt praktisch gleich; auf
 * inkompressiblen Daten (heic) sogar BESSER (dichtere Tabelle trainiert die Zaehler
 * mehr statt kalt/sparse) -> heic kippt zum welle-Byte-Sieg. */
#define TBITS  20
#define TSIZE  (1 << TBITS)
#define TMASK  (TSIZE - 1)

/* R20: Tabellengroesse zur LAUFZEIT. Einzelblock (kleine Dateien + docx/step, duenne
 * Marge) = 20 fuer max Ratio. Mehrblock (grosse Dateien mit RIESEN-Marge) = kleiner,
 * damit die parallelen Block-Tabellen in den L3 passen -> weniger Bandbreiten-Streit
 * zwischen den Threads -> bessere Kern-Skalierung. Beide Seiten waehlen per Datei-Flag. */
#define BLOCK_TBITS 17
static int      g_nctx = NCTX;   /* R211: Laufzeit-Kontexttiefe (grosse nb>3-Dateien leichter, docx/klein voll) */
static int      g_tbits = TBITS;
static size_t   g_tsize = TSIZE;
static uint32_t g_tmask = TMASK;
static void set_tbits(int tb){ g_tbits = tb; g_tsize = (size_t)1 << tb; g_tmask = (uint32_t)(g_tsize - 1); }

/* R5 (ALLE-FORMATE): adaptive Zaehler-Lernrate. Der Zaehler (uint16) traegt die
 * 12-Bit-Wahrscheinlichkeit in den unteren Bits; die 4 freien Bits zaehlen die
 * Treffer (0..15). FRISCHE Kontexte lernen SCHNELL (grosser Schritt), reife LANGSAM
 * (stabil). Fester Rate 5/128 (~>>4.7) vorher = langsamer Warmup -> teuer auf kleinen
 * Dateien (json/txt). RATE[cnt] = Rechts-Shift des Fehlers, klein->schnell. */
static const int RATE[16] = {1,1,1,2,2,2,3,3,3,3,3,3,3,3,3,3};  /* R365/R367: FULL-Pfad dichte-Zaehler-Rate, Decke 3 (Text/CAD, oft gesehen -> schnellste reife Anpassung; per-Pfad getrennt von weak) */
static const int RATE_W[16] = {1,1,1,2,2,3,3,4,4,5,5,5,5,5,5,5};  /* R367: WEAK-Pfad (nb>12: Foto/CAD-Rauschen), Decke 5 (rausch-dominiert -> stabile langsame Anpassung) */
static const int RATE_J[16] = {2,2,2,2,3,3,3,3,4,4,4,5,5,5,5,5};  /* R110: jpg cross-byte-Zaehler eigene Raten-Kurve (kein rate-1-Schnellstart -> sparse-Kontext ueberreagiert nicht auf 1 Sample; sanfte Rampe zur Decke 5) */
static const int RATE_JE[16] = {2,2,2,2,3,3,3,3,4,4,4,4,4,4,4,4};  /* R112: jpeg (Full-Pfad, kleiner 43KB-Block) eigene Raten-Kurve = R110-Rampe aber Decke 4 (kurzer Block konvergiert nie voll -> adaptivere Decke traegt den nicht-stationaeren Strom besser; jpg-stationaer will Decke 5) */
static const int RATE_H[16] = {1,1,1,2,2,2,2,2,2,2,2,2,2,2,2,2};  /* R31: heic dichte-Zaehler-Rate Decke 2 (SCHNELL) — dieser Container-Typ hat block-lokal schnell wechselnde Byte-Statistik, nicht stationaer; per Magic-Byte getrennt (Sweep 5>4>3>2 monoton, all-1 schlechter) */
static const int IRATE[16] = {1,1,1,2,2,2,3,3,4,4,6,6,6,6,6,6};  /* R304/R365: iprob-Rate, Decke 5->6 (indirekte Modelle = grosser sparse Zustandsraum, wollen LANGSAMERE/stabilere reife Anpassung — Gegenteil der dichten Zaehler) */
static const int IRATE_WB[16] = {1,1,1,2,2,2,3,3,4,4,9,9,9,9,9,9};  /* R382: weak-BINAER (heic/jpg) iprob-Rate — Decke 6->9, das Rausch-Regime will NOCH langsamere/stabilere iprob-Reifung (stl bleibt bei 6 per g_word-Gate) */
#define PMASK 0x0FFFu

/* R13: Match-Modell ("Echo"). MI = zusaetzlicher Mischer-Eingang, NW = Eingaenge. */
#define MI     NCTX          /* Index des kurzen Match-Eingangs */
#define MI2    (NCTX + 1)    /* R295: 2. (langer) Match-Eingang */
#define IND    (NCTX + 2)    /* R298: indirekter Bit-Historie-Eingang */
#define IND2   (NCTX + 3)    /* R299: 2. indirekter Bit-Historie-Eingang (komplementaerer Kontext) */
#define IND3   (NCTX + 4)    /* R300: 3. indirekter Bit-Historie-Eingang */
#define IND4   (NCTX + 5)    /* R305: 4. indirekter Bit-Historie-Eingang */
#define MI3    (NCTX + 6)    /* R77: heic BIT-Echo (Block-Raster-Byte sagt Bit-fuer-Bit voraus) */
#define JBIT   (NCTX + 7)    /* R89: jpg CROSS-BYTE Bit-Historie (Entropie-Codewort-Praefix ueber Byte-Grenzen; held-out 0.977 b/bit K=16) */
#define JBIT2  (NCTX + 8)    /* R90: 2. cross-byte Fenster (kuerzeres K = robuster Backoff, PPM-artig gemischt) */
#define JBIT3  (NCTX + 9)    /* R91: 3. cross-byte Fenster (noch kuerzeres K = 3. PPM-Backoff-Ordnung, "erst 2 dann 3 mischen") */
#define JBIT4  (NCTX + 10)   /* R94: 4. cross-byte Fenster (kuerzester Backoff additiv, {16,14,12,K4}) */
#define JBIT5  (NCTX + 11)   /* R96: 5. cross-byte Fenster (Ladder-Verdichtung, Sweep) */
#define JBIT6  (NCTX + 12)   /* R98: 6. cross-byte Fenster (Sweep) */
#define JBIT7  (NCTX + 13)   /* R99: 7. cross-byte Fenster (Sweep, Ladder-Gap) */
#define JBIT8  (NCTX + 14)   /* R101: 8. cross-byte Fenster (Sweep, WF_JB8) */
#define JBIT9  (NCTX + 15)   /* R102: 9. cross-byte Fenster (Sweep, WF_JB9) */
#define JBIT10 (NCTX + 16)   /* R104: 10. cross-byte Fenster (Sweep, WF_JB10) */
#define JBIT11 (NCTX + 17)   /* R105: 11. cross-byte Fenster (Sweep, WF_JB11) */
#define JBIT12 (NCTX + 18)   /* R106: 12. cross-byte Fenster (Sweep, WF_JB12) */
#define JSP    (NCTX + 19)   /* R113: jpg SPARSE-BIT cross-byte (zwei getrennte Bit-Gruppen: recent + far, PPM-Sparse-Bit statt contiguous K-Fenster; Huffman-Codewort-Praefix wiederholt sich ueber Codewort-Grenzen) */
#define JSP2   (NCTX + 20)   /* R114: 2. jpg SPARSE-BIT-Fenster an laengerem Offset (~2 Codewoerter zurueck, PPM-Backoff-Stapel auf der Sparse-Achse wie R90 auf der contiguous-Achse) */
#define JSP3   (NCTX + 21)   /* R115: 3. jpg SPARSE-BIT-Fenster (langes Ende der Sparse-Ladder wie R91s 3. contiguous-Fenster K=16) */
#define JSP4   (NCTX + 22)   /* R116: 4. jpg SPARSE-BIT-Fenster (Mittel-Gap-Verdichtung wie R94s 4. contiguous-Fenster K=13) */
#define JSP5   (NCTX + 23)   /* R118: 5. jpg SPARSE-BIT-Fenster (naechster Gap der Sparse-Ladder 10/11/13/16) */
#define JSP6   (NCTX + 24)   /* R120: 6. jpg SPARSE-BIT-Fenster (naechster Gap der Sparse-Ladder 8/10/11/13/16) */
#define JSP7   (NCTX + 25)   /* R122: 7. jpg SPARSE-BIT-Fenster mit ASYMMETRISCHER Gruppengroesse (GR-Split) */
#define JSP8   (NCTX + 26)   /* R123: 8. jpg SPARSE-BIT-Fenster, 2. asymmetrischer GR-Split (andere Balance, additiv) */
#define JSP9   (NCTX + 27)   /* R124: 9. jpg SPARSE-BIT-Fenster, 3. asymmetrischer GR-Split (recent-heavy Richtung, additiv) */
#define JSP10  (NCTX + 28)   /* R125: 10. jpg SPARSE-BIT-Fenster, 2. RECENT-heavy Split (stapelt die recent-heavy Richtung wie R122/R123 die far-heavy) */
#define JSP11  (NCTX + 29)   /* R126: 11. jpg SPARSE-BIT-Fenster, 3-GRUPPEN-Kontext (recent + mittel + far, NEUE Kontext-FORM: Interferenz ZWEIER Referenzen mit der Position) */
#define JSP12  (NCTX + 30)   /* R127: 2. 3-GRUPPEN-Fenster an anderen Offsets (stapelt die Interferenz-Klasse wie die 2-Gruppen-Asymmetrie R114/R125) */
#define JSP13  (NCTX + 31)   /* R128: 3. 3-GRUPPEN-Fenster (saettigt die Interferenz bei 2 oder stapelt weiter?) */
#define JSP14  (NCTX + 32)   /* R130: 4-GRUPPEN-Kontext (recent + 3 Referenzen gleichzeitig, hoehere Interferenz-Ordnung) */
#define JSP15  (NCTX + 33)   /* R131: 2. 4-GRUPPEN-Fenster (andere drei Referenz-Distanzen, stapelt die Ordnung?) */
#define JSP16  (NCTX + 34)   /* R132: 3. 4-GRUPPEN-Fenster (stapelt die Ordnung-4-Leiter zu 3?) */
#define JSP17  (NCTX + 35)   /* R133: 4. 4-GRUPPEN-Fenster (stapelt die Leiter zu 4?) */
#define JSP18  (NCTX + 36)   /* R137: 3-GRUPPEN mit ASYMMETRISCHER Gruppengroesse (R122-GR-Split-Prinzip auf die 3-Gruppen-Form) */
#define JSP19  (NCTX + 37)   /* R146: 5. 4-GRUPPEN-Fenster (Leiter zu 5 — R134-Frontier: stapelt die Ordnung-4-Leiter zu 5?) */
#define JFF    (NCTX + 38)   /* R148: jpg FF-DISTANZ-Regime (Bytes seit letztem 0xFF-Datenbyte × kurzes Bit-Fenster) — BYTE-EREIGNIS-abgeleitet, ORTHOGONAL zur Bit-Historie (Probe K=10×FFdist −0.25% held-out); Proxy fuer lokales DCT-Aktivitaets-Regime */
#define JFF2   (NCTX + 39)   /* R149: 2. FF-Distanz-Fenster (Regime × ANDERE Bit-Fensterbreite g_jfw2) — K-Ladder-Stapeln (R90/R114) auf der FF-Regime-Achse */
#define NW     (NCTX + 40)   /* Mischer-Eingaenge gesamt (R149: +JFF2) */
#define JB     14            /* R89: cross-byte Bit-Fenster-Laenge (Real-Codec-Sweep: K=14 Optimum je 128KB-Block; K=16-held-out-Probe war whole-file, per-Block zu sparse) */
#define JB2    12            /* R90: 2. cross-byte Fenster (12 vs JB=14: komplementaerer Backoff, PPM-Mix; Paar-Sweep-Optimum, JB2==JB=14 waere redundant) */
#define JB3    16            /* R91: 3. cross-byte Fenster (kuerzester Backoff; Sweep) */
#define JB4    13            /* R94: 4. cross-byte Fenster (Sweep) */
#define JB5    19            /* R96: 5. cross-byte Fenster (Sweep -> Ladder-Lang-Ende) */
#define JB6    10            /* R98: 6. cross-byte Fenster (Sweep -> Ladder-Kurz-Ende K=10) */
#define JB7    8             /* R99: 7. cross-byte Fenster (Sweep -> Ladder-Gap) */
#define JB8    11            /* R101: 8. cross-byte Fenster (Sweep -> Ladder-Gap 10<11<12, -216B) */
#define JB9    22            /* R102: 9. cross-byte Fenster (Sweep -> Ladder-Lang-Ende K=22, -170B) */
static int g_jb10 = 9;        /* R104: 10. cross-byte Fenster K=9 (Sweep-Optimum, Ladder-Gap zwischen 8 und 10; -83B) */
static int g_jb11 = 24;       /* R105: 11. cross-byte Fenster K=24 (Sweep-Optimum, Ladder-Lang-Ende jenseits 22; -30B) */
static int g_jb12 = 15;       /* R106: 12. cross-byte Fenster K=15 (Sweep-Optimum, Ladder-Mittel-Gap zwischen 14 und 16; -27B) */
static int g_jshift = 1;     /* R109: jpeg (Full-Pfad, kleiner Block) cross-byte K-Verkuerzung (WF_JSH) */
static const int g_jspoff = 10;    /* R113: jpg sparse-bit far-group Offset (Sweep-Optimum, sauberer Basin off=9/10/11 = -509/-704/-702; Bits ~1 kurzes Codewort zurueck) */
static const int g_jspgr  = 7;     /* R113: jpg sparse-bit Gruppengroesse (recent+far je 7 Bits = 14-bit-Kontext; off<GR kollidiert -> dilute) */
static const int g_jsp2off = 13;    /* R114: 2. sparse-bit far-group Offset (Sweep-Optimum, sauberer Ein-Peak 11/12/13/14/15 = -287/-313/-347/-284/-222; komplementaer zu JSP off=10, off2=10-Dup gibt nur -24) */
static const int g_jsp3off = 16;    /* R115: 3. sparse-bit far-group Offset (Sweep-Optimum, sauberer Ein-Peak 15/16/17 = -103/-122/-97; langes Ende der Sparse-Ladder komplementaer zu off=10/13; off3=10/13-Dup gibt +8/+19) */
static const int g_jsp4off = 11;    /* R116: 4. sparse-bit far-group Offset (Sweep-Optimum, sauberer Peak 8/9/11/12 = -82/-106/-151/-89; fuellt den Gap zwischen off=10 und off=13; Dups 10/13/16 geben +10/+40/+91) */
static const int g_jsp5off = 8;    /* R118: 5. sparse-bit far-group Offset (Sweep-Optimum, sauberer Basin 8/9 = -67/-59 unter der Ladder 10/11/13/16; Dups 10/11/13/16 geben +97/+99/+77/+98; off=7=GR Gruppen-Kollision +112) */
static const int g_jsp6off = 20;    /* R120: 6. sparse-bit far-group Offset (Sweep-Optimum, sauberer Basin 19/20 = -23/-24 am LANGEN Ende ueber der Ladder 8/10/11/13/16; Dups 8/10/11/13/16 geben +168/+101/+90/+69/+88; Mittel-Gaps 9/12/14/15 schwach) */
static const int g_jsp7gr1 = 4;    /* R122: 7. sparse-bit RECENT-Gruppengroesse (asymmetrischer GR-Split: KURZER 4-bit Praefix + LANGE 11-bit far-Referenz, 2^15-Tabelle; Sweep-Optimum, sauberer Peak far 10/11/12 = -543/-590/-548, off 6/7/8 = -474/-590/-493; symmetrisch 7+7 = +65 REDUNDANT zu den 6 GR=7-Fenstern) */
static const int g_jsp7gr2 = 11;   /* R122: 7. sparse-bit FAR-Gruppengroesse (asymmetrischer GR-Split) */
static const int g_jsp7off = 7;    /* R122: 7. sparse-bit far-group Offset (Sweep-Optimum) */
static const int g_jsp8gr1 = 4;    /* R123: 8. sparse-bit RECENT-Gruppengroesse (2. asymmetrischer far-heavy Split, KOMPLEMENTAER zu JSP7 4+11@7: hier 4+14@8, NOCH laengere 14-bit far-Referenz an laengerem Offset, 2^18-Tabelle; Sweep-Optimum, sauberer Peak far 13/14/15 = -350/-371/-344, off 8/9/10 = -371/-365/-303; JSP7-Klon 4+11@7 = +36 REDUNDANT) */
static const int g_jsp8gr2 = 14;   /* R123: 8. sparse-bit FAR-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp8off = 8;    /* R123: 8. sparse-bit far-group Offset (Sweep-Optimum) */
static const int g_jsp9gr1 = 11;   /* R124: 9. sparse-bit RECENT-Gruppengroesse (3. Split, RECENT-heavy Richtung: 11-bit recent-Praefix + 4-bit far-Referenz @off=15, 2^15-Tabelle; orthogonal zu den zwei FAR-heavy JSP7/JSP8 — schaerft WO im Codewort wir sind statt WELCHE Referenz; Sweep-Optimum, sauberer Peak recent 10/11/12 = -231/-240/-145, off 14/15/16 = -207/-240/-231, far 4/5 = -240/-218; 3. far-heavy 4+13@9 = -17 SUBSUMIERT von JSP7/8, JSP8-Klon +73) */
static const int g_jsp9gr2 = 4;    /* R124: 9. sparse-bit FAR-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp9off = 15;   /* R124: 9. sparse-bit far-group Offset (Sweep-Optimum) */
static const int g_jsp10gr1 = 10;  /* R125: 10. sparse-bit RECENT-Gruppengroesse (2. RECENT-heavy Split: 10-bit recent-Praefix + 4-bit far @off=18, 2^14-Tabelle; stapelt die recent-heavy Richtung wie R122/R123 die far-heavy — anderer Praefix-Laengen/Offset als JSP9 11+4@15, faengt eine WEITER zurueckliegende Codewort-Positions-Echo; Sweep-Optimum: off 17/18/19 = -120/-129/-95, recent 10/11/12/13 = -129/-122/-125/-87, far 3/4/5 = -99/-129/-104; JSP9-Klon 11+4@15 = +152, 2. far-heavy 4+14@8 = +71 subsumiert) */
static const int g_jsp10gr2 = 4;   /* R125: 10. sparse-bit FAR-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp10off = 18;  /* R125: 10. sparse-bit far-group Offset (Sweep-Optimum, laenger als JSP9 @15) */
static const int g_jsp11gr1 = 5;   /* R126: 11. sparse-bit 3-GRUPPEN-Kontext (recent 5-bit + mittel 5-bit @off=10 + far 5-bit @off=15, 2^15-Tabelle 64KB; NEUE Kontext-FORM — modelliert die gleichzeitige Korrelation ZWEIER Referenzen (@10 ~1.5 Codewoerter, @15 ~2 Codewoerter) MIT der aktuellen Position = Interferenz-Struktur, die kein 2-Gruppen-Fenster darstellt; BEWEIST jpg-Codewort-Praefix-Struktur ist NICHT rein Markov (mehrfach-Referenz-Kopplung); Sweep-Optimum: offm 9/10/11 = -185/-240/-185, offf 13/14/15/16 = -173/-211/-240/-198, GR1=5 (6=+96 dilute)) */
static const int g_jsp11gr2 = 5;   /* R126: 11. sparse-bit MITTEL-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp11gr3 = 5;   /* R126: 11. sparse-bit FAR-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp11offm = 10; /* R126: 11. sparse-bit MITTEL-Gruppen-Offset (Sweep-Optimum) */
static const int g_jsp11offf = 15; /* R126: 11. sparse-bit FAR-Gruppen-Offset (Sweep-Optimum) */
static const int g_jsp12gr1 = 5;   /* R127: 2. 3-GRUPPEN-Fenster (recent 5b + mittel 5b @off=9 + far 5b @off=19, 2^15=64KB) — STAPELT die Interferenz-Klasse: JSP11 sitzt bei (mittel@10, far@15), JSP12 bei (mittel@9, far@19) = ZWEI verschiedene Zweit-Referenz-Distanz-Paare; wie die 2-Gruppen-Asymmetrie je Richtung zu 2 Fenstern stapelt (R114/R125), stapelt die 3-Gruppen-Interferenz zu 2 Fenstern (-157 additiv zu JSP11 -240); Sweep-Optimum: offm 8/9/10 = -118/-157/-146, offf 18/19/20 = -144/-157/-137, GR=5/5/5 (GR1=6=+76 dilute, GR2=6=-152 marginal) */
static const int g_jsp12gr2 = 5;   /* R127: 2. 3-GRUPPEN MITTEL-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp12gr3 = 5;   /* R127: 2. 3-GRUPPEN FAR-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp12offm = 9;  /* R127: 2. 3-GRUPPEN MITTEL-Offset (Sweep-Optimum, kuerzer als JSP11 @10) */
static const int g_jsp12offf = 19; /* R127: 2. 3-GRUPPEN FAR-Offset (Sweep-Optimum, laenger als JSP11 @15 = 2. Zweit-Referenz-Distanz) */
static const int g_jsp13gr1 = 5;   /* R128: 3. 3-GRUPPEN-Fenster (recent 5b + mittel 5b @off=10 + far 5b @off=20, 2^15=64KB) — STAPELT die Interferenz-Klasse zu 3 Fenstern (nicht bei 2 gesaettigt): JSP11 (10,15) + JSP12 (9,19) + JSP13 (10,20) = eine LEITER der Zweit-Referenz-Distanzen (far 15/19/20); jedes Paar faengt eine eigene DCT-Kategorie-Kopplung; offf=19 (= JSP12) = +84 REDUNDANT bestaetigt den Mechanismus; Sweep-Optimum: offm 10/11/12 = -62/-49/-31, offf 20/21/22 = -62/-62/-52, GR=5/5/5 */
static const int g_jsp13gr2 = 5;   /* R128: 3. 3-GRUPPEN MITTEL-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp13gr3 = 5;   /* R128: 3. 3-GRUPPEN FAR-Gruppengroesse (Sweep-Optimum) */
static const int g_jsp13offm = 10; /* R128: 3. 3-GRUPPEN MITTEL-Offset (Sweep-Optimum) */
static const int g_jsp13offf = 20; /* R128: 3. 3-GRUPPEN FAR-Offset (Sweep-Optimum, 3. Zweit-Referenz-Distanz jenseits JSP12 @19) */
static const int g_jsp14gr1 = 5;   /* R130: 4-GRUPPEN-Kontext (recent 5b + 3 Ref 5b @10/@15/@19, 2^20=2MB) — hoehere Interferenz-ORDNUNG: die Codewort-Praefix-Position mit DREI Referenzen GLEICHZEITIG gekoppelt (10/15/19 = die Ladder-Distanzen der pairwise 3-Gruppen JSP11/12/13); traegt -96B ADDITIV, was die paarweisen 3-Gruppen (je 2 Ref) strukturell nicht sehen; GR=5 (2^20) noetig fuer die 4-Gruppen-Aufloesung (GR=4=2^16 nur -18); Sweep-Optimum: off2 9/10/11=-75/-96/-75, off3 14/15=-96, off4 18/19/20=-54/-96/-88 */
static const int g_jsp14gr2 = 5;   /* R130: 4-GRUPPEN 1. REF-Gruppengroesse */
static const int g_jsp14gr3 = 5;   /* R130: 4-GRUPPEN 2. REF-Gruppengroesse */
static const int g_jsp14gr4 = 5;   /* R130: 4-GRUPPEN 3. REF-Gruppengroesse */
static const int g_jsp14off2 = 10; /* R130: 4-GRUPPEN 1. Referenz-Offset (Sweep-Optimum) */
static const int g_jsp14off3 = 15; /* R130: 4-GRUPPEN 2. Referenz-Offset (Sweep-Optimum) */
static const int g_jsp14off4 = 19; /* R130: 4-GRUPPEN 3. Referenz-Offset (Sweep-Optimum) */
static const int g_jsp15gr1 = 5, g_jsp15gr2 = 5, g_jsp15gr3 = 5, g_jsp15gr4 = 5;   /* R131: 2. 4-GRUPPEN-Fenster (recent 5b + 3 Ref @9/@13/@22, 2^20) — die 4-Gruppen-ORDNUNG STAPELT zu 2 Fenstern wie die 3-Gr-Leiter (R126-128): ein anderes Dreifach-Referenz-Tripel (9/13/22 vs JSP14s 10/15/19), traegt -53B ADDITIV; Sweep-Optimum sauberer Basin: off2 8/9/10=-36/-53/-24, off3 12/13/14=-35/-53/-43, off4 21/22/23=-46/-53/-35 */
static const int g_jsp15off2 = 9, g_jsp15off3 = 13, g_jsp15off4 = 22;   /* R131: 2. Referenz-Tripel (Sweep-Optimum) */
static const int g_jsp16gr1 = 5, g_jsp16gr2 = 5, g_jsp16gr3 = 5, g_jsp16gr4 = 5;   /* R132: 3. 4-GRUPPEN-Fenster (recent 5b + 3 Ref @6/@7/@16, 2^20) — die Ordnung-4-Leiter STAPELT zu 3 Fenstern; das 3. Tripel ist KOMPAKT (6/7/16, alle drei nah) statt spread (JSP14 10/15/19, JSP15 9/13/22) und traegt -128B (> JSP15 -53!): das dichte Referenz-Ende fasst die HAEUFIGEN Dreifach-Kopplungen (R94/R98-Muster), die die spread-Tripel verpassten; sauberer 3D-Basin: off2 5/6/7=-66/-128/-118, off3 6/7/8=-101/-128/-122, off4 15/16/17=-64/-128/-100 */
static const int g_jsp16off2 = 6, g_jsp16off3 = 7, g_jsp16off4 = 16;   /* R132: 3. Referenz-Tripel (kompakt, Sweep-Optimum) */
static const int g_jsp17gr1 = 5, g_jsp17gr2 = 5, g_jsp17gr3 = 5, g_jsp17gr4 = 5;   /* R133: 4. 4-GRUPPEN-Fenster (recent 5b + 3 Ref @6/@10/@18, 2^20) — die Ordnung-4-Leiter STAPELT zu 4 Fenstern; das 4. Tripel (6/10/18) liegt ZWISCHEN kompakt (JSP16 6/7/16) und spread (JSP14 10/15/19), traegt -65B (Leiter JSP14 -96/JSP15 -53/JSP16 -128/JSP17 -65, KEIN geometrischer Abfall — jedes Tripel nach Kopplungs-Haeufigkeit); sauberer 3D-Basin: off2 5/6/7=-36/-65/-12, off3 9/10/11=-40/-65/-49, off4 17/18/19=-30/-65/-34 */
static const int g_jsp17off2 = 6, g_jsp17off3 = 10, g_jsp17off4 = 18;   /* R133: 4. Referenz-Tripel (Sweep-Optimum) */
static const int g_jsp19gr1 = 5, g_jsp19gr2 = 5, g_jsp19gr3 = 5, g_jsp19gr4 = 5;   /* R146: 5. 4-GRUPPEN-Fenster (recent 5b + 3 Ref, 2^20) — testet ob die Ordnung-4-Leiter zu 5 stapelt; Tripel distinkt von JSP14 10/15/19, JSP15 9/13/22, JSP16 6/7/16, JSP17 6/10/18 */
static const int g_jsp19off2 = 7, g_jsp19off3 = 12, g_jsp19off4 = 19;   /* R146: 5. Referenz-Tripel — Sweep-Optimum spread (kompakt 5/6/7=+59..+225 REDUNDANT mit JSP16/17, spread offen wie JSP14/15); Basin off4 17/18/19/20/21=0/-13/-23/-18/-21, off3 11/12/13=-17/-23/-19, off2 6/7/8=-12/-23/-6 */
static const int g_jsp18gr1 = 3, g_jsp18gr2 = 4, g_jsp18gr3 = 8;   /* R137: ASYMMETRISCHE 3-Gruppen-GR (R122-GR-Split-Prinzip auf die 3-Gruppen-Form): recent 3b + mittel 4b @off=5 + FAR-heavy 8b @off=9, 2^15=64KB. Die uniform-GR=5/5/5 3-Gruppen-Fenster (R126-128) loesen die far-Referenz nur auf 5 Bit auf; ein far-heavy 8-Bit-Fenster faengt die laengere DCT-Kategorie-Kopplung (wie R122s 2-Gruppen-far-heavy 4+11 die symmetrischen 7+7 schlug), additiv -355B. GR2=4/5/6 identisch (mittel+far ueberlappen -> 3/4/8 minimal); GR1=2/3/4=-243/-355/-135, GR3=7/8/9=-302/-355/-301. Sweep-Optimum */
static const int g_jsp18offm = 5, g_jsp18offf = 9;   /* R137: asym 3-Gruppen Offsets (2D-Basin: offm 4/5/6=-162/-355/-206, offf 7/8/9/10/11=-253/-302/-355/-231/-161) */
static int g_jsh = 0;        /* R109: effektiver K-Shift je Block (0=jpg weak, g_jshift=jpeg full) */
static int g_jthr = 12;      /* R109: Shift nur auf Fenster mit K>=g_jthr (0=alle) */
#define JEFF(K) ( (int)(K) >= g_jthr ? (int)(K)-g_jsh : (int)(K) )
#define JMASK(K) ( ((uint32_t)1 << ( JEFF(K) < 4 ? 4 : JEFF(K) )) - 1u )
#define IBITS  20            /* R298: Bit-Historie-Kontexttabelle 2^20 (order-3 x Bit-Knoten) */
#define ISUB   256           /* R302: iprob-Sub-Karte je Bit-Baum-Knoten */
#define GCRATE 13            /* R313/R366: Lernrate Kombinierer (12->13 nach R365-Raten-Retune: der Kombinierer will langsamere Anpassung) */
#define MMLEN  4             /* Kontextlaenge kurzer Match (Bytes) */
#define MMBITS 19            /* R357: kurzer Match 2^19 (byte-getunt: R295 waehlte 18 fuer Cache/Speed, 19 = byte-Optimum, weniger Kollisionen, +2% Zeit) */
#define MMLEN2 7             /* R358: langer Match Kontextlaenge 7 (byte-getunt: R295 waehlte 6, 7 = byte-Optimum -426, 8 schlechter) */
#define MMBITS2 18            /* R359: langer Match 2^18 (byte-getunt getrennt: lange Kontexte rekurrieren seltener -> kleinere Tabelle optimal, byte -29 UND kleiner/schneller als 19; R357 koppelte beide) */

typedef struct {
    uint16_t *t[NCTX];   /* je Kontext: P(bit=1) in 0..4095, Start 2048 */
    int w[2048][NW];     /* R310: 256 (Vorbyte x Match) x 8 Bit-Positionen */
    int wb[2048][NW];    /* R312: 2. Mischer, Kontext Vor-Vorbyte x Bit-Position */
    int wc[2048][NW];    /* R314: 3. Mischer, Mittel-Kontext (gelernt gewichtet) */
    int wcx;             /* aktueller Gewichts-Kontext = Vorbyte */
    int wcx_eff;         /* R310: wcx*8 + Bit-Position (per-Bit Mischer-Kontext) */
    int wcxb_eff;        /* R312: 2. Mischer Kontext */
    int wcxc_eff;        /* R314: 3. Mischer Kontext */
    int gcomb[16][3];    /* R314: 3 gelernte Kombinier-Gewichte */
    int cdA, cdB, cdC, cbp;   /* R313/R314: fuer Kombinierer-Update */
    uint64_t hist;       /* R9: rollierende Byte-Historie, 8 Bytes (fuer order-6) */
    uint32_t bhist;      /* R89: rollierende BIT-Historie (cross-byte, ueber Byte-Grenzen) */
    uint16_t *jff;       /* R148: jpg FF-Distanz-Regime-Tabelle (6 Buckets × 2^g_jfw) */
    uint32_t jffctx;     /* R148: gemerkter jff-Index fuer das Update */
    uint16_t *jff2;      /* R149: 2. FF-Distanz-Regime-Tabelle (6 Buckets × 2^g_jfw2) */
    uint32_t jffctx2;    /* R149: gemerkter jff2-Index fuer das Update */
    int ffdist;          /* R148: Bytes seit letztem 0xFF-Datenbyte (gekappt 63) */
    uint16_t *jt;        /* R89: cross-byte Bit-Kontext-Tabelle 2^JB (jpg) */
    uint32_t jctx;       /* R89: gemerkter jt-Index fuer das Update */
    uint16_t *jt2;       /* R90: 2. cross-byte Bit-Kontext-Tabelle 2^JB2 (jpg) */
    uint32_t jctx2;      /* R90: gemerkter jt2-Index fuer das Update */
    uint16_t *jt3;       /* R91: 3. cross-byte Bit-Kontext-Tabelle 2^JB3 (jpg) */
    uint32_t jctx3;      /* R91: gemerkter jt3-Index fuer das Update */
    uint16_t *jt4;       /* R94: 4. cross-byte Bit-Kontext */
    uint32_t jctx4;
    uint16_t *jt5;       /* R96: 5. cross-byte Bit-Kontext */
    uint32_t jctx5;
    uint16_t *jt6;       /* R98: 6. cross-byte Bit-Kontext */
    uint32_t jctx6;
    uint16_t *jt7;       /* R99: 7. cross-byte Bit-Kontext */
    uint32_t jctx7;
    uint16_t *jt8;       /* R101: 8. cross-byte Bit-Kontext */
    uint32_t jctx8;
    uint16_t *jt9;       /* R102: 9. cross-byte Bit-Kontext */
    uint32_t jctx9;
    uint16_t *jt10;      /* R104: 10. cross-byte Bit-Kontext */
    uint32_t jctx10;
    uint16_t *jt11;      /* R105: 11. cross-byte Bit-Kontext */
    uint32_t jctx11;
    uint16_t *jt12;      /* R106: 12. cross-byte Bit-Kontext */
    uint16_t *jsp;       /* R113: sparse-bit cross-byte (zwei getrennte Bit-Gruppen) */
    uint32_t jspctx;     /* R113: gemerkter jsp-Index */
    uint16_t *jsp2;      /* R114: 2. sparse-bit Fenster (laengerer Offset) */
    uint32_t jsp2ctx;    /* R114: gemerkter jsp2-Index */
    uint16_t *jsp3;      /* R115: 3. sparse-bit Fenster (langes Ende) */
    uint32_t jsp3ctx;    /* R115: gemerkter jsp3-Index */
    uint16_t *jsp4;      /* R116: 4. sparse-bit Fenster (Mittel-Gap) */
    uint32_t jsp4ctx;    /* R116: gemerkter jsp4-Index */
    uint16_t *jsp5;      /* R118: 5. sparse-bit Fenster */
    uint32_t jsp5ctx;    /* R118: gemerkter jsp5-Index */
    uint16_t *jsp6;      /* R120: 6. sparse-bit Fenster */
    uint16_t *jsp7;      /* R122: 7. sparse-bit Fenster (asymmetrischer GR-Split) */
    uint32_t jsp7ctx;    /* R122: gemerkter jsp7-Index */
    uint16_t *jsp8;      /* R123: 8. sparse-bit Fenster (2. asymmetrischer GR-Split) */
    uint32_t jsp8ctx;    /* R123: gemerkter jsp8-Index */
    uint16_t *jsp9;      /* R124: 9. sparse-bit Fenster (3. asymmetrischer GR-Split, recent-heavy) */
    uint32_t jsp9ctx;    /* R124: gemerkter jsp9-Index */
    uint16_t *jsp10;     /* R125: 10. sparse-bit Fenster (2. recent-heavy Split) */
    uint32_t jsp10ctx;   /* R125: gemerkter jsp10-Index */
    uint16_t *jsp11;     /* R126: 11. sparse-bit Fenster (3-Gruppen-Kontext) */
    uint32_t jsp11ctx;   /* R126: gemerkter jsp11-Index */
    uint16_t *jsp12;     /* R127: 2. 3-Gruppen-Fenster */
    uint32_t jsp12ctx;   /* R127: gemerkter jsp12-Index */
    uint16_t *jsp13;     /* R128: 3. 3-Gruppen-Fenster */
    uint32_t jsp13ctx;   /* R128: gemerkter jsp13-Index */
    uint16_t *jsp14;     /* R130: 4-Gruppen-Kontext */
    uint32_t jsp14ctx;   /* R130: gemerkter jsp14-Index */
    uint16_t *jsp15;     /* R131: 2. 4-Gruppen-Fenster */
    uint32_t jsp15ctx;   /* R131: gemerkter jsp15-Index */
    uint16_t *jsp16;     /* R132: 3. 4-Gruppen-Fenster */
    uint32_t jsp16ctx;   /* R132: gemerkter jsp16-Index */
    uint16_t *jsp17;     /* R133: 4. 4-Gruppen-Fenster */
    uint32_t jsp17ctx;   /* R133: gemerkter jsp17-Index */
    uint16_t *jsp19;     /* R146: 5. 4-Gruppen-Fenster */
    uint32_t jsp19ctx;   /* R146: gemerkter jsp19-Index */
    uint16_t *jsp18;     /* R137: asym 3-Gruppen-Fenster */
    uint32_t jsp18ctx;   /* R137: gemerkter jsp18-Index */
    uint32_t jsp6ctx;    /* R120: gemerkter jsp6-Index */
    uint32_t jctx12;
    int c0;              /* Bit-Baum-Praefix (fuehrende 1) */
    uint32_t idx[NCTX];  /* aktuelle Tabellen-Indizes */
    int st[NW];          /* zwischengespeicherte stretch-Werte (+Match) */
    int pr;              /* letzte gemischte Vorhersage 0..4095 (Mischer-Ausgang) */
    uint32_t pa[NCTX];   /* R3: pro-Byte vorberechneter Hash-Teil (ohne c0) */
    uint16_t apm[256 * 33]; /* R12: SSE/APM — Nachschaerfung je c0-Kontext */
    int apm_index;       /* gemerkter APM-Knoten fuer das Update */
    uint16_t apm2[256 * 33]; /* R15: 2. APM-Stufe, Kontext = Vorbyte */
    int apm2_index;
    uint16_t apm3[17 * 33];  /* R318: 3. APM-Stufe, Kontext = Match-Laenge-Bucket x Vorhersage-Bit */
    int apm3_index;
    uint16_t apm4[8 * 33];    /* R381: 4. SSE-Stufe (full-binaer pdf/docx/xlsx), Kontext = 00-Struktur */
    int apm4_index;
    /* R13 Match-Modell ("Echo") */
    uint32_t *mm_hash;   /* Kontext-Hash -> letzte Folgeposition */
    const unsigned char *mm_buf; /* Verlauf (data beim Packen, out beim Entpacken) */
    uint32_t mm_pos;     /* aktuelle Byte-Position */
    uint32_t mm_ptr;     /* Vorhersage-Quelle: buf[mm_ptr] sagt aktuelles Byte voraus */
    int mm_len;          /* Match-Laenge */
    uint32_t *mm_hash2;  /* R295: 2. (langer) Match */
    uint32_t mm_ptr2; int mm_len2;
    /* R298 indirektes Bit-Historie-Modell */
    uint8_t *ihist;      /* Kontext(order-3 x Bit-Knoten) -> 8-Bit-Historie */
    uint16_t iprob[ISUB*256]; /* R302: (Bit-Position x Zustand) -> P(bit=1) */
    uint32_t i_hidx; int i_state; int i_sub;
    uint8_t *ihist2;     /* R299 2. indirektes Modell, komplementaerer Kontext */
    uint16_t iprob2[ISUB*256];
    uint32_t i_hidx2; int i_state2;
    uint8_t *ihist3;     /* R300 3. indirektes Modell */
    uint16_t iprob3[ISUB*256];
    uint32_t i_hidx3; int i_state3;
    uint8_t *ihist4;     /* R305 4. indirektes Modell */
    uint16_t iprob4[ISUB*256];
    uint32_t i_hidx4; int i_state4;
    void *arena_base; size_t arena_len; unsigned char *arena_cur;  /* ZEIT R209: Ein-mmap-Arena je Modell (THP) */
} Model;

static uint32_t hashc(uint32_t a, uint32_t b) {
    uint32_t h = a * 2654435761u + b * 2246822519u;
    h ^= h >> 15; h *= 2654435761u; h ^= h >> 13;
    return h;
}

static int g_jpg;   /* R141: Vorwaerts-Deklaration (echte Definition unten) — jt/jsp nur bei jpg anlegen */
static int g_jfw;   /* R148: Vorwaerts-Deklaration (echte Definition unten) — FF-Distanz-Fensterbreite fuer jff-Alloc */
static int g_jfw2;  /* R149: Vorwaerts-Deklaration — 2. FF-Distanz-Fensterbreite fuer jff2-Alloc */
/* ZEIT R209: alle Modell-Tabellen aus EINER mmap-Arena (MADV_HUGEPAGE) — 1 mmap statt ~40 malloc
 * (weniger mmap-Lock bei 14 parallelen Bloecken), 2MB-Pages: ~45 statt ~21000 Faults je Modell,
 * weniger TLB-Walks im random-access Hot-Loop. Werte/Fuellungen UNVERAENDERT -> byte-identisch. */
static void *arena_take(Model *m, size_t bytes) {
    size_t align = (bytes >= (2u<<20)) ? (size_t)(2u<<20) : 4096u;
    uintptr_t cur = ((uintptr_t)m->arena_cur + align - 1) & ~(uintptr_t)(align - 1);
    m->arena_cur = (unsigned char *)(cur + bytes);
    return (void *)cur;
}
static void model_init(Model *m) {
    {   /* Arena-Groesse: jede Tabelle + Align-Polster; ungenutzter Ueberhang wird nie beruehrt (kostet nichts) */
        size_t need = (size_t)4 << 20;
        for (int i = 0; i < g_nctx; i++) need += g_tsize * sizeof(uint16_t) + (2u<<20);
        need += 4 * (((size_t)1 << IBITS) + 4096);                       /* ihist1-4 */
        need += (((size_t)1 << MMBITS) + ((size_t)1 << MMBITS2)) * sizeof(uint32_t) + 2*4096 + 2*(2u<<20);
        if (g_jpg) {
            need += ((size_t)6 * ((size_t)1 << g_jfw) + (size_t)6 * ((size_t)1 << g_jfw2)) * sizeof(uint16_t) + 2*4096;
            int jbs[12] = {JB,JB2,JB3,JB4,JB5,JB6,JB7,JB8,JB9,g_jb10,g_jb11,g_jb12};
            for (int k = 0; k < 12; k++) need += (((size_t)1 << jbs[k]) * sizeof(uint16_t)) + (2u<<20);
            need += 6 * ((((size_t)1 << (2*g_jspgr)) * sizeof(uint16_t)) + 4096);
            need += (((size_t)1 << (g_jsp7gr1+g_jsp7gr2)) + ((size_t)1 << (g_jsp8gr1+g_jsp8gr2)) + ((size_t)1 << (g_jsp9gr1+g_jsp9gr2)) + ((size_t)1 << (g_jsp10gr1+g_jsp10gr2))) * sizeof(uint16_t) + 4*4096;
            need += (((size_t)1 << (g_jsp11gr1+g_jsp11gr2+g_jsp11gr3)) + ((size_t)1 << (g_jsp12gr1+g_jsp12gr2+g_jsp12gr3)) + ((size_t)1 << (g_jsp13gr1+g_jsp13gr2+g_jsp13gr3)) + ((size_t)1 << (g_jsp18gr1+g_jsp18gr2+g_jsp18gr3))) * sizeof(uint16_t) + 4*(2u<<20);
            need += (((size_t)1 << (g_jsp14gr1+g_jsp14gr2+g_jsp14gr3+g_jsp14gr4)) + ((size_t)1 << (g_jsp15gr1+g_jsp15gr2+g_jsp15gr3+g_jsp15gr4)) + ((size_t)1 << (g_jsp16gr1+g_jsp16gr2+g_jsp16gr3+g_jsp16gr4)) + ((size_t)1 << (g_jsp17gr1+g_jsp17gr2+g_jsp17gr3+g_jsp17gr4)) + ((size_t)1 << (g_jsp19gr1+g_jsp19gr2+g_jsp19gr3+g_jsp19gr4))) * sizeof(uint16_t) + 5*(2u<<20);
        }
        m->arena_len = need;
#ifdef _WIN32
        m->arena_base = calloc(1, need);               /* Windows: plain zeroed heap (wie anonymes mmap) */
        if (!m->arena_base) { fprintf(stderr, "welle_fast: arena alloc failed\n"); exit(3); }
#else
        m->arena_base = mmap(NULL, need, PROT_READ|PROT_WRITE, MAP_PRIVATE|MAP_ANONYMOUS, -1, 0);
        if (m->arena_base == MAP_FAILED) { fprintf(stderr, "welle_fast: arena mmap failed\n"); exit(3); }
#ifdef MADV_HUGEPAGE
        madvise(m->arena_base, need, MADV_HUGEPAGE);   /* 2MB-Pages: ~45 statt ~21000 Faults je Modell, weniger TLB-Walks */
#endif
#endif
        m->arena_cur = (unsigned char *)m->arena_base;
    }
    for (int i = 0; i < NCTX; i++) {
        if (i >= g_nctx) { m->t[i] = NULL; continue; } /* R143: tote Byte-Ordnungs-Tabellen (weak-Pfad g_nctx=3: jpg/stl) nicht anlegen/fuellen — Predict/Update-Loop i<g_nctx dereferenziert sie nie, byte-neutral (R141-Muster, ZEIT) */
        m->t[i] = arena_take(m, g_tsize * sizeof(uint16_t));
        for (size_t j = 0; j < g_tsize; j++) m->t[i][j] = 2048;
    }
    for (int cx = 0; cx < 2048; cx++) {
        for (int i = 0; i < NW; i++) m->wb[cx][i] = 0;         /* R312: 2. Mischer neutral */
        for (int i = 0; i < NCTX; i++) m->wb[cx][i] = 65536 / NCTX;
        for (int i = 0; i < NW; i++) m->wc[cx][i] = 0;         /* R314: 3. Mischer neutral */
        for (int i = 0; i < NCTX; i++) m->wc[cx][i] = 65536 / NCTX;
        for (int i = 0; i < NCTX; i++) m->w[cx][i] = 65536 / NCTX;
        m->w[cx][MI] = 0;   /* R13: Match-Gewicht startet neutral, lernt sich ein */
        m->w[cx][MI2] = 0;  /* R295: 2. Match-Gewicht neutral */
        m->w[cx][IND] = 0;  /* R298: indirektes Modell neutral */
        m->w[cx][IND2] = 0; /* R299: 2. indirektes Modell neutral */
        m->w[cx][IND3] = 0; /* R300: 3. indirektes Modell neutral */
        m->w[cx][IND4] = 0; /* R305: 4. indirektes Modell neutral */
        m->w[cx][JBIT] = 0; m->w[cx][JBIT2] = 0; m->w[cx][JBIT3] = 0; m->w[cx][JBIT4] = 0; m->w[cx][JBIT5] = 0; m->w[cx][JBIT6] = 0; m->w[cx][JBIT7] = 0; m->w[cx][JBIT8] = 0; m->w[cx][JBIT9] = 0; m->w[cx][JBIT10] = 0; m->w[cx][JBIT11] = 0; m->w[cx][JBIT12] = 0; m->w[cx][JSP] = 0; m->w[cx][JSP2] = 0; m->w[cx][JSP3] = 0; m->w[cx][JSP4] = 0; m->w[cx][JSP5] = 0; m->w[cx][JSP6] = 0; m->w[cx][JSP7] = 0; m->w[cx][JSP8] = 0; m->w[cx][JSP9] = 0; m->w[cx][JSP10] = 0; m->w[cx][JSP11] = 0; m->w[cx][JSP12] = 0; m->w[cx][JSP13] = 0; m->w[cx][JSP14] = 0; m->w[cx][JSP15] = 0; m->w[cx][JSP16] = 0; m->w[cx][JSP17] = 0; m->w[cx][JSP18] = 0; m->w[cx][JSP19] = 0; m->w[cx][JFF] = 0; m->w[cx][JFF2] = 0; /* R92..R106: jpg cross-byte auf 1. Mischer; R113-146: JSP..JSP19 sparse-bit; R148: JFF FF-Distanz; R149: JFF2 */
    }
    m->ihist = arena_take(m, (size_t)1 << IBITS);   /* Arena ist kernel-zero -> calloc-gleich */
    m->ihist2 = arena_take(m, (size_t)1 << IBITS);
    m->ihist3 = arena_take(m, (size_t)1 << IBITS);
    m->ihist4 = arena_take(m, (size_t)1 << IBITS);
    for (int j = 0; j < ISUB*256; j++) { m->iprob[j] = 2048; m->iprob2[j] = 2048; m->iprob3[j] = 2048; m->iprob4[j] = 2048; }
    m->mm_hash = arena_take(m, ((size_t)1 << MMBITS) * sizeof(uint32_t));
    m->mm_hash2 = arena_take(m, ((size_t)1 << MMBITS2) * sizeof(uint32_t));
    if (g_jpg) {   /* R141: cross-byte jt/jsp-Tabellen (~50MB, K bis 24) NUR bei jpg anlegen+fuellen; die 15 Nicht-jpg lasen sie nie (Praedikt/Update g_jpg-gegatet) -> byte-identisch, spart Alloc+Init-Fuellung je Block */
    m->jff = arena_take(m, (size_t)6 * ((size_t)1 << g_jfw) * sizeof(uint16_t));  /* R148: FF-Distanz-Regime 6 Buckets × 2^g_jfw */
    for (size_t j = 0; j < (size_t)6 * ((size_t)1 << g_jfw); j++) m->jff[j] = 2048;
    m->jff2 = arena_take(m, (size_t)6 * ((size_t)1 << g_jfw2) * sizeof(uint16_t));  /* R149: 2. FF-Distanz-Regime 6 Buckets × 2^g_jfw2 */
    for (size_t j = 0; j < (size_t)6 * ((size_t)1 << g_jfw2); j++) m->jff2[j] = 2048;
    m->jt = arena_take(m, ((size_t)1 << JB) * sizeof(uint16_t));  /* R89: cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB); j++) m->jt[j] = 2048;
    m->jt2 = arena_take(m, ((size_t)1 << JB2) * sizeof(uint16_t)); /* R90: 2. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB2); j++) m->jt2[j] = 2048;
    m->jt3 = arena_take(m, ((size_t)1 << JB3) * sizeof(uint16_t)); /* R91: 3. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB3); j++) m->jt3[j] = 2048;
    m->jt4 = arena_take(m, ((size_t)1 << JB4) * sizeof(uint16_t)); /* R94: 4. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB4); j++) m->jt4[j] = 2048;
    m->jt5 = arena_take(m, ((size_t)1 << JB5) * sizeof(uint16_t)); /* R96: 5. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB5); j++) m->jt5[j] = 2048;
    m->jt6 = arena_take(m, ((size_t)1 << JB6) * sizeof(uint16_t)); /* R98: 6. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB6); j++) m->jt6[j] = 2048;
    m->jt7 = arena_take(m, ((size_t)1 << JB7) * sizeof(uint16_t)); /* R99: 7. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB7); j++) m->jt7[j] = 2048;
    m->jt8 = arena_take(m, ((size_t)1 << JB8) * sizeof(uint16_t)); /* R101: 8. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB8); j++) m->jt8[j] = 2048;
    m->jt9 = arena_take(m, ((size_t)1 << JB9) * sizeof(uint16_t)); /* R102: 9. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << JB9); j++) m->jt9[j] = 2048;
    m->jt10 = arena_take(m, ((size_t)1 << g_jb10) * sizeof(uint16_t)); /* R104: 10. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << g_jb10); j++) m->jt10[j] = 2048;
    m->jt11 = arena_take(m, ((size_t)1 << g_jb11) * sizeof(uint16_t)); /* R105: 11. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << g_jb11); j++) m->jt11[j] = 2048;
    m->jt12 = arena_take(m, ((size_t)1 << g_jb12) * sizeof(uint16_t)); /* R106: 12. cross-byte Bit-Kontext */
    for (size_t j = 0; j < ((size_t)1 << g_jb12); j++) m->jt12[j] = 2048;
    { size_t jspn = (size_t)1 << (2*g_jspgr);   /* R113: sparse-bit Tabelle 2^(2*GR) */
      m->jsp = arena_take(m, jspn * sizeof(uint16_t));
      for (size_t j = 0; j < jspn; j++) m->jsp[j] = 2048;
      m->jsp2 = arena_take(m, jspn * sizeof(uint16_t));   /* R114: 2. sparse-bit Fenster */
      for (size_t j = 0; j < jspn; j++) m->jsp2[j] = 2048;
      m->jsp3 = arena_take(m, jspn * sizeof(uint16_t));   /* R115: 3. sparse-bit Fenster */
      for (size_t j = 0; j < jspn; j++) m->jsp3[j] = 2048;
      m->jsp4 = arena_take(m, jspn * sizeof(uint16_t));   /* R116: 4. sparse-bit Fenster */
      for (size_t j = 0; j < jspn; j++) m->jsp4[j] = 2048;
      m->jsp5 = arena_take(m, jspn * sizeof(uint16_t));   /* R118: 5. sparse-bit Fenster */
      for (size_t j = 0; j < jspn; j++) m->jsp5[j] = 2048;
      m->jsp6 = arena_take(m, jspn * sizeof(uint16_t));   /* R120: 6. sparse-bit Fenster */
      for (size_t j = 0; j < jspn; j++) m->jsp6[j] = 2048;
      size_t jsp7n = (size_t)1 << (g_jsp7gr1 + g_jsp7gr2);   /* R122: 7. Fenster asymmetrische Tabelle 2^(GR1+GR2) */
      m->jsp7 = arena_take(m, jsp7n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp7n; j++) m->jsp7[j] = 2048;
      size_t jsp8n = (size_t)1 << (g_jsp8gr1 + g_jsp8gr2);   /* R123: 8. Fenster asymmetrische Tabelle 2^(GR1+GR2) */
      m->jsp8 = arena_take(m, jsp8n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp8n; j++) m->jsp8[j] = 2048;
      size_t jsp9n = (size_t)1 << (g_jsp9gr1 + g_jsp9gr2);   /* R124: 9. Fenster asymmetrische Tabelle 2^(GR1+GR2) */
      m->jsp9 = arena_take(m, jsp9n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp9n; j++) m->jsp9[j] = 2048;
      size_t jsp10n = (size_t)1 << (g_jsp10gr1 + g_jsp10gr2);   /* R125: 10. Fenster asymmetrische Tabelle */
      m->jsp10 = arena_take(m, jsp10n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp10n; j++) m->jsp10[j] = 2048;
      size_t jsp11n = (size_t)1 << (g_jsp11gr1 + g_jsp11gr2 + g_jsp11gr3);   /* R126: 11. Fenster 3-Gruppen-Tabelle */
      m->jsp11 = arena_take(m, jsp11n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp11n; j++) m->jsp11[j] = 2048;
      size_t jsp12n = (size_t)1 << (g_jsp12gr1 + g_jsp12gr2 + g_jsp12gr3);   /* R127: 2. 3-Gruppen-Tabelle */
      m->jsp12 = arena_take(m, jsp12n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp12n; j++) m->jsp12[j] = 2048;
      size_t jsp13n = (size_t)1 << (g_jsp13gr1 + g_jsp13gr2 + g_jsp13gr3);   /* R128: 3. 3-Gruppen-Tabelle */
      m->jsp13 = arena_take(m, jsp13n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp13n; j++) m->jsp13[j] = 2048;
      size_t jsp14n = (size_t)1 << (g_jsp14gr1 + g_jsp14gr2 + g_jsp14gr3 + g_jsp14gr4);   /* R130: 4-Gruppen-Tabelle */
      m->jsp14 = arena_take(m, jsp14n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp14n; j++) m->jsp14[j] = 2048;
      size_t jsp15n = (size_t)1 << (g_jsp15gr1 + g_jsp15gr2 + g_jsp15gr3 + g_jsp15gr4);   /* R131: 2. 4-Gruppen-Tabelle */
      m->jsp15 = arena_take(m, jsp15n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp15n; j++) m->jsp15[j] = 2048;
      size_t jsp16n = (size_t)1 << (g_jsp16gr1 + g_jsp16gr2 + g_jsp16gr3 + g_jsp16gr4);   /* R132: 3. 4-Gruppen-Tabelle */
      m->jsp16 = arena_take(m, jsp16n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp16n; j++) m->jsp16[j] = 2048;
      size_t jsp17n = (size_t)1 << (g_jsp17gr1 + g_jsp17gr2 + g_jsp17gr3 + g_jsp17gr4);   /* R133: 4. 4-Gruppen-Tabelle */
      m->jsp17 = arena_take(m, jsp17n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp17n; j++) m->jsp17[j] = 2048;
      size_t jsp19n = (size_t)1 << (g_jsp19gr1 + g_jsp19gr2 + g_jsp19gr3 + g_jsp19gr4);   /* R146: 5. 4-Gruppen-Tabelle */
      m->jsp19 = arena_take(m, jsp19n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp19n; j++) m->jsp19[j] = 2048;
      size_t jsp18n = (size_t)1 << (g_jsp18gr1 + g_jsp18gr2 + g_jsp18gr3);   /* R137: asym 3-Gruppen-Tabelle */
      m->jsp18 = arena_take(m, jsp18n * sizeof(uint16_t));
      for (size_t j = 0; j < jsp18n; j++) m->jsp18[j] = 2048; }
    }   /* R141: Ende g_jpg-gegatete jt/jsp-Anlage */
    m->bhist = 0;
    m->ffdist = 63;   /* R148: Start weit weg vom letzten FF */
    m->mm_buf = NULL; m->mm_pos = 0; m->mm_ptr = 0; m->mm_len = 0; m->mm_ptr2 = 0; m->mm_len2 = 0;
    /* R12: APM-Knoten mit der Identitaets-Kurve (squash) vorbelegen -> anfangs
     * gibt die Nachschaerfung die Eingabe unveraendert zurueck, lernt dann Bias. */
    for (int cx = 0; cx < 256; cx++)
        for (int j = 0; j < 33; j++) {
            int d = (j - 16) * 128, idx = d + 2048;
            if (idx < 0) idx = 0; else if (idx > 4095) idx = 4095;
            m->apm[cx * 33 + j] = m->apm2[cx * 33 + j] = (uint16_t)(SQUASH[idx] << 4);
            if (cx < 17) m->apm3[cx * 33 + j] = (uint16_t)(SQUASH[idx] << 4);  /* R318: identisch initialisiert = Durchlass */
            if (cx < 8) m->apm4[cx * 33 + j] = (uint16_t)(SQUASH[idx] << 4);  /* R381 */
        }
    m->hist = 0;
    m->c0 = 1;
    m->wcx = 0;
    for (int b = 0; b < 16; b++) { m->gcomb[b][0] = 32768; m->gcomb[b][1] = 32768; m->gcomb[b][2] = 0; }  /* R314: 3. Arm Start 0 = worst case R313 */
    m->apm_index = 0;
    m->apm3_index = 0;
    m->apm4_index = 0;
}

static void model_free(Model *m) {
#ifdef _WIN32
    free(m->arena_base);                   /* ZEIT R209: alle Tabellen leben in der Arena */
#else
    munmap(m->arena_base, m->arena_len);   /* ZEIT R209: alle Tabellen leben in der Arena */
#endif
    if (0) {
    for (int i = 0; i < NCTX; i++) free(m->t[i]);
    free(m->mm_hash); free(m->mm_hash2); free(m->ihist); free(m->ihist2); free(m->ihist3); free(m->ihist4);
    if (g_jpg) { free(m->jff); free(m->jff2); free(m->jt); free(m->jt2); free(m->jt3); free(m->jt4); free(m->jt5); free(m->jt6); free(m->jt7); free(m->jt8); free(m->jt9); free(m->jt10); free(m->jt11); free(m->jt12); free(m->jsp); free(m->jsp2); free(m->jsp3); free(m->jsp4); free(m->jsp5); free(m->jsp6); free(m->jsp7); free(m->jsp8); free(m->jsp9); free(m->jsp10); free(m->jsp11); free(m->jsp12); free(m->jsp13); free(m->jsp14); free(m->jsp15); free(m->jsp16); free(m->jsp17); free(m->jsp18); free(m->jsp19); }   /* R141: nur bei jpg angelegt */
    }
}

/* R12/R15: eine APM-Stufe anwenden (interpolieren) und Update-Knoten merken. */
static int apm_do(uint16_t *tab, int cx, int pr, int *idx_out) {
    int s = STRETCH[pr] + 2048;
    if (s < 0) s = 0; else if (s > 4094) s = 4094;
    int ai = s >> 7, aw = s & 127;
    uint16_t *ap = &tab[cx * 33];
    *idx_out = cx * 33 + ai + (aw >> 6);
    int r = (ap[ai] * (128 - aw) + ap[ai + 1] * aw) >> 11;
    return r < 1 ? 1 : (r > 4094 ? 4094 : r);
}
static void apm_up(uint16_t *tab, int idx, int bit) {
    int v = tab[idx];
    v += ((bit << 16) - v) >> 7;
    tab[idx] = (uint16_t)v;
}

/* R13: Hash der MMLEN Bytes, die bei Position pos enden. */
static uint32_t mm_hash_at(const unsigned char *buf, uint32_t pos) {
    uint32_t h = 0;
    for (int j = 0; j < MMLEN; j++) h = h * 0x9E3779B1u + buf[pos - j];
    h *= 2654435761u;
    return h >> (32 - MMBITS);
}
static uint32_t mm_hash_at2(const unsigned char *buf, uint32_t pos) {
    uint32_t h = 0;
    for (int j = 0; j < MMLEN2; j++) h = h * 0x9E3779B1u + buf[pos - j];
    h *= 2654435761u;
    return h >> (32 - MMBITS2);
}
/* R295: Match-Eingang berechnen (kurz oder lang, gleiche Logik). */
static int match_input(int mlen, uint32_t mptr, const unsigned char *buf, int c0) {
    if (mlen <= 0) return 0;
    int E = buf[mptr];
    int k = 31 - __builtin_clz((unsigned)c0);
    if ((E >> (8 - k)) == (c0 - (1 << k))) {
        int pb = (E >> (7 - k)) & 1;
        int mag = mlen; if (mag > 32) mag = 32;
        int sm = pb ? mag * 64 : -mag * 64;
        if (sm > 2047) sm = 2047; else if (sm < -2047) sm = -2047;
        return sm;
    }
    return 0;
}

/* R13: Match-Zustand am Byte-ENDE fortschreiben. buf[pos] muss geschrieben sein
 * (Encoder: data; Decoder: out, VOR diesem Aufruf gesetzt). */
static void model_byte_end(Model *m, int b, uint32_t pos) {
    if (m->mm_len > 0) {                       /* Match fortsetzen oder brechen */
        if (m->mm_buf[m->mm_ptr] == (unsigned char)b) {
            m->mm_ptr++; if (m->mm_len < 65535) m->mm_len++;
        } else m->mm_len = 0;
    }
    if (pos + 1 >= MMLEN) {                     /* Kontext-Hash aktualisieren */
        uint32_t h = mm_hash_at(m->mm_buf, pos);
        uint32_t prev = m->mm_hash[h];
        m->mm_hash[h] = pos + 1;
        if (m->mm_len == 0 && prev != 0 && prev >= MMLEN) {
            /* R17: Kontext verifizieren (Hash-Kollisionen filtern) -> nur echte Echos.
             * Verifiziert kann die Startkonfidenz hoeher sein (mm_len=12 gesweept). */
            int ok = 1;
            for (int j = 0; j < MMLEN; j++)
                if (m->mm_buf[prev - 1 - j] != m->mm_buf[pos - j]) { ok = 0; break; }
            if (ok) { m->mm_ptr = prev; m->mm_len = 12; }
        }
    }
    /* R295: langer Match (MMLEN2) analog, eigene Hashtabelle */
    if (m->mm_len2 > 0) {
        if (m->mm_buf[m->mm_ptr2] == (unsigned char)b) { m->mm_ptr2++; if (m->mm_len2 < 65535) m->mm_len2++; }
        else m->mm_len2 = 0;
    }
    if (pos + 1 >= MMLEN2) {
        uint32_t h = mm_hash_at2(m->mm_buf, pos);
        uint32_t prev = m->mm_hash2[h];
        m->mm_hash2[h] = pos + 1;
        if (m->mm_len2 == 0 && prev != 0 && prev >= MMLEN2) {
            int ok = 1;
            for (int j = 0; j < MMLEN2; j++)
                if (m->mm_buf[prev - 1 - j] != m->mm_buf[pos - j]) { ok = 0; break; }
            if (ok) { m->mm_ptr2 = prev; m->mm_len2 = 12; }
        }
    }
    m->mm_pos = pos + 1;
}

/* Kontext am Byte-Anfang: den c0-UNABHAENGIGEN Teil des Hashes EINMAL pro Byte
 * vorberechnen (R3). hashc(a,b)=a*C1+b*C2->avalanche; a haengt nur an hist (pro
 * Byte konstant), b=c0 (pro Bit). pa[i]=a*C1 -> im Bit-Loop nur noch +c0*C2 und
 * die Avalanche. Byte-IDENTISCHE Ausgabe, aber ~halbe Hash-Arithmetik pro Bit. */
/* R10: Kontext-Ordnungen (Byte-Reichweiten) als Satz stimmbar. "Welle"-Idee:
 * laengere Nachbar-Reichweite greift glatte/strukturierte Muster (R9: order-6
 * kippte stl). NCTX bleibt 6 -> keine Rechenzeit-Kosten. */
static const int ORDERS[NCTX] = {0,1,2,3,4,101,105,104};  /* R297: 5 dicht + Sparse -2-4 / -2-6 / -3-5 */
static const int ORDERS_W2[NCTX] = {0,2,6,3,4,101,105,104}; /* R369/R370: weak-Pfad ASCII-Variante (erste g_nctx=3 = {0,2,6}; order-6 faengt die CAD-Record-Struktur besser als order-4, R370-Sweep) */
static const int ORDERS_H[NCTX] = {0,1,2,3,121,130,107,105}; /* R61: heic 8 Kontexte (4 dicht 0-3 + 4 Sparse -12,-16/-24,-32/-4,-8/-2,-6) via g_nctx=8 (Voll-Pfad-Sentinel g_nctx==NCTX entkoppelt per !g_heic-Guards); heic-only -4B */ /* R59: heic Multi-Block-Raster ueber mm_buf — -4,-8 (1 Block R32/58) + -12,-16 (2 Bloecke) + -24,-32 (3-4 Bloecke); der Block-Raster reicht ueber die 8 hist-Bytes hinaus, mm_buf erreicht -16/-24/-32; heic-only Sweep -72B */ /* R58: heic ~8-Byte-Block-Periode -> -8-Anker-Strides (-2,-6 R32 + -4,-8/-6,-8 neu); heic-only Sweep -35B */ /* R32: heic (block-strukturiert) will REICHE Stride-Kontexte — Dichte 0-3 + Sparse -2,-6/-3,-5/-2,-4; g_nctx=7 (Sweep: nc4 1674862 -> nc6 1674851 -> nc7 1674848 abnehmend); nur heic via Magic getrennt */
static int g_word = 0;  /* R369: weak-Ordnungs-Variante (0={0,1,2} Rauschen, 1={0,2,4} ASCII-Text), per-Datei per Flag */
static int g_jpg = 0;   /* R89: jpg (Magic FFD8) -> cross-byte Bit-Historie-Eingang (JBIT); nur jpg-gegatet, alle anderen byte-identisch */
static int g_jfw = 13;   /* R148: jpg FF-Distanz-Regime kurzes Bit-Fenster (Bits von bhist neben dem FF-Distanz-Bucket); Sweep 6..10 */
static int g_jfw2 = 5;   /* R149: 2. FF-Distanz-Fensterbreite (K-Ladder, komplementaer zu g_jfw); Sweep */
static const int g_sstep = 472;  /* R103: jpg Konfidenz-SSE-Aggregat-Schritt; R117: 13-Fenster; R119: 14-Fenster (9 contiguous + 5 sparse) re-getunt, flache Basin 470-475 = -43, Peak steigt mit groesserem Aggregat */
static int g_gate = 100;   /* R107: jpg cross-byte-KONFIDENZ Mischer-GATE-Schwelle (wcxb-Bit 0x100 in hoch-Konfidenz-Regionen; Sweep-Optimum 100, flache Basin 0-200; -347B) */
static int g_gatemask = 1; /* R107: nur wcxb gegatet (mask=1 Sweep-Optimum; wcxc/beide schlechter) */
static int g_heic = 0;  /* R30 (Block-1 R30): weak-BINAER heic (Magic ftypheic) will EXTRA dichte Ordnung g_nctx=4 (order-3 -13B, R388); jpg (FFD8) hasst sie (+889) -> per Magic-Byte-Klassifikator getrennt (kein Format-Dekode, nur Kontexttiefe-Wahl), Flag 4 traegt die Wahl */
static int heic_nctx = 8; /* R61: 8. Kontext (-2,-6) via g_weak-Entkopplung, heic-only -4B */ /* R61: heic-Kontextzahl (Sweep 7 vs 8) */
#define HB(k) (m->mm_pos>=(uint32_t)(k)?m->mm_buf[m->mm_pos-(k)]:0)
#define HEICMIX (HB(8)==HB(16))  /* R60: heic block-stationaer (-8==-16 Block-Raster) als Mischer-0x80-Bit; prev==0xFF war fuer HEVC verschwendet (R378), COMBINATION-Achse orthogonal zu den Direkt-Strides R32/58/59 */
static void model_new_byte_ctx(Model *m) {
    for (int i = 0; i < g_nctx; i++) {
        int o = (g_heic ? ORDERS_H : (g_nctx != NCTX && g_word) ? ORDERS_W2 : ORDERS)[i];  /* R32: heic eigene Stride-reiche Kontext-Wahl */
        uint64_t hh = (o == 0) ? 0
                    : (o == 100) ? ((m->hist >> 8) & 0xFFFFull)                                   /* R297 sparse: Byte -2,-3 (skip -1) */
                    : (o == 101) ? (((m->hist >> 8) & 0xFFull) | (((m->hist >> 24) & 0xFFull) << 8)) /* R297 sparse: -2,-4 */
                    : (o == 102) ? ((m->hist & 0xFFull) | (((m->hist >> 16) & 0xFFull) << 8))       /* R297 sparse: -1,-3 */
                    : (o == 103) ? (((m->hist >> 8) & 0xFFull) | (((m->hist >> 24) & 0xFFull) << 8) | (((m->hist >> 40) & 0xFFull) << 16)) /* R297 sparse: -2,-4,-6 stride2 */
                    : (o == 104) ? (((m->hist >> 16) & 0xFFull) | (((m->hist >> 32) & 0xFFull) << 8)) /* R297 sparse: -3,-5 */
                    : (o == 105) ? (((m->hist >> 8) & 0xFFull) | (((m->hist >> 40) & 0xFFull) << 8))  /* R297 sparse: -2,-6 */
                    : (o == 106) ? (((m->hist >> 16) & 0xFFull) | (((m->hist >> 40) & 0xFFull) << 8)) /* R297 sparse: -3,-6 */
                    : (o == 107) ? (((m->hist >> 24) & 0xFFull) | (((m->hist >> 56) & 0xFFull) << 8)) /* R297 sparse: -4,-8 */
                    : (o == 109) ? (((m->hist >> 40) & 0xFFull) | (((m->hist >> 56) & 0xFFull) << 8)) /* R58 sparse: -6,-8 (heic ~8-Byte-Block-Periode) */
                    : (o == 108) ? (((m->hist >> 8) & 0xFFull) | (((m->hist >> 24) & 0xFFull) << 8) | (((m->hist >> 40) & 0xFFull) << 16) | (((m->hist >> 56) & 0xFFull) << 24)) /* R297 sparse: -2,-4,-6,-8 */
                    : (o == 121) ? ((m->mm_pos>=12?(uint64_t)m->mm_buf[m->mm_pos-12]:0) | ((m->mm_pos>=16?(uint64_t)m->mm_buf[m->mm_pos-16]:0)<<8)) /* R59 -12,-16 (heic 2-Block-Raster via mm_buf, reaching past -8) */
                    : (o == 130) ? ((m->mm_pos>=24?(uint64_t)m->mm_buf[m->mm_pos-24]:0) | ((m->mm_pos>=32?(uint64_t)m->mm_buf[m->mm_pos-32]:0)<<8)) /* R59 -24,-32 (heic 3-4-Block-Raster) */
                    : (o >= 8) ? m->hist
                    : (m->hist & ((1ull << (8 * o)) - 1));
        uint32_t folded = (uint32_t)hh ^ (uint32_t)(hh >> 32);
        m->pa[i] = ((uint32_t)(i * 0x9E3779B1u) ^ folded) * 2654435761u;
    }
    m->wcx = (int)(m->hist & 0x1Fu) | ((m->mm_len>0||m->mm_len2>0)?0x20:0) | (((m->hist&0xFFu)==((m->hist>>8)&0xFFu))?0x40:0) | (( (g_word && ((int)((m->hist&0xFFu)-((m->hist>>8)&0xFFu)) > (int)(((m->hist>>8)&0xFFu)-((m->hist>>16)&0xFFu)))) || (!g_word && (g_heic ? (HEICMIX) : (g_nctx==NCTX) ? ((m->hist&0xFFu)==0x00u) : ((m->hist&0xFFu)==0xFFu))) )?0x80:0) /* R374/R377: 0x80-Bit = ASCII-Kruemmung (g_word) ODER binaer-Struktur — full-Pfad (pdf/docx) prev==0x00, weak-Pfad (jpg/heic) prev==0xFF; freier Slot je g_word+Pfad */; /* R361 Lauf-Bit + R371 g_word-gegatetes KRUEMMUNGS-Bit (Delta steigend, nur ASCII/stl; wcx_eff max 2047<2048) */  /* R310: 32 Vorbyte x Match-Aktiv x 8 Bit-Position (best of sweep)
                                       * gesweept: mehr Saetze fragmentieren kleine Dateien). */
}

static int model_predict(Model *m) {
    int dot = 0, dotb = 0, dotc = 0;
    m->wcx_eff = (m->wcx << 3) | (31 - __builtin_clz((unsigned)m->c0));  /* R310: + Bit-Position 0..7 */
    m->wcxb_eff = (int)(((m->hist>>48)&0x1Full) << 3) | (m->wcx_eff & 7);  /* R312: order-7 (fern-dekorreliert von Stufe-1) x Bit-Position */
    m->wcxc_eff = (int)(((m->hist>>24)&0x1Full) << 3) | (m->wcx_eff & 7);  /* R314: order-4 Mittel-Kontext (fuellt zwischen Stufe-1-nah und Stufe-2-order-7-fern) */
    uint32_t c0c2 = (uint32_t)m->c0 * 2246822519u;
    for (int i = 0; i < g_nctx; i++) {
        uint32_t h = m->pa[i] + c0c2;      /* == hashc(a, c0), nur faktorisiert */
        h ^= h >> 15; h *= 2654435761u; h ^= h >> 13;
        uint32_t ix = h & g_tmask;
        m->idx[i] = ix;
        int p = m->t[i][ix] & PMASK;   /* untere 12 Bit = Wahrscheinlichkeit */
        int s = STRETCH[p];
        m->st[i] = s;
        dot += m->w[m->wcx_eff][i] * s;
        dotb += m->wb[m->wcxb_eff][i] * s;
        dotc += m->wc[m->wcxc_eff][i] * s;
    }
    /* R13: Match-Eingang ("Echo"). Wenn ein Match laeuft und das vorhergesagte Byte
     * mit den bisher codierten Bits (c0) uebereinstimmt, sagt es das naechste Bit
     * voraus; Staerke waechst mit der Match-Laenge. Sonst 0 (kein Beitrag). */
    m->st[MI]  = match_input(m->mm_len,  m->mm_ptr,  m->mm_buf, m->c0);
    m->st[MI2] = match_input(m->mm_len2, m->mm_ptr2, m->mm_buf, m->c0);
    dot += m->w[m->wcx_eff][MI]  * m->st[MI];
    dotb += m->wb[m->wcxb_eff][MI]  * m->st[MI];
    dotc += m->wc[m->wcxc_eff][MI]  * m->st[MI];
    dot += m->w[m->wcx_eff][MI2] * m->st[MI2];
    dotb += m->wb[m->wcxb_eff][MI2] * m->st[MI2];
    dotc += m->wc[m->wcxc_eff][MI2] * m->st[MI2];
    /* R77: heic BIT-ECHO — der Block-Raster-Byte HB(8) sagt das AKTUELLE Byte BIT-fuer-BIT
     * voraus (unter dem Byte: Echo-Praediktor statt Wert-Kontext), Staerke = Block-
     * Stationaritaet (HB(8)==HB(16)/HB(24)). Nur heic-gegatet -> st[MI3]=0 sonst ->
     * die 12 Nicht-heic byte-IDENTISCH (w*0=0, err*0=0). */
    if (g_heic && m->mm_pos >= 8) {
        int E8 = m->mm_buf[m->mm_pos - 8];
        int conf = 4;
        if (m->mm_pos >= 16 && m->mm_buf[m->mm_pos - 16] == E8) conf += 16;
        if (m->mm_pos >= 24 && m->mm_buf[m->mm_pos - 24] == E8) conf += 16;
        m->st[MI3] = match_input(conf, m->mm_pos - 8, m->mm_buf, m->c0);
    } else m->st[MI3] = 0;
    if (g_heic) {   /* ZEIT R210: byte-neutral — st[MI3]=0 fuer die 16 Nicht-heic, w*0=+0 auf alle 3 Mischer (R208-Muster) */
    dot  += m->w[m->wcx_eff][MI3]   * m->st[MI3];
    dotb += m->wb[m->wcxb_eff][MI3] * m->st[MI3];
    dotc += m->wc[m->wcxc_eff][MI3] * m->st[MI3];
    }
    /* R298: indirektes Bit-Historie-Modell — Kontext (order-2 x Bit-Knoten) waehlt einen
     * 8-Bit-Historie-Zustand; eine adaptive Karte Zustand->P sagt das Bit voraus (Lupe fuer
     * schwache sub-Byte-Struktur im entropie-codierten Strom). */
    {
        m->i_sub = m->c0 & 0xFF;   /* R302: voller Bit-Baum-Knoten (256) als iprob-Subkontext */
        uint32_t hh = (uint32_t)(m->hist & 0xFFFFFFu) * 2654435761u ^ (uint32_t)m->c0 * 2246822519u;
        hh ^= hh >> 15;
        m->i_hidx = hh & (((uint32_t)1 << IBITS) - 1);
        m->i_state = m->ihist[m->i_hidx];
        int ip = m->iprob[m->i_sub*256 + m->i_state] & 4095;
        m->st[IND] = STRETCH[ip];
        dot += m->w[m->wcx_eff][IND] * m->st[IND];
        dotb += m->wb[m->wcxb_eff][IND] * m->st[IND];
        dotc += m->wc[m->wcxc_eff][IND] * m->st[IND];
    }
    {   /* R299: 2. indirektes Modell, komplementaerer Kontext order-5 */
        uint64_t hc = m->hist & 0xFFFFFFFFFFull;
        uint32_t hh = ((uint32_t)hc * 2654435761u ^ (uint32_t)(hc >> 32) * 40503u) ^ (uint32_t)m->c0 * 2246822519u;
        hh ^= hh >> 15;
        m->i_hidx2 = hh & (((uint32_t)1 << IBITS) - 1);
        m->i_state2 = m->ihist2[m->i_hidx2];
        int ip2 = m->iprob2[m->i_sub*256 + m->i_state2] & 4095;
        m->st[IND2] = STRETCH[ip2];
        dot += m->w[m->wcx_eff][IND2] * m->st[IND2];
        dotb += m->wb[m->wcxb_eff][IND2] * m->st[IND2];
        dotc += m->wc[m->wcxc_eff][IND2] * m->st[IND2];
    }
    {   /* R300: 3. indirektes Modell, SPARSE-Kontext (Bytes -2,-4,-6) x Bit-Knoten */
        uint32_t sc = (uint32_t)((m->hist >> 8) & 0xFFull) | (uint32_t)(((m->hist >> 24) & 0xFFull) << 8) | (uint32_t)(((m->hist >> 40) & 0xFFull) << 16);
        uint32_t hh = sc * 2654435761u ^ (uint32_t)m->c0 * 2246822519u;
        hh ^= hh >> 15;
        m->i_hidx3 = hh & (((uint32_t)1 << IBITS) - 1);
        m->i_state3 = m->ihist3[m->i_hidx3];
        int ip3 = m->iprob3[m->i_sub*256 + m->i_state3] & 4095;
        m->st[IND3] = STRETCH[ip3];
        dot += m->w[m->wcx_eff][IND3] * m->st[IND3];
        dotb += m->wb[m->wcxb_eff][IND3] * m->st[IND3];
        dotc += m->wc[m->wcxc_eff][IND3] * m->st[IND3];
    }
    {   /* R305: 4. indirektes Modell, Kontext gesweept */
        uint32_t c4 = (uint32_t)(m->hist & 0xFFull);   /* R305: order-1 (bester 4. Kontext) */
        uint32_t hh = c4 * 2654435761u ^ (uint32_t)m->c0 * 2246822519u;
        hh ^= hh >> 15;
        m->i_hidx4 = hh & (((uint32_t)1 << IBITS) - 1);
        m->i_state4 = m->ihist4[m->i_hidx4];
        int ip4 = m->iprob4[m->i_sub*256 + m->i_state4] & 4095;
        m->st[IND4] = STRETCH[ip4];
        dot += m->w[m->wcx_eff][IND4] * m->st[IND4];
        dotb += m->wb[m->wcxb_eff][IND4] * m->st[IND4];
        dotc += m->wc[m->wcxc_eff][IND4] * m->st[IND4];
    }
    /* R89: jpg CROSS-BYTE Bit-Historie — die letzten JB Bits (ueber Byte-Grenzen) sagen
     * das naechste Bit voraus; faengt Entropie-Codewort-Praefixe die die byte-alignten
     * Kontexte NICHT sehen (Probe: heic weiss, jpg -2.26% held-out). Nur jpg-gegatet ->
     * st[JBIT]=0 sonst -> alle 16 Nicht-jpg byte-identisch (Gewicht bleibt 0). */
    if (g_jpg) {
        { int d = m->ffdist; int fb = d==0?0:(d<=2?1:(d<=5?2:(d<=12?3:(d<=30?4:5))));   /* R148: FF-Distanz-Bucket (6) × kurzes Bit-Fenster (2^g_jfw) — byte-ereignis-abgeleitetes DCT-Aktivitaets-Regime, orthogonal zur reinen Bit-Historie */
          m->jffctx = (uint32_t)fb * (1u << g_jfw) + (m->bhist & ((1u << g_jfw) - 1u));
          m->st[JFF] = STRETCH[m->jff[m->jffctx] & PMASK];
          m->jffctx2 = (uint32_t)fb * (1u << g_jfw2) + (m->bhist & ((1u << g_jfw2) - 1u));   /* R149: FF-Regime × 2. Fensterbreite */
          m->st[JFF2] = STRETCH[m->jff2[m->jffctx2] & PMASK]; }
        m->jctx = m->bhist & JMASK(JB);
        int jp = m->jt[m->jctx] & PMASK;
        m->st[JBIT] = STRETCH[jp];
        m->jctx2 = m->bhist & JMASK(JB2);
        m->st[JBIT2] = STRETCH[m->jt2[m->jctx2] & PMASK];
        m->jctx3 = m->bhist & JMASK(JB3);
        m->st[JBIT3] = STRETCH[m->jt3[m->jctx3] & PMASK];
        m->jctx4 = m->bhist & JMASK(JB4);
        m->st[JBIT4] = STRETCH[m->jt4[m->jctx4] & PMASK];
        m->jctx5 = m->bhist & JMASK(JB5);
        m->st[JBIT5] = STRETCH[m->jt5[m->jctx5] & PMASK];
        m->jctx6 = m->bhist & JMASK(JB6);
        m->st[JBIT6] = STRETCH[m->jt6[m->jctx6] & PMASK];
        m->jctx7 = m->bhist & JMASK(JB7);
        m->st[JBIT7] = STRETCH[m->jt7[m->jctx7] & PMASK];
        m->jctx8 = m->bhist & JMASK(JB8);
        m->st[JBIT8] = STRETCH[m->jt8[m->jctx8] & PMASK];
        m->jctx9 = m->bhist & JMASK(JB9);
        m->st[JBIT9] = STRETCH[m->jt9[m->jctx9] & PMASK];
        m->jctx10 = m->bhist & JMASK(g_jb10);
        m->st[JBIT10] = STRETCH[m->jt10[m->jctx10] & PMASK];
        m->jctx11 = m->bhist & JMASK(g_jb11);
        m->st[JBIT11] = STRETCH[m->jt11[m->jctx11] & PMASK];
        m->jctx12 = m->bhist & JMASK(g_jb12);
        m->st[JBIT12] = STRETCH[m->jt12[m->jctx12] & PMASK];
        { uint32_t mr = (1u << g_jspgr) - 1u;   /* R113: sparse-bit = recent GR Bits + far GR Bits @Offset (zwei getrennte Gruppen) */
          m->jspctx = ((m->bhist & mr) << g_jspgr) | ((m->bhist >> g_jspoff) & mr);
          m->st[JSP] = STRETCH[m->jsp[m->jspctx] & PMASK];
          m->jsp2ctx = ((m->bhist & mr) << g_jspgr) | ((m->bhist >> g_jsp2off) & mr);   /* R114: 2. Fenster laengerer Offset */
          m->st[JSP2] = STRETCH[m->jsp2[m->jsp2ctx] & PMASK];
          m->jsp3ctx = ((m->bhist & mr) << g_jspgr) | ((m->bhist >> g_jsp3off) & mr);   /* R115: 3. Fenster langes Ende */
          m->st[JSP3] = STRETCH[m->jsp3[m->jsp3ctx] & PMASK];
          m->jsp4ctx = ((m->bhist & mr) << g_jspgr) | ((m->bhist >> g_jsp4off) & mr);   /* R116: 4. Fenster Mittel-Gap */
          m->st[JSP4] = STRETCH[m->jsp4[m->jsp4ctx] & PMASK];
          m->jsp5ctx = ((m->bhist & mr) << g_jspgr) | ((m->bhist >> g_jsp5off) & mr);   /* R118: 5. Fenster */
          m->st[JSP5] = STRETCH[m->jsp5[m->jsp5ctx] & PMASK];
          m->jsp6ctx = ((m->bhist & mr) << g_jspgr) | ((m->bhist >> g_jsp6off) & mr);   /* R120: 6. Fenster */
          m->st[JSP6] = STRETCH[m->jsp6[m->jsp6ctx] & PMASK];
          { uint32_t mr1 = (1u << g_jsp7gr1) - 1u, mr2 = (1u << g_jsp7gr2) - 1u;   /* R122: 7. Fenster ASYMMETRISCHE Gruppen (recent GR1 + far GR2) */
            m->jsp7ctx = ((m->bhist & mr1) << g_jsp7gr2) | ((m->bhist >> g_jsp7off) & mr2);
            m->st[JSP7] = STRETCH[m->jsp7[m->jsp7ctx] & PMASK]; }
          { uint32_t mr1 = (1u << g_jsp8gr1) - 1u, mr2 = (1u << g_jsp8gr2) - 1u;   /* R123: 8. Fenster 2. ASYMMETRISCHE Gruppen */
            m->jsp8ctx = ((m->bhist & mr1) << g_jsp8gr2) | ((m->bhist >> g_jsp8off) & mr2);
            m->st[JSP8] = STRETCH[m->jsp8[m->jsp8ctx] & PMASK]; }
          { uint32_t mr1 = (1u << g_jsp9gr1) - 1u, mr2 = (1u << g_jsp9gr2) - 1u;   /* R124: 9. Fenster 3. ASYMMETRISCHE Gruppen (recent-heavy) */
            m->jsp9ctx = ((m->bhist & mr1) << g_jsp9gr2) | ((m->bhist >> g_jsp9off) & mr2);
            m->st[JSP9] = STRETCH[m->jsp9[m->jsp9ctx] & PMASK]; }
          { uint32_t mr1 = (1u << g_jsp10gr1) - 1u, mr2 = (1u << g_jsp10gr2) - 1u;   /* R125: 10. Fenster 2. recent-heavy */
            m->jsp10ctx = ((m->bhist & mr1) << g_jsp10gr2) | ((m->bhist >> g_jsp10off) & mr2);
            m->st[JSP10] = STRETCH[m->jsp10[m->jsp10ctx] & PMASK]; }
          { uint32_t mr1 = (1u << g_jsp11gr1) - 1u, mr2 = (1u << g_jsp11gr2) - 1u, mr3 = (1u << g_jsp11gr3) - 1u;   /* R126: 11. Fenster 3-Gruppen (recent+mittel+far) */
            m->jsp11ctx = ((m->bhist & mr1) << (g_jsp11gr2 + g_jsp11gr3)) | (((m->bhist >> g_jsp11offm) & mr2) << g_jsp11gr3) | ((m->bhist >> g_jsp11offf) & mr3);
            m->st[JSP11] = STRETCH[m->jsp11[m->jsp11ctx] & PMASK]; }
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp12gr1) - 1u, mr2 = (1u << g_jsp12gr2) - 1u, mr3 = (1u << g_jsp12gr3) - 1u;   /* R127: 2. 3-Gruppen-Fenster; auf dem kleinen jpeg-Full-Pfad (g_nctx==NCTX) gegatet -> jpeg byte-identisch (das sparse @19-Fenster verduennt den 43KB-Einzelblock), strikte Obermenge */
            m->jsp12ctx = ((m->bhist & mr1) << (g_jsp12gr2 + g_jsp12gr3)) | (((m->bhist >> g_jsp12offm) & mr2) << g_jsp12gr3) | ((m->bhist >> g_jsp12offf) & mr3);
            m->st[JSP12] = STRETCH[m->jsp12[m->jsp12ctx] & PMASK]; } else m->st[JSP12] = 0;
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp13gr1) - 1u, mr2 = (1u << g_jsp13gr2) - 1u, mr3 = (1u << g_jsp13gr3) - 1u;   /* R128: 3. 3-Gruppen-Fenster; jpeg-Full-Pfad gegatet (strikte Obermenge) */
            m->jsp13ctx = ((m->bhist & mr1) << (g_jsp13gr2 + g_jsp13gr3)) | (((m->bhist >> g_jsp13offm) & mr2) << g_jsp13gr3) | ((m->bhist >> g_jsp13offf) & mr3);
            m->st[JSP13] = STRETCH[m->jsp13[m->jsp13ctx] & PMASK]; } else m->st[JSP13] = 0;
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp14gr1) - 1u, mr2 = (1u << g_jsp14gr2) - 1u, mr3 = (1u << g_jsp14gr3) - 1u, mr4 = (1u << g_jsp14gr4) - 1u;   /* R130: 4-Gruppen-Kontext; jpeg-Full-Pfad gegatet (strikte Obermenge) */
            m->jsp14ctx = ((m->bhist & mr1) << (g_jsp14gr2 + g_jsp14gr3 + g_jsp14gr4)) | (((m->bhist >> g_jsp14off2) & mr2) << (g_jsp14gr3 + g_jsp14gr4)) | (((m->bhist >> g_jsp14off3) & mr3) << g_jsp14gr4) | ((m->bhist >> g_jsp14off4) & mr4);
            m->st[JSP14] = STRETCH[m->jsp14[m->jsp14ctx] & PMASK]; } else m->st[JSP14] = 0;
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp15gr1) - 1u, mr2 = (1u << g_jsp15gr2) - 1u, mr3 = (1u << g_jsp15gr3) - 1u, mr4 = (1u << g_jsp15gr4) - 1u;   /* R131: 2. 4-Gruppen-Fenster; jpeg-Full-Pfad gegatet */
            m->jsp15ctx = ((m->bhist & mr1) << (g_jsp15gr2 + g_jsp15gr3 + g_jsp15gr4)) | (((m->bhist >> g_jsp15off2) & mr2) << (g_jsp15gr3 + g_jsp15gr4)) | (((m->bhist >> g_jsp15off3) & mr3) << g_jsp15gr4) | ((m->bhist >> g_jsp15off4) & mr4);
            m->st[JSP15] = STRETCH[m->jsp15[m->jsp15ctx] & PMASK]; } else m->st[JSP15] = 0;
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp16gr1) - 1u, mr2 = (1u << g_jsp16gr2) - 1u, mr3 = (1u << g_jsp16gr3) - 1u, mr4 = (1u << g_jsp16gr4) - 1u;   /* R132: 3. 4-Gruppen-Fenster; jpeg-Full-Pfad gegatet */
            m->jsp16ctx = ((m->bhist & mr1) << (g_jsp16gr2 + g_jsp16gr3 + g_jsp16gr4)) | (((m->bhist >> g_jsp16off2) & mr2) << (g_jsp16gr3 + g_jsp16gr4)) | (((m->bhist >> g_jsp16off3) & mr3) << g_jsp16gr4) | ((m->bhist >> g_jsp16off4) & mr4);
            m->st[JSP16] = STRETCH[m->jsp16[m->jsp16ctx] & PMASK]; } else m->st[JSP16] = 0;
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp17gr1) - 1u, mr2 = (1u << g_jsp17gr2) - 1u, mr3 = (1u << g_jsp17gr3) - 1u, mr4 = (1u << g_jsp17gr4) - 1u;   /* R133: 4. 4-Gruppen-Fenster; jpeg-Full-Pfad gegatet */
            m->jsp17ctx = ((m->bhist & mr1) << (g_jsp17gr2 + g_jsp17gr3 + g_jsp17gr4)) | (((m->bhist >> g_jsp17off2) & mr2) << (g_jsp17gr3 + g_jsp17gr4)) | (((m->bhist >> g_jsp17off3) & mr3) << g_jsp17gr4) | ((m->bhist >> g_jsp17off4) & mr4);
            m->st[JSP17] = STRETCH[m->jsp17[m->jsp17ctx] & PMASK]; } else m->st[JSP17] = 0;
          if (g_nctx != NCTX) { uint32_t mr1 = (1u << g_jsp19gr1) - 1u, mr2 = (1u << g_jsp19gr2) - 1u, mr3 = (1u << g_jsp19gr3) - 1u, mr4 = (1u << g_jsp19gr4) - 1u;   /* R146: 5. 4-Gruppen-Fenster */
            m->jsp19ctx = ((m->bhist & mr1) << (g_jsp19gr2 + g_jsp19gr3 + g_jsp19gr4)) | (((m->bhist >> g_jsp19off2) & mr2) << (g_jsp19gr3 + g_jsp19gr4)) | (((m->bhist >> g_jsp19off3) & mr3) << g_jsp19gr4) | ((m->bhist >> g_jsp19off4) & mr4);
            m->st[JSP19] = STRETCH[m->jsp19[m->jsp19ctx] & PMASK]; } else m->st[JSP19] = 0;
          { uint32_t mr1 = (1u << g_jsp18gr1) - 1u, mr2 = (1u << g_jsp18gr2) - 1u, mr3 = (1u << g_jsp18gr3) - 1u;   /* R137: asym 3-Gruppen (recent GR1 + mittel GR2 @offm + far GR3 @offf) */
            m->jsp18ctx = ((m->bhist & mr1) << (g_jsp18gr2 + g_jsp18gr3)) | (((m->bhist >> g_jsp18offm) & mr2) << g_jsp18gr3) | ((m->bhist >> g_jsp18offf) & mr3);
            m->st[JSP18] = STRETCH[m->jsp18[m->jsp18ctx] & PMASK]; } }
    } else { m->st[JBIT] = 0; m->st[JBIT2] = 0; m->st[JBIT3] = 0; m->st[JBIT4] = 0; m->st[JBIT5] = 0; m->st[JBIT6] = 0; m->st[JBIT7] = 0; m->st[JBIT8] = 0; m->st[JBIT9] = 0; m->st[JBIT10] = 0; m->st[JBIT11] = 0; m->st[JBIT12] = 0; m->st[JSP] = 0; m->st[JSP2] = 0; m->st[JSP3] = 0; m->st[JSP4] = 0; m->st[JSP5] = 0; m->st[JSP6] = 0; m->st[JSP7] = 0; m->st[JSP8] = 0; m->st[JSP9] = 0; m->st[JSP10] = 0; m->st[JSP11] = 0; m->st[JSP12] = 0; m->st[JSP13] = 0; m->st[JSP14] = 0; m->st[JSP15] = 0; m->st[JSP16] = 0; m->st[JSP17] = 0; m->st[JSP18] = 0; m->st[JSP19] = 0; m->st[JFF] = 0; m->st[JFF2] = 0; }
    /* R107: cross-byte-KONFIDENZ als Mischer-GATE — in hoch-konfidenten jpg-Regionen
     * gewichtet der 2./3. Mischer die cross-byte-Vorhersage anders (COMBINATION-Achse,
     * orthogonal zum Mischer-EINGANG R89-92 und zum SSE-TRAEGER R95). wcxb/wcxc nutzen
     * nur 0..255 -> Bit 0x100 frei; jpg-gegatet -> alle 16 Nicht-jpg byte-identisch. */
    if (g_jpg) {
        int a = 0;
        a += m->st[JBIT]<0?-m->st[JBIT]:m->st[JBIT];      a += m->st[JBIT2]<0?-m->st[JBIT2]:m->st[JBIT2];
        a += m->st[JBIT3]<0?-m->st[JBIT3]:m->st[JBIT3];    a += m->st[JBIT4]<0?-m->st[JBIT4]:m->st[JBIT4];
        a += m->st[JBIT5]<0?-m->st[JBIT5]:m->st[JBIT5];    a += m->st[JBIT6]<0?-m->st[JBIT6]:m->st[JBIT6];
        a += m->st[JBIT7]<0?-m->st[JBIT7]:m->st[JBIT7];    a += m->st[JBIT8]<0?-m->st[JBIT8]:m->st[JBIT8];
        a += m->st[JBIT9]<0?-m->st[JBIT9]:m->st[JBIT9];
        if (a >= g_gate) {
            if (g_gatemask & 1) m->wcxb_eff |= 256;
            if (g_gatemask & 2) m->wcxc_eff |= 256;
        }
    }
    if (g_jpg) {   /* ZEIT R208: byte-neutral — st[JBIT..JFF2]=0 fuer alle 16 Nicht-jpg (else-Zweig oben), also w*0=0 auf alle 3 Mischer; die ~99 mult-adds/Bit uebersprungen (kein Byte-Effekt, spart Arithmetik auf heic/pdf/stl/...) */
    dotb += m->wb[m->wcxb_eff][JBIT]  * m->st[JBIT];
    dotb += m->wb[m->wcxb_eff][JBIT2] * m->st[JBIT2];
    dotb += m->wb[m->wcxb_eff][JBIT3] * m->st[JBIT3];
    dotb += m->wb[m->wcxb_eff][JBIT4] * m->st[JBIT4];
    dotb += m->wb[m->wcxb_eff][JBIT5] * m->st[JBIT5];
    dotb += m->wb[m->wcxb_eff][JBIT6] * m->st[JBIT6];
    dotb += m->wb[m->wcxb_eff][JBIT7] * m->st[JBIT7];
    dotb += m->wb[m->wcxb_eff][JBIT8] * m->st[JBIT8];
    dotb += m->wb[m->wcxb_eff][JBIT9] * m->st[JBIT9];
    dotb += m->wb[m->wcxb_eff][JBIT10] * m->st[JBIT10];
    dotb += m->wb[m->wcxb_eff][JBIT11] * m->st[JBIT11];
    dotb += m->wb[m->wcxb_eff][JBIT12] * m->st[JBIT12];
    dotb += m->wb[m->wcxb_eff][JSP]    * m->st[JSP];   /* R113: sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP2]   * m->st[JSP2];  /* R114: 2. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP3]   * m->st[JSP3];  /* R115: 3. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP4]   * m->st[JSP4];  /* R116: 4. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP5]   * m->st[JSP5];  /* R118: 5. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP6]   * m->st[JSP6];  /* R120: 6. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP7]   * m->st[JSP7];  /* R122: 7. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP8]   * m->st[JSP8];  /* R123: 8. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP9]   * m->st[JSP9];  /* R124: 9. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP10]  * m->st[JSP10]; /* R125: 10. sparse-bit auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP11]  * m->st[JSP11]; /* R126: 11. sparse-bit (3-Gruppen) auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP12]  * m->st[JSP12]; /* R127: 2. 3-Gruppen-Fenster auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP13]  * m->st[JSP13]; /* R128: 3. 3-Gruppen-Fenster auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP14]  * m->st[JSP14]; /* R130: 4-Gruppen-Kontext auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP15]  * m->st[JSP15]; /* R131: 2. 4-Gruppen-Fenster auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP16]  * m->st[JSP16]; /* R132: 3. 4-Gruppen-Fenster auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP17]  * m->st[JSP17]; /* R133: 4. 4-Gruppen-Fenster auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP18]  * m->st[JSP18]; /* R137: asym 3-Gruppen auf dotb */
    dotb += m->wb[m->wcxb_eff][JSP19]  * m->st[JSP19]; /* R146: 5. 4-Gruppen-Fenster auf dotb */
    dotb += m->wb[m->wcxb_eff][JFF]    * m->st[JFF];   /* R148: FF-Distanz-Regime auf dotb */
    dotb += m->wb[m->wcxb_eff][JFF2]   * m->st[JFF2];  /* R149: 2. FF-Fenster auf dotb */
    /* R92: jpg cross-byte auch auf 1. (nah/Bit-Position R310) + 3. (mitte order-4) Mischer =
     * COMBINATION-Achse (R309-R314): jeder Mischer gewichtet die cross-byte-Vorhersage unter
     * anderem Selektions-Kontext; st[JBIT*]=0 sonst -> byte-identisch by construction. */
    dot  += m->w[m->wcx_eff][JBIT]   * m->st[JBIT];
    dot  += m->w[m->wcx_eff][JBIT2]  * m->st[JBIT2];
    dot  += m->w[m->wcx_eff][JBIT3]  * m->st[JBIT3];
    dot  += m->w[m->wcx_eff][JBIT4]  * m->st[JBIT4];
    dot  += m->w[m->wcx_eff][JBIT5]  * m->st[JBIT5];
    dot  += m->w[m->wcx_eff][JBIT6]  * m->st[JBIT6];
    dot  += m->w[m->wcx_eff][JBIT7]  * m->st[JBIT7];
    dot  += m->w[m->wcx_eff][JBIT8]  * m->st[JBIT8];
    dot  += m->w[m->wcx_eff][JBIT9]  * m->st[JBIT9];
    dot  += m->w[m->wcx_eff][JBIT10] * m->st[JBIT10];
    dot  += m->w[m->wcx_eff][JBIT11] * m->st[JBIT11];
    dot  += m->w[m->wcx_eff][JBIT12] * m->st[JBIT12];
    dot  += m->w[m->wcx_eff][JSP]    * m->st[JSP];   /* R113: sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP2]   * m->st[JSP2];  /* R114: 2. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP3]   * m->st[JSP3];  /* R115: 3. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP4]   * m->st[JSP4];  /* R116: 4. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP5]   * m->st[JSP5];  /* R118: 5. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP6]   * m->st[JSP6];  /* R120: 6. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP7]   * m->st[JSP7];  /* R122: 7. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP8]   * m->st[JSP8];  /* R123: 8. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP9]   * m->st[JSP9];  /* R124: 9. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP10]  * m->st[JSP10]; /* R125: 10. sparse-bit auf dot */
    dot  += m->w[m->wcx_eff][JSP11]  * m->st[JSP11]; /* R126: 11. sparse-bit (3-Gruppen) auf dot */
    dot  += m->w[m->wcx_eff][JSP12]  * m->st[JSP12]; /* R127: 2. 3-Gruppen-Fenster auf dot */
    dot  += m->w[m->wcx_eff][JSP13]  * m->st[JSP13]; /* R128: 3. 3-Gruppen-Fenster auf dot */
    dot  += m->w[m->wcx_eff][JSP14]  * m->st[JSP14]; /* R130: 4-Gruppen-Kontext auf dot */
    dot  += m->w[m->wcx_eff][JSP15]  * m->st[JSP15]; /* R131: 2. 4-Gruppen-Fenster auf dot */
    dot  += m->w[m->wcx_eff][JSP16]  * m->st[JSP16]; /* R132: 3. 4-Gruppen-Fenster auf dot */
    dot  += m->w[m->wcx_eff][JSP17]  * m->st[JSP17]; /* R133: 4. 4-Gruppen-Fenster auf dot */
    dot  += m->w[m->wcx_eff][JSP18]  * m->st[JSP18]; /* R137: asym 3-Gruppen auf dot */
    dot  += m->w[m->wcx_eff][JSP19]  * m->st[JSP19]; /* R146: 5. 4-Gruppen-Fenster auf dot */
    dot  += m->w[m->wcx_eff][JFF]    * m->st[JFF];   /* R148: FF-Distanz-Regime auf dot (COMBINATION R92) */
    dot  += m->w[m->wcx_eff][JFF2]   * m->st[JFF2];  /* R149: 2. FF-Fenster auf dot */
    dotc += m->wc[m->wcxc_eff][JBIT]  * m->st[JBIT];
    dotc += m->wc[m->wcxc_eff][JBIT2] * m->st[JBIT2];
    dotc += m->wc[m->wcxc_eff][JBIT3] * m->st[JBIT3];
    dotc += m->wc[m->wcxc_eff][JBIT4] * m->st[JBIT4];
    dotc += m->wc[m->wcxc_eff][JBIT5] * m->st[JBIT5];
    dotc += m->wc[m->wcxc_eff][JBIT6] * m->st[JBIT6];
    dotc += m->wc[m->wcxc_eff][JBIT7] * m->st[JBIT7];
    dotc += m->wc[m->wcxc_eff][JBIT8] * m->st[JBIT8];
    dotc += m->wc[m->wcxc_eff][JBIT9] * m->st[JBIT9];
    dotc += m->wc[m->wcxc_eff][JBIT10] * m->st[JBIT10];
    dotc += m->wc[m->wcxc_eff][JBIT11] * m->st[JBIT11];
    dotc += m->wc[m->wcxc_eff][JBIT12] * m->st[JBIT12];
    dotc += m->wc[m->wcxc_eff][JSP]    * m->st[JSP];   /* R113: sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP2]   * m->st[JSP2];  /* R114: 2. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP3]   * m->st[JSP3];  /* R115: 3. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP4]   * m->st[JSP4];  /* R116: 4. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP5]   * m->st[JSP5];  /* R118: 5. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP6]   * m->st[JSP6];  /* R120: 6. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP7]   * m->st[JSP7];  /* R122: 7. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP8]   * m->st[JSP8];  /* R123: 8. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP9]   * m->st[JSP9];  /* R124: 9. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP10]  * m->st[JSP10]; /* R125: 10. sparse-bit auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP11]  * m->st[JSP11]; /* R126: 11. sparse-bit (3-Gruppen) auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP12]  * m->st[JSP12]; /* R127: 2. 3-Gruppen-Fenster auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP13]  * m->st[JSP13]; /* R128: 3. 3-Gruppen-Fenster auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP14]  * m->st[JSP14]; /* R130: 4-Gruppen-Kontext auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP15]  * m->st[JSP15]; /* R131: 2. 4-Gruppen-Fenster auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP16]  * m->st[JSP16]; /* R132: 3. 4-Gruppen-Fenster auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP17]  * m->st[JSP17]; /* R133: 4. 4-Gruppen-Fenster auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP18]  * m->st[JSP18]; /* R137: asym 3-Gruppen auf dotc */
    dotc += m->wc[m->wcxc_eff][JSP19]  * m->st[JSP19]; /* R146: 5. 4-Gruppen-Fenster auf dotc */
    dotc += m->wc[m->wcxc_eff][JFF]    * m->st[JFF];   /* R148: FF-Distanz-Regime auf dotc (COMBINATION R92) */
    dotc += m->wc[m->wcxc_eff][JFF2]   * m->st[JFF2];  /* R149: 2. FF-Fenster auf dotc */
    }   /* ZEIT R208: Ende g_jpg-Mischer-Adds */
    int bp = (m->wcx_eff & 7) | ((m->mm_len>0||m->mm_len2>0)?8:0);   /* R313b: Bit-Position x Match-Aktiv */
    int dA = dot >> 16, dB = dotb >> 16, dC = dotc >> 16;
    m->cdA = dA; m->cdB = dB; m->cdC = dC; m->cbp = bp;
    int d = (m->gcomb[bp][0] * dA + m->gcomb[bp][1] * dB + m->gcomb[bp][2] * dC) >> 16;
    if (g_heic && (m->hist&0xFFFFu)==0x0000u) {  /* R83b: Emulation-Prevention HARD-Bias — nach 00 00 Byte<=3, top-6-Bits=0; solange nur Nullen dekodiert (c0 Potenz von 2) und im oberen Bit-Bereich -> Bit=0 vorhersagen */
        int k = 31 - __builtin_clz((unsigned)m->c0);
        if (k < 6 && (m->c0 & (m->c0 - 1)) == 0) d -= 256;
    }
    if (d > 2047) d = 2047; else if (d < -2047) d = -2047;
    m->pr = SQUASH[d + 2048];
    if (m->pr < 1) m->pr = 1; else if (m->pr > 4094) m->pr = 4094;
    /* R12: APM-Nachschaerfung, Kontext = c0 (Bit-Baum-Knoten, gut trainiert auch
     * auf kleinen Dateien). Interpoliere 2 Knoten, merke den naeheren furs Update.
     * m->pr (Mischer-Ausgang) bleibt fuer das Training unveraendert. */
    int prc;
    m->apm4_index = 0;
    if (g_nctx < NCTX || g_heic) {  /* R379/R380: Weak-Pfad; R61: heic auch bei g_nctx=8 (heic/jpg/stl) bekommt SSE (war R212 ganz aus) — je Inhalt gestuft */
        int r1 = apm_do(m->apm, m->c0, m->pr, &m->apm_index);
        if (g_word) {  /* stl (weak-ASCII, match-reich): volle Match-Laengen-SSE wie Full-Pfad, pures Endstufen-Mittel */
            int r2 = apm_do(m->apm2, (int)(m->hist & 0xFFu  /* R386: stl gross/voll-trainiert */), r1, &m->apm2_index);
            int base = (r1 + r2 + 1) >> 1;
            int mctx=0, bestlen=0, bestpb=0, kk=31-__builtin_clz((unsigned)m->c0);
            if(m->mm_len>0){int E=m->mm_buf[m->mm_ptr]; if((E>>(8-kk))==(m->c0-(1<<kk))){bestlen=m->mm_len;bestpb=(E>>(7-kk))&1;}}
            if(m->mm_len2>0){int E=m->mm_buf[m->mm_ptr2]; if((E>>(8-kk))==(m->c0-(1<<kk))&&m->mm_len2>bestlen){bestlen=m->mm_len2;bestpb=(E>>(7-kk))&1;}}
            if(bestlen>0){int lb=bestlen>=32?7:bestlen>=16?6:bestlen>=12?5:bestlen>=8?4:bestlen>=6?3:bestlen>=4?2:bestlen>=2?1:0; mctx=1+lb*2+bestpb;}
            int r3=apm_do(m->apm3,mctx,base,&m->apm3_index);
            prc=(base+3*r3+2)>>2;
        } else {  /* R380: heic/jpg (weak-binaer) — apm2 mit VOLLEM Vorbyte (0xFF statt 0x3F) + leichte 00-Struktur-SSE (apm3, Gewicht 1/8); mit Mischer-Ausgang verankert */
            int r2 = apm_do(m->apm2, (int)(m->hist & 0xFFu), r1, &m->apm2_index);
            int base = (r1 + r2 + 1) >> 1;
            int sctx = ((m->hist & 0xFFu) == 0x00u) ? 1 : 0;
            int r3 = apm_do(m->apm3, sctx, base, &m->apm3_index);
            prc = (m->pr + ((3*base + r3 + 2) >> 2) + 1) >> 1;
            if (g_heic) {  /* R85: Emulation-SSE (apm4) NUR heic — jpg (FFD8, entropie-codiert) hat keine 00-00-Emulation-Struktur (+192 Dilution sonst) */
                int ec = ((m->hist & 0xFFFFu) == 0x0000u) ? ((m->c0 >= 64) ? 2 : 1) : 0;  /* 00 00 Kontext x c0-Praefix (value<=3 top-Bits), sonst slot 0; R381 3-Traeger-Muster */
                int r4 = apm_do(m->apm4, ec, prc, &m->apm4_index);
                prc = (3 * prc + r4 + 2) >> 2;
            } else if (g_jpg) {  /* R95: jpg cross-byte-KONFIDENZ als SSE-Traeger (apm4 fuer jpg frei — heic/full-binaer nutzen es in exklusiven Zweigen, per-Block-Isolation = keine Kollision; R85-Muster: SSE schaerft ohne Dilution; jpg-gegatet -> alle anderen byte-identisch) */
                int s0=m->st[JBIT], s1=m->st[JBIT2], s2=m->st[JBIT3], s3=m->st[JBIT4], s4=m->st[JBIT5], s5=m->st[JBIT6], s6=m->st[JBIT7], s7=m->st[JBIT8], s8=m->st[JBIT9];
                int s9=m->st[JSP], s10=m->st[JSP2], s11=m->st[JSP3], s12=m->st[JSP4], s13=m->st[JSP5];  /* R117: 4 + R119: 5. sparse-bit Fenster ins Konfidenz-Aggregat */
                int a = (s0<0?-s0:s0)+(s1<0?-s1:s1)+(s2<0?-s2:s2)+(s3<0?-s3:s3)+(s4<0?-s4:s4)+(s5<0?-s5:s5)+(s6<0?-s6:s6)+(s7<0?-s7:s7)+(s8<0?-s8:s8)
                      +(s9<0?-s9:s9)+(s10<0?-s10:s10)+(s11<0?-s11:s11)+(s12<0?-s12:s12)+(s13<0?-s13:s13);  /* R119: Aggregat-|stretch| aller 14 Fenster (9 contiguous + 5 sparse) = Konfidenz */
                int ec = (a / g_sstep) >= 7 ? 7 : (a / g_sstep);   /* R103: 8 lineare Konfidenz-Buckets, Schritt g_sstep (9-Fenster-Aggregat, re-getunt) */
                int r4 = apm_do(m->apm4, ec, prc, &m->apm4_index);
                prc = (3 * prc + r4 + 2) >> 2;             /* leichte SSE-Nachschaerfung 1/4 (Sweep: bl=1 schlaegt 1/2) */
            }
        }
    }
    else { int r1 = apm_do(m->apm, m->c0, m->pr, &m->apm_index);
           int r2 = apm_do(m->apm2, (int)(m->hist & 0xFFu), r1, &m->apm2_index);  /* R385/R386: apm2 mit VOLLEM Vorbyte fuer ALLE (full-binaer R385 + full-ASCII obj/step R386; tiny-Text neutral, stl 0xFF Zeile 414) */
           int base = (r1 + r2 + 1) >> 1;
           /* R318: Match-Laengen-SSE — Kontext = laengster zustimmender Match (Bucket) x Vorhersage-Bit */
           int mctx = 0, bestlen = 0, bestpb = 0, kk = 31 - __builtin_clz((unsigned)m->c0);
           if (m->mm_len > 0) { int E = m->mm_buf[m->mm_ptr];
               if ((E >> (8 - kk)) == (m->c0 - (1 << kk))) { bestlen = m->mm_len; bestpb = (E >> (7 - kk)) & 1; } }
           if (m->mm_len2 > 0) { int E = m->mm_buf[m->mm_ptr2];
               if ((E >> (8 - kk)) == (m->c0 - (1 << kk)) && m->mm_len2 > bestlen) { bestlen = m->mm_len2; bestpb = (E >> (7 - kk)) & 1; } }
           if (bestlen > 0) {
               int lb = bestlen>=32?7 : bestlen>=16?6 : bestlen>=12?5 : bestlen>=8?4 : bestlen>=6?3 : bestlen>=4?2 : bestlen>=2?1:0;
               mctx = 1 + lb * 2 + bestpb; }
           int r3 = apm_do(m->apm3, mctx, base, &m->apm3_index);
           prc = (base + 3 * r3 + 2) >> 2;
           if (!g_word) {  /* R381: full-binaer (pdf/docx/xlsx) — 00-Struktur-SSE leicht ueber die Match-SSE */
               int sctx = ((m->hist & 0xFFu) == 0x00u) ? 1 : 0;
               int r4 = apm_do(m->apm4, sctx, prc, &m->apm4_index);
               prc = (3 * prc + r4 + 2) >> 2;
           } }
    if (prc < 1) prc = 1; else if (prc > 4094) prc = 4094;
    return prc;
}

static void model_update(Model *m, int bit) {
    /* R12/R15: beide APM-Stufen ziehen (nur wenn APM aktiv, R212) */
    { apm_up(m->apm, m->apm_index, bit); apm_up(m->apm2, m->apm2_index, bit); apm_up(m->apm3, m->apm3_index, bit); apm_up(m->apm4, m->apm4_index, bit); }  /* R379: SSE-Update jetzt auch Weak-Pfad */
    int err = ((bit << 12) - m->pr);          /* -4095..4095 */
    {   /* R313: Kombinier-Gewichte per Gradient (Start 32768 = R312-Mittel) */
        int ga = m->gcomb[m->cbp][0] + ((m->cdA * err) >> GCRATE);
        int gb = m->gcomb[m->cbp][1] + ((m->cdB * err) >> GCRATE);
        int gc = m->gcomb[m->cbp][2] + ((m->cdC * err) >> GCRATE);
        if (ga < 0) ga = 0; else if (ga > 65535) ga = 65535;
        if (gb < 0) gb = 0; else if (gb > 65535) gb = 65535;
        if (gc < 0) gc = 0; else if (gc > 65535) gc = 65535;
        m->gcomb[m->cbp][0] = ga; m->gcomb[m->cbp][1] = gb; m->gcomb[m->cbp][2] = gc;
    }
    for (int i = 0; i < g_nctx; i++) {
        /* Zaehler-Update mit adaptiver Rate (R5): frisch=schnell, reif=langsam */
        uint16_t *c = &m->t[i][m->idx[i]];
        int cv = *c;
        int p = cv & PMASK;
        int cnt = cv >> 12;
        int rr = (g_nctx==NCTX && !g_heic)?RATE[cnt]:(g_heic?RATE_H[cnt]:RATE_W[cnt]);  /* R367 per-Pfad Lernrate; R31: heic eigene schnelle Decke */
        if (!g_word && (m->hist&0xFFu)==0xFFu && rr>1) rr--;  /* R375: nach JPEG-Marker (prev==0xFF, binaer) 1 Schritt schneller — Entropie-Strom startet neu am Segment */
        if (g_nctx==NCTX && !g_heic && !g_word && (m->hist&0xFFu)==0x00u && rr>1) rr--;  /* R376: nach 0x00 (full-Pfad binaer: pdf/docx) 1 Schritt schneller — 0x00-Kontext schaerfer (H(next|00)=7.19<7.99), pdf-Stream-0x00-Cluster; nur full-Pfad (weak-jpg wollte es nicht) */
        p += ((bit << 12) - p) >> rr;
        if (p < 1) p = 1; else if (p > 4095) p = 4095;
        if (cnt < 15) cnt++;
        *c = (uint16_t)((cnt << 12) | p);
        /* Mixer-Gewicht: Gradient */
        int nw = m->w[m->wcx_eff][i] + ((m->st[i] * err) >> 10);
        if (nw < -(1 << 20)) nw = -(1 << 20);
        else if (nw > (1 << 20)) nw = (1 << 20);
        m->w[m->wcx_eff][i] = nw;
        int bw0 = m->wb[m->wcxb_eff][i] + ((m->st[i] * err) >> 10);
        if (bw0 < -(1 << 20)) bw0 = -(1 << 20); else if (bw0 > (1 << 20)) bw0 = (1 << 20);
        m->wb[m->wcxb_eff][i] = bw0;
        int cw0 = m->wc[m->wcxc_eff][i] + ((m->st[i] * err) >> 10);
        if (cw0 < -(1 << 20)) cw0 = -(1 << 20); else if (cw0 > (1 << 20)) cw0 = (1 << 20);
        m->wc[m->wcxc_eff][i] = cw0;
    }
    /* R13: Match-Gewicht trainieren */
    {
        int nw = m->w[m->wcx_eff][MI] + ((m->st[MI] * err) >> 10);
        if (nw < -(1 << 20)) nw = -(1 << 20);
        else if (nw > (1 << 20)) nw = (1 << 20);
        m->w[m->wcx_eff][MI] = nw;
        int bwm = m->wb[m->wcxb_eff][MI] + ((m->st[MI] * err) >> 10);
        if (bwm < -(1 << 20)) bwm = -(1 << 20); else if (bwm > (1 << 20)) bwm = (1 << 20);
        m->wb[m->wcxb_eff][MI] = bwm;
        int cwm = m->wc[m->wcxc_eff][MI] + ((m->st[MI] * err) >> 10);
        if (cwm < -(1 << 20)) cwm = -(1 << 20); else if (cwm > (1 << 20)) cwm = (1 << 20);
        m->wc[m->wcxc_eff][MI] = cwm;
        int nw2 = m->w[m->wcx_eff][MI2] + ((m->st[MI2] * err) >> 10);
        if (nw2 < -(1 << 20)) nw2 = -(1 << 20);
        else if (nw2 > (1 << 20)) nw2 = (1 << 20);
        m->w[m->wcx_eff][MI2] = nw2;
        int bwm2 = m->wb[m->wcxb_eff][MI2] + ((m->st[MI2] * err) >> 10);
        if (bwm2 < -(1 << 20)) bwm2 = -(1 << 20); else if (bwm2 > (1 << 20)) bwm2 = (1 << 20);
        m->wb[m->wcxb_eff][MI2] = bwm2;
        int cwm2 = m->wc[m->wcxc_eff][MI2] + ((m->st[MI2] * err) >> 10);
        if (cwm2 < -(1 << 20)) cwm2 = -(1 << 20); else if (cwm2 > (1 << 20)) cwm2 = (1 << 20);
        m->wc[m->wcxc_eff][MI2] = cwm2;
        if (g_heic) {   /* ZEIT R210: byte-neutral — st[MI3]=0 sonst -> g=0 -> w unveraendert (Store = verschwendet); uebersprungen fuer die 16 Nicht-heic (R208-Muster) */
        int nw3 = m->w[m->wcx_eff][MI3] + ((m->st[MI3] * err) >> 10);
        if (nw3 < -(1 << 20)) nw3 = -(1 << 20); else if (nw3 > (1 << 20)) nw3 = (1 << 20);
        m->w[m->wcx_eff][MI3] = nw3;
        int bwm3 = m->wb[m->wcxb_eff][MI3] + ((m->st[MI3] * err) >> 10);
        if (bwm3 < -(1 << 20)) bwm3 = -(1 << 20); else if (bwm3 > (1 << 20)) bwm3 = (1 << 20);
        m->wb[m->wcxb_eff][MI3] = bwm3;
        int cwm3 = m->wc[m->wcxc_eff][MI3] + ((m->st[MI3] * err) >> 10);
        if (cwm3 < -(1 << 20)) cwm3 = -(1 << 20); else if (cwm3 > (1 << 20)) cwm3 = (1 << 20);
        m->wc[m->wcxc_eff][MI3] = cwm3;
        }
        int nwi = m->w[m->wcx_eff][IND] + ((m->st[IND] * err) >> 10);
        if (nwi < -(1 << 20)) nwi = -(1 << 20);
        else if (nwi > (1 << 20)) nwi = (1 << 20);
        m->w[m->wcx_eff][IND] = nwi;
        int bwi = m->wb[m->wcxb_eff][IND] + ((m->st[IND] * err) >> 10);
        if (bwi < -(1 << 20)) bwi = -(1 << 20); else if (bwi > (1 << 20)) bwi = (1 << 20);
        m->wb[m->wcxb_eff][IND] = bwi;
        int cwi = m->wc[m->wcxc_eff][IND] + ((m->st[IND] * err) >> 10);
        if (cwi < -(1 << 20)) cwi = -(1 << 20); else if (cwi > (1 << 20)) cwi = (1 << 20);
        m->wc[m->wcxc_eff][IND] = cwi;
        int nwi2 = m->w[m->wcx_eff][IND2] + ((m->st[IND2] * err) >> 10);
        if (nwi2 < -(1 << 20)) nwi2 = -(1 << 20);
        else if (nwi2 > (1 << 20)) nwi2 = (1 << 20);
        m->w[m->wcx_eff][IND2] = nwi2;
        int bwi2 = m->wb[m->wcxb_eff][IND2] + ((m->st[IND2] * err) >> 10);
        if (bwi2 < -(1 << 20)) bwi2 = -(1 << 20); else if (bwi2 > (1 << 20)) bwi2 = (1 << 20);
        m->wb[m->wcxb_eff][IND2] = bwi2;
        int cwi2 = m->wc[m->wcxc_eff][IND2] + ((m->st[IND2] * err) >> 10);
        if (cwi2 < -(1 << 20)) cwi2 = -(1 << 20); else if (cwi2 > (1 << 20)) cwi2 = (1 << 20);
        m->wc[m->wcxc_eff][IND2] = cwi2;
        int nwi3 = m->w[m->wcx_eff][IND3] + ((m->st[IND3] * err) >> 10);
        if (nwi3 < -(1 << 20)) nwi3 = -(1 << 20);
        else if (nwi3 > (1 << 20)) nwi3 = (1 << 20);
        m->w[m->wcx_eff][IND3] = nwi3;
        int bwi3 = m->wb[m->wcxb_eff][IND3] + ((m->st[IND3] * err) >> 10);
        if (bwi3 < -(1 << 20)) bwi3 = -(1 << 20); else if (bwi3 > (1 << 20)) bwi3 = (1 << 20);
        m->wb[m->wcxb_eff][IND3] = bwi3;
        int cwi3 = m->wc[m->wcxc_eff][IND3] + ((m->st[IND3] * err) >> 10);
        if (cwi3 < -(1 << 20)) cwi3 = -(1 << 20); else if (cwi3 > (1 << 20)) cwi3 = (1 << 20);
        m->wc[m->wcxc_eff][IND3] = cwi3;
        int nwi4 = m->w[m->wcx_eff][IND4] + ((m->st[IND4] * err) >> 10);
        if (nwi4 < -(1 << 20)) nwi4 = -(1 << 20);
        else if (nwi4 > (1 << 20)) nwi4 = (1 << 20);
        m->w[m->wcx_eff][IND4] = nwi4;
        int bwi4 = m->wb[m->wcxb_eff][IND4] + ((m->st[IND4] * err) >> 10);
        if (bwi4 < -(1 << 20)) bwi4 = -(1 << 20); else if (bwi4 > (1 << 20)) bwi4 = (1 << 20);
        m->wb[m->wcxb_eff][IND4] = bwi4;
        int cwi4 = m->wc[m->wcxc_eff][IND4] + ((m->st[IND4] * err) >> 10);
        if (cwi4 < -(1 << 20)) cwi4 = -(1 << 20); else if (cwi4 > (1 << 20)) cwi4 = (1 << 20);
        m->wc[m->wcxc_eff][IND4] = cwi4;
        /* R89: jpg cross-byte Bit-Gewicht (nur dotb, wie IND); st[JBIT]=0 sonst -> 0-Update */
        /* R89/R90/R91 dotb + R92 dot/dotc: alle 3 Mischer je cross-byte-Eingang trainieren;
         * st[JBIT*]=0 sonst -> 0-Update -> byte-identisch. */
        if (g_jpg) {   /* ZEIT R208: byte-neutral — st[JBIT..JFF2]=0 sonst -> g=0 -> w unveraendert (Store zurueck = verschwendet); Loop uebersprungen fuer die 16 Nicht-jpg */
        int jidx[33]; jidx[32] = JFF2; jidx[31] = JFF; jidx[30] = JSP19; jidx[0] = JBIT; jidx[1] = JBIT2; jidx[2] = JBIT3; jidx[3] = JBIT4; jidx[4] = JBIT5; jidx[5] = JBIT6; jidx[6] = JBIT7; jidx[7] = JBIT8; jidx[8] = JBIT9; jidx[9] = JBIT10; jidx[10] = JBIT11; jidx[11] = JBIT12; jidx[12] = JSP; jidx[13] = JSP2; jidx[14] = JSP3; jidx[15] = JSP4; jidx[16] = JSP5; jidx[17] = JSP6; jidx[18] = JSP7; jidx[19] = JSP8; jidx[20] = JSP9; jidx[21] = JSP10; jidx[22] = JSP11; jidx[23] = JSP12; jidx[24] = JSP13; jidx[25] = JSP14; jidx[26] = JSP15; jidx[27] = JSP16; jidx[28] = JSP17; jidx[29] = JSP18; /* R94..R106 Fenster; R113-133 sparse-bit 1-17 */
        for (int jj = 0; jj < 33; jj++) {
            int ji = jidx[jj];
            int g = (m->st[ji] * err) >> 10;
            int va = m->w[m->wcx_eff][ji] + g;
            if (va < -(1 << 20)) va = -(1 << 20); else if (va > (1 << 20)) va = (1 << 20);
            m->w[m->wcx_eff][ji] = va;
            int vb = m->wb[m->wcxb_eff][ji] + g;
            if (vb < -(1 << 20)) vb = -(1 << 20); else if (vb > (1 << 20)) vb = (1 << 20);
            m->wb[m->wcxb_eff][ji] = vb;
            int vc = m->wc[m->wcxc_eff][ji] + g;
            if (vc < -(1 << 20)) vc = -(1 << 20); else if (vc > (1 << 20)) vc = (1 << 20);
            m->wc[m->wcxc_eff][ji] = vc;
        }
        }   /* ZEIT R208: Ende g_jpg-Gewichts-Update */
    }
    /* R298: indirektes Modell nachziehen (Zustands-Karte + Bit-Historie fortschreiben) */
    {
        const int *IR = (g_nctx!=NCTX && !g_word && !g_heic)?IRATE_WB:IRATE;  /* R382: weak-binaer jpg eigene langsame iprob-Decke 9; R31: heic per Magic getrennt -> zurueck auf globale schnelle Decke 6 (R382 mass heic +14 bei Decke 9, jpg -131 dominierte; jetzt trennbar) */
        uint16_t *pv = &m->iprob[m->i_sub*256 + m->i_state];
        int ip = *pv & 4095, ic = *pv >> 12;
        ip += ((bit << 12) - ip) >> IR[ic];
        if (ip < 1) ip = 1; else if (ip > 4095) ip = 4095;
        if (ic < 15) ic++;
        *pv = (uint16_t)((ic << 12) | ip);
        m->ihist[m->i_hidx] = (uint8_t)((m->i_state << 1) | bit);
        uint16_t *pv2 = &m->iprob2[m->i_sub*256 + m->i_state2];
        int ip2 = *pv2 & 4095, ic2 = *pv2 >> 12;
        ip2 += ((bit << 12) - ip2) >> IR[ic2];
        if (ip2 < 1) ip2 = 1; else if (ip2 > 4095) ip2 = 4095;
        if (ic2 < 15) ic2++;
        *pv2 = (uint16_t)((ic2 << 12) | ip2);
        m->ihist2[m->i_hidx2] = (uint8_t)((m->i_state2 << 1) | bit);
        uint16_t *pv3 = &m->iprob3[m->i_sub*256 + m->i_state3];
        int ip3 = *pv3 & 4095, ic3 = *pv3 >> 12;
        ip3 += ((bit << 12) - ip3) >> IR[ic3];
        if (ip3 < 1) ip3 = 1; else if (ip3 > 4095) ip3 = 4095;
        if (ic3 < 15) ic3++;
        *pv3 = (uint16_t)((ic3 << 12) | ip3);
        m->ihist3[m->i_hidx3] = (uint8_t)((m->i_state3 << 1) | bit);
        uint16_t *pv4 = &m->iprob4[m->i_sub*256 + m->i_state4];
        int ip4 = *pv4 & 4095, ic4 = *pv4 >> 12;
        ip4 += ((bit << 12) - ip4) >> IR[ic4];
        if (ip4 < 1) ip4 = 1; else if (ip4 > 4095) ip4 = 4095;
        if (ic4 < 15) ic4++;
        *pv4 = (uint16_t)((ic4 << 12) | ip4);
        m->ihist4[m->i_hidx4] = (uint8_t)((m->i_state4 << 1) | bit);
    }
    /* R89: cross-byte Bit-Kontext nachziehen (nur jpg) + Bit-Historie fortschreiben */
    if (g_jpg) {
        uint16_t *jcs[33]; jcs[32] = &m->jff2[m->jffctx2]; jcs[31] = &m->jff[m->jffctx]; jcs[30] = &m->jsp19[m->jsp19ctx]; jcs[0] = &m->jt[m->jctx]; jcs[1] = &m->jt2[m->jctx2]; jcs[2] = &m->jt3[m->jctx3]; jcs[3] = &m->jt4[m->jctx4]; jcs[4] = &m->jt5[m->jctx5]; jcs[5] = &m->jt6[m->jctx6]; jcs[6] = &m->jt7[m->jctx7]; jcs[7] = &m->jt8[m->jctx8]; jcs[8] = &m->jt9[m->jctx9]; jcs[9] = &m->jt10[m->jctx10]; jcs[10] = &m->jt11[m->jctx11]; jcs[11] = &m->jt12[m->jctx12]; jcs[12] = &m->jsp[m->jspctx]; jcs[13] = &m->jsp2[m->jsp2ctx]; jcs[14] = &m->jsp3[m->jsp3ctx]; jcs[15] = &m->jsp4[m->jsp4ctx]; jcs[16] = &m->jsp5[m->jsp5ctx]; jcs[17] = &m->jsp6[m->jsp6ctx]; jcs[18] = &m->jsp7[m->jsp7ctx]; jcs[19] = &m->jsp8[m->jsp8ctx]; jcs[20] = &m->jsp9[m->jsp9ctx]; jcs[21] = &m->jsp10[m->jsp10ctx]; jcs[22] = &m->jsp11[m->jsp11ctx]; jcs[23] = &m->jsp12[m->jsp12ctx]; jcs[24] = &m->jsp13[m->jsp13ctx]; jcs[25] = &m->jsp14[m->jsp14ctx]; jcs[26] = &m->jsp15[m->jsp15ctx]; jcs[27] = &m->jsp16[m->jsp16ctx]; jcs[28] = &m->jsp17[m->jsp17ctx]; jcs[29] = &m->jsp18[m->jsp18ctx];
        for (int ji = 0; ji < 33; ji++) {
            uint16_t *jc = jcs[ji];
            int jp = *jc & PMASK, jcnt = *jc >> 12;
            jp += ((bit << 12) - jp) >> (g_jsh ? RATE_JE[jcnt] : RATE_J[jcnt]);
            if (jp < 1) jp = 1; else if (jp > 4095) jp = 4095;
            if (jcnt < 15) jcnt++;
            *jc = (uint16_t)((jcnt << 12) | jp);
        }
    }
    m->bhist = (m->bhist << 1) | (uint32_t)bit;
    /* Bit in Bit-Baum + ggf. Byte-Historie */
    m->c0 = (m->c0 << 1) | bit;
    if (m->c0 >= 256) {
        int byte = m->c0 & 0xFF;
        m->hist = (m->hist << 8) | (uint64_t)byte;
        if (byte == 0xFF) m->ffdist = 0; else if (m->ffdist < 63) m->ffdist++;  /* R148: FF-Distanz fortschreiben (byte-symmetrisch enc/dec) */
        m->c0 = 1;
    }
}

/* ---- binaerer Range-Coder (32-bit, x1/x2) ---- */
typedef struct { uint32_t x1, x2; unsigned char *out; size_t n, cap; } Enc;
static void enc_init(Enc *e) { e->x1 = 0; e->x2 = 0xFFFFFFFFu; e->n = 0;
    e->cap = 1 << 20; e->out = malloc(e->cap); }
static void enc_put(Enc *e, unsigned char b) {
    if (e->n == e->cap) { e->cap <<= 1; e->out = realloc(e->out, e->cap); }
    e->out[e->n++] = b;
}
static void enc_bit(Enc *e, int bit, int p1) {
    uint32_t xmid = e->x1 + (uint32_t)(((uint64_t)(e->x2 - e->x1) * p1) >> 12);
    if (xmid < e->x1) xmid = e->x1; else if (xmid >= e->x2) xmid = e->x2 - 1;
    if (bit) e->x2 = xmid; else e->x1 = xmid + 1;
    while (((e->x1 ^ e->x2) & 0xFF000000u) == 0) {
        enc_put(e, (unsigned char)(e->x2 >> 24));
        e->x1 <<= 8; e->x2 = (e->x2 << 8) | 0xFF;
    }
}
static void enc_flush(Enc *e) {
    for (int i = 0; i < 4; i++) { enc_put(e, (unsigned char)(e->x1 >> 24));
        e->x1 <<= 8; }
}

typedef struct { uint32_t x1, x2, x; const unsigned char *in; size_t n, pos; } Dec;
static unsigned char dec_get(Dec *d) { return d->pos < d->n ? d->in[d->pos++] : 0; }
static void dec_init(Dec *d, const unsigned char *in, size_t n) {
    d->x1 = 0; d->x2 = 0xFFFFFFFFu; d->in = in; d->n = n; d->pos = 0; d->x = 0;
    for (int i = 0; i < 4; i++) d->x = (d->x << 8) | dec_get(d);
}
static int dec_bit(Dec *d, int p1) {
    uint32_t xmid = d->x1 + (uint32_t)(((uint64_t)(d->x2 - d->x1) * p1) >> 12);
    if (xmid < d->x1) xmid = d->x1; else if (xmid >= d->x2) xmid = d->x2 - 1;
    int bit;
    if (d->x <= xmid) { bit = 1; d->x2 = xmid; }
    else { bit = 0; d->x1 = xmid + 1; }
    while (((d->x1 ^ d->x2) & 0xFF000000u) == 0) {
        d->x1 <<= 8; d->x2 = (d->x2 << 8) | 0xFF;
        d->x = (d->x << 8) | dec_get(d);
    }
    return bit;
}

/* ---- CM-Kern auf einem Puffer (ein Modell, seriell) ---- */
static unsigned char *cm_encode(const unsigned char *data, size_t sz, size_t *outn) {
    g_jsh = (g_jpg && g_nctx == NCTX) ? g_jshift : 0;   /* R109: jpeg-Full-Pfad kuerzere cross-byte-K */
    Model m; model_init(&m);
    m.mm_buf = data;                       /* R13: Verlauf = Eingabe */
    Enc e; enc_init(&e);
    for (size_t i = 0; i < sz; i++) {
        int byte = data[i];
        m.mm_pos = (uint32_t)i;
        model_new_byte_ctx(&m);
        for (int b = 7; b >= 0; b--) {
            int bit = (byte >> b) & 1;
            int p = model_predict(&m);
            enc_bit(&e, bit, p);
            model_update(&m, bit);
        }
        model_byte_end(&m, byte, (uint32_t)i);   /* R13: Match fortschreiben */
    }
    enc_flush(&e);
    model_free(&m);
    *outn = e.n;
    return e.out;
}
static void cm_decode(const unsigned char *in, size_t inn, unsigned char *out, size_t sz) {
    g_jsh = (g_jpg && g_nctx == NCTX) ? g_jshift : 0;   /* R109: jpeg-Full-Pfad kuerzere cross-byte-K */
    Model m; model_init(&m);
    m.mm_buf = out;                        /* R13: Verlauf = bisher decodierte Ausgabe */
    Dec d; dec_init(&d, in, inn);
    for (size_t i = 0; i < sz; i++) {
        int byte = 0;
        m.mm_pos = (uint32_t)i;
        model_new_byte_ctx(&m);
        for (int b = 7; b >= 0; b--) {
            int p = model_predict(&m);
            int bit = dec_bit(&d, p);
            model_update(&m, bit);
            byte = (byte << 1) | bit;
        }
        out[i] = (unsigned char)byte;            /* R13: erst schreiben ... */
        model_byte_end(&m, byte, (uint32_t)i);   /* ... dann Match fortschreiben */
    }
    model_free(&m);
}

static void put32(unsigned char *p, uint32_t v) { for (int i = 0; i < 4; i++) p[i] = (v >> (8*i)) & 0xFF; }
static uint32_t get32(const unsigned char *p) { uint32_t v = 0; for (int i = 0; i < 4; i++) v |= (uint32_t)p[i] << (8*i); return v; }

/* R6 (ALLE-FORMATE): MEHRKERN via unabhaengige Bloecke. Grosse Dateien werden in
 * BLOCK-Stuecke geteilt, jedes Stueck hat sein EIGENES Modell -> byte-exakt &
 * deterministisch, egal wie viele Kerne. Der schnelle R5-Warmup (adaptive Rate)
 * macht die Block-Reset-Kosten klein. Kleine Dateien (<= BLOCK) bleiben EIN Block
 * im alten Format (flag 0/1) -> byte-IDENTISCH, alle kleinen Siege geschuetzt. */
#define BLOCK (1u << 17)   /* R209: 256->128 KB — mehr Bloecke -> alle 16 Kerne auf grossen Dateien, −16% Zeit */

typedef struct { const unsigned char *in; size_t insz; unsigned char *pay; size_t payn; int israw; } Job;
static void *enc_worker(void *arg) {
    Job *j = arg;
    size_t sn; unsigned char *s = cm_encode(j->in, j->insz, &sn);
    if (sn >= j->insz) { j->israw = 1; free(s); j->pay = (unsigned char*)j->in; j->payn = j->insz; }
    else { j->israw = 0; j->pay = s; j->payn = sn; }
    return NULL;
}
typedef struct { const unsigned char *pay; size_t payn; unsigned char *out; size_t outsz; int israw; } DJob;
static void *dec_worker(void *arg) {
    DJob *j = arg;
    if (j->israw) memcpy(j->out, j->pay, j->outsz);
    else cm_decode(j->pay, j->payn, j->out, j->outsz);
    return NULL;
}

/* R144/R145: x86-Call/Jmp-Ziel-Vorfilter (Ziel-Vorhersage aus Position). E8/E9 = CALL/JMP rel32;
 * rel32 = Ziel - naechster-Befehl. Zwei Aufrufe DERSELBEN Funktion aus verschiedenen Positionen
 * haben VERSCHIEDENE rel32 -> Match-Modell sieht keine Wiederkehr. Umrechnung rel->absolut
 * (abs = rel + Position + 5) macht identische Ziele zu IDENTISCHEN 4 Bytes -> matchbar/geclustert.
 * R145: 0x00/0xFF-Top-Byte-MASKE (zustandsbehaftet, Standard-x86-Filter) — konvertiert nur wo das
 * Ziel-Top-Byte 0x00/0xFF ist (echte Call-Ziele im PIE-Bereich), laesst zufaellige .rodata-Bytes
 * die als E8/E9 aussehen UNVERAENDERT -> keine Daten-Verwuerfelung. Reine reversible Vorhersage-
 * Transform auf ROHEN Bytes (wie Byte-Ebenen-Delta), KEIN Format-Dekode; exakt invertierbar (enc/dec
 * spiegeln denselben Positions-Scan + dieselbe Maske). */
static int bcj_test_ms(unsigned char b) { return b == 0 || b == 0xFF; }
static void bcj_x86(unsigned char *data, size_t n, int enc) {
    if (n < 5) return;
    size_t size = n - 4, pos = 0;
    uint32_t mask = 0, ip = 5;
    for (;;) {
        size_t p = pos;
        while (p < size && (data[p] & 0xFE) != 0xE8) p++;
        size_t d = p - pos;
        pos = p;
        if (p >= size) break;
        if (d > 2) mask = 0;
        else {
            mask >>= (unsigned)d;
            if (mask != 0 && (mask > 4 || mask == 3 || bcj_test_ms(data[p + (mask >> 1) + 1]))) {
                mask = (mask >> 1) | 4; pos++; continue;
            }
        }
        if (bcj_test_ms(data[p + 4])) {
            uint32_t v = ((uint32_t)data[p+4] << 24) | ((uint32_t)data[p+3] << 16)
                       | ((uint32_t)data[p+2] << 8) | (uint32_t)data[p+1];
            uint32_t cur = ip + (uint32_t)pos;
            pos += 5;
            v = enc ? (v + cur) : (v - cur);
            if (mask != 0) {
                unsigned sh = (mask & 6) << 2;
                if (bcj_test_ms((unsigned char)(v >> sh))) {
                    v ^= (((uint32_t)0x100 << sh) - 1);
                    v = enc ? (v + cur) : (v - cur);
                }
                mask = 0;
            }
            data[p+1] = (unsigned char)v;
            data[p+2] = (unsigned char)(v >> 8);
            data[p+3] = (unsigned char)(v >> 16);
            data[p+4] = (unsigned char)(0 - ((v >> 24) & 1));
        } else {
            mask = (mask >> 1) | 4; pos++;
        }
    }
}

/* R151: x86 0F 8x Jcc (bedingter Sprung) rel32-Ziel-Vorfilter — ergaenzt bcj_x86 (das nur E8/E9
 * fasst). 0F 80..0F 8F = Jcc rel32, 6-Byte-Befehl (2-Byte-Opcode + rel32). Die 2-Byte-Praefix
 * (0F 8x) wird NIE veraendert (nur das rel32 an p+2..p+5) -> enc/dec finden EXAKT dieselben
 * Positionen, der Scan ist praefix-invariant -> exakt reversibel OHNE Top-Byte-Maske. rel->abs
 * (abs = rel + Position + 6) macht identische Branch-Ziele zu IDENTISCHEN 4 Bytes (matchbar).
 * Reine reversible Vorhersage-Transform auf ROHEN Bytes (wie bcj_x86), KEIN Format-Dekode. */
static void bcj_jcc(unsigned char *data, size_t n, int enc) {
    if (n < 6) return;
    size_t p = 0;
    while (p + 6 <= n) {
        if (data[p] == 0x0F && (data[p+1] & 0xF0) == 0x80) {
            uint32_t v = (uint32_t)data[p+2] | ((uint32_t)data[p+3] << 8)
                       | ((uint32_t)data[p+4] << 16) | ((uint32_t)data[p+5] << 24);
            uint32_t cur = (uint32_t)(p + 6);
            v = enc ? (v + cur) : (v - cur);
            data[p+2] = (unsigned char)v;       data[p+3] = (unsigned char)(v >> 8);
            data[p+4] = (unsigned char)(v >> 16); data[p+5] = (unsigned char)(v >> 24);
            p += 6;
        } else p++;
    }
}

/* R152: x86-64 rip-relative disp32-Datenzugriff-Vorfilter (lea/mov [rip+disp32]). Der Standard-BCJ
 * (E8/E9 R144 + Jcc R151) fasst nur BRANCH-Ziele; PIC-ELF greift auf jeden globalen Wert via
 * rip+disp32 zu (lib.so 3583, davon 2766 auf wiederkehrende Ziele). Muster REX(40-4f) + opcode
 * {8b/8d/89/03/2b/63} + ModRM(mod=00,rm=101 -> &0xC7==0x05) + disp32; kein SIB/imm -> 7-Byte-Befehl,
 * disp32 an p+3..p+6. Die 3-Byte-Praefix wird NIE veraendert -> enc/dec praefix-invariant, exakt
 * reversibel OHNE Maske. abs = disp + Position + 7 macht identische Global-Referenzen zu IDENTISCHEN
 * 4 Bytes (matchbar/geclustert), genau wie E8/E9 (R144). Reine reversible Roh-Byte-Transform, KEIN
 * Format-Dekode. */
static int bcj_rip_op(unsigned char b) {
    /* R152-Satz {mov/lea/add/sub/movsxd} + R154: die restlichen 1-Byte-ALU-Opcodes mit
     * rip-relativem (r/m,r)/(r,r/m)-Form (7-Byte-Befehl, REX-gegatet = 3-Byte-Gate wie R152).
     * Jeder referenziert Globals via rip+disp32; der REX-Gate haelt False-Positives niedrig. */
    switch (b) {
        case 0x8b: case 0x8d: case 0x89: case 0x03: case 0x2b: case 0x63: /* R152 */
        case 0x01: case 0x09: case 0x11: case 0x19: case 0x21: case 0x29: case 0x31: case 0x39: /* add/or/adc/sbb/and/sub/xor/cmp  m,r */
        case 0x0b: case 0x13: case 0x1b: case 0x23: case 0x33: case 0x3b: /* or/adc/sbb/and/xor/cmp  r,m */
        case 0x85: case 0x87: /* test / xchg */
            return 1;
    }
    return 0;
}
/* R155: SSE/SSE2 2-Byte-0F-Opcodes mit rip-relativem Speicheroperand (Const-Load/Store). */
static int bcj_sse_op(unsigned char b) {
    switch (b) {
        case 0x10: case 0x11: /* movups/movss/movsd */
        case 0x28: case 0x29: /* movaps */
        case 0x6f: case 0x7f: /* movdqa/movdqu */
        case 0xd6:            /* movq store */
        case 0x12: case 0x13: case 0x16: case 0x17: /* movlp/movhp */
        case 0x2a: case 0x2c: case 0x2d: /* cvtsi2ss/cvt(t)ss2si */
        case 0x2e: case 0x2f: /* ucomiss/comiss */
        case 0x51: case 0x54: case 0x57: /* sqrt/and/xor */
        case 0x58: case 0x59: case 0x5c: case 0x5e: /* add/mul/sub/div */
            return 1;
    }
    return 0;
}
/* Wandelt das disp32 an data[dp..dp+3] rel<->abs; die Praefix-Bytes (0..dp-1) bleiben UNVERAENDERT
 * -> praefix-invariant, exakt reversibel. cur = Position DES NAECHSTEN Befehls (rip-Basis). */
static void bcj_rip_conv(unsigned char *data, size_t dp, uint32_t cur, int enc) {
    uint32_t v = (uint32_t)data[dp] | ((uint32_t)data[dp+1] << 8)
               | ((uint32_t)data[dp+2] << 16) | ((uint32_t)data[dp+3] << 24);
    v = enc ? (v + cur) : (v - cur);
    data[dp]   = (unsigned char)v;        data[dp+1] = (unsigned char)(v >> 8);
    data[dp+2] = (unsigned char)(v >> 16); data[dp+3] = (unsigned char)(v >> 24);
}
static void bcj_rip(unsigned char *data, size_t n, int enc) {
    if (n < 7) return;
    size_t p = 0;
    while (p + 6 <= n) {
        /* R152: REX + mov/lea/add/sub/movsxd [rip+disp32] (REX-pflichtig = wenig False-Positives) */
        if (p + 7 <= n && data[p] >= 0x40 && data[p] <= 0x4f && bcj_rip_op(data[p+1])
            && (data[p+2] & 0xC7) == 0x05) {
            bcj_rip_conv(data, p + 3, (uint32_t)(p + 7), enc); p += 7; continue;
        }
        /* R153: REX + C7 /0 mov [rip+disp32], imm32 (REX-pflichtig; disp32 GEFOLGT von imm32) */
        if (p + 11 <= n && data[p] >= 0x40 && data[p] <= 0x4f && data[p+1] == 0xC7
            && data[p+2] == 0x05) {
            bcj_rip_conv(data, p + 3, (uint32_t)(p + 11), enc); p += 11; continue;
        }
        /* R154: REX + 0F {B6/B7/BE/BF} movzx/movsx [rip+disp32] (8B, 4-Byte-Gate = sehr sauber)
         * (81/83/FF-Formen getestet + verworfen: 81 neutral, FF +1, 83 lib +143 = Regression) */
        if (p + 8 <= n && data[p] >= 0x40 && data[p] <= 0x4f && data[p+1] == 0x0F
            && (data[p+2]==0xB6||data[p+2]==0xB7||data[p+2]==0xBE||data[p+2]==0xBF)
            && (data[p+3] & 0xC7) == 0x05) {
            bcj_rip_conv(data, p + 4, (uint32_t)(p + 8), enc); p += 8; continue;
        }
        /* R155: SSE/SSE2 [rip+disp32] loads/stores (movups/movaps/movss/movsd/movdqu/movq +
         * const-load arith): [66/F2/F3]? [REX]? 0F op ModRM(mod00 rm101) disp32. Praefix/opcode/
         * ModRM nie veraendert -> praefix-invariant, exakt reversibel; keep-if-smaller schuetzt. */
        {   size_t o = 0;
            if (data[p]==0x66||data[p]==0xF2||data[p]==0xF3) o = 1;
            if (p+o < n && data[p+o] >= 0x40 && data[p+o] <= 0x4f) o += 1;
            if (p+o+3+4 <= n && data[p+o]==0x0F && bcj_sse_op(data[p+o+1])
                && (data[p+o+2] & 0xC7) == 0x05) {
                size_t len = o + 3 + 4;
                bcj_rip_conv(data, p + o + 3, (uint32_t)(p + len), enc); p += len; continue;
            }
        }
        p++;
    }
}

static int compress_file(const char *inp, const char *outp) {
    FILE *f = fopen(inp, "rb"); if (!f) return 1;
    fseek(f, 0, SEEK_END); long sz = ftell(f); fseek(f, 0, SEEK_SET);
    unsigned char *data = malloc(sz ? sz : 1);
    if (sz && fread(data, 1, sz, f) != (size_t)sz) { fclose(f); free(data); return 1; }
    fclose(f);

    FILE *o = fopen(outp, "wb"); if (!o) { free(data); return 1; }

    if ((size_t)sz <= BLOCK) {
        /* --- Einzelblock: max Ratio (TB20) --- */
        set_tbits(TBITS); g_nctx = NCTX;
        g_jpg = (sz >= 2 && data[0]==0xFF && data[1]==0xD8) ? 1 : 0;  /* R108: kleine jpeg (Full-Pfad) auch cross-byte-gaten, Flag 6 */
        size_t en; unsigned char *es = cm_encode(data, (size_t)sz, &en);
        if ((uint64_t)en + 8 < (uint64_t)sz) {
            unsigned char flag = g_jpg ? 6 : 1; fwrite(&flag, 1, 1, o);
            unsigned char hdr[8]; for (int i=0;i<8;i++) hdr[i]=((uint64_t)sz>>(8*i))&0xFF;
            fwrite(hdr, 1, 8, o); fwrite(es, 1, en, o);
        } else {
            unsigned char flag = 0; fwrite(&flag, 1, 1, o);
            if (sz) fwrite(data, 1, (size_t)sz, o);
        }
        free(es); fclose(o); free(data); return 0;
    }

    /* --- Mehrblock (flag 2), parallel --- */
    uint32_t nb = (uint32_t)(((size_t)sz + BLOCK - 1) / BLOCK);
    set_tbits(nb <= 12 ? TBITS : BLOCK_TBITS);  /* R21 */
    g_heic = 0;  /* R30: heic-Magic (ftypheic bei Offset 4) -> weak-binaer will g_nctx=4 (order-3), jpg NICHT */
    if (nb > 12 && sz >= 12 && data[4]=='f'&&data[5]=='t'&&data[6]=='y'&&data[7]=='p'
        && data[8]=='h'&&data[9]=='e'&&data[10]=='i'&&data[11]=='c') g_heic = 1;
    g_jpg = 0;  /* R89: jpg-Magic (FFD8 SOI) -> cross-byte Bit-Historie-Eingang (JBIT), sonst byte-identisch */
    if (nb > 12 && sz >= 2 && !g_heic && data[0]==0xFF && data[1]==0xD8) g_jpg = 1;
    g_nctx = (nb > 12) ? (g_heic ? heic_nctx : 3) : NCTX; /* R352 + R30 + R61 heic_nctx */
    /* R369: weak-Pfad ASCII-Klassifikator — strukturierter CAD-Text (stl) will {0,2,4} statt {0,1,2};
     * heic/jpg-Rauschen bleibt {0,1,2}. Nur relevant im weak-Pfad (nb>12); Flag 3 statt 2 traegt die Wahl. */
    g_word = 0;
    {  /* R372: ASCII-Klassifikator fuer ALLE Multiblock-Dateien (auch full-Pfad obj/step) -> Kruemmungs-Bit */
        size_t na = 0, lim = (size_t)sz < 262144 ? (size_t)sz : 262144;
        for (size_t i = 0; i < lim; i++) { unsigned char c = data[i];
            if ((c >= 0x20 && c <= 0x7E) || (c >= 0x09 && c <= 0x0D)) na++; }
        if (lim && na * 100 >= lim * 75) g_word = 1;  /* >=75% druckbares ASCII -> Text */
    }
    int bcj = 0;  /* R144: x86-ELF -> BCJ-Vorfilter (Call/Jmp-Ziel-Vorhersage), Flag 7; nur bin/lib matchen -> 15 andere byte-identisch */
    if (!g_word && !g_heic && !g_jpg && sz >= 4
        && data[0]==0x7F && data[1]=='E' && data[2]=='L' && data[3]=='F') {
        bcj_rip(data, (size_t)sz, 1); bcj_jcc(data, (size_t)sz, 1); bcj_x86(data, (size_t)sz, 1); bcj = 1;  /* R152/R151: LIFO rip-enc, jcc-enc, e8e9-enc */
    }
    Job *jobs = calloc(nb, sizeof(Job));
    pthread_t *th = calloc(nb, sizeof(pthread_t));
    for (uint32_t b = 0; b < nb; b++) {
        size_t off = (size_t)b * BLOCK;
        jobs[b].in = data + off;
        jobs[b].insz = (off + BLOCK <= (size_t)sz) ? BLOCK : ((size_t)sz - off);
    }
    long ncpu = portable_ncpu();
    for (uint32_t b = 0; b < nb; ) {
        uint32_t k = 0;
        for (; k < (uint32_t)ncpu && b + k < nb; k++)
            pthread_create(&th[b+k], NULL, enc_worker, &jobs[b+k]);
        for (uint32_t j = 0; j < k; j++) pthread_join(th[b+j], NULL);
        b += k;
    }
    /* R7: schlanker Header. origlen ist redundant (=BLOCK ausser letzter Block,
     * ableitbar aus filesize), bflag passt ins MSB von complen. Statt 5+9n nur
     * 9+4n Bytes -> spart 5n-4 pro Mehrblock-Datei (staerkt die knappe Ratio-Fuehrung). */
    unsigned char flag = bcj ? 7 : (g_word ? 3 : (g_heic ? 4 : (g_jpg ? 5 : 2))); fwrite(&flag, 1, 1, o);  /* R369: 3=ASCII-weak; R30: 4=heic-order-3; R89: 5=jpg cross-byte-Bit; R144: 7=BCJ-x86-ELF */
    unsigned char szb[8]; for (int i=0;i<8;i++) szb[i]=((uint64_t)sz>>(8*i))&0xFF;
    fwrite(szb, 1, 8, o);
    for (uint32_t b = 0; b < nb; b++) {          /* Tabelle: complen (MSB=israw) */
        unsigned char t[4];
        put32(t, (uint32_t)jobs[b].payn | (jobs[b].israw ? 0x80000000u : 0));
        fwrite(t, 1, 4, o);
    }
    for (uint32_t b = 0; b < nb; b++) {          /* Nutzlasten */
        fwrite(jobs[b].pay, 1, jobs[b].payn, o);
        if (!jobs[b].israw) free(jobs[b].pay);
    }
    fclose(o); free(jobs); free(th); free(data);
    return 0;
}

static int decompress_file(const char *inp, const char *outp) {
    FILE *f = fopen(inp, "rb"); if (!f) return 1;
    fseek(f, 0, SEEK_END); long fsz = ftell(f); fseek(f, 0, SEEK_SET);
    unsigned char *blob = malloc(fsz ? fsz : 1);
    if (fsz && fread(blob, 1, fsz, f) != (size_t)fsz) { fclose(f); free(blob); return 1; }
    fclose(f);
    if (fsz < 1) { free(blob); return 1; }
    unsigned char flag = blob[0];

    FILE *o = fopen(outp, "wb"); if (!o) { free(blob); return 1; }
    if (flag == 0) {                              /* roh */
        if (fsz > 1) fwrite(blob + 1, 1, (size_t)(fsz - 1), o);
        fclose(o); free(blob); return 0;
    }
    if (flag == 1 || flag == 6) {                 /* Einzelblock CM (6 = jpeg cross-byte, R108) */
        set_tbits(TBITS); g_nctx = NCTX; g_jpg = (flag == 6) ? 1 : 0;
        if (fsz < 9) { fclose(o); free(blob); return 1; }
        uint64_t n = 0; for (int i=0;i<8;i++) n |= (uint64_t)blob[1+i] << (8*i);
        unsigned char *out = malloc(n ? n : 1);
        cm_decode(blob + 9, (size_t)(fsz - 9), out, (size_t)n);
        if (n) fwrite(out, 1, (size_t)n, o);
        fclose(o); free(out); free(blob); return 0;
    }
    g_word = (flag == 3) ? 1 : 0;  /* R369: weak-Ordnungs-Variante aus Flag */
    g_heic = (flag == 4) ? 1 : 0;  /* R30: heic-order-3 aus Flag */
    g_jpg = (flag == 5) ? 1 : 0;   /* R89: jpg cross-byte-Bit aus Flag */
    /* flag == 2/3: Mehrblock, parallel (R7 schlanker Header) */
    if (fsz < 9) { fclose(o); free(blob); return 1; }
    uint64_t fbytes = 0; for (int i=0;i<8;i++) fbytes |= (uint64_t)blob[1+i] << (8*i);
    uint32_t nb = (uint32_t)((fbytes + BLOCK - 1) / BLOCK);
    set_tbits(nb <= 12 ? TBITS : BLOCK_TBITS);
    g_nctx = (nb > 12) ? (g_heic ? heic_nctx : 3) : NCTX; /* R352 + R30 + R61 heic_nctx */              /* R214: nb>3-Dateien 3 Kontexte (+APM-Skip R212) — sichere Margen, docx/klein voll */
    const unsigned char *tab = blob + 9;
    const unsigned char *pay = tab + (size_t)nb * 4;
    DJob *jobs = calloc(nb, sizeof(DJob));
    pthread_t *th = calloc(nb, sizeof(pthread_t));
    size_t total = 0, poff = 0;
    for (uint32_t b = 0; b < nb; b++) {
        uint32_t e = get32(tab + (size_t)b*4);
        jobs[b].israw = (e >> 31) & 1;
        uint32_t cl = e & 0x7FFFFFFFu;
        size_t off = (size_t)b * BLOCK;
        uint32_t ol = (off + BLOCK <= fbytes) ? BLOCK : (uint32_t)(fbytes - off);
        jobs[b].outsz = ol; jobs[b].payn = cl; jobs[b].pay = pay + poff;
        poff += cl; total += ol;
    }
    unsigned char *out = malloc(total ? total : 1);
    size_t ooff = 0;
    for (uint32_t b = 0; b < nb; b++) { jobs[b].out = out + ooff; ooff += jobs[b].outsz; }
    long ncpu = portable_ncpu();
    for (uint32_t b = 0; b < nb; ) {
        uint32_t k = 0;
        for (; k < (uint32_t)ncpu && b + k < nb; k++)
            pthread_create(&th[b+k], NULL, dec_worker, &jobs[b+k]);
        for (uint32_t j = 0; j < k; j++) pthread_join(th[b+j], NULL);
        b += k;
    }
    if (flag == 7) { bcj_x86(out, total, 0); bcj_jcc(out, total, 0); bcj_rip(out, total, 0); }  /* R144/R151/R152: BCJ invertieren (e8e9, jcc, rip; LIFO) vor dem Schreiben */
    if (total) fwrite(out, 1, total, o);
    fclose(o); free(out); free(jobs); free(th); free(blob);
    return 0;
}

int main(int argc, char **argv) {
    if (argc != 4) {
        fprintf(stderr, "usage: %s c|d IN OUT\n", argv[0]);
        return 2;
    }
    init_tables();
#if defined(__linux__) && defined(PR_SET_THP_DISABLE)
    prctl(PR_SET_THP_DISABLE, 0, 0, 0, 0);   /* ZEIT R209: geerbtes THP-Disable loeschen — MADV_HUGEPAGE soll ueberall greifen */
#endif
    if (argv[1][0] == 'c') return compress_file(argv[2], argv[3]);
    if (argv[1][0] == 'd') return decompress_file(argv[2], argv[3]);
    fprintf(stderr, "unknown mode\n");
    return 2;
}
