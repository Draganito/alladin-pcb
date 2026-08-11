# Alladin PCB — Bedienungsanleitung für Anfänger

Alladin ist für **Hobby und Bastler**: schnell von der Idee zu einer
JLCPCB-fertigen Platine (ESP32, Smart-Home, kleine Robotik-Boards) —
kein volles Profi-EDA, sondern ein schlanker Start.

Diese Anleitung erklärt Schritt für Schritt, wie man mit **Alladin PCB**
ein eigenes, produktionsreifes 2-Layer-Board baut — von der leeren Fläche
bis zur fertig gerouteten Platine mit GND-Netz, 5V-Versorgung, Vias und
einer Kupferfläche (Pour). Als durchgehendes Praxisbeispiel dient die
denkbar einfachste sinnvolle Schaltung: eine **LED mit Vorwiderstand an
5V/GND**. Die gleichen Schritte (Netz anlegen → verbinden → routen →
Via setzen → Zone ziehen) funktionieren identisch für jede größere
Schaltung, z.B. ein ESP32-Board.

Die Oberfläche der App ist auf Englisch, diese Anleitung ist auf
Deutsch — Knopfbeschriftungen werden deshalb immer in Anführungszeichen
mit dem englischen Original zitiert, z.B. „Route traces".

## 0. Grundbegriffe

Bevor es losgeht, kurz die wichtigsten Bausteine, die überall in der App
auftauchen:

| Begriff | Bedeutung |
|---|---|
| **Footprint / Bauteil** | Ein platziertes Teil (Widerstand, LED, Stecker, ESP32-Modul, …) mit seinen Anschlussflächen (Pads). |
| **Pad** | Ein einzelner Lötanschluss eines Footprints. |
| **Netz (Net)** | Eine elektrische Verbindung, die aus mehreren Pads besteht, die alle das gleiche Potential führen sollen — z.B. "GND" oder "5V". Ein Netz ist zunächst nur eine *logische* Verknüpfung, noch keine Kupferbahn. |
| **Track (Leiterbahn)** | Die echte Kupferverbindung zwischen zwei Punkten desselben Netzes. |
| **Via** | Ein durchkontaktiertes Loch, das eine Leiterbahn von der Ober- (F.Cu) auf die Unterseite (B.Cu) wechseln lässt (oder umgekehrt). |
| **Zone / Pour** | Eine großflächige Kupferfläche (meist für GND), die frei gezeichnet und automatisch gefüllt wird. |
| **Layer** | Kupferlage: F.Cu (oben) und B.Cu (unten) bei einem 2-Layer-Board. |
| **DRC** | Design Rule Check — die Prüfung, ob Abstände/Kurzschlüsse den Fertigungsregeln (hier: JLCPCB) entsprechen. |

## 1. App starten

Wer das fertige Debian-Paket installiert hat (siehe README, Abschnitt
„Download"), startet einfach:

```bash
alladin-pcb
```

Wer stattdessen mit dem Quellcode arbeitet, startet im Terminal im
Alladin-Projektordner (dort, wo `Cargo.toml` und diese Anleitung liegen):

```bash
cargo run -p alladin-pcb
```

Ohne weitere Argumente startet **immer die grafische Oberfläche**, nie
direkt mit einer geöffneten Datei — du musst eine vorhandene Datei danach
über „Open…" laden (siehe Abschnitt 8). Mit Argumenten (z.B.
`cargo run -p alladin-pcb -- --help`) läuft die gleiche Datei stattdessen
als Kommandozeilen-Tool (CLI) — für headless Hilfen wie `new-board` oder
`download-part`. Die KI steuert Alladin über **MCP** in der GUI (nicht
über die CLI); für diese Anleitung brauchst du die CLI nicht.

## 2. Neues Board anlegen

Direkt nach dem Start siehst du den Bildschirm „New board":

1. Optional: **„Import outline DXF…"** — geschlossene Kontur aus LibreCAD
   oder FreeCAD (eine `LWPOLYLINE`, oder ein Ring aus `LINE`/`ARC`).
   Dann kommen die Maße aus der DXF; Width/Height/Corner radius entfallen.
2. Sonst: **Width (mm)** / **Height (mm)** — Außenmaße, z.B. `40` x `30`.
3. **Layers** — für ein normales 2-Layer-Board bei der Standardeinstellung lassen.
4. **Corner radius (mm)** — abgerundete Ecken, `0` für rechteckig (nur ohne DXF).
5. Auf **„Create board"** klicken.

Danach bist du im Haupteditor. Falls du das Board nie siehst (zu weit
rein-/rausgezoomt), oben auf **„Fit to board"** klicken.

## 3. Bauteile besorgen

Rechts im Seitenpanel gibt es zwei Bereiche:

- **„Place part"** — eine Liste bereits vorhandener Bauteil-Vorlagen
  (Templates), inklusive ein paar eingebauter Standardteile (0603/0805-
  Widerstände/Kondensatoren, Stiftleisten, …).
- **„Download part (LCSC)"** — ein Textfeld für eine LCSC-Bestellnummer
  (z.B. `C2040`). Nummer eintragen, Enter oder **„Download"** klicken —
  die App lädt Footprint/Pads automatisch aus der LCSC/EasyEDA-Datenbank
  und legt sie lokal in deiner Parts-Datenbank ab. Danach taucht das Teil
  dauerhaft in der „Place part"-Liste auf.

Für das LED-Beispiel reichen die eingebauten 0805-Bauteile:

- 1x Widerstand 0805 (z.B. 330 Ω, für eine normale rote/grüne LED an 5V)
- 1x LED 0805 (falls kein LED-Template vorhanden ist: per LCSC-Nummer
  einer 0805-LED herunterladen, z.B. eine gängige C-Nummer wie `C2286`)

## 4. Bauteile platzieren

1. In der Liste „Place part" auf das gewünschte Bauteil klicken — der
   Cursor ist jetzt im **Place**-Modus.
2. Mit der Maus über das Board fahren: es erscheint eine „Geister"-Vorschau.
3. **Taste `R`** dreht die Vorschau in 90°-Schritten, solange man sich
   noch im Place-Modus befindet.
4. Klick auf die gewünschte Position setzt das Bauteil endgültig.
5. **Escape** verlässt den Place-Modus wieder (zurück zu „Select").

Platziere so den Widerstand und die LED nebeneinander irgendwo auf dem
Board, mit etwas Abstand zwischen den Pads (macht das spätere Routen
einfacher).

**Später korrigieren:** Im **„Select"**-Modus (Standardwerkzeug, kein
Knopf nötig) kannst du ein Bauteil anklicken und bei gehaltener Maustaste
verschieben; `R` dreht ein *ausgewähltes* Bauteil ebenfalls in 90°-
Schritten. `Delete`/`Backspace` entfernt das ausgewählte Bauteil.

## 5. Netze anlegen und sinnvoll benennen

Ganz unten im rechten Seitenpanel steht die Überschrift **„Nets (n)"**
mit einer Liste aller bisher existierenden Netze. Jeder Netzname dort ist
ein **editierbares Textfeld** — du kannst z.B. „Net1" direkt anklicken
und in `GND` bzw. `5V` umbenennen. Der Name ist rein kosmetisch (für dich
in der Liste und in der BOM), hat aber keine Sonderfunktion in der Software
selbst — Alladin behandelt „GND" nicht anders als „Net7".

Netze entstehen aber erst dadurch, dass du zwei Pads miteinander
verbindest — das macht das Werkzeug **„Connect pins"**:

1. Oben im Panel auf **„Connect pins"** klicken.
2. Ersten Pin anklicken (er wird markiert, Hinweistext: „First pin
   selected — click the pin to connect it to.").
3. Zweiten Pin anklicken → beide werden auf dasselbe Netz gelegt (ist
   noch keines von beiden zugeordnet, entsteht ein neues Netz; ist eines
   der beiden schon in einem Netz, kommt das andere Pad dazu).
4. **Shift-Klick** auf einen Pin entfernt ihn wieder aus seinem Netz.
5. **Escape** beendet den aktuellen Verbindungsvorgang (falls ein erster
   Pin schon ausgewählt war).

## 6. Praxisbeispiel: LED an 5V/GND

Angenommen du hast einen Widerstand (Pin 1/Pin 2) und eine LED (Anode/
Kathode) platziert, plus irgendeine Spannungsquelle auf dem Board (z.B.
ein Stecker-Pad, das später extern mit 5V/GND versorgt wird). Verbinde
mit „Connect pins" wie folgt:

1. **5V-Quelle-Pad** ↔ **Widerstand Pin 1** → neues Netz entsteht, in der
   „Nets"-Liste umbenennen in `5V`.
2. **Widerstand Pin 2** ↔ **LED Anode** → neues Netz, z.B. `LED_A` nennen
   (kein Massepotential, Name frei wählbar).
3. **LED Kathode** ↔ **GND-Quelle-Pad** → neues Netz, umbenennen in `GND`.

Damit hast du elektrisch exakt die klassische Reihenschaltung
`5V → Widerstand → LED → GND` abgebildet — genau die Grundschaltung, die
in praktisch jeder späteren, größeren Platine (z.B. eine Status-LED an
einem Mikrocontroller-Board) in dieser Form wiederkehrt.

Wenn du mehrere Pads auf einmal auf `5V` bzw. `GND` legen willst (z.B.
mehrere Entstörkondensatoren), verbinde sie einfach nacheinander mit dem
schon bestehenden Netz — jeder weitere „Connect pins"-Klick auf ein
bereits belegtes Pad hängt das neue Pad einfach mit an.

## 7. Leiterbahnen routen

Jetzt bestehen die Netze nur logisch — als gestrichelte Ratsnest-Linien
sichtbar, aber noch kein echtes Kupfer. Das ändert **„Route traces"**:

1. Oben auf **„Route traces"** klicken.
2. Startpin anklicken (muss bereits einem Netz zugeordnet sein).
3. Maus bewegen — es erscheint eine **Live-Vorschau** der Leiterbahn mit
   geführten 45°/orthogonalen Schenkeln. Hindernisse werden nicht
   automatisch umgangen und fremde Bahnen nicht verschoben; bei Kollision
   bleibt die Vorschau ungültig (rot), bis du einen freien Weg legst.
4. Ziel-Pin (gleiches Netz!) anklicken → die Bahn wird fest übernommen.
   Fertige Segmente kannst du im Select-Modus anklicken und ziehen
   (Segment-Drag), ohne neue Knicke zu erzeugen.

Zusätzliche Tasten während einer laufenden Route:

| Taste | Wirkung |
|---|---|
| `Leertaste` | Fixiert die aktuelle Ecke, damit die Bahn dort einen festen Knick macht, bevor man weiterzieht. |
| `V` | Setzt an der aktuellen Position ein Via und wechselt für den Rest der Bahn auf die andere Kupferlage. |
| `Backspace` | Nimmt die letzte fixierte Ecke wieder zurück (solange noch am Routen). |
| `Escape` | Bricht die aktuell laufende Route komplett ab. |

Route so `5V → Widerstand`, `Widerstand → LED`, `LED → GND`. Fertige
Bahnen lassen sich im „Select"-Modus wieder anklicken; `Delete`/
`Backspace` entfernt die ganze Bahn (das Netz selbst bleibt bestehen,
nur das Kupfer verschwindet — man kann sie danach jederzeit neu routen).

## 8. Vias gezielt setzen

Manchmal will man ein Via **nicht** mitten in einer laufenden Route
(`V`-Taste, siehe oben), sondern gezielt an einer bestimmten Stelle
setzen — z.B. um GND von der Oberseite auf eine GND-Kupferfläche auf der
Unterseite „festzunageln" (Stitching-Via). Dafür gibt es **„Place vias"**:

1. Oben auf **„Place vias"** klicken.
2. Erst auf ein Pad klicken, das **schon einem Netz zugeordnet ist** —
   damit legst du fest, welches Netz gestitcht wird (Hinweis erscheint:
   „Stitching net "GND" — click to place a via.").
3. Jeder weitere Klick irgendwo auf dem Board setzt dort ein Via auf
   genau diesem Netz — beliebig oft wiederholbar.
4. `Escape` beendet den Vorgang / setzt die Netzauswahl zurück, damit du
   für das nächste Via ein anderes Netz wählen kannst.

## 9. Kupferfläche (GND-Zone/Pour) zeichnen

Für eine flächige GND-Massefläche (reduziert Störungen, ist bei fast
jedem echten Board Standard) gibt es **„Draw zone"**:

1. Im rechten Panel zuerst **Net** (z.B. `GND`) und **Layer** (`F.Cu`
   oder `B.Cu`) im Dropdown auswählen — *bevor* man zeichnet.
2. Oben auf **„Draw zone"** klicken.
3. Nacheinander auf die Eckpunkte des gewünschten Umrisses klicken.
4. Zum Schließen entweder wieder auf den **ersten** gesetzten Punkt
   klicken oder **Enter** drücken — die Fläche wird sofort automatisch
   gefüllt (alle vorhandenen Pads/Bahnen anderer Netze werden dabei
   korrekt mit Abstand ausgespart).

Änderst du danach noch etwas am Board (neue Bahn, verschobenes Bauteil),
kannst du jederzeit oben auf **„Refill zones"** klicken, um alle Zonen
neu berechnen zu lassen.

Tipp: Zeichne die GND-Fläche über den ganzen freien Bereich der
Rückseite (B.Cu) und verbinde sie über 1-2 Stitching-Vias (Abschnitt 8)
mit dem GND-Netz auf der Vorderseite.

## 10. Sichtbare Layer / Ansicht

Unten im oberen Panel gibt es Checkboxen (**Outline**, **Pads**,
**Tracks**, **Vias**, **Zones**, **Mounting holes**), mit denen man
einzelne Ebenen ein-/ausblenden kann, um bei komplexeren Boards den
Überblick zu behalten. Zoomen: Mausrad über dem Board. Verschieben der
Ansicht: mit gedrückter Maustaste über eine freie Fläche ziehen (nicht
über ein Bauteil, sonst wird stattdessen das Bauteil verschoben).

## 11. Speichern

- **„Save"** — speichert unter dem bisherigen Dateinamen (`*.json`).
- **„Save As…"** — speichert unter neuem Namen/Pfad.
- **„Open…"** — lädt eine vorhandene Alladin-Board-`.json`.

Der aktuelle Dateiname (oder „(unsaved)") steht direkt daneben im oberen
Panel.

## 12. Kontrolle vor der Fertigung

Einen separaten DRC-Lauf braucht Alladin nicht: Das Programm arbeitet
**correct-by-construction** — Aktionen, die die JLCPCB-Fertigungsregeln
(Abstände, Mindestbreiten) verletzen würden, werden gar nicht erst
zugelassen. Was auf dem Board liegt, ist damit per Konstruktion
regelkonform. Vor der Bestellung trotzdem drei Dinge prüfen:

1. **Elektrisch vollständig?** Die Ratsnest-Anzeige (dünne Luftlinien)
   muss leer sein — jede sichtbare Linie ist eine noch nicht geroutete
   Verbindung.
2. **Zonen aktuell?** Erscheint oben die Warnung „⚠ Zones may be
   stale …", einmal **„Refill zones"** klicken.
3. **Zweitmeinung vom Fertiger:** Nach dem Gerber-Upload zeigt JLCPCB
   eine eigene DFM-Analyse und eine Bestückungsvorschau — dort die
   Platinenkontur, die Lagen und die Ausrichtung gepolter Bauteile
   (Pin-1-Marker) kontrollieren.

## 13. Fertigungsdaten erzeugen

Oben auf **„Export manufacturing files…"** klicken und einen Ordner
wählen. Alladin schreibt dort nativ (ohne KiCad) das komplette
JLCPCB-SMT-Paket:

1. `<name>_gerbers.zip` — Gerber + Bohrdateien
2. `<name>_cpl.csv` — Bestückungspositionen (Pick & Place)
3. `<name>_bom.csv` — Stückliste mit LCSC-Nummern

ZIP bei JLCPCB hochladen; BOM und CPL dort für die Bestückung angeben.

## 14. Tastenkürzel-Übersicht

| Taste | Kontext | Wirkung |
|---|---|---|
| `R` | Place-Modus / ausgewähltes Bauteil | 90°-Drehung |
| `Escape` | überall | Aktuellen Vorgang abbrechen / Werkzeug-Zustand zurücksetzen |
| `Enter` | Draw-Zone-Modus | Zonenumriss schließen |
| `Leertaste` | Route-Modus | Ecke fixieren |
| `V` | Route-Modus | Via setzen + Kupferlage wechseln |
| `Backspace` | Route-Modus (aktive Route) | Letzte fixierte Ecke zurücknehmen |
| `Delete` / `Backspace` | Select-Modus, etwas ausgewählt | Bauteil oder Bahn löschen |
| `Ctrl+Z` | Editor (nicht im Textfeld) | Letzte Board-Änderung rückgängig |
| `Ctrl+Y` / `Ctrl+Shift+Z` | Editor | Wiederherstellen |
| `Shift`-Klick | Connect-Modus, auf ein Pad | Pad aus seinem Netz entfernen |
| Mausrad | überall über dem Board | Zoomen |
| Ziehen auf freier Fläche | überall | Ansicht verschieben (Pan) |

## 15. Typische Anfängerfehler

- **Netz nicht umbenannt** → am Ende hat man zehn Netze namens „Net3",
  „Net7" usw. und weiß nicht mehr, was was ist. Gleich beim Anlegen in
  der „Nets"-Liste sinnvoll benennen (`GND`, `5V`, `3V3`, …).
- **Zone vor der Netzliste gezeichnet** → die Zone kann kein Netz
  auswählen, wenn das Netz noch gar nicht existiert. Erst verbinden
  („Connect pins"), dann Zone zeichnen.
- **Vergessen, „Refill zones" zu klicken** nach nachträglichen Änderungen
  am Board — die Zone bleibt sonst optisch auf dem alten Stand.
- **Direkt bestellen ohne Endkontrolle** — vor dem Export einmal
  Ratsnest prüfen (keine offenen Verbindungen), „Refill zones" klicken
  und nach dem Upload JLCPCBs eigene DFM-/Bestückungsvorschau ansehen
  (siehe Abschnitt 12).
- **Bauteil verschieben, ohne vorher zu prüfen, ob Bahnen mitgezogen
  wurden** — Alladin hält Netzverbindungen beim Verschieben stabil,
  bereits geroutete Bahnen können nach einem großen Sprung aber wieder
  Kollisionen bekommen; nach dem Verschieben kurz durchs Board scrollen
  und ggf. betroffene Bahnen neu routen.

Mit diesen Schritten lässt sich jede Schaltung aufbauen, die aus
Bauteilen + Netzen + Kupferflächen besteht — vom einfachen LED-Beispiel
oben bis zu einem kompletten ESP32-USB-Board.

## 16. Parts zwischen Desktop und Web

Die Web-Version (WASM) hat **keinen** LCSC-Download und kein MCP.
Beim Speichern schreibt Alladin die genutzten Nicht-Builtin-Parts mit
in die Board-`.json` (`embedded_parts`). Workflow:

1. Auf dem Desktop Parts per LCSC laden (Abschnitt 3), platzieren und
   das Board speichern.
2. In der Web-Version dieselbe Board-`.json` öffnen — die Footprints
   kommen mit; für schon platzierte Parts brauchst du keinen Import.

**Optional:** **„Export parts…"** / **„Import parts…"** für eine
portable `alladin-parts.json`, wenn du Vorlagen brauchst, die noch
nicht auf dem Board liegen.

Mit diesen 16 Abschnitten deckt diese Anleitung das manuelle Platzieren,
Netze setzen, Routen und den Fertigungsexport ab.
