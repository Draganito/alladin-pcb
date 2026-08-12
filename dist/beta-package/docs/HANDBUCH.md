# Alladin PCB — Das Handbuch

Vollständige Bedienungsanleitung von A bis Z, Stand v0.2.0-beta.1.
English version: [MANUAL.md](MANUAL.md).
Die Oberfläche der App ist auf Englisch; Knopfbeschriftungen werden hier
immer in Anführungszeichen mit dem englischen Original zitiert, z. B.
„Route traces". Für einen geführten Einstieg mit durchgehendem
Praxisbeispiel siehe [ANLEITUNG_FUER_ANFAENGER.md](../ANLEITUNG_FUER_ANFAENGER.md) —
dieses Handbuch hier ist das Nachschlagewerk, das *alles* abdeckt.

---

## Inhaltsverzeichnis

1. [Was ist Alladin PCB — das Konzept](#1-was-ist-alladin-pcb--das-konzept)
2. [Installation und Start](#2-installation-und-start)
3. [Neues Board anlegen](#3-neues-board-anlegen)
4. [Der Editor im Überblick](#4-der-editor-im-überblick)
5. [Ansicht und Navigation](#5-ansicht-und-navigation)
6. [Bauteile: Datenbank, LCSC-Download, eigene Templates](#6-bauteile-datenbank-lcsc-download-eigene-templates)
7. [Bauteile platzieren und bearbeiten](#7-bauteile-platzieren-und-bearbeiten)
8. [Netze](#8-netze)
9. [Leiterbahnen routen](#9-leiterbahnen-routen)
10. [Vias](#10-vias)
11. [Zonen und Power-Planes](#11-zonen-und-power-planes)
12. [Silkscreen (Bestückungsdruck)](#12-silkscreen-bestückungsdruck)
13. [Trace-, Via- und Raster-Einstellungen](#13-trace--via--und-raster-einstellungen)
14. [Speichern, Öffnen, Dateien](#14-speichern-öffnen-dateien)
15. [Fertigungsdaten exportieren und bei JLCPCB bestellen](#15-fertigungsdaten-exportieren-und-bei-jlcpcb-bestellen)
16. [Parts-Transfer Desktop ↔ Web](#16-parts-transfer-desktop--web)
17. [KI-Steuerung über MCP](#17-ki-steuerung-über-mcp)
18. [Kommandozeile (CLI)](#18-kommandozeile-cli)
19. [Alle Tastenkürzel](#19-alle-tastenkürzel)
20. [Meldungen verstehen und Probleme lösen](#20-meldungen-verstehen-und-probleme-lösen)
21. [Bewusste Grenzen des Programms](#21-bewusste-grenzen-des-programms)
22. [Glossar](#22-glossar)

---

## 1. Was ist Alladin PCB — das Konzept

Alladin ist für **Hobby und Bastler**: schnell von der Idee zu einer
**JLCPCB**-fertigen 1-/2-Layer-Platine — typisch ESP32, Smart-Home,
kleine Robotik-Boards. Kein vollwertiger Profi-EDA-Ersatz, sondern ein
schlanker Weg zum bestellbaren Board (optional mit KI über MCP).

Alladin PCB ist ein interaktiver Editor mit einem Grundprinzip, das ihn
von klassischen PCB-Programmen unterscheidet: **correct-by-construction**.
Klassische Programme lassen dich erst einmal alles zeichnen und melden
Regelverstöße hinterher im Design Rule Check (DRC). Alladin dreht das um:
Aktionen, die die Fertigungsregeln verletzen würden, werden **gar nicht
erst zugelassen**. Ein Bauteil, das zu nah an einem anderen landet, wird
rot angezeigt und lässt sich dort nicht absetzen; eine Leiterbahn, die
einen Kurzschluss verursachen würde, wird nicht übernommen. Was auf dem
Board liegt, ist damit per Konstruktion regelkonform.

Die Regeln dahinter sind die echten **JLCPCB-Fertigungsregeln** (Abstände
abhängig vom Kupfergewicht: 0,10 mm bei 1 oz, 0,16 mm bei 2 oz;
Mindest-Leiterbahnbreiten; Via-Geometrien). Deshalb gibt es auch keinen
eingebauten nachgelagerten DRC-Knopf — er wäre immer grün.

Weitere Eckpfeiler:

- **Eigenes Board-Format**: Boards sind `.json`-Dateien in Alladins
  eigenem Format. Es gibt keinen KiCad-Import/-Export in der Oberfläche;
  KiCad wird für nichts benötigt.
- **Native Fertigung**: Gerber-, Bohr-, Bestückungs- (CPL) und
  Stücklisten-Dateien (BOM) schreibt Alladin selbst, direkt im
  JLCPCB-Format.
- **Manuelles Routing**: Geführte 45°/orthogonale Leiterbahnen und
  Segment-Drag; kein externer Autorouter.
- **KI-steuerbar (Desktop)**: Mini-MCP für Parts, Placement, Netzliste,
  manuelles Kupfer-Routing und Speichern (Kapitel 17). Zone-Fill bleibt GUI.
- **Portable Boards**: Beim Speichern werden genutzte Nicht-Builtin-
  Parts mitgeschrieben (`embedded_parts`) — eine `.json` öffnet auf
  Desktop und Web. Optionaler Bibliotheks-Export/Import für
  Ersatzteile (Kapitel 16).

## 2. Installation und Start

### 2.1 Debian-Paket (empfohlen)

Die Release-Datei ist ein einziges Debian-Paket, erhältlich auf der
[Releases-Seite](https://github.com/Draganito/alladin-pcb/releases):

```bash
sudo apt install ./alladin-pcb_<version>_amd64.deb
alladin-pcb
```

Das Paket installiert:

| Pfad | Inhalt |
|---|---|
| `/usr/bin/alladin-pcb` | Das Programm |
| `/usr/share/alladin-pcb/cursor-setup/` | Fertiges Cursor/MCP-Setup für KI-Steuerung |
| `/usr/share/doc/alladin-pcb/` | Dokumentation und Lizenzhinweise |

Voraussetzungen: Debian/Ubuntu auf x86-64, X11 oder Wayland,
glibc 2.39+ (prüfbar mit `ldd --version`).

### 2.2 Aus dem Quellcode

```bash
git clone https://github.com/Draganito/alladin-pcb.git
cd alladin-pcb
cargo build --release -p alladin-pcb
./target/release/alladin-pcb
```

Benötigt ein aktuelles stabiles Rust. Tests: `cargo test --workspace`.

### 2.3 Startvarianten

- `alladin-pcb` — startet die grafische Oberfläche.
- `alladin-pcb --allow-ai-write` — GUI mit freigeschaltetem
  KI-Schreibzugriff über MCP (Kapitel 17). Ohne dieses Flag kann eine
  KI das Board nur lesen, nicht verändern.
- `alladin-pcb <unterkommando> …` — jedes weitere Argument schaltet in
  den Kommandozeilenmodus ohne GUI (Kapitel 18). `alladin-pcb --help`
  listet alle Unterkommandos.

Beim GUI-Start wird das zuletzt geöffnete Board automatisch im
Hintergrund geladen (Statusanzeige „⏳ Board wird geladen…"). Der
Fenstertitel ist immer „Alladin PCB"; der aktuelle Dateiname steht in
der Werkzeugleiste.

## 3. Neues Board anlegen

Der Startbildschirm „Alladin PCB — New board" (später jederzeit über
„New board..." erreichbar) fragt die Grunddaten ab:

| Feld | Bedeutung | Standard |
|---|---|---|
| „Import outline DXF…" | Optionale Platinenkontur aus LibreCAD/FreeCAD (geschlossene `LWPOLYLINE` oder ein geschlossener Ring aus `LINE`/`ARC`; Bögen/Bulges werden tesselliert). Maße kommen dann aus der DXF; Eckenradius entfällt. | — |
| „Width (mm)" / „Height (mm)" | Außenmaße der Platine (1–500 mm), unbenutzt wenn DXF geladen | 50 × 30 |
| „Layers" | 1 oder 2 Kupferlagen | 2 |
| „Copper weight" | Kupfergewicht „1oz" oder „2oz". Bestimmt den verbindlichen Mindestabstand: 0,10 mm (1 oz) bzw. 0,16 mm (2 oz) — nach JLCPCB-Regeln, im ganzen Programm nicht abschaltbar. | 1oz |
| „Corner radius (mm)" | Eckenradius der Platinenkontur, 0 = rechteckig (unbenutzt mit DXF) | 1,0 |

„Create board" legt das Board an und wechselt in den Editor. Bei
unzulässigen Werten erscheint „Invalid dimensions: …" und der Knopf ist
gesperrt. **Achtung:** „New board..." aus dem Editor heraus fragt nicht
nach ungespeicherten Änderungen — vorher speichern.

**DXF-Tipps:** Nur eine geschlossene Außenkontur exportieren (keine
Bemaßung, keine Blöcke). LibreCAD: geschlossene Polylinie (Bögen als
Bulge ok). FreeCAD: eine als Geometrie exportierte Skizze wird oft zu
`LINE`/`ARC` — Alladin verbindet sie, wenn genau ein geschlossener Ring
entsteht. Reine Kreise, leere Dateien und übrig gebliebene Hilfslinien
werden abgelehnt.

## 4. Der Editor im Überblick

Der Editor besteht aus drei Bereichen:

**Obere Werkzeugleiste** (umbricht bei schmalem Fenster):

- Statuszeile: „Alladin PCB — 2-layer, 1oz board", daneben der
  KI-Status: „🔒 AI-Schreibzugriff aus (nur lesen via MCP)" oder
  „🔓 AI-Schreibzugriff aktiv (MCP)".
- Dateiverwaltung: „Fit to board", „New board...", „Open...", „Save",
  „Save As…", „Export manufacturing files…"
  mit Zahnrad „⚙" für dessen Einstellungen, daneben der Dateiname
  (oder „(unsaved)").
- Werkzeuge (anklickbare Umschalter): „Connect pins", „Route traces",
  „Place vias", „Draw zone", „Place silk text", „Place silk dot" —
  plus „Refill zones" als Aktion. Einen „Select"-Knopf gibt es nicht:
  **Auswählen ist der Grundzustand**, erreichbar jederzeit mit `Escape`.
- Einstellungen: Leiterbahnbreite, Via-Maße, „Reset", „Snap to grid",
  „Grid (mm)".
- Sichtbarkeit („Show:"): Checkboxen für Outline, Pads, Tracks, Vias,
  Zones, B.Cu, Mounting holes, Ratsnest.

**Zeichenfläche** (Mitte): das Board auf dunkelgrünem Grund, mit
Rasterpunkten wenn Snap aktiv ist.

**Rechtes Seitenpanel**: „Place part" (Bauteilliste), „Download part
(LCSC)", „Add part to database...", „Parts (n)" (platzierte Teile),
Auswahl-Details, „Nets (n)" (Netzliste), „Power/ground planes" und —
je nach aktivem Werkzeug — dessen Hilfe- und Eingabefelder.

Rote Textzeilen unter der Werkzeugleiste oder im Seitenpanel sind
Fehlermeldungen der jeweils letzten Aktion (Kapitel 20).

## 5. Ansicht und Navigation

| Aktion | Wirkung |
|---|---|
| Mausrad über der Zeichenfläche | Zoomen |
| Ziehen auf freier Fläche | Ansicht verschieben (Pan). Nicht über einem Bauteil beginnen — sonst wird das Bauteil verschoben. Während einer laufenden Route ist Pan deaktiviert. |
| „Fit to board" | Ansicht auf das ganze Board einpassen |

Die „Show:"-Checkboxen blenden Ebenen ein/aus:

- **„Outline"**, **„Pads"**, **„Tracks"**, **„Vias"**, **„Zones"**,
  **„Mounting holes"** — die jeweiligen Objektarten.
- **„Show back copper (B.Cu)"** — die Unterseite wird normalerweise
  abgedunkelt dargestellt (gleicher Netz-Farbton, dunkler); diese
  Checkbox blendet sie ganz aus.
- **„Ratsnest"** — die dünnen Luftlinien, die zeigen, welche
  Verbindungen eines Netzes noch keine Leiterbahn haben. Jedes Netz hat
  seine eigene Farbe; Ratsnest, Bahnen und Pads eines Netzes teilen sie
  sich.

In der Netzliste kann man mit „○/◉" ein einzelnes Netz **highlighten**:
alle anderen Netze werden stark abgedunkelt — sehr nützlich, um auf
vollen Boards einer Verbindung zu folgen.

Beim Überfahren eines Pads erscheint ein Tooltip `REF.Pinnummer`, bei
bekannten Pin-Funktionen z. B. `U10.3 (VDD)`.

## 6. Bauteile: Datenbank, LCSC-Download, eigene Templates

### 6.1 Eingebaute Bauteile

Unter „Place part" stehen immer bereit: „2-pin THT (2.54mm pitch)",
„4-pin THT header (2.54mm pitch)", „SOIC-8 (1.27mm pitch)", „Wire pad
(solder, 2mm)", „Mounting hole (M2, NPTH)", „Mounting hole (M2.5,
NPTH)", „Mounting hole (M3, NPTH)".

Jedes Montageloch erzwingt automatisch einen **Schraubenkopf-Freiraum**:
Kupfer (Bahnen, Vias, Pads, Zonen-Füllungen) bleibt aus einem Kreis vom
vollen Bohrdurchmesser um den Lochmittelpunkt heraus — ein kupferfreier
Ring von einer halben Bohrweite über die Wand hinaus, dimensioniert so,
dass ein normaler Zylinder-/Linsenkopf nie auf Kupfer aufliegt (M3-Kopf
5,5 mm im 6,4-mm-Freikreis seines 3,2-mm-Lochs). Große Unterlegscheiben
können darüber hinausreichen — dann selbst mehr Abstand halten.

### 6.2 LCSC-Download

Der wichtigste Weg zu echten Bauteilen: unter „Download part (LCSC)"
eine LCSC-Bestellnummer (z. B. `C2040`) eintragen und „Download"
klicken. Alladin lädt den echten Footprint (Pads mit Form, Größe,
Position), die Pin-Funktionsnamen (GND/VDD/DIN/…, soweit verfügbar) und
die Kategorie aus der LCSC/EasyEDA-Datenbank und speichert alles
dauerhaft in deiner persönlichen Teile-Datenbank
(`~/.local/share/alladin-pcb/parts.sqlite3`). Heruntergeladene Teile
erscheinen in aufklappbaren Kategorien („Kategorie (Anzahl)") in der
„Place part"-Liste.

Löschen: „✖" neben einem Teil entfernt es aus der Datenbank, „🗑" neben
einer Kategorie die ganze Kategorie — beides mit Bestätigungsdialog
(„This cannot be undone.").

### 6.3 Eigene einfache Templates

„Add part to database..." öffnet ein Formular für einfache eigene
Footprints: „Name", „Ref. prefix" (z. B. `R` für Widerstände), „Pins"
(1–64), „Pitch (mm)", „Pad radius (mm)", „Description", „Category".
„Save to parts database" speichert es dauerhaft. Für komplexe Footprints
ist der LCSC-Download fast immer der bessere Weg.

## 7. Bauteile platzieren und bearbeiten

### 7.1 Platzieren

1. Bauteil in der „Place part"-Liste anklicken → Platzier-Modus, eine
   „Geist"-Vorschau hängt am Cursor. **Grün** = Position erlaubt,
   **rot** = Kollision (dort wird nicht platziert).
2. `R` dreht die Vorschau in 90°-Schritten („Rotation: n°" im Panel).
3. Klick setzt das Bauteil. Der Modus bleibt aktiv — weiterklicken
   platziert weitere Exemplare.
4. `Escape` oder „Cancel placement (Esc)" beendet den Modus.

**Matrix-Platzierung**: Im Panel lassen sich „Rows", „Cols",
„Pitch X (mm)", „Pitch Y (mm)" einstellen (Standard 1×1) — ein Klick
setzt dann das ganze Raster auf einmal, z. B. 5×4 LEDs mit 12,7 mm
Abstand. Beim Ziehen nahe der Board-Mittelachsen rasten gelbe
Hilfslinien ein.

### 7.2 Auswählen, Verschieben, Drehen, Löschen

Im Grundzustand (Select): Bauteil anklicken → gelber Auswahlring, im
Panel erscheinen Position/Rotation und die Hinweise „Drag it on the
board to move. R to rotate, Del to remove."

- **Verschieben**: anklicken und ziehen. Der Geist zeigt grün/rot, ob
  die Zielposition legal ist; Loslassen auf Rot lässt das Teil
  zurückschnappen. Mit aktivem „Snap to grid" rastet die Position aufs
  Raster.
- **Drehen**: `R` (wird verweigert, wenn die gedrehte Lage kollidieren
  würde).
- **Löschen**: `Delete`/`Backspace` oder „✖" in der „Parts"-Liste.
- **„Pin-1-Punkt (Silk)"**: Checkbox beim ausgewählten Teil — setzt
  einen Bestückungsdruck-Punkt an Pin 1, der mit dem Bauteil mitwandert
  (wichtig für gepolte Teile wie LEDs).
- **„Pin-1 auf allen Teilen"**: unter der Parts-Liste — derselbe Punkt
  per Batch auf jedem Bauteil mit Pads, mit denselben JLCPCB-Silk-
  Regeln; ohne Pads oder ohne Platz wird übersprungen (eine Zeile
  Status darunter). Ein Ctrl+Z macht den ganzen Batch rückgängig.

Bereits verlegte Bahnen wandern beim Verschieben **nicht** mit; die
Netz-Zugehörigkeit bleibt aber erhalten (Ratsnest zeigt die offene
Verbindung wieder an). Nach größeren Umbauten betroffene Bahnen löschen
und neu routen.

## 8. Netze

Ein Netz ist die logische Aussage „diese Pads gehören elektrisch
zusammen". Netze entstehen mit dem Werkzeug **„Connect pins"**:

1. „Connect pins" aktivieren.
2. Ersten Pin anklicken (cyanfarbener Ring, Meldung „First pin
   selected — click the pin to connect it to.").
3. Zweiten Pin anklicken → beide liegen auf demselben Netz. Hat schon
   einer ein Netz, kommt der andere dazu; haben beide keins, entsteht
   ein neues Netz „NetN".
4. **Shift-Klick** auf einen Pin entfernt ihn aus seinem Netz.
5. Klick ins Leere oder `Escape` bricht die laufende Auswahl ab.

Die Liste **„Nets (n)"** im Seitenpanel zeigt jedes Netz mit
Pin-Anzahl. Der Name ist ein direkt editierbares Textfeld — gleich
sinnvoll benennen (`GND`, `5V`, `3V3`, `DATA` …), das Commit erfolgt
mit Enter oder Verlassen des Feldes. „✖" löscht das ganze Netz: alle
Pins werden getrennt und **das gesamte Kupfer des Netzes (Bahnen, Vias)
wird entfernt**. „○/◉" schaltet das Highlight (Kapitel 5).

## 9. Leiterbahnen routen

Das Herzstück. Werkzeug **„Route traces"**:

1. Startpin anklicken — er muss bereits einem Netz zugeordnet sein
   (sonst: „This pin has no net yet — connect it to one first.").
2. Maus bewegen: Die Live-Vorschau zeigt geführte 45°/orthogonale
   Schenkel. Hindernisse werden nicht automatisch umgangen; bei Kollision
   bleibt die Vorschau ungültig (rot).
3. Zielpin **desselben Netzes** anklicken → die Bahn wird übernommen.

### 9.1 Tasten während einer laufenden Route

| Taste | Wirkung |
|---|---|
| `Leertaste` | Aktuelle Ecke fixieren (fester Knick), dann weiterziehen |
| `V` | An der Cursorposition ein Via setzen und auf die andere Kupferlage wechseln |
| `Backspace` | Letzte fixierte Ecke zurücknehmen |
| `Escape` | Route komplett abbrechen |

Das Panel zählt mit („n corner(s) fixed.").

### 9.2 Die Farben der Vorschau

| Darstellung | Bedeutung |
|---|---|
| Durchgezogene Linie in Netzfarbe | Dieser Weg ist frei und würde so übernommen |
| Durchgezogene **rote** Linie | Hier ist aktuell kein legaler Weg (Kollision oder zu nah an der Boardkante) |

### 9.3 Fertige Bahnen ändern

Im Select-Grundzustand:

- **Anklicken** wählt die Bahn/das Via aus („Selected: trace/via …").
- **Ziehen an einem Segment** formt die Bahn um — mit derselben
  Live-Logik (grün/rot) wie beim Neurouten. Vias lassen
  sich nicht per Drag verschieben.
- `Delete`/`Backspace` löscht **die ganze zusammenhängende Bahn** (das
  Netz bleibt bestehen; das Ratsnest zeigt die Verbindung wieder als
  offen, sie kann jederzeit neu geroutet werden).

## 10. Vias

Drei Wege zu einem Via:

1. **Mitten in einer Route**: Taste `V` (Kapitel 9.1) — Via an der
   Cursorposition, Rest der Bahn läuft auf der anderen Lage weiter.
2. **Stitching-Vias** mit dem Werkzeug **„Place vias"**: Zuerst ein Pad
   anklicken, das schon einem Netz zugeordnet ist — das legt das Netz
   fest (gelbe Meldung: „Stitching net "GND" — click to place a
   via."). Jeder weitere Klick aufs Board setzt dort ein Via dieses
   Netzes. Klick auf ein anderes Pad wechselt das Netz; `Escape` setzt
   zurück. Typischer Einsatz: eine Kupferfläche auf der Rückseite mit
   dem gleichen Netz auf der Vorderseite „vernähen".
3. **Rechtsklick auf ein Pad** → Kontextmenü **„Add via near pin"**:
   setzt ein Via direkt neben dem Pad samt kurzem Verbindungsstück.
   Ist der Naturpunkt blockiert, hängt eine Vorschau am Cursor
   (grün/rot) und der nächste Klick setzt Via+Stück an einer legalen
   Stelle; `Escape` bricht ab.

Via-Durchmesser und Bohrung stellt man vorher in der Werkzeugleiste ein
(Kapitel 13).

## 11. Zonen und Power-Planes

### 11.1 Freie Zone zeichnen („Draw zone")

1. Werkzeug „Draw zone" aktivieren.
2. Im Seitenpanel **zuerst** „Net:" (z. B. `GND`) und „Layer:"
   („F.Cu"/„B.Cu") wählen.
3. Eckpunkte des Umrisses nacheinander anklicken (orange Vorschau).
4. Schließen: wieder auf den **ersten** Punkt klicken (ab 3 Punkten,
   der erste Punkt bekommt einen Ring), `Enter` drücken oder „Finish
   outline" klicken. „Cancel" verwirft.

Die Fläche wird sofort gefüllt: alle fremden Pads, Bahnen und Vias
werden automatisch mit korrektem Abstand ausgespart, eigene
Netz-Mitglieder werden angebunden.

### 11.2 Ganzflächige Planes mit einem Klick

Unter **„Power/ground planes"** im Seitenpanel: „Solid F.Cu plane" bzw.
„Solid B.Cu plane" ankreuzen und im Dropdown daneben das Netz wählen —
Alladin füllt die komplette Boardfläche der Lage mit diesem Netz.
Netzwechsel im Dropdown füllt neu; Abwählen entfernt die Plane.

### 11.3 Aktualisieren („Refill zones")

Zonenfüllungen sind Momentaufnahmen. Nach Änderungen am Board erscheint
in der Werkzeugleiste die Warnung „⚠ Zones may be stale … — click
Refill zones". Ein Klick auf **„Refill zones"** berechnet alle Zonen im
Hintergrund neu (Statusanzeige „⏳ zone refill…"). Vor dem
Fertigungsexport immer einmal refillen.

## 12. Silkscreen (Bestückungsdruck)

- **„Place silk text"**: Im Panel „Text:" eingeben, „Side:" wählen
  („Front (F.SilkS)" / „Back (B.SilkS)"), optional „Rotate 90°" und
  Größe mit „−/+" — dann auf dem Board platzieren (grün/rot-Geist wie
  bei Bauteilen; leerer Text wird nicht gesetzt). Alladin verwendet die
  Hershey-Futural-Strichschrift: **die Vorschau ist exakt das, was im
  Gerber landet**.
- **„Place silk dot"**: setzt runde Markierungspunkte (Seite und
  Durchmesser im Panel einstellbar) — z. B. für Pin-1-Markierungen an
  Stellen, wo die automatische „Pin-1-Punkt"-Checkbox (Kapitel 7.2)
  nicht reicht.
- **Bearbeiten**: Im Select-Grundzustand anklicken (gelber Rahmen) —
  dann verschieben per Drag, `R` dreht Texte, „−/+" ändert die Größe,
  `Delete` löscht.

Bauteil-Referenzen (R1, U10, …) werden im Editor angezeigt, aber
**nicht** ins Gerber exportiert — der Bestückungsdruck bleibt sauber;
die Bestückung läuft über die CPL-Datei.

## 13. Trace-, Via- und Raster-Einstellungen

In der Werkzeugleiste, gültig für alles **künftig** verlegte Kupfer
(bestehendes bleibt unverändert):

| Feld | Standard | Bedeutung |
|---|---|---|
| „Trace width (mm):" | 0,25 | Breite neuer Leiterbahnen (Minimum: JLCPCB-Regel) |
| „Via diameter (mm):" | 0,60 | Außendurchmesser neuer Vias |
| „Via drill (mm):" | 0,30 | Bohrung neuer Vias |
| „Reset" | — | Setzt die drei Werte auf 0,25/0,6/0,3 zurück |
| „Snap to grid" | an | Platzieren und Verschieben rastet aufs Raster |
| „Grid (mm):" | 1,0 | Rasterweite (0,05–50 mm), nur aktiv bei Snap |

## 14. Speichern, Öffnen, Dateien

- **„Save"** speichert unter dem aktuellen Pfad (beim ersten Mal wie
  „Save As..."), **„Save As..."** unter neuem Namen, **„Open..."** lädt
  eine Alladin-`.json`-Datei (Dateidialog-Filter „Aladin PCB board",
  `*.json`). Fehler erscheinen rot („Couldn't open/save board: …").
  Beim Speichern werden genutzte Nicht-Builtin-Parts mitgeschrieben
  (siehe Kapitel 16).
- **Automatisches Nachladen**: Alladin überwacht die geöffnete Datei
  (~alle 300 ms). Wird sie extern verändert — z. B. von einem
  CLI-Kommando oder Skript — lädt Alladin sie neu („Board reloaded
  from disk."). Eine kaputte Datei wird abgewiesen, der letzte gute
  Stand bleibt erhalten.
- **Backups**: Es gibt keinen automatischen Backup-Mechanismus mehr;
  unter einem neuen Dateinamen speichern.
- Es gibt **keine** Ctrl+S/Ctrl+O-Tastenkürzel — Speichern läuft über
  die Knöpfe.

## 15. Fertigungsdaten exportieren und bei JLCPCB bestellen

**„Export manufacturing files..."** klicken, Zielordner wählen —
Alladin schreibt nativ (ohne KiCad) drei Dateien:

| Datei | Inhalt |
|---|---|
| `<name>_gerbers.zip` | Kupferlagen, Lötstopplack, Bestückungsdruck, Kontur (Edge Cuts) und Excellon-Bohrdateien |
| `<name>_cpl.csv` | Bestückungspositionen (Pick & Place): Designator, Mid X/Y, Layer, Rotation |
| `<name>_bom.csv` | Stückliste mit LCSC-Bestellnummern |

Bestellablauf bei JLCPCB:

1. Auf [jlcpcb.com](https://jlcpcb.com) das Gerber-Zip hochladen. Die
   Vorschau prüfen (Kontur, Lagen, Bohrungen).
2. Platinenoptionen wählen; **Copper weight** muss zur Einstellung im
   Board passen (1 oz/2 oz).
3. „SMT Assembly" aktivieren, BOM- und CPL-Datei hochladen.
4. Im Bauteil-Matching prüfen, dass jede BOM-Zeile einem
   JLCPCB-Lagerteil zugeordnet ist (die LCSC-Nummern aus dem
   Alladin-Download passen direkt).
5. In der Bestückungsvorschau (2D/3D) die Ausrichtung gepolter
   Bauteile kontrollieren (Pin-1-Marker), dann bestellen.

Vorher: einmal „Refill zones" und speichern.


## 16. Parts-Transfer Desktop ↔ Web

Die experimentelle Web-Version (WASM) hat keinen LCSC-Download und kein
MCP. Boards tragen ihre Parts mit:

1. Desktop: benötigte LCSC-Parts laden, platzieren, dann **„Save"**.
   Alladin schreibt die genutzten Nicht-Builtin-Templates in dieselbe
   `.json` (`embedded_parts`).
2. Web: diese Board-Datei mit **„Open…"** öffnen — Footprints kommen
   aus dem Embed; für schon platzierte Parts ist kein separater
   Parts-Import nötig.
3. Manuell weiter routen und Fertigungs-Zip herunterladen.

**Optional — ganze Bibliothek:** **„Export parts…"** / **„Import
parts…"** schreibt bzw. lädt eine portable `alladin-parts.json`, wenn
du Vorlagen brauchst, die noch nicht auf dem Board liegen.

Es gibt keinen Netzwerk-Proxy für LCSC im Browser. Bei öffentlichem
Hosting der WASM-Version gilt AGPL §13 (Corresponding Source anbieten —
dieses Repository). Öffentliche Demo auf GitHub Pages:
[https://draganito.github.io/alladin-pcb/](https://draganito.github.io/alladin-pcb/).



## 17. KI-Steuerung über MCP

Alladin hat auf dem **Desktop** einen eingebauten MCP-Server. Eine KI
kann ein Board anlegen, Teile beschaffen und platzieren, die Netzliste
verdrahten, Kupfer mit denselben Clearance-Gates wie der manuelle
45°-Router legen und ihre Arbeit selbst prüfen. Zone-Fill bleibt in der
GUI. Es gibt keinen klassischen Autorouter.

### 17.1 Einrichtung

1. GUI mit `alladin-pcb --allow-ai-write` starten.
2. Inhalt von `contrib/cursor-setup/` (bzw. im Deb:
   `/usr/share/alladin-pcb/cursor-setup/`) in den Cursor-Projektordner
   kopieren (`.cursor/` und `.cursorignore`).
3. MCP-URL: `http://127.0.0.1:8642/mcp`.

### 17.2 Tool-Referenz (22 Tools)

Read-only (immer verfügbar):

| Tool | Zweck |
|---|---|
| `board_summary` | Überblick / Todo |
| `get_footprints` | Platzierte Footprints |
| `get_nets` | Netze und Pins |
| `list_parts` | Alle platzierbaren Templates der Parts-Bibliothek |
| `check_board` | Prüfbericht (Netzliste komplett? Kupfer verbunden? Zonen aktuell? DFM-Befunde) |
| `get_routing_scene` | Pads, Tracks/Vias, offene Kupfer-Brücken, Routing-Regeln |
| `probe_route` | Batch-Clearance-Check für vorgeschlagene Polylinien (+ Vias) |
| `probe_placement` | Dry-Run Place/Move-DFM-Probe; optional `search_radius_mm` für nächsten legalen Spot |
| `suggest_route` | Serverseitiger octilinearer A*-Pfadfinder (45°-Stil, keine 90°-Ecken); Schreibrecht nur mit `commit=true` nötig |

Schreibend (brauchen `--allow-ai-write`):

| Tool | Zweck |
|---|---|
| `new_board` | Frisches Board anlegen (verweigert, ein offenes Board ungefragt zu verwerfen) |
| `download_lcsc_part` | LCSC → Parts-DB |
| `place_footprint` | Template platzieren (dieselben DFM-Gates wie die GUI) |
| `move_footprint` | Platziertes Teil verschieben/drehen |
| `place_parts` | Atomares Mehrfach-Platzieren (max. 50, ein Undo); optional `pins`-Netzmap; Antwort mit `open_bridges`-Score |
| `move_parts` | Atomares Mehrfach-Verschieben (max. 50, ein Undo); Antwort mit `open_bridges`-Score |
| `remove_footprint` | Platziertes Teil entfernen |
| `connect_pins` | Netzliste (zwei Pins verbinden) |
| `disconnect_pin` | Einen Pin vom Netz nehmen |
| `add_pin_stitching_via` | Stitching-Via + Stub neben einem Pin (oder allen Pads eines Netzes), automatisch platziert wie das GUI-„Via neben Pin"; landet nie auf oder zu nah an einem Lötpad (auch nicht same-net) — enge Pins werden abgelehnt statt kompromittiert. Jede Via-Platzierung (GUI / MCP / Layerwechsel) lehnt auch das Landen auf einer Leiterbahn ab (auch same-net) — das Bohrloch kappt die Spur |
| `rename_net` | Netz sauber benennen (`5V`, `GND`, …) |
| `save_board` | Board speichern |
| `commit_route` | Geprüfte Kupferbahn legen (gleiche Gates wie GUI-Preview) |
| `ripup_wire` | Bahn nahe einem Punkt oder alles Kupfer eines Netzes entfernen |

### 17.3 Floorplan-Ablauf

Bevorzugt `probe_placement` (optional mit `search_radius_mm`) als Dry-Run, dann `place_parts` / `move_parts` für atomare Batches (ein Ctrl+Z). Optional pro Teil `pins` (`{"1":"GND","2":"3V3"}`) setzt Netze im selben Schritt. Der `open_bridges`-Score in der Antwort (`sum_mm` / `max_mm` / `top`) ist das Ratsnest-Signal für vorausschauendes Platzieren — vor dem Routen klein halten. Einzelnes `place_footprint` / `move_footprint` bleibt für Einzeledits.

### 17.4 Kupfer-Routing-Ablauf

Schnellster Weg: `suggest_route` — ein serverseitiger octilinearer A*-Pfadfinder. Netz plus zwei Pins (`"REF.PIN"`) oder Punkte angeben, und er sucht einen legalen 45°-Stil-Pfad auf einer Lage (jedes Teilstück horizontal/vertikal/45°, keine 90°-Ecken, keine Vias) mit exakt denselben Clearance- und Rand-Gates wie unten — das Ergebnis ist direkt commit-fähig. Mit `commit=true` wird es im selben Aufruf gelegt, sonst das zurückgegebene `route_candidate` an `commit_route` geben. Stellschrauben: `step_mm` (Gitterweite, Standard 0,5), `bend_penalty_mm` (höher = gerader), `max_expansions` (Suchbudget).

Manueller Ablauf (volle ästhetische Kontrolle, Mehrlagen-Routen mit Vias):

1. `get_routing_scene` — `open_bridges` (kürzeste Pad-Paare zwischen Kupferinseln).
2. Polylinien vorschlagen (`segments` mit `layer` + `points_mm`; Mehrlagen mit `vias_mm` an den Übergängen).
3. `probe_route` — Kandidaten im Batch prüfen (grün/rot = GUI-Preview). Bei Blockade nennt das Ergebnis das genaue Teilstück und die Items im Weg (Art, Netz, Footprint, Lage, Position) — die KI kann gezielt drumherum routen. Zum Platinenrand gilt standardmäßig ein **Komfort-Abstand von 1,0 mm** (Routen am 0,2-mm-Fab-Limit provoziert DFM-Warnungen der Fab); per `edge_margin_mm` kann ein Kandidat bewusst näher heran, bis zum harten 0,2-mm-Minimum.
4. `commit_route` — ersten freien Kandidaten schreiben (Ctrl+Z macht rückgängig). Der Commit prüft zusätzlich die Konnektivität: Eine Bahn, die die Kupferinseln des Netzes nicht wirklich verbindet (falsche Lage, endet im Leeren), wird zurückgerollt und abgelehnt — die Antwort meldet `bridge_closed` und die Inselzahl vorher/nachher.
5. `check_board`, bis `open_nets` leer ist. Bei Blockade: Ecken, andere Lage + Via, oder `ripup_wire`.

Jeder MCP-Schreibzugriff läuft durch dieselben JLCPCB-DFM-Gates und
dieselbe Ctrl+Z-Undo-Historie wie deine eigenen GUI-Gesten — du kannst
alles, was die KI getan hat, jederzeit zurücknehmen. Zone-Fill bleibt GUI.



## 18. Kommandozeile (CLI)

Ohne Argumente startet die GUI. Mit Unterbefehl läuft Alladin headless:

| Befehl | Zweck |
|---|---|
| `new-board <path>` | Leeres Board anlegen (`--width-mm`, `--height-mm`, `--layers`, `--copper-oz`, `--corner-radius-mm`) |
| `download-part <C-Nr>` | LCSC-Teil in die Parts-DB laden |
| `connect <board> <ref1> <pin1> <ref2> <pin2>` | Zwei Pins auf dasselbe Netz legen |
| `list-nets <board>` | Netze auflisten |
| `list-footprints <board>` | Footprints auflisten |
| `board-summary <board>` | Kompakter Überblick |

Fertigungsexport läuft über die GUI. MCP deckt Board-Anlage, Parts,
Placement, Netzliste, manuelles Kupfer-Routing, Prüfung und Speichern ab.


## 19. Alle Tastenkürzel

| Taste | Kontext | Wirkung |
|---|---|---|
| `Escape` | überall | Zurück zum Select-Grundzustand; bricht laufende Vorgänge ab (Platzierung, Verbindung, Route, Trace-Drag, Zonenumriss, Stitching-Netz, Pin-Via) |
| `R` | Platzier-Modus | Vorschau +90° drehen |
| `R` | Select, Bauteil gewählt | Bauteil +90° drehen (nur wenn kollisionsfrei) |
| `R` | Select, Silk-Text gewählt / Silk-Text-Modus | Text +90° drehen |
| `Leertaste` | laufende Route | Ecke fixieren |
| `V` | laufende Route | Via setzen + Lage wechseln |
| `Backspace` | laufende Route mit fixierten Ecken | Letzte Ecke zurücknehmen |
| `Delete` / `Backspace` | Select, etwas gewählt | Bauteil / ganze Bahn / Silk-Element löschen |
| `Ctrl+Z` / `Cmd+Z` | Editor (nicht in Textfeld) | Letzte Board-Änderung rückgängig (bis 40 Schritte) |
| `Ctrl+Y` / `Ctrl+Shift+Z` | Editor (nicht in Textfeld) | Wiederherstellen |
| `Enter` | Zonen-Modus | Umriss schließen und füllen |
| `Enter` | Textfelder (Netzname, LCSC) | Eingabe bestätigen |
| `Shift`+Klick | Connect-Modus, auf Pad | Pad aus seinem Netz entfernen |
| Mausrad | Zeichenfläche | Zoomen |
| Ziehen (freie Fläche) | Zeichenfläche | Ansicht verschieben |
| Rechtsklick | auf Pad | Kontextmenü „Add via near pin" |

Bewusst **nicht** vorhanden: Ctrl+S/Ctrl+O, Lagen-Hotkeys außer `V` in der Route.

## 20. Meldungen verstehen und Probleme lösen

Alladin lehnt unzulässige Aktionen ab und sagt warum. Die häufigsten
Meldungen:

| Meldung | Bedeutung / Abhilfe |
|---|---|
| „This pin has no net yet — connect it to one first." | Routen startet nur an Pins mit Netz. Erst „Connect pins". |
| „this leg collides with something or comes too close to the board edge" | Der aktuelle Streckenabschnitt ist blockiert. Anderen Weg ziehen, Ecke früher fixieren oder per `V` die Lage wechseln. |
| „final leg comes within X.XXmm of the board edge" | Der Live-Pfad verletzt den Kantenabstand — Weg anpassen. |
| „can't fix a corner here — move the mouse first, or this leg is blocked" | `Leertaste` an ungültiger Stelle. |
| „no clear route here yet to drop a via onto" / „can't place a via here: …" | `V` an blockierter Stelle — Via braucht auf beiden Lagen Platz. |
| „Stitching net "…" — click to place a via." | Kein Fehler: Das Via-Werkzeug wartet auf die Zielposition. |
| „⚠ Zones may be stale …" | Board hat sich seit dem letzten Füllen geändert → „Refill zones". |
| „Couldn't open/save board: …" | Dateisystem-Problem (Pfad, Rechte); Details in der Meldung. |
| „Board reloaded from disk." | Kein Fehler: Die Datei wurde extern geändert und neu geladen. |
| Rote / blockierte Live-Vorschau | Zwischen letzter Ecke und Cursor ist der geführte Weg blockiert. Anderen Weg ziehen oder Ecken mit `Leertaste` setzen. |

Grundregel: **Eine Ablehnung heißt „hier gerade nicht legal", nicht
„kaputt".** Alladin erlaubt nichts, was die Fertigungsregeln verletzen
würde — der Weg zur Lösung ist ein anderer Pfad, eine andere Lage oder
mehr Platz, nie „fester ziehen".

## 21. Bewusste Grenzen des Programms

- **Leichtes Undo.** Ctrl+Z / Ctrl+Y stellen aktuelle **Board**-
  Änderungen wieder her (Platzierung, Netze, Kupfer, Zonen, Silk) mit
  begrenzter Historie — nicht Kamera, Werkzeug oder Parts-Datenbank.
  Zonen-Fill / Refill / Solid-Plane zählen jeweils als ein Schritt.
  Regelmäßig speichern für längere Versionierung; es gibt kein Autosave.
- **Kein Messwerkzeug.** Abstände prüft man über das Raster oder die
  Positionsanzeige ausgewählter Objekte.
- **Kein KiCad-Import/-Export in der Oberfläche.** Alladins `.json` ist
  das einzige Board-Format; die Fertigung läuft nativ.
- **Kein eingebauter DRC-Knopf** — per Konstruktion unnötig
  (Kapitel 1).
- **1–2 Kupferlagen, JLCPCB-Regelwerk.** Mehr Lagen oder andere
  Fertiger-Regelsätze sind derzeit nicht vorgesehen.

## 22. Glossar

| Begriff | Bedeutung |
|---|---|
| **Footprint** | Die Platinen-Geometrie eines Bauteils: Pads, Bohrungen, Umriss |
| **Pad** | Einzelner Lötanschluss eines Footprints |
| **Netz (Net)** | Logische Gruppe von Pads mit gleichem Potential (z. B. GND) |
| **Track / Leiterbahn** | Kupferverbindung auf einer Lage |
| **Via** | Durchkontaktiertes Loch zwischen den Kupferlagen |
| **Zone / Pour / Plane** | Große gefüllte Kupferfläche eines Netzes |
| **F.Cu / B.Cu** | Kupfer vorn (oben) / hinten (unten) |
| **Ratsnest** | Luftlinien der noch nicht gerouteten Verbindungen |
| **Segment-Drag** | Fertiges Track-Segment verschieben, Knickzahl bleibt |
| **Stitching-Via** | Via, das Flächen/Bahnen desselben Netzes über Lagen hinweg verbindet |
| **Clearance** | Vorgeschriebener Mindestabstand zwischen Kupfer verschiedener Netze |
| **DRC** | Design Rule Check — bei Alladin per Konstruktion immer erfüllt |
| **Gerber / Excellon** | Industrieformate für Fertigungsdaten / Bohrdaten |
| **BOM** | Stückliste (Bill of Materials) |
| **CPL** | Bestückungspositionen (Component Placement List) |
| **LCSC** | Bauteilhändler; dessen C-Nummern nutzt der Teile-Download und die BOM |
| **MCP** | Model Context Protocol — Mini-Surface für Parts + Netzliste (Desktop) |
| **embedded_parts** | Nicht-Builtin-Footprints, die beim Speichern in der Board-`.json` stecken |
| **alladin-parts** | Optionale portable Bibliotheks-JSON (Vorlagen, die noch nicht auf dem Board liegen) |
