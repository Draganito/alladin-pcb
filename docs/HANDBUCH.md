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
16. [Externer Autorouter (KiCadRoutingTools)](#16-externer-autorouter-kicadroutingtools)
17. [KI-Steuerung über MCP](#17-ki-steuerung-über-mcp)
18. [Kommandozeile (CLI)](#18-kommandozeile-cli)
19. [Alle Tastenkürzel](#19-alle-tastenkürzel)
20. [Meldungen verstehen und Probleme lösen](#20-meldungen-verstehen-und-probleme-lösen)
21. [Bewusste Grenzen des Programms](#21-bewusste-grenzen-des-programms)
22. [Glossar](#22-glossar)

---

## 1. Was ist Alladin PCB — das Konzept

Alladin PCB ist ein interaktiver Editor für 1- und 2-Layer-Platinen mit
einem Grundprinzip, das ihn von klassischen PCB-Programmen unterscheidet:
**correct-by-construction**. Klassische Programme lassen dich erst einmal
alles zeichnen und melden Regelverstöße hinterher im Design Rule Check
(DRC). Alladin dreht das um: Aktionen, die die Fertigungsregeln verletzen
würden, werden **gar nicht erst zugelassen**. Ein Bauteil, das zu nah an
einem anderen landet, wird rot angezeigt und lässt sich dort nicht
absetzen; eine Leiterbahn, die einen Kurzschluss verursachen würde, wird
nicht übernommen. Was auf dem Board liegt, ist damit per Konstruktion
regelkonform.

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
- **Interaktives Routing mit Intelligenz**: Beim manuellen Ziehen einer
  Leiterbahn weicht die Vorschau automatisch Hindernissen aus
  (Walkaround), sucht Wege per A*-Suche und kann im Weg liegende fremde
  Bahnen zur Seite schieben (Shove) — mit Live-Vorschau, was passieren
  würde.
- **KI-steuerbar**: Ein eingebauter MCP-Server erlaubt es einem
  KI-Assistenten (z. B. in Cursor), das Board über geprüfte Werkzeuge
  aufzubauen — mit denselben Regeln wie ein menschlicher Nutzer
  (Kapitel 17).
- **Optionaler externer Autorouter**: Für das automatische Verlegen
  vieler Netze bindet Alladin das eigenständige MIT-Projekt
  [KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)
  als Subprozess ein (Kapitel 16).

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
| `/usr/share/alladin-pcb/KiCadRoutingTools/` | Der externe Autorouter, fertig gebündelt |
| `/usr/share/alladin-pcb/cursor-setup/` | Fertiges Cursor/MCP-Setup für KI-Steuerung |
| `/usr/share/doc/alladin-pcb/` | Dokumentation und Lizenzhinweise |

Die Python-Abhängigkeiten des Autorouters (`numpy`, `scipy`, `shapely`)
werden von apt automatisch mitinstalliert — kein venv, kein pip, nichts
zu bauen. Voraussetzungen: Debian/Ubuntu auf x86-64, X11 oder Wayland,
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
| „Width (mm)" / „Height (mm)" | Außenmaße der Platine (1–500 mm) | 50 × 30 |
| „Layers" | 1 oder 2 Kupferlagen | 2 |
| „Copper weight" | Kupfergewicht „1oz" oder „2oz". Bestimmt den verbindlichen Mindestabstand: 0,10 mm (1 oz) bzw. 0,16 mm (2 oz) — nach JLCPCB-Regeln, im ganzen Programm nicht abschaltbar. | 1oz |
| „Corner radius (mm)" | Eckenradius der Platinenkontur, 0 = rechteckig | 1,0 |

„Create board" legt das Board an und wechselt in den Editor. Bei
unzulässigen Werten erscheint „Invalid dimensions: …" und der Knopf ist
gesperrt. **Achtung:** „New board..." aus dem Editor heraus fragt nicht
nach ungespeicherten Änderungen — vorher speichern.

## 4. Der Editor im Überblick

Der Editor besteht aus drei Bereichen:

**Obere Werkzeugleiste** (umbricht bei schmalem Fenster):

- Statuszeile: „Alladin PCB — 2-layer, 1oz board", daneben der
  KI-Status: „🔒 AI-Schreibzugriff aus (nur lesen via MCP)" oder
  „🔓 AI-Schreibzugriff aktiv (MCP)".
- Dateiverwaltung: „Fit to board", „New board...", „Open...", „Save",
  „Save As...", „Export manufacturing files...", „Autoroute (extern)…"
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
2. Maus bewegen: Die Live-Vorschau zeigt den Weg, den Alladin verlegen
   würde — mit 45°/90°-Knicken, automatisch um Hindernisse herum
   (Walkaround), bei Bedarf mit A*-Wegsuche über mehrere Hindernisse.
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
| **Gestrichelte** Linie in Netzfarbe + **orange gestrichelte** fremde Bahnen | Shove-Vorschau: Der Weg ist möglich, wenn Alladin die orange markierten fremden Bahnen zur Seite schiebt. Klick auf den Zielpin führt genau das aus. |

Shove verschiebt fremde Bahnen nur so, dass alle Regeln weiterhin
eingehalten werden — geht das nicht, bleibt die Vorschau rot.

### 9.3 Fertige Bahnen ändern

Im Select-Grundzustand:

- **Anklicken** wählt die Bahn/das Via aus („Selected: trace/via …").
- **Ziehen an einem Segment** formt die Bahn um — mit derselben
  Live-Logik (grün/rot, Walkaround) wie beim Neurouten. Vias lassen
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
- **Automatisches Nachladen**: Alladin überwacht die geöffnete Datei
  (~alle 300 ms). Wird sie extern verändert — z. B. von einem
  CLI-Kommando oder Skript — lädt Alladin sie neu („Board reloaded
  from disk."). Eine kaputte Datei wird abgewiesen, der letzte gute
  Stand bleibt erhalten.
- **Backups**: Vor dem Übernehmen eines Autorouter-Ergebnisses schreibt
  Alladin automatisch `<board>.before-autoroute.json`.
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

## 16. Externer Autorouter (KiCadRoutingTools)

Alladin selbst verlegt Bahnen nur interaktiv. Für das automatische
Routen vieler Netze auf einmal bindet es das eigenständige Projekt
[KiCadRoutingTools](https://github.com/drandyhaas/KiCadRoutingTools)
(MIT, Andy Haas) als Subprozess ein — unverändert, austauschbar,
optional.

### 16.1 Einrichtung (einmal pro Rechner)

Mit dem Debian-Paket ist das Tool schon installiert. In der
Werkzeugleiste auf das Zahnrad „⚙" klicken → Fenster **„Autoroute
(extern) settings"**:

| Feld | Wert (deb-Installation) |
|---|---|
| „Tool folder" | `/usr/share/alladin-pcb/KiCadRoutingTools` |
| „Python" | `python3` |
| „Track width (mm)" / „Via diameter (mm)" / „Via drill (mm)" | Standardwerte passen (0,25/0,6/0,3) |
| „Clearance (mm)" | fest nach JLCPCB-Regel, nicht editierbar |
| „Extra arguments" | leer (für Spezialfälle, z. B. `--bus`) |

**„Diagnose"** klicken — sechs Prüfpunkte müssen grün sein (Python,
`route.py`, numpy, scipy, shapely, `route.py --help`). Dann **„Save"**.
Bei Quellcode-Installationen zeigt „Copy setup instructions" die
Klon-/Installationsschritte.

### 16.2 Einen Lauf starten

1. **„Autoroute (extern)…"** klicken.
2. Im Dialog die zu routenden Netze ankreuzen (vorausgewählt: alle
   Netze mit mehr als einem Pad), **„Route n net(s)"**.
3. Live-Log verfolgen (je nach Board Sekunden bis Minuten); „Cancel
   run" bricht ab.
4. Nach „Finished." zeigt der Bericht: „x/y requested net(s) routed",
   Ergebnis von „DRC check" und „Connectivity check" sowie die Zahl der
   wartenden neuen Bahnen/Vias.
5. **Wichtig**: Die Ergebnisse liegen noch *nicht* im Board. **„Merge
   into board"** übernimmt sie (vorher wird automatisch
   `<board>.before-autoroute.json` gesichert); **„Discard"** verwirft
   sie.

## 17. KI-Steuerung über MCP

Alladin hat einen eingebauten MCP-Server (Model Context Protocol), über
den ein KI-Assistent — etwa in Cursor — das **live geöffnete Board**
steuern kann. Die KI benutzt dabei exakt dieselben geprüften Operationen
wie ein Mensch: Was gegen die Fertigungsregeln verstoßen würde, wird
abgelehnt. Alladin ist der Wächter; die KI kann nichts Illegales bauen.

### 17.1 Einrichtung

1. Alladin mit Schreibfreigabe starten:

   ```bash
   alladin-pcb --allow-ai-write
   ```

   Ohne dieses Flag funktionieren nur die Lese-Tools; jede
   Schreiboperation wird mit klarer Meldung verweigert. Der Status ist
   in der Werkzeugleiste sichtbar („🔓 AI-Schreibzugriff aktiv (MCP)").

2. Den **Inhalt** des mitgelieferten Setups in den Projektordner
   kopieren, den man in Cursor öffnet — sowohl `.cursor/` als auch
   `.cursorignore` (Deb-Installation: aus
   `/usr/share/alladin-pcb/cursor-setup/`, Quellbaum:
   `contrib/cursor-setup/`):

   | Datei | Zweck |
   |---|---|
   | `.cursor/mcp.json` | Verbindet Cursor mit `http://127.0.0.1:8642/mcp` |
   | `.cursor/rules/alladin-mcp.mdc` | Arbeitsregeln für die KI: nur MCP-Tools, knapp berichten, mit `board_summary` starten, bei Timeouts `get_job_status` pollen |
   | `.cursorignore` | Versteckt Board-`.json`-Dateien vor der KI — die MCP-Tools sind ihr einziger Weg zum Board |

Der Server lauscht nur auf localhost (Port 8642), ohne
Authentifizierung — er ist für die lokale Maschine gedacht. Er läuft,
sobald die GUI offen ist.

### 17.2 Hintergrund-Jobs, Timeouts und der eine Job-Slot

Schnelle Tools antworten innerhalb von 3 Sekunden. Rechenintensive
Operationen (Zonenfüllung, Routing-Suche, Kontinuitätsprüfung, Export,
Batches) laufen als **Hintergrund-Job** — davon gibt es genau **einen
Slot**. Daraus folgen drei Verhaltensregeln für KI-Clients:

- Antwortet ein langsames Tool mit „no reply within Ns", läuft der Job
  **weiter**. `get_job_status` alle paar Sekunden aufrufen, bis
  `running` `null` ist — `last_finished.result` enthält dann genau die
  Antwort, die der ursprüngliche Aufruf geliefert hätte.
- Die Operation **niemals erneut absetzen**, nur weil die Antwort
  ausblieb — sie würde ein zweites Mal ausgeführt.
- Solange ein Job läuft, werden weitere schreibende Aufrufe sofort als
  „busy" abgewiesen (reine Lese-Tools antworten weiterhin).

Der externe Autorouter hat seinen eigenen Status-Kanal:
`start_external_autoroute` kehrt sofort zurück,
`get_external_autoroute_status` liefert `idle`/`running`/`done`/
`failed`. Das Übernehmen des Ergebnisses („Merge into board") bleibt
bewusst ein manueller Klick in der GUI.

### 17.3 Tool-Referenz (32 Tools)

**Lesen — immer erlaubt, auch ohne `--allow-ai-write`:**

| Tool | Liefert |
|---|---|
| `get_editor_state` | Live-UI-Zustand: aktives Werkzeug, laufende Route/Zone, Auswahl, Meldungen |
| `get_board_overview` | Dateipfad, Maße, Lagen, Zähler (Netze, Teile, Bahnen, Vias, Zonen) |
| `get_nets` | Alle Netze mit Pin-Zugehörigkeit |
| `get_zones` | Alle Zonen mit Netz, Lage, Umriss- und Insel-Anzahl |
| `get_footprints` | Alle platzierten Teile mit Position, Rotation, Pad-Netzen |
| `get_job_status` | Status des Hintergrund-Job-Slots + volles Ergebnis des letzten Jobs |
| `get_external_autoroute_status` | Status des externen Autorouter-Laufs |

**Langsame Analysen — kein Schreibflag nötig, belegen aber den Job-Slot:**

| Tool | Liefert |
|---|---|
| `board_summary` | Das Gesamtbild in einem Aufruf: Maße, Regeln, was noch unfertig ist (Pins ohne Netz, unterbrochene Netze). Empfohlener erster Aufruf. |
| `check_net_continuity` | Prüft physische Kupfer-Durchgängigkeit (Pads+Bahnen+Vias+Zonen), optional für ein einzelnes Netz (`net_name`) |

**Schreiben — nur mit `--allow-ai-write`:**

| Gruppe | Tools |
|---|---|
| Board | `create_board` (nur auf dem New-board-Bildschirm), `save_board` (ohne `path` = Save, mit = Save As) |
| Teile | `place_footprint`, `download_lcsc_part` (nicht batchbar), `register_part` |
| Netze | `connect_pins`, `rename_net` |
| Routing automatisch | `route_pins` (Punkt-zu-Punkt-Wegsuche, gleiche Lage, keine Vias) |
| Routing manuell (Drag-Familie) | `start_route` → `route_to` → `fix_corner` / `undo_last_corner` / `drop_via_and_switch_layer` → `finish_route` bzw. `cancel_route` — das MCP-Pendant zur Maus samt `Leertaste`/`Backspace`/`V`/`Escape` |
| Vias | `add_via` (freies Stitching-Via, muss Netz-Kupfer berühren), `add_pin_stitching_via` (Via + Stichleitung direkt am Pad, mit automatischer Ausweichsuche) |
| Zonen | `add_zone` (Polygon auf `front`/`back`), `refill_zones` |
| Silkscreen | `add_silk_text` |
| Fertigung | `export_manufacturing_files` (Gerber-Zip + CPL + BOM in einen Ordner) |
| Autorouter | `start_external_autoroute` (optional `nets`, `extra_args`) |
| Batch | `run_batch` — führt eine Liste von Operationen in einem Durchgang aus (`operations: [{"tool": …, "args": {…}}]`, `stop_on_error` standardmäßig an). Batchbar sind alle Schreib-Tools außer `download_lcsc_part` und `start_external_autoroute`. |

**Bewährtes Muster für den KI-Boardaufbau:** Teile per
`download_lcsc_part` besorgen → Platzierung/Netze/Routen/Zonen/Speichern
als `run_batch` → mit `board_summary` und `check_net_continuity`
verifizieren.

## 18. Kommandozeile (CLI)

Jedes Argument außer `--allow-ai-write` startet Alladin ohne GUI als
Kommandozeilenwerkzeug: Board-Datei laden → Operation ausführen →
speichern, Prozess endet. Damit lassen sich Boards skripten. Übersicht
mit `alladin-pcb --help`, Details je Kommando mit
`alladin-pcb <kommando> --help`.

| Kommando | Zweck | Wichtigste Argumente |
|---|---|---|
| `new-board <datei>` | Leeres Board anlegen | `--width-mm` (50), `--height-mm` (30), `--layers` (2), `--copper-oz` (1), `--corner-radius-mm` (1) |
| `list-templates` | Alle Footprint-Vorlagen auflisten (eingebaut + Datenbank) | — |
| `download-part <C-Nr>` | LCSC-Teil in die Datenbank laden | z. B. `C2040`; verweigert Duplikate |
| `update-part <C-Nr>` | Bereits geladenes Teil neu von LCSC holen und überschreiben | — |
| `place-part <board>` | Teil platzieren | `--template` (Name aus `list-templates`), `--x-mm`, `--y-mm`, `--rotation-deg` |
| `connect <board>` | Zwei Pins auf ein Netz legen | `--ref1`/`--pin1`, `--ref2`/`--pin2` |
| `route <board>` | Bahn zwischen zwei verbundenen Pins suchen und verlegen (eine Lage, keine Vias) | `--ref1`/`--pin1`, `--ref2`/`--pin2` |
| `add-via <board>` | Stitching-Via setzen (muss Netz-Kupfer berühren) | `--net`, `--x-mm`, `--y-mm`, `--diameter-mm` (0.6), `--drill-mm` (0.3) |
| `add-zone <board>` | Kupferfläche anlegen und füllen | `--net`, `--layer front\|back`, `--points-file` (JSON-Polygon `[{"x_mm":…,"y_mm":…},…]`) |
| `refill-zones <board>` | Alle Zonen neu füllen | — |
| `list-zones <board>` | Zonen auflisten | — |
| `set-outline <board>` | Platinenkontur ersetzen | genau eines von `--from-kicad <datei>` (nur Edge.Cuts) oder `--points-file` (mehrere Polygone = Ausschnitte); verweigert, wenn Bestehendes herausfallen würde |
| `register-part <name>` | Einfaches eigenes Teil registrieren | `--reference-prefix`, genau eines von `--pin-count` (Padreihe, `--pitch-mm`, `--pad-radius-mm`) oder `--hole-diameter-mm` (NPTH-Loch), `--exclude-from-bom`, `--category` |
| `export-manufacturing <board> <ordner>` | Gerber-Zip + CPL + BOM schreiben | — |
| `autoroute-external <board>` | Externen Autorouter blockierend laufen lassen und Ergebnis einpflegen (Backup `*.before-autoroute.json` entsteht automatisch) | `--nets` (wiederholbar; ohne = alle Mehrfach-Pad-Netze), `--tool-dir`, `--extra-args` |

Koordinaten sind Millimeter mit Ursprung in der Board-Mitte; negative
Werte funktionieren (`--x-mm -10`). Eine typische Skript-Pipeline:

```bash
alladin-pcb new-board board.json --width-mm 50 --height-mm 30
alladin-pcb download-part C2040
alladin-pcb place-part board.json --template "…" --x-mm 0 --y-mm 0
alladin-pcb connect board.json --ref1 U1 --pin1 1 --ref2 R1 --pin2 1
alladin-pcb route board.json --ref1 U1 --pin1 1 --ref2 R1 --pin2 1
alladin-pcb add-zone board.json --net Net1 --layer front --points-file pour.json
alladin-pcb refill-zones board.json
alladin-pcb export-manufacturing board.json ./fab
```

Läuft die GUI mit derselben Datei parallel, lädt sie externe Änderungen
automatisch nach (Kapitel 14) — man sieht dem Skript also live zu.

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
| `Enter` | Zonen-Modus | Umriss schließen und füllen |
| `Enter` | Textfelder (Netzname, LCSC) | Eingabe bestätigen |
| `Shift`+Klick | Connect-Modus, auf Pad | Pad aus seinem Netz entfernen |
| Mausrad | Zeichenfläche | Zoomen |
| Ziehen (freie Fläche) | Zeichenfläche | Ansicht verschieben |
| Rechtsklick | auf Pad | Kontextmenü „Add via near pin" |

Bewusst **nicht** vorhanden: Ctrl+Z/Ctrl+Y (siehe Kapitel 21),
Ctrl+S/Ctrl+O, Lagen-Hotkeys außer `V` in der Route.

## 20. Meldungen verstehen und Probleme lösen

Alladin lehnt unzulässige Aktionen ab und sagt warum. Die häufigsten
Meldungen:

| Meldung | Bedeutung / Abhilfe |
|---|---|
| „This pin has no net yet — connect it to one first." | Routen startet nur an Pins mit Netz. Erst „Connect pins". |
| „this leg collides with something or comes too close to the board edge" | Der aktuelle Streckenabschnitt ist blockiert. Anderen Weg ziehen, Ecke früher fixieren oder per `V` die Lage wechseln. |
| „route found, but comes within X.XXmm of the board edge" | Der gefundene Weg verletzt den Kantenabstand — Weg anpassen. |
| „can't fix a corner here — move the mouse first, or this leg is blocked" | `Leertaste` an ungültiger Stelle. |
| „no clear route here yet to drop a via onto" / „can't place a via here: …" | `V` an blockierter Stelle — Via braucht auf beiden Lagen Platz. |
| „Stitching net "…" — click to place a via." | Kein Fehler: Das Via-Werkzeug wartet auf die Zielposition. |
| „⚠ Zones may be stale …" | Board hat sich seit dem letzten Füllen geändert → „Refill zones". |
| „Couldn't open/save board: …" | Dateisystem-Problem (Pfad, Rechte); Details in der Meldung. |
| „Board reloaded from disk." | Kein Fehler: Die Datei wurde extern geändert und neu geladen. |
| Suchabbrüche („no path", „search too complex") | Zwischen Start und Cursor existiert aktuell kein legaler Weg (oder er ist zu verwinkelt). Zwischenecken mit `Leertaste` setzen und in Etappen routen. |

Grundregel: **Eine Ablehnung heißt „hier gerade nicht legal", nicht
„kaputt".** Alladin erlaubt nichts, was die Fertigungsregeln verletzen
würde — der Weg zur Lösung ist ein anderer Pfad, eine andere Lage oder
mehr Platz, nie „fester ziehen".

## 21. Bewusste Grenzen des Programms

- **Kein Undo/Redo.** Es gibt keinen globalen Rückgängig-Verlauf.
  Abgefedert wird das durch das Grundprinzip: Illegales passiert erst
  gar nicht, Löschen verlangt bei Datenbank-Teilen eine Bestätigung,
  Bahnen lassen sich jederzeit neu routen, und vor Autorouter-Merges
  entsteht automatisch eine Backup-Datei. Trotzdem: **regelmäßig
  speichern**, gern unter neuen Namen als manuelle Versionierung.
- **Kein Messwerkzeug.** Abstände prüft man über das Raster oder die
  Positionsanzeige ausgewählter Objekte.
- **Kein KiCad-Import/-Export in der Oberfläche.** Alladins `.json` ist
  das einzige Board-Format; die Fertigung läuft nativ. (Intern nutzt
  nur der externe Autorouter einen KiCad-Austausch.)
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
| **Walkaround** | Automatisches Umfahren von Hindernissen beim Ziehen |
| **Shove** | Beiseiteschieben fremder Bahnen, um Platz zu schaffen |
| **Stitching-Via** | Via, das Flächen/Bahnen desselben Netzes über Lagen hinweg verbindet |
| **Clearance** | Vorgeschriebener Mindestabstand zwischen Kupfer verschiedener Netze |
| **DRC** | Design Rule Check — bei Alladin per Konstruktion immer erfüllt |
| **Gerber / Excellon** | Industrieformate für Fertigungsdaten / Bohrdaten |
| **BOM** | Stückliste (Bill of Materials) |
| **CPL** | Bestückungspositionen (Component Placement List) |
| **LCSC** | Bauteilhändler; dessen C-Nummern nutzt der Teile-Download und die BOM |
| **MCP** | Model Context Protocol — die Schnittstelle, über die eine KI Alladin steuert |
