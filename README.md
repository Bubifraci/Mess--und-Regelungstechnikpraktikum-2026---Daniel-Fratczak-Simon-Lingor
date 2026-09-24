# Praktikum Mess- und Regelungstechnik 2026 - Daniel Fratczak, Simon Lingor

Sourcecode und weiterer relevanter Code für unsere Bearbeitung des Praktikums Mess- und Regelungstechnik 2026.

Dies ist die README-Datei für das Hardwarepraktikum SS26 an der JMU Würzburg. Hier wird beschrieben, wie man die Anwendung ausführt – fokussiert auf die selbst geschriebenen ROS2-Nodes.

Zur Bearbeitung der Aufgaben und zur Aufnahme der Bag-Files wurden ROS2 Jazzy und ROS2 Humble unter Linux Mint genutzt.

Da wir jeweils mit unterschiedlichen ROS2-Versionen gearbeitet haben (Humble und Jazzy), ist die Implementierung wie folgt strukturiert:

* **ROS2 Jazzy:** Implementierung des Giovanni-Reglers (inkl. Simulation) auf Basis von Odometriedaten.
* **ROS2 Humble:** Implementierung des Giovanni-Reglers (inkl. Simulation) auf Basis von AMCL-Daten.

---

## Ordnerstruktur

* **`tools/`** $\rightarrow$ Werkzeuge abseits von ROS2 (größtenteils in Python geschrieben).
* **`Jazzy/`** $\rightarrow$ Sourcecode für ROS2 Jazzy (basiert auf Odometriedaten).
* **`Humble/`** $\rightarrow$ Sourcecode für ROS2 Humble (basiert auf AMCL-Daten).

---

## Für `tools`

In `tools/` finden sich zwei Ordner: `datModifier` und `plotter`.

### 1. `datModifier`

`datModifier` ist das Werkzeug zum Modifizieren der `.dat`-Routen.

* **Verwendung:**
1. Die zu bearbeitende Route in denselben Ordner kopieren und in `path.dat` umbenennen.
2. Im Code existiert die Variable `scale` (Standardwert: `0.5`). Dieser Wert kann je nach gewünschter Skalierung angepasst werden.
3. Nach der Einrichtung das Skript ausführen:
`python3 pathModifier.py`
*(Voraussetzungen: `matplotlib` und `numpy`)*
4. Die modifizierte Route wird in die Datei `modified.dat` geschrieben. Zudem öffnet sich ein Plot, der die ursprüngliche Route mit der modifizierten Route vergleicht.



### 2. `plotter`

Im Ordner `plotter/` befinden sich verschiedene Auswertungstools:

* **`plot map/plot_comma.py`** $\rightarrow$ Vergleicht den Odometrie- und AMCL-Pfad auf einer Karte (`map.yaml` inkl. passender `map.pgm`-Datei). Dazu die entsprechenden Dateien im Ordner (`map`, `pathDataAMCL` und `pathOdom`) ersetzen und `plot_comma.py` ausführen.
* **`plot sim/plot_comma.py`** $\rightarrow$ Visualisiert die geloggte Route der `giovanni_sim`-Node. Dazu die geloggte `pos.dat`-Datei in diesen Ordner kopieren und das Skript ausführen.
* **`standardplotter/plot_comma.py`** $\rightarrow$ Visualisiert beliebige Routen unserer Nodes. Dazu die gewünschte Datei in `pos.dat` umbenennen, in den Ordner kopieren und das Skript ausführen.

---

## Für `Jazzy`

* **`vidPfade/`** $\rightarrow$ Enthält die `.dat`-Files und Bag-Files für die odometriebasierte Pfadverfolgung aus dem Video.
* **`Beispielspfade/`** $\rightarrow$ Enthält die bereitgestellten Beispielpfade.
* **`giovanni/`** $\rightarrow$ Enthält den ursprünglichen Giovanni-Controller aus der Vorlesung Regelungstechnik (nicht die eigene Regler-Implementierung!).
* **`volksbot/`** $\rightarrow$ Enthält den ROS2-Workspace sowie den eigentlichen Sourcecode für das Praktikum.

---

## Für `Humble`

* **`my_path_generator/`** $\rightarrow$ Enthält das Package zum Loggen von Odometrie- und AMCL-Routen. Die Nodes sind in `amcl_path_generator.cpp` und `odom_path_generator.cpp` implementiert.
* **`giovanni/`** $\rightarrow$ Enthält den bereitgestellten Giovanni-Controller sowie dessen Simulation.

---

## Befehlsübersicht & Ausführung

### Allgemeine Befehle & Mapping

**RViz2 starten (für das Abspielen von Bag-Files):**
`ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true`

**Bag-File abspielen:**
`ros2 bag play [NAME] --clock`

**Cartographer ausführen:**
`ros2 launch volksbot cartographer2d.launch`

**Aufgenommene Karte speichern:**
`ros2 run nav2_map_server map_saver_cli -f [NAME]`

---

### Ausführung unter ROS2 Jazzy (Odometrie-basiert)

**Pfadverfolgung mittels Odometrie:**
`ros2 run giovanni gio_node --ros-args -p path:=[PFAD DER .dat DATEI] -p speed:=[SPEED]`
*(Empfohlener Wert für `[SPEED]`: `0.15`)*

**Path-Logger für X- & Y-Koordinaten (funktioniert auch mit Bag-Files):**
`ros2 run path_logger logger`
*(Erstellt die Datei `odomXYData.txt` im `ros2_ws`-Ordner. Kann z. B. mit GnuPlot visualisiert werden: Spalte 1 = X, Spalte 2 = Y)*

---

### Ausführung unter ROS2 Humble (AMCL-basiert)

**Simulation starten:**
`ros2 run giovanni gio_sim_path --ros-args -p path:="[PFAD DER .dat DATEI]"`

**AMCL-Pfad loggen:**
`ros2 run my_path_generator amcl_path_generator`

**Odometrie-Pfad loggen:**
`ros2 run my_path_generator odom_path_generator`

**Giovanni-Controller auf Basis von AMCL-Daten starten:**
`ros2 run giovanni gio_volksbot --ros-args -p path:="[PFAD DER .dat DATEI]" -p speed:=[SPEED]`
*(Empfohlener Wert für `[SPEED]`: `0.15`)*
