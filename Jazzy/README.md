# Hardwarepraktikum SS26 – JMU Würzburg

Dies ist das README-file für das Hardwarepraktikum SS26 an der JMU Würzburg. Hier wird beschrieben, wie man was zum laufen bekommt, jedoch nur für die selbstgeschriebenen ros2 Nodes.

Es wurde ros2 jazzy unter Mint Linux zur Bearbeitung der Aufgaben und für die Aufnahme der Bag-Files genutzt.

---

## Ordnerstruktur

* **`vidPfade`** – Hier befinden sich die `.dat` files und bag files für die odometriebasierte Pfadverfolgung, die im Video angesprochen wurden.
* **`Beispielspfade`** – Enthält die zur verfügunggestellten Beispielspfade.
* **`giovanni`** – Enthält den giovanni-Controller, so wie wir ihn in Regelungstechnik zur Verfügung gestellt bekommen haben. Dies ist nicht die implementierung des Reglers!
* **`volksbot`** – Der Workspace und Sourcecode für das Praktikum ist hier zu finden.

---

## Befehle

### Starten von rviz2 für das Abspielen von aufgenommenen Bag-Files:
```bash
$ ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true
```

### Abspielen eines Bag-Files:
```bash
$ ros2 bag play [NAME] --clock
```

### Cartographer ausführen:
```bash
$ ros2 launch volksbot cartographer2d.launch
```

### Speichern der aufgenommenen Karte:
```bash
$ ros2 run nav2_map_server map_saver_cli -f [NAME]
```

### Pfadverfolgung mittels Odometrie:
```bash
$ ros2 run giovanni gio_node --ros-args -p path:=[PFAD DER .dat DATEI] -p speed:=[SPEED]
```
* Empfehlung für `[SPEED]` ist `0.15`

### Path logger für X- & Y-Koordinaten des Roboters (Funktioniert auch mit Bag-Files):
```bash
$ ros2 run path_logger logger
```
* Erstellt eine Datei names "odomXYData.txt" im ros2_ws Ordner. Diese kann mit dann z.B. mit GnuPlot geplottet werden.
  * Spalte 1 := `x`
  * Spalte 2 := `y`
