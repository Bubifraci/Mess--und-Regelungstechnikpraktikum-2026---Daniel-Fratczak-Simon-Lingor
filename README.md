# Praktikum Mess- und-Regelungstechnik 2026 - Daniel Fratczak, Simon Lingor
src Code und weiterer relevanter Code für unsere Bearbeitung vom Praktikum Mess- und Regelungstechnik 2026

Dies ist das README-file für das Hardwarepraktikum SS26 an der JMU Würzburg. Hier wird beschrieben, wie man was zum Laufen bekommt, jedoch nur für die selbstgeschriebenen ros2 Nodes.

Es wurde ROS2 jazzy unter Mint Linux genutzt, zur Bearbeitung der Aufgaben und für die Aufnahme der Bag-Files.

Ordnerstruktur:
tools -> Hier befinden sich alle Werkzeuge, die wir genutzt haben, abseits von ROS2 (meist unter Python)
Jazzy -> alle ROS2 src Dateien

**Für tools:**
Hier finden sich zwei Ordner: datModifier und plotter. 

datModifier ist das Werkzeug, welches wir zum Modifizieren der .dat Routen verwenden. Um dies zu verwenden, füge man die zu bearbeitende Route in denselben Ordner und nenne diese 'path.dat'. Innerhalb des Codes findet man eine Variable namens 'scale', standardmäßig ist diese auf 0.5 gesetzt. Je nachdem, wie die Route skaliert werden soll, kann man diesen Wert anpassen. Ist alles ordnungsgemäß eingerichtet, so kann man die Transformation der Route durchführen, indem man die Python-Datei ausführt: "python3 pathModifier.py" (matplotlib und numpy sind erforderlich!). Die modifizierte Route wird in die Datei "modified.dat" geschrieben und es öffnet sich ein Plot, welches die alte Route mit der modifizierten Route vergleicht. 

In plotter/ hingegen finden sich mehrere unterschiedliche Tools:
- plot map/plot_comma.py -> Dieses Tool vergleicht ein Odometrie- und AMCL-Weg auf einer Karte map.yaml (inkl. der passenden map.pgm Datei). Dazu ersetze man die jeweiligen Dateien in dem Ordner (map, pathDataAMCL und pathOdom) mit demselben Namen in dem Ordner und führe plot_comma.py aus.
- plot sim/plot_comma.py -> Dieses Tool visualisiert die geloggte Route von unserer Giovanni-Sim-Node. Auch hier gilt: In demselben Ordner die geloggte pos.dat Datei kopieren und die Python Datei ausführen.
- standardplotter/plot_comma.py -> Dieses Tool visualisiert beliebige Routen unserer Nodes. Dazu die Datei zu "pos.dat" umbenennen und in den Ordner kopieren. Dann die Python-Datei ausführen und die Route wird visualisiert.

**Für Jazzy:**
Im Ordner vidPfade befinden sich die .dat files und bag files für die odometriebasierte Pfadverfolgung, die im Video angesprochen wurden.
Der Ordner Beispielspfade enthält die zur verfügunggestellten Beispielpfade.
Der Ordner giovanni enthält den giovanni-Controller, so wie wir ihn in Regelungstechnik zur Verfügung gestellt bekommen haben. Dies ist nicht die implementierung des Reglers!
Der Workspace und Sourcecode für das Praktikum ist im Ordner volksbot zu finden.

Starten von rviz2 für das Abspielen von aufgenommenen Bag-Files:
$ ros2 run rviz2 rviz2 --ros-args -p use_sim_time:=true

Abspielen eines Bag-Files:
$ ros2 bag play [NAME] --clock

Cartographer ausführen:
$ ros2 launch volksbot cartographer2d.launch

Speichern der aufgenommenen Karte:
$ ros2 run nav2_map_server map_saver_cli -f [NAME]

Pfadverfolgung mittels Odometrie:
$ ros2 run giovanni gio_node --ros-args -p path:=[PFAD DER .dat DATEI] -p speed:=[SPEED]
-> Empfehlung für [SPEED] ist 0.15

Path logger für X- & Y-Koordinaten des Roboters (Funktioniert auch mit Bag-Files):
$ ros2 run path_logger logger 
-> Erstellt eine Datei names "odomXYData.txt" im ros2_ws Ordner. Diese kann mit dann z.B. mit GnuPlot geplottet werden. Spalte 1 = X, Spalte 2 = Y.
