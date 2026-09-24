# Praktikum Mess- und-Regelungstechnik 2026 - Daniel Fratczak, Simon Lingor
src Code und weiterer relevanter Code für unsere Bearbeitung vom Praktikum Mess- und Regelungstechnik 2026

Dies ist das README-file für das Hardwarepraktikum SS26 an der JMU Würzburg. Hier wird beschrieben, wie man was zum Laufen bekommt, jedoch nur für die selbstgeschriebenen ros2 Nodes.

Es wurde ROS2 jazzy unter Mint Linux genutzt, zur Bearbeitung der Aufgaben und für die Aufnahme der Bag-Files.

Ordnerstruktur:
tools -> Hier befinden sich alle Werkzeuge, die wir genutzt haben, abseits von ROS2 (meist unter Python)
Jazzy -> alle ROS2 src Dateien

Für tools:

Für Jazzy:
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
