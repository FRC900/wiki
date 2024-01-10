# RQT Custom Plugins #
The purpose of this page is to document the slightly too long process of creating a custom rqt plugin similar to the likes of rqt_graph, rqt_plot, and rqt_gui. We used custom rqt plugins to create a fake driver station that connects to our code in sim like how the real FRC driver station would.

==== Creating the GUI ====
Bonus of using RQT is the relative ease with which you can create a pop up GUI. However the only software I have found so far to do this is QT designer.[[http:*doc.qt.io/archives/qt-4.8/designer-manual.html]]. This software is ok, but not great for creating GUIs.

Start the designer from the main Ubuntu launcher. Hit the windows key, type qt, select QT4 Designer

The driver station sim UI is in ...zebROS_ws/src/rqt_driver_station_sim/resource/driverStatonSim.ui,  This can be opened using the Open... menu.

