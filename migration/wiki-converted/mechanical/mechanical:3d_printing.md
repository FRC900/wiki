##  Fusion F400 Printer ## 
   - The computer by the printer is where you will slice the part and start the print.

###  Machine Preparation ### 

####  Loading Filament #### 
  - Put the roll of filament into the spool holder on the right.
  - Manually feed the filament until approximately where the end of the filament is visible in the PTFE tube inside the main build enclosure. 
      - Use the orange wheel on the extruder assembly in the spool holder.
  - On the device screen, navigate to Macros and select "Load Filament." 
      - The printer should automatically heat up the hot end and feed the filament through and then stop automatically. 
  - Once filament is loaded, clear the extruded filament from the build area.
####  Preparing the Bed #### 
  - To clean bed:
      - Use a spray bottle of water or water/alcohol solution liberally on the bed. 
      - Wipe with a clean shop towel

  - Applying glue to the bed:
      - Apply in lines from the large glue stick covering the whole area that your print will cover.
      - Number of layers varies based on filament type.
          - For PLA, two layers: one vertical and one horizontal.
          - For iglidur, three layers: vertical, horizontal, and diagonal.
          - For other filaments apply the number of layers recommended by Meg or the manufacturer (https://www.fusion3design.com/f400-adhering-to-and-removing-parts-from-the-print-bed/). 
      - Leave the lid open for a few minutes for maximum effectiveness.

###  Part Preparation ### 
  - Export the part as an stl file.
      - In Solidworks: save as > file type "STL *.stl"
      - In Onshape: right click on the part studio tab near the bottom, select export, and select STL as the Format. 
  - Put the .stl onto the printer computer.
      - Historically, .stl files were uploaded to the orders folder within GrabCAD and downloaded on the computer.
  - Open Simplify3D
  - Add each of the parts you want to print to the bed using the "import" button on the left side of the screen.
  - To orient parts, press "ctrl+L" and select the face you wish to be down.
  - To duplicate parts, select the parts you wish to duplicate, press "ctrl+D" and enter the number of copies.
  - Once you are happy with the number and orientation of parts, select "Center and Arrange."
  - To make sure the print has the correct settings, select "Edit Process Settings."
      - Select the correct process from the dropdown near the top for the filament you are using.
      - The majority of the time, you only might need to change these settings:
          - How support is generated: none, from build plate only, or everywhere
              - Note: You can manually add supports on parts where the computer isn't getting it right in the main editor using one of the buttons on the right side. 
          - Infill percentage.
  - Once you are happy with the settings, exit that menu and click "prepare to print."
  - Double check the preview to make sure the support is in the right spots and everything looks right.
  - Save the .gcode file to the folder "FRC 3D printer files"

###  Printing the part ### 
  - Open Chrome on the printer computer and select the bookmark for the printer control panel.
  - Connect to the printer (will require a password, ask a mentor or another student that has used the printer).
  - Select "Upload and Print" and find your file (likely the first file in the list).
  - Watch the first few layers to make sure it's not doing anything weird.
  - Come back in anywhere from a few minutes to many hours to collect your print

###  Getting parts off of the print bed ### 
  - Use the spatula to separate the print from the bed.
      - Make sure the spatula stays sliding on the print bed when you are trying to get parts off. 
      - This may require a large amount of force, make sure not to have hands in the path.
  - If a part rolls below the bed, move it using the screen on the printer and retrieve your part. 