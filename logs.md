#  Session Logs

## Session 0 – *09/09/2025*
- **Project Definition**  
  - Discussed general project organization  
  - Missions not yet revealed  
- **Decisions**  
  - Study previous CanSat team's GitHub as reference  
  - Start designing our own motherboard  
  - Focus: modular design → adaptable to different peripherals  
- **Progress**  
  - Listed components and sensors to implement  
- **Goals for Next Session**  
  - Learn basics of required tools:  
    - Onshape  
    - FreeRTOS   
    - GitHub etiquette

## Session 1 – *15/09/2025*
- **Project Definition**  
  - Roles distribution
  - Missions not yet revealed  
- **Decisions**  
  - Study previous CanSat team's GitHub as reference  
  - Focus on testing the sensors while we wait for the rules
- **Progress**  
  - Ordered most of the sensors
  - Set deadlines
- **Goals for Next Session**  
  - Work on the test motherboard

## Session 2 – *23/09/2025*
- **Project Definition**  
  - Missions not yet revealed
- **Decisions**  
  - Reverse engineer previous team's code and implementation
- **Progress**  
  - Analyzed last year's code structure and libraries
  - Studied communication protocols and sensor integration
- **Goals for Next Session**  
  - Complete analysis of previous implementation
  - Prepare for mission announcement

## Session 3 – *30/09/2025*
- **Project Definition**  
  - **NEW: Mission requirements published**
- **Decisions**  
  - Finalize reverse engineering of previous codebase
  - Begin planning for new mission requirements
- **Progress**  
  - Completed analysis of libraries and implementation
  - Documented communication protocols from previous design
- **Goals for Next Session**  
  - Start GNSS receptor configuration
  - Adapt code structure to new missions

## Session 4 – *07/10/2025*
- **Project Definition**  
  - Mission requirements defined
- **Decisions**  
  - Focus on GNSS receptor implementation
- **Progress**  
  - Started GNSS module configuration
  - Tested basic positioning functionality
- **Goals for Next Session**  
  - Complete GNSS integration
  - Begin communication protocol development

## Session 5 – *14/10/2025*
- **Project Definition**  
  - Mission requirements defined
- **Decisions**  
  - Continue GNSS optimization
- **Progress**  
  - Advanced GNSS receptor implementation
  - Improved positioning accuracy and data parsing
- **Goals for Next Session**  
  - Finalize GNSS functionality
  - Begin sensor data integration

## Session 6 – *21/10/2025*
- **Project Definition**  
  - Second report completion and project planning
- **Decisions**  
  - Parallel development across all technical areas
  - Focus on integration testing
- **Progress**  
  - **Communications**: SX1276 module testing and investigation with ground station
  - **3D Design**: Prototype printing and parachute investigation
  - **Programming**: UBX message data extraction implementation
  - **Electronic Design**: LiDAR sensor investigation
- **Goals for Next Session**  
  - Continue module integration
  - Begin system-wide testing
  - Refine parachute design based on prototype results

## Session 7 – *28/10/2025*
- **Project Definition**  
  - Refining the architecture and completing the second status report.
  - Definition of primary (Atmospheric sounding) and secondary (Topo-mapping) missions.

- **Decisions**  
  - **Sensors**: Replacement of the accelerometer with the BNO055 (9-axis IMU) for better orientation data.
  - **Power**: Selection of a 9V Li-ion battery (600 mAh) for an estimated autonomy of 11 hours.
  - **MCU**: Confirmation of STM32L432KC as the main microcontroller.
- **Progress**  
  - **Electronic Design**: Schematic completed in KiCad; warnings cleared, but some errors remain.
  - **Station Sol**: Development of the Python GUI using Tkinter for real-time graphing and logging.
  - **Communications**: Testing GPS accuracy (latitude/longitude error analysis) and ground station reception.
  - **Parachute**: Research into drogue chute types and ballistics.
- **Goals for Next Session**  
  - Fix remaining schematic errors and prepare for routing.
  - Define specific team identity and branding.

## Session 8 – *04/11/2025*
- **Project Definition**  
  - Formalization of team identity and presentation to first-year students.

- **Decisions**  
  - **Team Name**: Adopted "Vortex" as the project name, referencing the spinning strategy.
  - **Mechanics**: Defined parachute diameters: 65cm for the main chute and 45cm for the second chute.
- **Progress**  
  - **Electronic Design**: Schematic fully completed without errors or warnings; ready for routing.
  - **Project Management**: Preparation of presentation materials for student recruitment.
- **Goals for Next Session**  
  - Create project logo and team assets.
  - Begin physical testing of communication modules at home.

## Session 9 – *11/11/2025*
- **Project Definition**  
  - Detailed task distribution among team members to parallelize workflow.

- **Decisions**  
  - Work Distribution: Amr (PCB Design), Ted (LoRa/Github), Juan Pablo (LoRa/Cameras), Abdel (Python mapping), Alexei (PCB/Flaps)
- **Progress**  
  - **Communications**: Retrieval of Nucleo boards and SX1276 modules for individual testing at home.
  - **Electronic Design**: Clarification of power supply modes (shorting PWR pin with E5V) for the Nucleo board.
  - **General**: Creation of the team logo.
- **Goals for Next Session**  
  - Select specific power inductors for the PCB.
  - Finalize sensor selection for the topography mission.

## Session 10 – *11/11/2025*
- **Project Definition**  
  - Preparation for the RCE (Rencontres des Clubs Espace) event in Paris.

- **Decisions**  
  - **Components**: Selection of Würth MAPI power inductors (shielded, high current).
  - **Sensors**: Selection of the Lightware LW20/C LiDAR/ToF sensor for the mapping mission.
- **Progress**  
  - **Events**: Logistics planning for the RCE event on November 22.
- **Goals for Next Session**  
  - Integrate RCE feedback into the project design.
  - Prepare the CANSAT 4 presentation.

## Session 11 – *25/11/2025*
- **Project Definition**  
  - Major design review following RCE feedback and submission of technical reports.

- **Decisions**  
  - **Routing Strategy**: Implementation of a 4-layer PCB (Signal, Ground, Power, Signal) with 9V, 5V, and 3.3V rails.
  - **Mechanics**: Validation of the spinning strategy using servo-controlled flaps.
  - **GPS Placement**: Moving GPS to the top of the CanSat based on RCE feedback.
- **Progress**  
  - **Electronic Design**: Completion of mainboard routing and design of a separate IHM (Interface Homme Machine) board.
  - **Communications**: Successful "Hello World" LoRa transmission and protocol definition.
  - **3D Design**: Modeling of the internal rack and aerodynamic flaps.
  - **Logistics**: Final component orders placed.
- **Goals for Next Session**  
  - Submit the "Dossier de définition".
  - Fix identified PCB connection errors.

  ## Session 12 – *02/12/2025*
- **Project Definition**  
  - Submission of the Definition Dossier defining team goals and preliminary designs.

- **Decisions**  
  - **Mechanics**: Focus shifted to advanced 3D modeling of the structure.

- **Progress**  
  - **Documentation**: "Dossier de définition" completed and sent.
  - **Electronic Design**: Identification of a critical routing mistake (GPS connected to Ven instead of LoRa) to be fixed in V2.
- **Goals for Next Session**  
  - Define specific PCB manufacturing rules.
  - Research handling protocols for sensitive sensors.

  ## Session 13 – *09/12/2025*
- **Project Definition**  
  - Refinement of hardware handling and manufacturing specifications.

- **Decisions**  
  - **Manufacturing**: Set IHM PCB track width to 0.5mm and vias to 0.6mm.
  - **Sensor Handling**: Strict protocol adopted for BMP581 (plastic tweezers only) due to fragility.

- **Progress**  
  - **Sensors**: Discovery of BNO055 PCB optimization guides and confirmation of temperature extraction from the BMP581.
  - **Station Sol**: Integration of battery status and temperature monitoring into the receiver logic.
- **Goals for Next Session**  
  - Solder and test the received V1 PCBs.
  - Present the Ground Station GUI.

    ## Session 14 – *16/12/2025*
- **Project Definition**  
  - Physical integration.

- **Decisions**  
  - **Data Flow**: Finalized architecture as GNSS > STM32 > SX1276 > Ground Station.

- **Progress**  
  - **Electronic Design**: Mainboard V1 received and soldered; IHM PCB completed.
  - **Station Sol**: Demonstration of the GUI with RSSI and sensor data integration.
  - **Sensors**: Waiting for specific connectors (DF52) for the LiDAR.
- **Goals for Next Session**  
  - Power testing of the PCB (LDO and Buck converters).
  - Perform wind tunnel testing for the mechanical structure.


    ## Session – *23/12/2025*
- **Project Definition**  
  - Critical hardware testing and validation phase during the break.

- **Decisions**  
  - **PCB V2**: Added correction of the first buck converter footprint to the V2 task list.
  - **Sensor Handling**: Strict protocol adopted for BMP581 (plastic tweezers only) due to fragility.

- **Progress**  
  - **Electronic Design**: Confirmed LDO outputs clean 3.3V and Buck converter outputs clean 5V.
  - **Mechanics**: Successful wind tunnel tests.
- **Goals for Next Session**  
  -  Resume project work after the exam period.

## Session 15 – *13/01/2026*
- **Project Definition**  
  - Work on tasks defined in the roadmap

- **Decisions**  
  - Organize the Github repository

- **Progress**  
  - **Main PCB**: Managed to light a led.
  
- **Goals for Next Session**  
  -  Station Sol 3D Modelling


