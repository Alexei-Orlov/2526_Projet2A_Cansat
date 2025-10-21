# CanSat 2025-2026
ENSEA's CanSat team project for the 2025–2026 edition  

---

## 📋 Objective
Build a **satellite the size of a 33cl can** capable of completing different missions during a **120m drop**.

---

## 👥 Team Members
| Name | Role |
|------|------|
| Alexeï DOUILLARD | 3D Modeling |
| Juan Pablo BARONA CIFUENTES | Communications |
| Ted KOYAZANDE | Ground Station |
| Amr TAOUIS | PCB Design |
| Abdelmoughit HAJJI LAAMOURI | Code and Firmware |

---

## 📝 Session Logs

### Session 0 – *09/09/2025*
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

### Session 1 – *15/09/2025*
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

### Session 2 – *23/09/2025*
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

### Session 3 – *30/09/2025*
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

### Session 4 – *07/10/2025*
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

### Session 5 – *14/10/2025*
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

### Session 6 – *21/10/2025*
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

---

## 🔧 Technical Sections

### 📡 Communications & Software
- **GNSS Implementation**
  - Configuration and testing of GNSS receptor
  - Data parsing and accuracy optimization
  - Integration with main system architecture
- **Communication Protocols**
  - Analysis of previous team's communication structure
  - Development of robust data transmission system
  - Error handling and data validation
- **Current Focus**
  - SX1276 module testing with ground station
  - UBX message data extraction and parsing

### 🔌 Electronic Design
- **PCB Development**
  - Modular motherboard design
  - Component selection and integration
  - Power management and distribution
- **Sensor Integration**
  - Testing and calibration of various sensors
  - Signal processing and data acquisition
  - Peripheral interface design
- **Current Focus**
  - LiDAR sensor testing and evaluation
  - Component validation for final design

### 🖥️ Ground Station
- **Station Development**
  - Data reception and processing
  - Real-time monitoring interface
  - Communication link management
- **Data Visualization**
  - Telemetry display systems
  - Mission control dashboard
  - Data logging and analysis
- **Current Focus**
  - Integration testing with SX1276 modules
  - Communication protocol validation

### 🎨 3D Design
- **Mechanical Structure**
  - Can-sized satellite enclosure design
  - Component placement and mounting
  - Aerodynamic considerations for descent
- **Prototyping**
  - 3D modeling and simulation
  - Material selection and testing
  - Integration with electronic components
- **Current Focus**
  - Prototype printing and fit testing
  - Parachute design validation and testing

---

## 🗺️ Roadmap
- [ ] Learn required tools  
- [x] Define missions (November)  
- [ ] Motherboard design  
- [ ] Peripheral integration  
- [ ] Drop test & validation  

---

## 📚 Resources
- [Previous ENSEA CanSat GitHub](#)  
- [Onshape Docs](#)  
- [FreeRTOS Guide](#)  

---

## 🛰️ About CanSat
CanSat competitions challenge teams to design, build, and launch a **can-sized satellite** that completes scientific or engineering missions during descent. 
