# CanSat 2025-2026
ENSEA's CanSat team project for the 2025–2026 edition  

---

## 📋 Objective
Build a **satellite the size of a 33cl can** capable of completing different missions during a **150m drop**.


### 🛰️ Current strategy :

- **Main mission : Parachute deployment**
  - Kirigami inspired drogue chute (CF links) acting as a protective cap on the Cansat
  - Use the drogue chute to deploy the main parachute
  - Release the drogue chute with an string and elastic band tied to a servomotor
 
- **Secondary mission : Topography**
  - ***Plan A :***
  -  Use a 100m time of flight (TOF) sensor to map the ground
  -  Induce a rotation thanks to an helicoïdal shaped can
  -  Precisely place the mapped point with the help of an Inertial Measurment Unit (IMU) and a barometer
  - ***Plan B :***
  -  Use a high resolution camera to film the whole descent
  -  Create a topological map thanks to an AI after the mission.
     
- **Tertiary mission : Recording of the landing**
    - Using an onboard first person view (FPV) camera
      
- **Bonus mission : Live video feed to the ground station**
    - Display all recoreded data and live feed of the descend to the ground station
  
- **330mL volume limit**
    - Fit everything under a 330mL limit and a 350g limit
---

## 👥 Team Members
| Name | Role |
|------|------|
| Alexeï DOUILLARD | 3D Modeling, Satellite integration |
| Juan Pablo BARONA CIFUENTES | Communications |
| Ted KOYAZANDE | Ground Station |
| Amr TAOUIS | PCB Design |
| Abdelmoughit HAJJI LAAMOURI | Code and Firmware |

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
  - Peripheral interface design (Buttons, leds and a screen)
- **Current Focus**
  - LiDAR sensor testing and evaluation

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
  - Integration testing with SX1276 modules (Working with a nucealo board)
  - Communication protocol validation

### 🎨 3D Design
- **Mechanical Structure**
  - Can-sized satellite enclosure design
  - Component placement and mounting
  - Aerodynamic considerations for descent : spiral shape + winglets
- **Prototyping**
  - 3D modeling and simulation
  - Material selection and testing (PLA for now)
  - Integration with electronic components
- **Current Focus**
  - Prototype printing and fit testing
  - Parachute design validation and testing

---

## 🗺️ Roadmap
- [X] Learn required tools  
- [x] Define missions
- [ ] Motherboard design  (In prgoress)
- [ ] Peripheral integration  
- [ ] Drop test & validation  

---

## 📚 Resources
- [Previous ENSEA CanSat GitHub](https://github.com/mathieupommery/CANSAT_ARES_ENSEA)
- [Onshape Docs](https://cad.onshape.com/documents?resourceType=folder&nodeId=558406456f078be48d4c722b&column=modifiedAt&sortOrder=asc)
- [Kirigami inspierd parachute](https://www.nature.com/articles/s41586-025-09515-9#Fig1)
- [FreeRTOS Guide](https://www.youtube.com/watch?v=OPrcpbKNSjU)  

---

## 🛰️ About CanSat
CanSat competitions challenge teams to design, build, and launch a **can-sized satellite** that completes scientific or engineering missions during descent. 
