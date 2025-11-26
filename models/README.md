# Hardware & Design Documentation

> "I'll Just Make One Quick Adjustment..." - Famous last words that spawned infinite versions 

## Table of Contents

1. [Quick Use Guide](#quick-use-guide)
2. [Design Timeline](#design-timeline)
3. [International Round](#international-round)
   1. [Phase 1: Minor Improvement from the National](#)
   2. [Phase 2: Lidar Mounting](#)
   3. [Phase 3: New Camera Mounting](#)
4. [National Round](#)
   1. [Phase 1: Open Challenge](#phase-1-open-challenge)
   2. [Phase 2: Obstacle Challenge](#phase-2-obstacle-challenge)
   3. [Phase 3: Chassis Modification](#phase-3-chassis-modification)
5. [License](#license)
6. [Contributing](#contributing)

---

## Quick Use Guide

All 3D models in this repository are provided in three formats for maximum compatibility:

### File Format Usage

#### **STL Files (.stl)**
- **Purpose**: Ready for 3D printing
- **Usage**: Import directly into your 3D printer slicer software
- **Recommended for**: Rapid prototyping, direct printing
- **Software compatibility**: Cura, PrusaSlicer, Simplify3D, and most slicers

#### **STEP Files (.step)**
- **Purpose**: Design modification and engineering
- **Usage**: Open in CAD software for editing, analysis, or integration
- **Recommended for**: Design modifications, dimensional analysis, assembly planning
- **Software compatibility**: SolidWorks, Fusion 360, FreeCAD, Onshape, CATIA

### Print Settings Recommendations
- **Layer Height**: 0.2mm (standard quality)
- **Infill**: 20-30% for structural components
- **Support**: Required for overhangs >60°
- **Material**: We mainly use PLA

---

## Design Timeline

### National Round

| Date | Phase | Version Tag | Component | Key Changes/Reason | Files |
|------|-------|-------------|-----------|-------------------|-------|
| 2025-Aug | Phase 1 | P1V1 | Jetson Enclosure | Initial design for open challenge | [STEP](./National_Round/Jetson_Enclosure(v1).step) \| [STL](./National_Round/Jetson_Enclosure(v1).stl) |
| 2025-Aug | Phase 1 | P1V2 | RealSense Stand | Basic camera mounting solution | [STEP](./National_Round/Intel_RealSense_Stand(v1).step) \| [STL](./National_Round/Intel_RealSense_Stand(1).stl) |
| 2025-Aug | Phase 2 | P2V1 | Jetson Enclosure | Improved ventilation and cable management | [STEP](./National_Round/Jetson_Enclosure(v2).step) \| [STL](./National_Round/Jetson_Enclosure(v2).stl) |
| 2025-Aug | Phase 2 | P2V2 | RealSense Stand | Height adjustment: +2cm for better FOV | [STEP](./National_Round/Intel_RealSense_Stand_2_cm(v2).step) \| [STL](./National_Round/Intel_RealSense_Stand_2_cm(v2).stl) |
| 2025-Aug | Phase 3 | P3V1 | RealSense Stand | Fine-tuned height: +2.5cm optimal positioning | [STEP](./National_Round/Intel_RealSense_Stand_2_half_cm(v3).step) \| [STL](./National_Round/Intel_RealSense_Stand_2_half_cm(v3).stl) |

### International Round
| Date | Phase | Version Tag | Component | Key Changes/Reason | Files |
|------|-------|-------------|-----------|-------------------|-------|
| 2025-Sep | Phase 1 | P1V1 | MainChassis | Improve from National Round | [STEP](./International_Round/MainChassis.step) | [STL](./International_Round/MainChassis.stl) |
| 2025-Sep | Phase 2 | P2V1 | Lidar Mounting | Replace with our previous RealSense place with Lidar Mounting place | [STEP](./International_Round/LidarMount.step) \| [STL](./International_Round/LidarMount.stl) |
| 2025-Sep | Phase 3 | P3V1 | New RealSense Mounting | Because we replace where RealSense used to be mounted with the Lidar | [STEP](./International_Round/RealSenseCam.step) \| [STL](./International_Round/RealSenseCam.stl) |


## License

This project is licensed under the **CERN Open Hardware Licence Version 2 - Strongly Reciprocal (CERN-OHL-S)** - see the [LICENSE](../LICENSE) file for details.

### Why CERN-OHL-S?
The CERN-OHL-S license is specifically designed for open source hardware projects and ensures that:
- All derivative works remain open source
- Modifications must be shared back to the community
- Commercial use is permitted with proper attribution
- Hardware designs are treated appropriately (unlike software-focused licenses)

### Usage Rights
-  **Commercial use** - Use in commercial products and services
-  **Modification** - Adapt and improve the designs
-  **Distribution** - Share the original and modified designs
-  **Patent use** - Protection against patent claims on the licensed designs
-  **Private use** - Use for personal and internal projects

### Conditions
-  **Source disclosure** - Modified designs must be made available
-  **License and copyright notice** - Must be included with distributions
-  **Same license** - Derivative works must use CERN-OHL-S or compatible license
-  **State changes** - Document modifications made to original designs

### Attribution
When using these designs, please provide attribution:
```
WRO AUPP Team - 3D Models for Autonomous Robot Platform
Licensed under CERN-OHL-S v2
Repository: https://github.com/Paulen101/WRO_AUPP
Original Design: [Component Name] v[Version] by WRO AUPP Team
```

### Alternative License Option
If you prefer a Creative Commons approach, this project is also available under **CC BY-SA 4.0**, which provides similar copyleft protections for creative works.

---

## Contributing

We welcome contributions to improve these designs! Please see our [Contributing Guidelines](../CONTRIBUTING.md) for details on:

-  Submitting design improvements
-  Reporting issues with current designs
-  Improving documentation
-  Sharing test results and performance data

### Design Standards
- All models should be provided in both STEP and STL formats
- Include design rationale and testing data
- Follow established naming conventions
- Document any special printing requirements

---
 
*WRO AUPP Team - Autonomous Robot Platform Development*