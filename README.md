# Electrical Grid Protection Analysis with Omicron Testing

This repository contains comprehensive test reports and analyses for three critical electrical protection systems: **Overcurrent Protection (OCP) Relay**, **Differential Protection Relay**, and **Distance Protection Relay**. The tests were conducted using OMICRON test equipment (CMC256plus, CMC156) and software (Version 4.31). The goal is to validate relay settings, performance, and compliance with protection standards under various fault scenarios.

---

## Table of Contents
1. [Differential Protection Relay](#differential-protection-relay)
2. [Distance Protection Relay](#distance-protection-relay)
3. [Overcurrent Protection (OCP) Relay](#overcurrent-protection-ocp-relay)
4. [Key Findings](#key-findings)
5. [Challenges and Solutions](#challenges-and-solutions)
6. [Repository Structure](#repository-structure)
7. [How to Use](#how-to-use)

---

## Differential Protection Relay
**Test Object**: Transformer (YY0 Vector Group, 110 kV, 114 MVA)  
**Test Equipment**: OMICRON CMC256plus (TF294W)  
**Key Tests**:  
- **Operating Characteristics**: Validated Idiff and Ibias thresholds for line-to-line and line-to-earth faults.  
- **Harmonic Restraint**: Confirmed immunity to 2nd harmonic inrush currents (20% threshold).  
- **Trip Time Validation**: All trip times within 0.05–0.06 s tolerance.  

**Results**:  
- 100% pass rate across 20+ test points.  
- Consistent trip times and harmonic restraint performance.  
- [Full Report](differential_protection_report.pdf)  

---

## Distance Protection Relay
**Test Object**: Transmission Line (4.28 Ω impedance, 57° line angle)  
**Test Equipment**: OMICRON CMC156 (NC693N)  
**Key Tests**:  
- **Zone Settings**: Z1 (instantaneous), Z2-Z5 (time-delayed) for phase and earth faults.  
- **Fault Types**: L1-E, L2-L3, L1-L2-L3, etc., with impedance plane validation.  
- **Timing Accuracy**: ≤5% deviation from nominal trip times.  

**Results**:  
- All faults detected within configured impedance zones.  
- Robust performance across 50+ test scenarios.  
- [Full Report](distance_protection_report.pdf)  

---

## Overcurrent Protection (OCP) Relay
**Test Object**: ABB REF615 (UMZ10)  
**Test Equipment**: OMICRON CMC156 (NC693N)  
**Key Tests**:  
- **Ramping Tests**: Pick-up/drop-off thresholds for phase (1.4–9.6 A) and earth (160–240 mA).  
- **Overcurrent Elements**: Definite-time curves for I>, I>>, I>>> (1–10 A).  
- **Pulse Testing**: Validated high-current (8A) and earth fault responses.  

**Results**:  
- 17/17 tests passed with ≤3% tolerance.  
- Successful single-phase configuration for earth fault testing.  
- [Full Report](OCP_relay.pdf)  

---

## Key Findings
- **Accuracy**: All relays operated within configured tolerances (1.5–5%).  
- **Selectivity**: Distance relay zones prevented overreach/underreach.  
- **Harmonic Immunity**: Differential relay ignored 2nd harmonic inrush (20% threshold).  
- **Speed**: Fastest trip time: 0.05 s (differential), 0.02 s (distance).  

---

## Challenges and Solutions
1. **Binary Input Reversal**  
   - *Issue*: CMC156 binary inputs (start/trip) misconfigured.  
   - *Fix*: Reversed trigger logic in OMICRON software.  

2. **Earth Fault Testing**  
   - *Issue*: Incorrect 3-phase setup for earth ramping.  
   - *Fix*: Switched to single-phase configuration.  

3. **Tolerance Calibration**  
   - *Issue*: Overcurrent tests failed at 0.5% tolerance.  
   - *Fix*: Adjusted to 1.5% tolerance for currents <10 A.  

---

## Repository Structure



---

## How to Use
1. **Review Reports**: Open PDFs for detailed test methodologies, graphs, and results.  
2. **Replicate Tests**: Use OMICRON test modules (Diff Configuration, Advanced Distance, Ramping) with the settings described.  
3. **Validate Configurations**: Compare your relay parameters (CT ratios, zone settings) with those in the reports.  

**Software**: OMICRON Test Universe 4.31 or later.  
**Hardware**: CMC256plus, CMC156.  

---

**Contributors**: Yazan Eissa, Lawal Ibrahim Okikiola  
**License**: CC BY-NC-SA 4.0  
**Contact**: [ibromatics38@gmail.com] | [[GitHub Profile](https://github.com/ibromatics38)]
