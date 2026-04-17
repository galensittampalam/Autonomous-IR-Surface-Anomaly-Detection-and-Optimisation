# Autonomous-IR-Surface-Anomaly-Detection-and-Optimisation
This project investigates the feasibility of using low-cost infrared (IR) reflectance sensors for detecting and characterising surface anomalies using the Pololu 3Pi+ robot.
The system extends traditional line-following approaches by enabling the detection, measurement, and classification of surface features such as cracks and discontinuities through reflectance analysis.

## Abstract
This investigation evaluates the optimisation of surface anomaly detecion using the infrared (IR) sensors on the 3Pi+ Robot
The Baseline.... According to experimental results....

## Overview 
Surface inspection is a key task in robotics applications such as industrial quality control and infrastructure monitoring. However, detecting small-scale surface defects using low-cost sensors presents challenges due to noise, sensor bias, and spatial averaging effects.
This project addresses these challenges by implementing a structured sensing and control framework that enables:
- Real-time anomaly detection  
- Estimation of feature width using encoder feedback  
- Normalised reflectance measurement (Darkness %)  
- Robust detection using signal filtering  
- A two-pass scanning strategy for improved accuracy  



## Methodology
### Baseline Data Collection
- Robot traverses controlled experimental maps  
- IR readings recorded alongside encoder position  
- Data used to analyse sensor response and establish detection thresholds  

### Improved Detection System
- Real-time anomaly detection based on reflectance deviation  
- Width estimation using entry/exit position tracking  
- Controlled motion ensures accurate spatial mapping  

### Two-Pass Scanning Strategy
- **Pass 1**: Initial detection of candidate regions  
- **Backtracking**: Robot repositions before anomaly  
- **Pass 2**: Refined detection using adaptive thresholds  

This approach improves robustness against noise and ensures more reliable boundary detection.

## Signal Processing

To improve detection reliability, three filtering techniques were evaluated:

- **Simple Moving Average (SMA)** – basic smoothing  
- **Exponential Moving Average (EMA)** – responsive noise reduction  
- **2nd Order Butterworth Filter** – advanced smoothing  

These filters were compared to evaluate the trade-off between noise reduction and detection responsiveness.

## Experimental Setup

Experiments were conducted using controlled maps with predefined surface anomalies, including variations in:

- Feature width  
- Contrast % 
- Shape and spacing  

This allowed systematic evaluation of detection performance and sensor limitations.

## Repository Structure
- code → Arduino implementation (.ino files)
- maps → Experimental surface maps
- data → Collected sensor data 
- figures → Graphs and results
- docs → Report and supporting documents





 
