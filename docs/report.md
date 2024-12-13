# Human-Robot Collaboration for Search and Retrieval on Unitree Go 2 Platform

## Table of Contents
* [Abstract](#abstract)
* [Introduction](#1-introduction)
* [Related Work](#2-related-work)
* [Technical Approach](#3-technical-approach)
* [Evaluation and Results](#4-evaluation-and-results)
* [Discussion and Conclusions](#5-discussion-and-conclusions)
* [References](#6-references)

---

## Abstract

**Human-Robot Collaboration for Search and Retrieval** aims to improve search and retrieval tasks in complex and hazardous environments. The project leverages the **Unitree Go 2 robot dog**, **Raspberry Pi 5**, and software tools including **ROS 2**, **YOLO-World**, and **Vosk** for autonomous exploration, real-time object detection, and voice-controlled human-robot interaction. The system balances autonomy with human guidance, achieving efficient navigation and user-friendly interaction. Initial results show sub-second response times and high object detection accuracy, demonstrating the system’s potential across various domains.

---

## 1. Introduction

### Motivation & Objective
The goal is to develop a system that improves the efficiency and safety of search and retrieval tasks in environments like disaster zones, warehouses, or industrial facilities. By combining robotics and voice commands, the project reduces manual effort and minimizes risks.

### State of the Art & Its Limitations
Existing systems focus on either fully autonomous robots or manual operations. Fully autonomous robots struggle with adaptability, while manual methods expose users to risks and inefficiencies. Current systems rarely integrate effective human-robot collaboration for dynamic environments.

### Novelty & Rationale
This project integrates real-time object detection using **YOLO-World**, voice recognition with **Vosk**, and modular ROS 2 nodes for seamless collaboration. By combining autonomy with user interaction, the system offers adaptability and improved usability.

### Potential Impact
If successful, this project will:
- Enhance operational efficiency in search and retrieval tasks.
- Improve safety for operators in complex environments.
- Provide a scalable solution for diverse domains, including industrial and emergency applications.

### Challenges
- Localizing of ROS 2 and the SDKs we used on the Raspberry Pi 5.
- Real-time performance with limited computational resources on the Raspberry Pi 5.
- Integration of navigation, object detection, and voice command nodes.

### Requirements for Success
- **Skills:** Python & C++ programming, ROS 2, and machine learning expertise.
- **Resources:** Unitree Go 2 robot, Raspberry Pi 5, and external microphone and speaker.
- **Software:** ROS 2.

### Metrics of Success

The following metrics will be used to evaluate the performance of the system across key functional areas:

#### **1. Object Detection & Voice Command Recognition Performance**
- **Accuracy:**  
  Measure the system's ability to correctly identify objects and interpret voice commands.  
  *Target*: Object detection accuracy > 90%, voice command recognition rate > 90%.  

- **Inference Speed & Latency:**  
  Time taken to process object detection and voice commands.  
  *Target*: Inference latency < 500ms for each operation.  

- **Error Rate:**  
  Rate of false positives/negatives in object detection and incorrect voice command interpretations.  
  *Target*: Combined error rate < 15%.

#### **2. Autonomous Exploration & Navigation**
- **Localization Accuracy:**  
  Evaluate the precision of the robot’s location estimates during exploration.  
  *Target*: Average localization error < 0.5m.  

- **Path Planning Efficiency:**  
  Measure the ability of the robot to compute and follow optimal paths to the target object.  
  *Target*: Robot minimizes redundant movements and reaches the target efficiently.  

- **Collision Avoidance:**  
  Evaluate the system's ability to navigate around obstacles without collisions.  
  *Target*: Zero collisions during testing.  

#### **3. Task Execution with Different Difficulty Levels**
- **Task Success Rate:**  
  The percentage of tasks completed successfully across varying difficulty levels (e.g., complex environments, overlapping objects).  
  *Target*: Success rate > 80%.  

- **Execution Time:**  
  Time taken to locate and retrieve the target object from the moment the command is issued.  
  *Target*: Average execution time < 30 seconds per task for simple search tasks.  

- **Error Handling:**  
  Evaluate the system's ability to recover from interruptions or errors (e.g., misdetections, voice misinterpretations).  
  *Target*: Recovery success rate > 80%.  

#### **4. Usability & Setup Performance**
- **Setup Time:**  
  Measure the time required to initialize the system and prepare it for operation.  
  *Target*: Setup time < 5 minutes.  

- **Error Recovery:**  
  Assess the time and success rate of recovering from setup or runtime errors.  
  *Target*: Recovery time < 1 minute, recovery success rate > 80%.  

---

# 2. Related Work

### go2\_ros2\_sdk
- **Description:** Foundational tools and APIs for mapping, navigation, and motion control.
- **Link:** [go2\_ros2\_sdk on GitHub](https://github.com/nesl/go2_ros2_sdk)

### m-explore
- **Description:** Framework for autonomous exploration and mapping in ROS 2.
- **Link:** [m-explore on GitHub](https://github.com/nesl/m-explore-ros2)

### YOLO-World
- **Description:** YOLO-based object detection for real-time, descriptive recognition.
- **Link:** [YOLO-World](https://www.yoloworld.cc/)

### Vosk
- **Description:** Lightweight speech recognition with real-time processing.
- **Link:** [Vosk](https://alphacephei.com/vosk/)

### From Human-Human Collaboration to Human-AI Collaboration
- **Description:** Provides theoretical grounding for designing intuitive human-robot interfaces.
- **Citation:** Dakuo Wang, Elizabeth Churchill, Pattie Maes, Xiangmin Fan, Ben Shneiderman, Yuanchun Shi, and Qianying Wang. 2020. From Human-Human Collaboration to Human-AI Collaboration: Designing AI Systems That Can Work Together with People. In Extended Abstracts of the 2020 CHI Conference on Human Factors in Computing Systems (CHI EA '20). Association for Computing Machinery, New York, NY, USA, 1–6. https://doi.org/10.1145/3334480.3381069


---

# 3. Technical Approach

### Key Components
- **Hardware:**
   - Unitree Go 2 robot
   - Raspberry Pi 5
   - External microphone and speaker
- **Software:**
   - ROS 2
   - YOLO for object detection
   - Vosk for voice recognition
   - pyttsx3 for text-to-speech feedback

### Workflow

#### Key Functionalities by Node
- **Unitree Go2 Main Node:**
  - Collects sensor data (e.g., camera, LiDAR) and publishes to topics.
- **Voice Command Node:**
  - Processes user commands and manages interaction with other nodes.
- **Object Detection Node:**
  - Detects objects in real-time and publishes results to relevant topics.
- **Explore/Navigation Node:**
  - Navigates autonomously to locate the specified goal/item.

#### 1. Initialization
The system begins with all nodes in an OFF state:
- **Unitree Go2 Main Node**
- **Voice Command Node**
- **Object Detection Node**
- **Explore/Navigation Node**

When the system is started:
- The **Unitree Go2 Main Node** is activated to manage sensor data collection and publish it to their respective topics.
- The **Voice Command Node** enters standby mode, awaiting interaction.
- The **Object Detection Node** and **Explore/Navigation Node** remain in standby mode.

#### 2. Voice Command Interaction
- **Listening for Wake Word:**
   - The **Voice Command Node** listens for a wake word (e.g., “Hi Ben”) from the user.
   - Once the wake word is detected, the **Voice Command Node** becomes active.

- **Listening for Commands:**
   - The user provides a search command, specifying the object to find (e.g., “Search for a bottle”).
   - The command is processed, and the required item is identified.

- **Command Confirmation:**
   - The **Voice Command Node** confirms the received command and required object with the user.
   - If the user says "No," the node resets to listening for a new command.
   - If the user says "Yes," the command is forwarded for processing.

#### 3. Object Detection and Exploration
- **Activating Search Nodes:**
   - The **Voice Command Node** publishes the search word to the `/object_to_detect` topic.
   - It also confirms the object to detect and publishes it to the `/confirmed_object_to_detect` topic.

- **Object Detection Node Activation:**
   - The **Object Detection Node** becomes active, receiving input from the `/object_to_detect` topic.
   - It starts detecting the specified object using YOLO-World and publishes results to:
     - `/annotated_image`: Annotated images of detected objects.
     - `/detected_objects`: Metadata of detected objects.

- **Explore/Navigation Node Activation:**
   - The **Explore/Navigation Node** uses input from:
     - `/confirmed_object_to_detect` topic.
     - `/detected_objects` topic.
   - It begins autonomous exploration and navigation using **m-explore** to locate the specified item.


#### 4. Item Search and Result Processing
- **Item Matching:**
   - The **Object Detection Node** and **Explore/Navigation Node** continuously process sensor and camera data.
   - Once the specified item is found:
     - A match is checked from the `/confirmed_object_to_detect` topic and `/detected_objects` topic.
     - If the item matches the specified search, the system progresses.
   - If no match is found, the system continues searching.

- **Item Found:**
   - When the item is successfully located:
     - The **Explore/Navigation Node** stops the search process.
     - An annotated image of the detected item is saved.
     - The **Voice Command Node** notifies the user that the item has been located.
     - A message is sent to the `/object_to_detect` topic to stop object detection.

#### 5. Post-Search Process
- After the search is complete:
  - The **Unitree Go2 Main Node** remains active to manage ongoing system functionality.
  - The **Voice Command Node** returns to standby mode, ready for the next command.
  - The **Object Detection Node** and **Explore/Navigation Node** also return to standby mode.


---

## 4. Evaluation and Results

## 5. Discussion and Conclusions

## 6. References
