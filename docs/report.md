# Table of Contents
* Abstract
* [Introduction](#1-introduction)
* [Related Work](#2-related-work)
* [Technical Approach](#3-technical-approach)
* [Evaluation and Results](#4-evaluation-and-results)
* [Discussion and Conclusions](#5-discussion-and-conclusions)
* [References](#6-references)

---

# Abstract

**Human-Robot Collaboration for Search and Retrieval** aims to improve search and retrieval tasks in complex and hazardous environments. The project leverages the **Unitree Go 2 robot dog**, **Raspberry Pi 5**, and software tools including **ROS 2**, **YOLO-World**, and **Vosk** for autonomous exploration, real-time object detection, and voice-controlled human-robot interaction. The system balances autonomy with human guidance, achieving efficient navigation and user-friendly interaction. Initial results show sub-second response times and high object detection accuracy, demonstrating the system’s potential across various domains.

---

# 1. Introduction

### Motivation & Objective
The goal is to develop a system that improves the efficiency and safety of search and retrieval tasks in environments like disaster zones, warehouses, or industrial facilities. By combining robotics and voice commands, the project reduces manual effort and minimizes risks.

### State of the Art & Its Limitations
Existing systems focus on either fully autonomous robots or manual operations. Fully autonomous robots struggle with adaptability, while manual methods expose users to risks and inefficiencies. Current systems rarely integrate effective human-robot collaboration for dynamic environments.

### Novelty & Rationale
This project integrates real-time object detection using **YOLO-World**, voice recognition with **Vosk**, and modular ROS 2 nodes for seamless collaboration. By combining autonomy with user interaction, the system offers adaptability and improved usability.

### Potential Impact
If successful, this project will:
- Enhance operational efficiency in search and retrieval tasks.
- Improve safety for operators in hazardous environments.
- Provide a scalable solution for diverse domains, including industrial and emergency applications.

### Challenges
- Ensuring reliable voice recognition in noisy environments.
- Real-time performance with limited computational resources on the Raspberry Pi 5.
- Seamless integration of navigation, detection, and interaction nodes.

### Requirements for Success
- **Skills:** Python & C++ programming, ROS 2, and machine learning expertise.
- **Resources:** Unitree Go 2 robot, Raspberry Pi 5, and external microphone and speaker.
- **Software:** ROS 2.

### Metrics of Success
- **Response Time:** Commands executed in under one second.
- **Detection Accuracy:** High reliability for object recognition in varied scenarios.
- **User Feedback:** Positive user satisfaction scores.

---

# 2. Related Work

### go2\_ros2\_sdk
- **Description:** Foundational tools and APIs for mapping, navigation, and motion control.
- **Link:** [go2\_ros2\_sdk on GitHub](https://github.com/nesl/go2_ros2_sdk)

### m-explore
- **Description:** Framework for autonomous exploration and mapping in ROS 2.
- **Link:** [m-explore on GitHub](https://github.com/nesl/m-explore-ros2)

### yoloworld
- **Description:** YOLO-based object detection for real-time, descriptive recognition.
- **Link:** [yoloworld](https://www.yoloworld.cc/)

### Vosk
- **Description:** Lightweight speech recognition with real-time processing.
- **Link:** [Vosk](https://alphacephei.com/vosk/)

### From Human-Human Collaboration to Human-AI Collaboration
- **Description:** Provides theoretical grounding for designing intuitive human-robot interfaces.
- **Citation:** Amershi, S., et al., Communications of the ACM, 2019.

---

# 3. Technical Approach

### Key Components
1. **Hardware:**
   - Unitree Go 2 robot
   - Raspberry Pi 5
   - External microphone and speaker
2. **Software:**
   - ROS 2
   - YOLO for object detection
   - Vosk for voice recognition
   - pyttsx3 for text-to-speech feedback

### Workflow
1. **Initialization:** Activate nodes for navigation, object detection, and voice commands.
2. **Exploration:** Use **m-explore** for autonomous mapping.
3. **Object Detection:** Perform real-time recognition using YOLO.
4. **Voice Interaction:** Recognize user commands with Vosk.
5. **Feedback:** Provide system status and results using pyttsx3.

### Modular Node Design
- **Voice Command Node:** Processes wake words and search commands.
- **Object Detection Node:** Detects and publishes object information.
- **Explore/Navigation Node:** Executes autonomous search and retrieval tasks.

# 4. Evaluation and Results

# 5. Discussion and Conclusions

# 6. References
