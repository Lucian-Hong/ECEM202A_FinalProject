# Project Proposal

## 1. Motivation & Objective

The goal of Human-Robot Collaboration for Search and Retrieval is to develop an advanced robotic system that enhances search and retrieval tasks in complex or hazardous environments. Current manual methods are time-consuming and pose safety risks to human operators. By integrating machine learning, autonomous navigation, and voice-controlled interactions, we aim to develop a system that improves efficiency, safety, and user experience.

## 2. State of the Art & Its Limitations

Currently, search and retrieval operations rely on human efforts or partially automated systems. Existing robotic solutions either focus solely on autonomous navigation without human interaction or require constant manual control, limiting their effectiveness in unpredictable environments, and the lack of human-machine collaboration makes them less adaptable and more resource intensive.

## 3. Novelty & Rationale

Our project integrates real-time object recognition (using YOLO), voice command processing (using Whisper), and voice feedback (using pyttsx3) on a portable Raspberry Pi 5 AI Kit with battery attached to the Unitree Go 2 robot dog. This combination of technologies supports autonomous operation and human interaction, making the system adaptable to various scenarios. 

## 4. Potential Impact

If successful, the project can improve how search and retrieval tasks are performed in challenging environments, and technically advance the use of machine learning and human-robot collaboration. Broadly, it can improve safety for operators, reduce workload, and increase the speed and accuracy of search operations in industrial, emergency, and hazardous settings.

## 5. Challenges

Key challenges include maintaining real-time object recognition performance on Raspberry Pi 5, integrating the components within ROS 2 framework, and ensuring reliable voice recognition in different environments. Risk factors include hardware limitations, potential latency issues, and accurate communication between the robot and human operator.

## 6. Requirements for Success

To ensure the success of this project, the following skills and resources are essential:

### Technical Skills
- **Programming Language**: Proficiency in Python for development.
- **Robotics Framework**: Familiarity with ROS 2 for integrating various system components.
- **Machine Learning**: Experience with training and deploying YOLO for object recognition and Whisper for voice recognition.

### Hardware Resources
- **Unitree Go 2 Robot Dog**: The main platform for autonomous navigation and interaction.
- **Raspberry Pi 5**: Used for running software components within ROS 2 framework, processing object recognition, and handling voice commands.

### Software Libraries
- **YOLO**: Real-time object detection.
- **Whisper**: Voice recognition and command processing.
- **pyttsx3**: Text-to-speech library for generating audio feedback.

## 7. Metrics of Success

The following metrics will be used to assess the project's success:

- **Object Recognition Accuracy**: Measured by the percentage of correctly identified objects during exploration.
- **Voice Command Reliability**: The successful execution rate of voice commands in various environments.
- **Operator Feedback**: User satisfaction scores collected from trials.
- **Task Completion Time**: The time taken to identify and retrieve objects compared to existing manual methods.

## 8. Execution Plan

## Execution Plan
### Key Tasks
1. **System Setup**:
   - Integrate the Raspberry Pi 5 with ROS 2 to control the Unitree Go 2 robot dog.
   - Configure and test hardware connections.

2. **Software Development**:
   - Implement ROS 2 architecture for the integration of navigation, object recognition, and voice interaction.
   - Develop the machine learning-based object recognition module using YOLO.
   - Build the voice command interface using Whisper for recognition and pyttsx3 for feedback.

3. **Testing and Iteration**:
   - Conduct extensive tests in controlled environments to ensure reliability.
   - Optimize software for real-time performance on the Raspberry Pi 5.

4. **Performance Evaluation**:
   - Collect data based on success metrics such as accuracy, response times, and user feedback.
   - Refine the system based on test results and operator input.

### Task Partition
- **Johnson Liu**:
- **Yi Han**:
- **Pinhao Hong**:

## 9. Related Work

### 9.a. Papers
List the key papers that you have identified relating to your project idea, and describe how they related to your project. Provide references (with full citation in the References section below).

### 9.b. Datasets
- **COCO Dataset**: Utilized for training object recognition models. [COCO](https://cocodataset.org/)
- **Open Speech Datasets**: Used for voice command training. [Open Speech](https://openslr.org/)

### 9.c. Software
- **YOLO**: Real-time object detection. [YOLO](https://pjreddie.com/darknet/yolo/)
- **Whisper by OpenAI**: Voice recognition. [Whisper](https://github.com/openai/whisper)
- **pyttsx3**: Text-to-speech library for Python. [pyttsx3](https://pyttsx3.readthedocs.io/en/latest/)

## References
- Unitree Go2 ROS2 SDK Project: https://github.com/abizovnuralem/go2_ros2_sdk
- COCO Dataset: https://cocodataset.org/
- Open Speech Datasets: https://openslr.org/
- YOLO: https://pjreddie.com/darknet/yolo/
- Whisper: https://github.com/openai/whisper
- pyttsx3: https://pyttsx3.readthedocs.io/en/latest/
