# Design and Control of an Inverted Pendulum System

An **Arduino-based inverted pendulum system** that integrates control theory, embedded hardware, and mechanical design to achieve real-time stabilization. The system balances a pendulum in its upright position using sensor feedback and a PID control algorithm.  

---

## 📌 Project Overview  
The inverted pendulum is a classical problem in control systems and robotics, widely used to test control strategies due to its **unstable dynamics**.  

In this project, we:  
- Derived the system dynamics using **Lagrangian mechanics**  
- Simulated the model in **Python**  
- Designed and implemented a **PID controller**  
- Built the physical prototype using **3D printed structures**  
- Integrated **Arduino Uno, DC motor, motor driver, and encoders** into a closed-loop control system  

The result is a lightweight, real-time balancing system that can resist small disturbances.  

---

## ⚙️ Hardware Setup  
- **Controller**: Arduino Uno  
- **Motor Driver**: L293D  
- **Actuator**: 12V DC N20 micro-geared motor with encoder (for cart displacement)  
- **Sensor**: 600 PPR rotary encoder (for pendulum angle measurement)  
- **Power Supply**: 12V DC  
- **Structure**: Modeled in **SolidWorks**, fabricated via **3D printing**  

---

## 🧮 Control System  
- **Modeling**: Lagrangian mechanics used to derive nonlinear equations of motion  
- **Simulation**: Verified in **Python** prior to hardware implementation  
- **Controller**: PID controller for stabilization  
  - Input: Pendulum angle (θ) from encoder  
  - Output: Motor actuation signal  
- **Objective**: Maintain pendulum at upright position (θ = 180°) while regulating cart displacement  

---

## 📂 Repository Structure  
- `/Initial python simulation` → Python scripts for system modeling and simulation  
- `/microcontroller` → Arduino code for motor control, encoder feedback, and PID algorithm  
- `/system design` → SolidWorks CAD files, assembly drawings, and all 3D printed parts  
  

---

## 🚀 How It Works  
1. The **rotary encoder** measures pendulum angle in real time.  
2. The **Arduino Uno** computes the error (desired upright angle – current angle).  
3. The **PID controller** generates the control signal.  
4. The **L293D motor driver** drives the DC motor to move the cart accordingly.  
5. This feedback loop keeps the pendulum upright.  

---

## 🎯 Applications  
- Benchmark system for **control theory** and **robotics research**  
- Foundation for advanced control techniques (LQR, state feedback, adaptive control)  
- Educational demonstration of **real-time embedded control**  
- Can be extended to **self-balancing robots**, **ball-and-beam systems**, or **drone gimbals**  

---

## 🛠️ Future Improvements  
- Implement advanced control algorithms (LQR, MPC, Reinforcement Learning)  
- Add state estimation with **Kalman Filter**  
- Use higher-torque motors for faster disturbance rejection  
- Upgrade to **ROS2** and integrate with simulations in **Gazebo/Simulink**

## 📝 License  
This project is open-source under the **MIT License**.  
