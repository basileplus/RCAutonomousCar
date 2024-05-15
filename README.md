
# Autonomous RC Car Project

## Overview

This project aims to develop an autonomous RC car using reinforcement learning and lidar sensors. The project involves training a neural network in a simulated environment using Webots and transferring the trained model to a real RC car for real-world testing.

<img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/022129c9-feba-4c57-9f09-c53270332cac" alt="Voiture" width="500"/>

## Project Details

### Simulation

The simulation environment for this project was set up using Webots, a robotic simulation software. Webots allows for creating a realistic virtual environment where the reinforcement learning model can be trained efficiently and safely before transferring it to a real-world scenario.

<img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/eaf13dee-a31c-4978-9791-05a28c6d7b8e" alt="Voiture" width="500"/>

### Real car


The real-world implementation of the autonomous RC car involves several key components and technologies. Here is an overview of the hardware and software used:

#### Components

- **Raspberry Pi**: The central processing unit for the RC car, running the Python code and handling sensor data.
- **Python Code**: Custom scripts developed to control the car, process sensor inputs, and execute the trained neural network model.
- **Lidar**: A lidar sensor is mounted on the car to provide real-time distance measurements, crucial for obstacle detection and navigation.
- **Motors**: The RC car is equipped with Dynamixel motors for precise control of steering and propulsion.
- **TT02 Kit**: The Tamiya TT02 chassis kit provides a robust and reliable base for the RC car, supporting the integration of all other components.

<img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/df9dc62e-74a0-4777-90d5-bba6b7506e64" alt="Voiture" width="500"/>


### Reinforcement Learning Process

The model has been trained using reinforcement learning in the Webots simulation environment. The algorithm we used is Proximal Policy Optimization (PPO).

![RL](https://github.com/basileplus/RCAutonomousCar/assets/115778954/9ac5ec91-c4c4-46f8-a27c-70ee85187bcc)


### Simulation to Real World Transfer

The trained model is transferred from the simulation environment to the real RC car for real-world testing. This process involves several steps to ensure a smooth transition and optimal performance:

**Improving Similarity**:
   - Adjust physical parameters to make the simulation more realistic.
   - Use realistic coefficients of friction, car mass, motor power, number of driving wheels, steering angle, and lidar parameters.

**Domain Randomization**:

Implemented domain randomization to introduce randomness, preventing overfitting to the simulation environment and enhancing robustness in real-world scenarios[^1].


1. **Measurement Noise**:

    Add Gaussian noise to sensor measurements (e.g., Lidar) to mimic real-world sensor imperfections.

    <img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/8dbb5e40-76e0-4cd1-a650-630a39e4622c" alt="Voiture" width="500"/>


3. **Action Noise**:

    Add randomness to car actions (e.g., steering angle, speed) to handle real-world variations.

   For instance, the steering response of the simuated car for a given steering command had the following non-linear shape :

    <img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/a6af6301-1179-4d2e-a64f-f034e2836410" alt="Voiture" width="500"/>

    Using a random parameter $b$ we introduce randomness which increases the robustness of the model.

5. **Random Obstacles**:

    Add random obstacles to the track to improve model robustness.

    <img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/ca4bd269-c1ac-48b4-b42a-f16c36ba1f9c" alt="Voiture" width="500"/>


### Performance and Results

#### Simulation Results
- Successfully trained the RL model in Webots with domain randomization.
- Improved model robustness by adding randomness to sensor measurements and car actions.

#### Real-World Testing
- Transferred the trained model to the real RC car.
- Observed the performance in a racing event, demonstrating the effectiveness and areas for improvement.

  <img src="https://github.com/basileplus/RCAutonomousCar/assets/115778954/a56ed84d-6255-48dc-8cc0-24abf9a7cf84" alt="Voiture" width="500"/>



**Observations**:
- The car performed well in simulations and had promising results in real-world testing.
- Encountered zigzagging behavior in straight lines due to discrepancies in simulation parameters and real-world conditions.

**Suggested Improvements**:
- Optimize the real-world car code for better performance seems the most relevant thing to do.
- Use Progressive Neural Networks for continuous learning in real-world conditions[^2].
- Consider simpler simulations for potentially better sim2real transfer[^3].



## Additional notes:
Further details can be found in the ``report.pdf`` file.

## References

[^1]: Tobin, J., Fong, R., Ray, A., Schneider, J., Zaremba, W., & Abbeel, P. (2017). Domain Randomization for Transferring Deep Neural Networks from Simulation to the Real World. Available at [https://arxiv.org/abs/1703.06907](https://arxiv.org/abs/1703.06907).

[^2]: Rusu, A. A., Rabinowitz, N. C., Desjardins, G., Soyer, H., Kirkpatrick, J., Kavukcuoglu, K., Pascanu, R., & Hadsell, R. (2022). Progressive Neural Networks. Available at [https://arxiv.org/abs/1606.04671](https://arxiv.org/abs/1606.04671).

[^3]: Truong, J., Rudolph, M., Yokoyama, N. H., Chernova, S., Batra, D., & Rai, A. (2023). Rethinking Sim2Real: Lower Fidelity Simulation Leads to Higher Sim2Real Transfer in Navigation. In Proceedings of The 6th Conference on Robot Learning. Available at [https://proceedings.mlr.press/v205/truong23a.html](https://proceedings.mlr.press/v205/truong23a.html).


