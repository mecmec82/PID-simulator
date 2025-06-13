import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

# PID Controller class
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        self.previous_error = error
        return output

# Streamlit app
st.title("PID Controller Simulation")

# User inputs
initial_value = st.number_input("Initial Value", value=0.0)
target_value = st.number_input("Target Value", value=1.0)
Kp = st.number_input("Kp", value=1.0)
Ki = st.number_input("Ki", value=0.0)
Kd = st.number_input("Kd", value=0.0)
simulation_time = st.number_input("Simulation Time (seconds)", value=10.0)
dt = st.number_input("Time Step (seconds)", value=0.01)

# Initialize PID controller
pid = PIDController(Kp, Ki, Kd, target_value)

# Simulation
time = np.arange(0, simulation_time, dt)
values = []
current_value = initial_value
rise_time = None
settling_time = None
for t in time:
    output = pid.update(current_value, dt)
    current_value += output * dt
    values.append(current_value)
    if rise_time is None and current_value >= 0.95 * target_value:
        rise_time = t
    if settling_time is None and abs(current_value - target_value) <= 0.05 * target_value:
        settling_time = t

# Plot results
fig, ax = plt.subplots()
ax.plot(time, values, label="System Response")
ax.axhline(y=target_value, color='r', linestyle='--', label="Target Value")
if rise_time is not None:
    ax.axvline(x=rise_time, color='g', linestyle='--', label="Rise Time")
if settling_time is not None:
    ax.axvline(x=settling_time, color='b', linestyle='--', label="Settling Time")
ax.set_xlabel("Time (s)")
ax.set_ylabel("Value")
ax.legend()

st.pyplot(fig)

# Display rise time and settling time
if rise_time is not None:
    st.write(f"**Rise Time:** {rise_time:.2f} seconds")
else:
    st.write("**Rise Time:** Not reached 95% of target value")

if settling_time is not None:
    st.write(f"**Settling Time:** {settling_time:.2f} seconds")
else:
    st.write("**Settling Time:** Not within ±5% of target value")
