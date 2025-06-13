import streamlit as st
import numpy as np
import matplotlib.pyplot as plt

# PID Controller class with anti-windup
class PIDController:
    def __init__(self, Kp, Ki, Kd, setpoint, output_limits=(-np.inf, np.inf)):
        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd
        self.setpoint = setpoint
        self.integral = 0
        self.previous_error = 0
        self.output_limits = output_limits

    def update(self, current_value, dt):
        error = self.setpoint - current_value
        self.integral += error * dt
        derivative = (error - self.previous_error) / dt
        output = self.Kp * error + self.Ki * self.integral + self.Kd * derivative
        
        # Anti-windup: Clamp the integral term if output exceeds limits
        if output > self.output_limits[1]:
            output = self.output_limits[1]
            self.integral -= error * dt
        elif output < self.output_limits[0]:
            output = self.output_limits[0]
            self.integral -= error * dt
        
        self.previous_error = error
        return output

# Streamlit app
st.title("PID Controller Simulation with Anti-Windup")

# Sidebar for user inputs
st.sidebar.title("Controls")
initial_value = st.sidebar.slider("Initial Value", min_value=0.0, max_value=10.0, value=0.0)
target_value = st.sidebar.slider("Target Value", min_value=0.0, max_value=10.0, value=1.0)
Kp = st.sidebar.slider("Kp", min_value=0.0, max_value=100.0, value=1.0)
Ki = st.sidebar.slider("Ki", min_value=0.0, max_value=100.0, value=0.0)
Kd = st.sidebar.slider("Kd", min_value=0.0, max_value=10.0, value=0.0)
simulation_time = st.sidebar.slider("Simulation Time (seconds)", min_value=1.0, max_value=100.0, value=10.0)
dt = st.sidebar.slider("Time Step (seconds)", min_value=0.001, max_value=1.0, value=0.01)
delay_time = st.sidebar.slider("Delay Time (seconds)", min_value=0.0, max_value=5.0, value=0.5)
output_min = st.sidebar.slider("Output Min", min_value=-10.0, max_value=0.0, value=-10.0)
output_max = st.sidebar.slider("Output Max", min_value=0.0, max_value=10.0, value=10.0)

# Initialize PID controller
pid = PIDController(Kp, Ki, Kd, target_value, output_limits=(output_min, output_max))

# Second-order system parameters
omega_n = 1.0  # Natural frequency
zeta = 0.5    # Damping ratio
# Simulation
time = np.arange(0, simulation_time, dt)
values = []
current_value = initial_value
rise_time = None
settling_time = None
delay_buffer = [initial_value] * int(delay_time / dt)  # Buffer to simulate pure delay

for t in time:
    delayed_value = delay_buffer.pop(0)
    delay_buffer.append(current_value)
    
    # Second-order system response
    acceleration = -2 * zeta * omega_n * delayed_value - omega_n**2 * current_value
    current_value += acceleration * dt
    
    # PID controller update
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
