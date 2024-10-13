import socket
import time
import message_pb2
from google.protobuf.timestamp_pb2 import Timestamp

import matplotlib as mpl
import matplotlib.pyplot as plt
import numpy as np
import keyboard
import threading


def get_timestamp():
    msg = Timestamp()
    msg.FromNanoseconds(time.time_ns())
    return msg

class Sim:
    SIM_DT_SEC = 0.01  # Simulation time step in seconds
    SIM_DT_MS = int(SIM_DT_SEC * 1000)  # Simulation time step in milliseconds
    def __init__(self):

        self._target_position = np.array([50.0, 50.0])
        self._target_estimate = np.array([np.nan, np.nan])
        self._sensor_position = np.array([0.0, 0.0])

        self.omega = 0.05
        self.step_size = 1.0

        self.start_time = time.perf_counter()
        self.prev_time = None
        
        # Register the key press handlers for WASD
        keyboard.on_press_key("w", lambda _: self.on_w())
        keyboard.on_press_key("s", lambda _: self.on_s())
        keyboard.on_press_key("a", lambda _: self.on_a())
        keyboard.on_press_key("d", lambda _: self.on_d())

        # mutex locks
        self.sensor_position_lock = threading.Lock()
        self.target_position_lock = threading.Lock()
        self.target_estimate_lock = threading.Lock()

        self.timer_thread = threading.Thread(target=self.spin).start()
        print("Simulation started\n")

    @property
    def target_position(self):
        with self.target_position_lock:
            return self._target_position
    
    @target_position.setter
    def target_position(self, value):
        with self.target_position_lock:
            self._target_position[0] = np.clip(value[0], 0, 100)
            self._target_position[1] = np.clip(value[1], 0, 100)
    
    @property
    def target_estimate(self):
        with self.target_estimate_lock:
            return self._target_estimate
    
    @target_estimate.setter
    def target_estimate(self, value):
        with self.target_estimate_lock:
            self._target_estimate[0] = np.clip(value[0], 0, 100)
            self._target_estimate[1] = np.clip(value[1], 0, 100)

    @property
    def sensor_position(self):
        with self.sensor_position_lock:
            return self._sensor_position
    
    @sensor_position.setter
    def sensor_position(self, value):
        with self.sensor_position_lock:
            self._sensor_position[0] = np.clip(value[0], 0, 100)
            self._sensor_position[1] = np.clip(value[1], 0, 100)

    def on_w(self):
        self.target_position[1] += self.step_size
    
    def on_s(self):
        self.target_position[1] -= self.step_size

    def on_a(self):
        self.target_position[0] -= self.step_size
    
    def on_d(self):
        self.target_position[0] += self.step_size
    
    def tick(self):
        # Calculate time elapsed since start
        t = time.perf_counter() - self.start_time
        
        if self.prev_time:
            dt = t - self.prev_time
        else:
            dt = self.SIM_DT_SEC
        self.prev_time = t
        # print(f"dt (ms): {int(dt*1000)}")
        
        self.sensor_position[0] = 50.0 + 25.0 * np.sin(2.0 * np.pi * self.omega * t)
        self.sensor_position[1] = 50.0 + 25.0 * np.cos(2.0 * np.pi * self.omega * t)
    
    def spin(self):
        try:
            while True:
                self.tick()
                time.sleep(self.SIM_DT_SEC)
        except KeyboardInterrupt:
            print("Exiting simulation")
        # threading.Timer(self.SIM_DT_SEC, self.tick).start()
    
    def get_bearing_measurement(self, noise_variance=None) -> float:
        # Calculate the bearing measurement in radians with optional additive Gaussian noise
        bearing = np.arctan2(self.target_position[1] - self.sensor_position[1], self.target_position[0] - self.sensor_position[0])
        if noise_variance:
            bearing += np.random.normal(0, noise_variance)
        return bearing
    

class SimFigure(Sim):
    VIS_DT_SEC = 0.1
    VIS_DT_MS = int(VIS_DT_SEC * 1000)

    def __init__(self):
        super().__init__()
        mpl.rcParams['keymap.save'] = ''
        plt.ion()

        self.fig, self.ax = plt.subplots()
        self.ax.axis('equal')
        plt.xlim(0, 100)
        plt.ylim(0, 100)
        
        self.ax.set_title("Use WASD keys to move the target")

        # Create a plot object for the target
        self.target_plot, = self.ax.plot(self.target_position[0], self.target_position[1], 'ro', markersize=10, label='Target')
        self.sensor_plot, = self.ax.plot(self.sensor_position[0], self.sensor_position[1], 'bo', markersize=2, label='Sensor')
        self.estimate_plot, = self.ax.plot(self.target_estimate[0], self.target_estimate[1], 'go', markersize=5, label='Estimate')

        # plot the sensor trajectory
        sensor_trajectory_x = np.array([50.0 + 25.0 * np.sin(s) for s in np.linspace(0, 2.0 * np.pi, 100)])
        sensor_trajectory_y = np.array([50.0 + 25.0 * np.cos(w) for w in np.linspace(0, 2.0 * np.pi, 100)])
        self.ax.plot(sensor_trajectory_x, sensor_trajectory_y, 'b--', label='Sensor trajectory')

    def spin_vis(self):
        # Set up a timer to call `update_sensor` every 100ms
        timer = self.fig.canvas.new_timer(interval=self.VIS_DT_MS)  # 100ms interval
        timer.add_callback(self.update_sensor)
        timer.start()

        # Connect the key press event handler
        # self.fig.canvas.mpl_connect('key_press_event', self.on_key_press)

        # Display the plot
        plt.show(block=False)
        plt.pause(0.1)
        print("Simulation figure started\n")

    # Define the function to update the target position on the plot
    def update_plot(self):
        # Update both target and sensor on the plot
        self.target_plot.set_data([self.target_position[0]], [self.target_position[1]])
        self.sensor_plot.set_data([self.sensor_position[0]], [self.sensor_position[1]])
        if not np.any(np.isnan(self.target_estimate)):
            self.estimate_plot.set_data([self.target_estimate[0]], [self.target_estimate[1]])
        # print(f"Sensor position: {self.sensor_position}, Target position: {self.target_position}, Time: {time.perf_counter() - self.start_time:.2f}")
        self.fig.canvas.draw_idle()
        plt.pause(0.01)

    def update_sensor(self):
        # Update the plot with the new sensor position
        self.update_plot()


class TargetEstimator:
    DT_SEC = 0.1
    def __init__(self, sim, socket):
        self.sim = sim
        self.socket = socket
        self.listen_thread = threading.Thread(target=self.listen).start()

    def listen(self):
        try:
            while True:
                target_msg = message_pb2.SensorMessage()
                target_msg.ParseFromString(self.socket.recv(1024))
                self.sim.target_estimate = np.array([target_msg.position.x, target_msg.position.y])
        except KeyboardInterrupt:
            print(f"Exiting {self.__class__.__name__}")

class BaseSensor:
    def __init__(self, sim, socket):
        self.sim = sim
        self.socket = socket
        self.measure_thread = threading.Thread(target=self.spin).start()
    
    def spin(self):
        try:
            while True:
                sensor_msg = message_pb2.SensorMessage()
                self.measure(sensor_msg)
                self.socket.sendall(sensor_msg.SerializeToString())
                time.sleep(self.DT_SEC)
        except KeyboardInterrupt:
            print(f"Exiting {self.__class__.__name__}")

    def measure(self, sensor_msg):
        raise NotImplementedError


class BearingSensor(BaseSensor):
    DT_SEC = 0.1
    DT_MS = int(DT_SEC * 1000)
    def __init__(self, sim, socket):
        super().__init__(sim, socket)
    
    def measure(self, sensor_msg):
        bearing_msg = message_pb2.BearingMessage()
        bearing_msg.timestamp.CopyFrom(get_timestamp())
        bearing_msg.bearing = self.sim.get_bearing_measurement()
        sensor_msg.bearing.CopyFrom(bearing_msg)


class PositionSensor(BaseSensor):
    DT_SEC = 0.1
    DT_MS = int(DT_SEC * 1000)
    def __init__(self, sim, socket):
        super().__init__(sim, socket)
    
    def measure(self, sensor_msg):
        position_msg = message_pb2.PositionMessage()
        position_msg.timestamp.CopyFrom(get_timestamp())
        position_msg.x = self.sim.sensor_position[0]
        position_msg.y = self.sim.sensor_position[1]
        sensor_msg.position.CopyFrom(position_msg)


def main():
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect(('127.0.0.1', 8080))
    try:
        sim = SimFigure()
        bearing_sensor = BearingSensor(sim, client_socket)
        position_sensor = PositionSensor(sim, client_socket)
        target_estimator = TargetEstimator(sim, client_socket)
        sim.spin_vis() # this needs to be called last to start the visualization which is blocking
    except KeyboardInterrupt:
        client_socket.close()
        print("Exited gracefully")


if __name__ == "__main__":
    main()
