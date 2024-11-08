import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from ship_maneuvering_model import ShipModel
import HydroPara_PI3_alternative1 as HydroPara_PI3
import matplotlib.animation as animation
from matplotlib.patches import Rectangle

class ShipSim:
    def __init__(self, root):
        self.model = ShipModel()
        self.root = root
        self.root.title("Ship Trajectory Dashboard")
        self.root.geometry("800x600")
        self.frame = ttk.Frame(self.root)
        self.frame.pack(fill=tk.BOTH, expand=True)
        self.setup_gui()
        self.data = np.array 
        self.eta  = np.array
        self.nu = np.array
        self.predict = False
        self.file_path = ''

    def setup_gui(self):
        animate_button = tk.Button(self.root, text="Animate Ship", command=self.animate_file)
        animate_button.pack(pady=10)

        select_button = tk.Button(self.root, text="Predict", command=self.predict)
        select_button.pack(pady=10)

        anomaly_button = tk.Button(self.root, text="Anomaly Detection", command=self.anomaly_detection)
        anomaly_button.pack(pady=5)

        accept_model_button = tk.Button(self.root, text="Accept New Model", command=self.accept_new_model)
        accept_model_button.pack(pady=5)

        generate_model_button = tk.Button(self.root, text="Generate New Model", command=self.generate_new_model)
        generate_model_button.pack(pady=5)


    def load_data(self, file, U0):
        df = pd.read_csv(file)
        df['Surge Speed'] = df['Surge Speed'] - U0
        return df

    def predict_trajectory(self, df):
        eta, nu = self.model.predict(
            HydroPara_PI3,
            df['Surge Speed'][0],
            df['Sway Speed'][0],
            df['Yaw Rate'][0],
            df['Heading'][0],
            df['x'][0],
            df['y'][0],
            df['Rudder Angle'],
            df['Wind Direction'],
            df['Wind Speed']
        )
        return eta, nu

    def make_plots(self, data, eta, nu):
        u = data['Surge Speed']
        v = data['Sway Speed']
        r = data['Yaw Rate']
        x = data['x']
        y = data['y']

        time_array = np.arange(0, len(nu[:, 0]))

        # Create figure for plotting
        fig = plt.figure(figsize=(10, 6))
        plot_pos = plt.subplot2grid((3, 6), (0, 0), rowspan=3, colspan=3)
        plot_u = plt.subplot2grid((3, 6), (0, 3), rowspan=1, colspan=3)
        plot_v = plt.subplot2grid((3, 6), (1, 3), rowspan=1, colspan=3)
        plot_r = plt.subplot2grid((3, 6), (2, 3), rowspan=1, colspan=3)

        # Plotting Surge Speed (u)
        plot_u.plot(time_array, nu[:, 0], label="predicted")
        plot_u.plot(time_array, u, label="true values")
        plot_u.set_xlabel('Time [s]')
        plot_u.set_ylabel('u [m/s]')
        plot_u.grid()
        plot_u.legend()

        # Plotting Sway Speed (v)
        plot_v.plot(time_array, nu[:, 1], label="predicted")
        plot_v.plot(time_array, v, label="true values")
        plot_v.set_xlabel('Time [s]')
        plot_v.set_ylabel('v [m/s]')
        plot_v.grid()
        plot_v.legend()

        # Plotting Yaw Rate (r)
        plot_r.plot(time_array, nu[:, 2], label="predicted")
        plot_r.plot(time_array, r, label="true values")
        plot_r.set_xlabel('Time [s]')
        plot_r.set_ylabel('r [rad/s]')
        plot_r.grid()
        plot_r.legend()

        # Plotting Position (x, y)
        plot_pos.plot(eta[:, 1], eta[:, 0], label="predicted")
        plot_pos.plot(y, x, label="true values")
        plot_pos.set_xlabel('y [m] - East')
        plot_pos.set_ylabel('x [m] - North')
        plot_pos.legend()
        plot_pos.grid()

        plt.tight_layout()

        # Embed the plot in the tkinter window
        for widget in self.frame.winfo_children():
            widget.destroy()
        canvas = FigureCanvasTkAgg(fig, master=self.frame)
        canvas.draw()
        canvas.get_tk_widget().pack()

    def animate_ship(self, data):
        x = data['x'].values
        y = data['y'].values
        heading = data['Heading'].values
        
        fig, ax = plt.subplots()
        ax.set_xlim(min(x) - 30, max(x) + 30)
        ax.set_ylim(min(y) - 30, max(y) + 30)
        ax.set_xlabel('x [m] - East')
        ax.set_ylabel('y [m] - North')

        def update(frame_num):
            ax.clear()
            ship_width, ship_length = 120, 10

            ship = Rectangle((x[0] - ship_width / 2, y[0] - ship_length / 2), ship_width, ship_length, angle=heading[0], color='darkblue')
            ax.add_patch(ship)
            ship.set_xy((x[frame_num] - ship_width / 2, y[frame_num] - ship_length / 2))
            ship = heading[frame_num]
            ax.set_title(f"File Path: {self.file_path} degrees")
            ax.plot(x, y, 'g--', label='Trajectory')
            if self.predict:
                data = self.data
                shifted_data = data.shift(-frame_num)
                self.eta, self.nu = self.predict_trajectory(shifted_data)
                ax.plot(self.eta[:, 0], self.eta[:, 1], 'r--', label='Predicted Trajectory')

            fig.canvas.draw()
            return ship

        ani = animation.FuncAnimation(fig, update, frames=len(x), interval=200, blit=False)

        # Embed the animation in the tkinter window
        for widget in self.frame.winfo_children():
            widget.destroy()
        canvas = FigureCanvasTkAgg(fig, master=self.frame)
        canvas.draw()
        canvas.get_tk_widget().pack()

        # Keep a reference to the animation to prevent it from being garbage collected
        self.frame.ani = ani

    def predict(self):
            if self.predict == False:
                try:
                    # df = self.load_data(self.file_path, self.model.U0)
                    self.predict = True
                    
                    # self.make_plots(self.data, eta, nu)
                except Exception as e:
                    messagebox.showerror("Error", f"An error occurred: {e}")
            else:
                self.predict = False

    def anomaly_detection(self):
        messagebox.showinfo("Anomaly Detection", "Anomaly detection functionality is not yet implemented.")

    def accept_new_model(self):
        messagebox.showinfo("Accept New Model", "Accepting the new model functionality is not yet implemented.")

    def generate_new_model(self):
        messagebox.showinfo("Generate New Model", "Generating a new model functionality is not yet implemented.")

    def animate_file(self):
        self.file_path = filedialog.askopenfilename()
        if self.file_path:
            try:
                self.data = self.load_data(self.file_path, self.model.U0)
                self.animate_ship(self.data)
            except Exception as e:
                messagebox.showerror("Error", f"An error occurred: {e}")


if __name__ == "__main__":
    root = tk.Tk()
    app = ShipSim(root)
    root.mainloop()

