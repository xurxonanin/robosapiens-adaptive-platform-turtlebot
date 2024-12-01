import tkinter as tk
from tkinter import filedialog, messagebox, ttk
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import ship_maneuvering_model
import HydroPara_PI3_alternative1 as HydroPara_PI3
import matplotlib.animation as animation
from matplotlib.patches import Rectangle
import matplotlib
import random
matplotlib.use('TkAgg')

import json


# List of available models
available_models = ["ShipModel_M1", "ShipModel_M2", "ShipModel_M7", "ShipModel_M12", "ShipModel_MS"]

#RA probe and effector 
from rpio.clientLibraries.rpclpy.node import Node
import time 
class Probe(Node):
    def __init__(self, config ,verbose=True):
        super().__init__(config=config,verbose=verbose)

        self._name = "probe"
        self.logger.info("Probe instantiated")
        self.plant = True

        #<!-- cc_init START--!>
        # user includes here
        #<!-- cc_init END--!>
    def effector(self, msg):
        self.plant  = False

        


    def register_callbacks(self):
        return super().register_event_callback(event_key="/model", callback= self.effector)
        

class Effector(Node):
    def __init__(self, config ,verbose=True, plant = True):
        super().__init__(config=config,verbose=verbose)

        self._name = "effector"
        self.logger.info("effector instantiated")
        self.plant  = plant 


        #<!-- cc_init START--!>
        # user includes here
        #<!-- cc_init END--!>
    def effector(self, msg):
        self.plant  = False
        


    def register_callbacks(self):
        return super().register_event_callback(event_key="/model", callback= self.effector)

class ShipSim:
    def __init__(self, root):
        self.predicttion_model = random.choice(available_models)
        self.predicttion_new_model = random.choice(available_models)
        self.model = getattr(ship_maneuvering_model, self.predicttion_model)()
        self.new_model = getattr(ship_maneuvering_model, self.predicttion_new_model)()
        self.root = root
        self.root.title("Ship Trajectory Dashboard")
        self.root.geometry("800x600")
        self.frame = ttk.Frame(self.root)
        self.frame.pack(fill=tk.BOTH, expand=True)
        self.setup_gui()
        self.data = np.array 
        self.eta  = np.array
        self.nu = np.array
        self.file_path = ''
        self.probe = Probe(config='config.yaml')
        self.probe.start()

    def setup_gui(self):
        animate_button = tk.Button(self.root, text="Animate Ship", command=self.animate_file)
        animate_button.pack(side = "left", padx=30, pady=5)

        select_button = tk.Button(self.root, text="New Model", command=self.predict_botton, justify= "left")
        select_button.pack(side = "left", padx=30, pady=5)

        anomaly_button = tk.Button(self.root, text="Anomaly Detection", command=self.anomaly_detection)
        anomaly_button.pack(side = "left", padx=30, pady=5)

        accept_model_button = tk.Button(self.root, text="Accept New Model", command=self.accept_new_model)
        accept_model_button.pack(side = "left", padx=30, pady=5)

        generate_model_button = tk.Button(self.root, text="Decline New Model", command=self.decline_new_model)
        generate_model_button.pack(side = "left", padx=30, pady=5)


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
    def new_predict_trajectory(self, df):
        eta, nu = self.new_model.predict(
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


    def animate_ship(self, data):
        x = data['x'].values
        y = data['y'].values
        heading = data['Heading'].values
        
        fig, ax = plt.subplots()
        ax.set_xlim(min(x), max(x))
        x_scale = max(x) - min(x)
        ax.set_ylim(min(y), max(y))
        y_scale = max(y) - min(y)
        ax.set_xlabel('x [m] - East')
        ax.set_ylabel('y [m] - North')

        def update(frame_num):
            ax.clear()
            ship_width, ship_length = int(x_scale/30), int(y_scale/30)

            ship = Rectangle((x[0] - ship_width / 2, y[0] - ship_length / 2), ship_width, ship_length, angle=heading[0], color='darkblue')
            ax.add_patch(ship)
            ship.set_xy([x[frame_num] - ship_width / 2, y[frame_num] - ship_length / 2])
            ship = heading[frame_num]
            # ax.set_title(f"File Path: {self.file_path} degrees")

            ax.plot(x, y, label='Current Trajectory')
            _data = self.data
            shifted_data = _data.shift(-frame_num)
            self.eta, self.nu = self.predict_trajectory(shifted_data)
            ax.plot(self.eta[:, 0], self.eta[:, 1], 'r--', label='Currecnt Prediction:'+ self.predicttion_model)
            
            self.probe.publish_event(event_key='/ship_status', message= json.dumps({'ship_prediction_model':self.predicttion_model, 'Surge Speed':shifted_data['Surge Speed'][0],
            'Sway Speed':shifted_data['Sway Speed'][0],
            'Yaw Rate': shifted_data['Yaw Rate'][0],
            'Heading': shifted_data['Heading'][0],
            'x': shifted_data['x'].to_list(),
            'y': shifted_data['y'].to_list()}))
            self.probe.publish_event(event_key='/weather_condition', message= json.dumps({'Rudder Angle': shifted_data['Rudder Angle'].tolist(),
            'Wind Direction': shifted_data['Wind Direction'].tolist(),
            'Wind Speed':shifted_data['Wind Speed'].tolist()}))  
            
            if self.predicttion_model != self.predicttion_new_model:
                _eta, _nu = self.new_predict_trajectory(shifted_data)
                ax.plot(_eta[:, 0], _eta[:, 1], 'g--', label='New Prediction:'+ self.predicttion_new_model)
                ax.set_title(self.predicttion_model+ "->"+self.predicttion_new_model)
            else: 
                ax.set_title(self.predicttion_model)
            fig.legend(loc = "lower left")
            fig.canvas.draw()
            return ship

        ani = animation.FuncAnimation(fig, update, frames=len(x), interval=1000, blit=False)

        # Embed the animation in the tkinter window
        for widget in self.frame.winfo_children():
            widget.destroy()
        canvas = FigureCanvasTkAgg(fig, master=self.frame)
        canvas.draw()
        canvas.get_tk_widget().pack()

        # Keep a reference to the animation to prevent it from being garbage collected
        self.frame.ani = ani

    def predict_botton(self):

        try:
            # df = self.load_data(self.file_path, self.model.U0)
            # messagebox.showinfo("go to ->"+ self.predicttion_new_model)
            self.predicttion_new_model= random.choice(available_models)
            self.new_model = getattr(ship_maneuvering_model, self.predicttion_new_model)()    
            
        #     # self.make_plots(self.data, eta, nu)
        except Exception as e:
            messagebox.showerror("Error", f"An error occurred: {e}")


    def anomaly_detection(self):
        messagebox.showinfo("Anomaly Detection", "Anomaly detection functionality is not yet implemented.")

    def accept_new_model(self):
        self.predicttion_model = self.predicttion_new_model
        self.model = getattr(ship_maneuvering_model, self.predicttion_model)()  
        # messagebox.showinfo("Accept New Model", "Accepting the new model functionality is not yet implemented.")

    def decline_new_model(self):
        self.predicttion_new_model = self.predicttion_model

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

