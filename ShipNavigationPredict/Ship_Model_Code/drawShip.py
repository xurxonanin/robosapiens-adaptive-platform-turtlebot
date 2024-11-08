import tkinter as tk

def draw_ship(canvas):
    # Draw the body of the ship
    canvas.create_rectangle(50, 100, 150, 150, fill='blue', outline='black')
    
    # Draw the mast
    canvas.create_rectangle(90, 50, 110, 100, fill='brown', outline='black')
    
    # Draw the sail
    canvas.create_polygon(110, 50, 90, 80, 110, 80, fill='white', outline='black')
    
    # Draw the water
    canvas.create_rectangle(0, 150, 200, 200, fill='lightblue', outline='')

# Set up the main window
root = tk.Tk()
root.title("Ship Logo Example")
root.geometry("200x250")

# Create a canvas widget
canvas = tk.Canvas(root, width=200, height=200)
canvas.pack()

# Call the function to draw the ship
draw_ship(canvas)

# Run the Tkinter main loop
root.mainloop()
