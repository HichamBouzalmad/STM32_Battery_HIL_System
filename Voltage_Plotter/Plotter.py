import serial
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import re
import time
try:
    ser = serial.Serial("COM8", 115200, timeout=1)
except:
    print("Could not open port. Is the board plugged in?")
    exit()
y_data = []
x_time = []

fig, ax = plt.subplots()
line, = ax.plot([], [], lw=2) 
ax.set_ylim(3.0, 4.5)       
ax.set_xlim(0, 100)
ax.grid(True)

def on_key_press(event):
    if event.key == 'c':
        ser.write(b'C')
    elif event.key == 'd':
        ser.write(b'D')
    elif event.key == 'n':
        ser.write(b'N')
    elif event.key == 'x':
        ser.write(b'X')
    elif event.key == 'e':
        ser.write(b'E')

def update(frame):
    while ser.in_waiting > 0:    
        try:
            data = ser.readline().decode('utf-8').strip()
            x = re.search(r"(\d+\.\d+)", data)
            
            if x :
                voltage = float(x.group(1))
                y_data.append(voltage)
                x_time.append(len(y_data))
            if len(y_data) > 100:
                y_data.pop(0)
                x_time.pop(0)

        except Exception as e:
            print(f"Error: {e}")

    line.set_data(range(len(y_data)), y_data)
    return line,
ani = animation.FuncAnimation(fig, update, interval=50)
fig.canvas.mpl_connect('key_press_event', on_key_press)
plt.show()
