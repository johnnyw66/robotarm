import matplotlib.pyplot as plt
import numpy as np


# Function to compute the two arm angles from a given an (x, y) position
def inverse_kinematics(x, y, L):
 
    #print("Point x=",x,"y=",y)
    r = np.sqrt(x**2 + y**2)

    if (r > 2*L):
        raise ValueError("Too Big")

    full_angle = np.atan2(y,x)
    
    hbase = r*0.5
    theta2 = np.pi - 2*np.asin(hbase/L)  # Angle of 2nd Arm - based on isosceles triangle

    # alpha is the angle of the isosceles triangle - from one its common basepoints to the apex point
    alpha = theta2/2.0 #(np.pi - (np.pi-theta2))/2
    #print("2 Common angles",alpha)

    # alpha is effective rotation of our base line -done by rotating 2nd arm
    # by moving the 2nd arm we have completed rotating our base line by alpha
    # we now need to work out the difference from the full angle to rotate our
    # first arm section by.

    theta1 = full_angle - alpha

    return theta1, theta2

# Function to compute the end positions of the arm sections given the angles
def compute_positions(L, theta1, theta2):
    # End of the first section
    x1 = L * np.cos(theta1)
    y1 = L * np.sin(theta1)
    
    # End of the second section
    x2 = x1 + L * np.cos(theta1 + theta2)
    y2 = y1 + L * np.sin(theta1 + theta2)
    
    return (x1, y1), (x2, y2)



def generate_line(start, end, num_points):
    x_start, y_start = start
    x_end, y_end = end
    return [
        (
            x_start + t * (x_end - x_start),
            y_start + t * (y_end - y_start)
        )
        for t in [i / (num_points - 1) for i in range(num_points)]
    ]

def generate_circle(start, radius, num_points):
    x_start, y_start = start
    return [
        (
            x_start + radius * np.cos(2 * np.pi * i / num_points),
            y_start + radius * np.sin(2 * np.pi * i / num_points)
        )
        for i in range(num_points)
    ]



# Length of each section
L = 1.0

# Define the path as a series of (x, y) coordinates
#path = generate_line((0.5, 0.5),(1.5, 1.5), 10)
path = generate_circle((0.5, 0.5),1.2, 50)
#path = generate_line((0.5, 0.5),(1.2, 1.2), 50)
#path=[(1,1)]

# Initialize lists to hold the positions of the joints
joint1_positions = []
end_effector_positions = []

# Compute the joint angles and positions for each point in the path
for (x, y) in path:
    theta1, theta2 = inverse_kinematics(x, y, L)
    joint1, end_effector = compute_positions(L, theta1, theta2)
    cx,cy = end_effector
    errx, erry = np.abs(cx - x), np.abs(cy - y)
    if (errx + erry > 0.0001):
        print("TOO BIG AN ERROR")     
    joint1_positions.append(joint1)
    end_effector_positions.append(end_effector)

# Plotting the path and the robotic arm's positions
fig, ax = plt.subplots()

# Plot the path
path_x, path_y = zip(*path)
ax.plot(path_x, path_y, 'k--', label="Desired Path")

# Plot the robotic arm's positions
for i in range(len(joint1_positions)):
    joint1 = joint1_positions[i]
    end_effector = end_effector_positions[i]
    
    # Plot the first section
    ax.plot([0, joint1[0]], [0, joint1[1]], 'b-')
    
    # Plot the second section
    ax.plot([joint1[0], end_effector[0]], [joint1[1], end_effector[1]], 'r-')
    ax.plot([end_effector[0]], [end_effector[1]], 'go', markersize=2)

# Annotate the final position
ax.annotate('positions', end_effector_positions[-1], textcoords="offset points", xytext=(0,10), ha='center')

# Set plot limits and labels
ax.set_xlim(-L*2, L*2)
ax.set_ylim(-L*2, L*2)
ax.set_aspect('equal', adjustable='box')
ax.set_xlabel('x')
ax.set_ylabel('y')
ax.legend()
ax.grid(True)

plt.title("2D Robotic Arm Drawing Path")
plt.show()


