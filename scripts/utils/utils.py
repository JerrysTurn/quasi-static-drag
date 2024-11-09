import numpy as np
import matplotlib.pyplot as plt

def is_circle_inside_rotated_rectangle(dragger, pullee):
    # objects info
    circle_x, circle_y, _ = dragger.q
    r = dragger.r
    rect_cx, rect_cy, rect_angle = pullee.q
    rect_width, rect_height = pullee.width, pullee.height
    
    # Calculate half dimensions of the rectangle
    half_width = rect_width / 2
    half_height = rect_height / 2

    # Define the rotation matrix based on the rectangle's orientation
    rotation_matrix = np.array([
        [np.cos(rect_angle), -np.sin(rect_angle)],
        [np.sin(rect_angle),  np.cos(rect_angle)]
    ])

    # Transform the circle center into the rotated rectangle's reference frame
    relative_position = np.array([circle_x, circle_y]) - np.array([rect_cx, rect_cy])
    rotated_circle_center = rotation_matrix.T @ relative_position

    # Calculate the new boundaries based on the rotated frame
    rotated_rect_x_min = -half_width
    rotated_rect_x_max = half_width
    rotated_rect_y_min = -half_height
    rotated_rect_y_max = half_height

    # Check if the rotated circle's center is within the rectangle boundaries in the rotated frame
    return (
        rotated_rect_x_min + r <= rotated_circle_center[0] <= rotated_rect_x_max - r and
        rotated_rect_y_min + r <= rotated_circle_center[1] <= rotated_rect_y_max - r
    )

def squareInfo2EqRadius(width, height):
        area = width * height
        inertia = (width * height**3) / 12 + (height * width**3) / 12
        eq_radius = np.sqrt(inertia / area)
        return eq_radius

def get_rotation(theta):
    rotation_matrix = np.array([[np.cos(theta), -np.sin(theta), 0],
                                [np.sin(theta),  np.cos(theta), 0],
                                [            0,              0, 1]])
    return rotation_matrix

def get_jacobian(x_r, y_r):
    
    jacobian_matrix = np.eye(3)
    jacobian_matrix[0, 2] = -y_r
    jacobian_matrix[1, 2] =  x_r

    return jacobian_matrix

def show_possible_velocity(velocity_candidate):
    # Extracting x, y, and z coordinates for the points
    X = velocity_candidate[:, 0]
    Y = velocity_candidate[:, 1]
    Z = velocity_candidate[:, 2]

    # Generate a list of colors using a colormap
    num_points = len(velocity_candidate)
    colors = plt.cm.viridis(np.linspace(0, 1, num_points))  # 'viridis' is an example colormap

    # Create a 3D plot
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')

    # Plot the points
    ax.scatter(X, Y, Z, color=colors, s=10)  # 's' is the size of the points

    # Set labels
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_zlabel('Z')

    # Set the limits of the plot
    ax.set_xlim([-0.3, 0.3])
    ax.set_ylim([-0.3, 0.3])
    ax.set_zlim([-0.3, 0.3])

    plt.title('3D Points in 3D Space')
    plt.show()