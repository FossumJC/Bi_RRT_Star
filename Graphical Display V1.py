import numpy as np
from matplotlib.patches import Circle, Wedge
from matplotlib.collections import PatchCollection
import matplotlib.pyplot as plt


def sample(obstacles: list, iterations: int):
    """
    Builds a list of samples that are i.i.d. and obstacle free in the search space
    :param obstacles: The list of patches that was generated for the space
    :param iterations: The number of free points wished to be found in the search space
    :return: A list of i.i.d. tuples that are found in the free space
    """
    samples = []  # Create an empty list
    current = 0  # Set current number of iterations to zero
    while current < iterations:
        x = np.random.rand(iterations-current)  # Creates an array of random x values
        y = np.random.rand(iterations-current)  # Creates an array of random y values
        for x1, y1 in zip(x, y):  # Loops through given arrays using the zip function
            if in_obstacle_detection((x1, y1), obstacles):  # Check if the current random point is in an obstacle
                pass  # Do nothing if TRUE
            else:
                samples.append((x1, y1))  # Add the valid point to the list of samples
                current = len(samples)  # Set the current number of iterations to the number of samples in the list

    return samples


def steer(start: tuple, goal: tuple, d: float) -> tuple:
    """
    Return a point in the direction of the goal, that is 'd' distance away from start
    :param start: start location
    :param goal: goal location
    :param d: distance away from start
    :return: point in the direction of the goal, distance away from start
    """
    start, end = np.array(start), np.array(goal)
    v = end - start
    u = v / (np.sqrt(np.sum(v ** 2)))
    steered_point = start + u * d
    return tuple(steered_point)


def dist_between_points(a: tuple, b: tuple) -> float:
    """
    Return the Euclidean distance between two points
    :param a: First point in the form of a tuple written (x,y)
    :param b: Second point in the form of a tuple written (x,y)
    :return: Euclidean distance between point a and point b
    """
    distance = np.linalg.norm(np.array(b) - np.array(a))
    return distance


def points_along_line(start: tuple, end: tuple, r: float):
    """
    Equally-spaced points along a line defined by start, end, with resolution r
    :param start: Starting point in the form of a tuple written (x,y)
    :param end: Ending point in the form of a tuple written (x,y)
    :param r: Maximum distance between points
    :return: List of points along line from start to end, separated by distance r
    """
    d = dist_between_points(start, end)
    n_points = int(np.ceil(d / r))
    points = []
    if n_points > 1:
        step = d / (n_points - 1)
        for i in range(n_points):
            next_point = steer(start, end, i * step)
            points.append(next_point)
    return points


def obstacle_free(obstacles: list, points_on_line: list):
    """

    :param obstacles: The list of patches that was generated for the space
    :param points_on_line: The list of points that were created by the 'points_along_line' function
    :return: True if the line is obstacle free. False if one or more points are in an obstacle
    """
    number_of_points = len(points_on_line)
    counter = 0
    for point in points_on_line:
        if in_obstacle_detection(point, obstacles):
            return False
        else:
            counter += 1
    if counter == number_of_points:
        return True
    else:
        return False


def nearest(graph, point):
    """
    Check the distance between a given point in the search space and find the closest vertex in terms of Euclidean
    distance. Do this by calculating the vector created between the two points (given point and vertex point(s))
    and minimize that vector by switching out other vertices,
    :return: A single vertex.
    """
    pass


def near(graph, point, magnitude):
    """
    Find all of the vertices that are within a closed ball (3D) or closed circle (2D) with a given radius centered at x.
    :return: A list of vertices.
    """
    pass


def cost(point):
    """
    This function calculates the unique cost of a path from the Start position to any given vertex.
    :return: A cost value (numerical number) in this case it would be the sum of the Euclidean distance between the
    Start position to the end vertex through all the parent vertices.
    """
    pass


def bi_rrt_star_body(start: tuple, goal: tuple, iterations: int, obstacles: list):
    """

    :param start: The desired start position as a tuple written (x,y)
    :param goal: The desired goal position as a tuple written (x,y)
    :param iterations: The number of free points wished to be found in the search space
    :param obstacles: The list of obstacles created with the 'create_obstacles' function
    :return: Graph-prime object that contains the Start vertex as root and other extended vertices and edges identified
    separately from the originals as vertices-prime and edges-prime respectively.
    """
    # vertex = [start]
    # edges = []
    # index = 0
    # while index < iterations:
    #     graph/tree = (vertex,edges)
    #     point_sample = sample(create_obstacles(patches, 5), index)
    #     index += 1
    #     (vertex, edges) = bi_rrt_star_extend(graph/tree, point_sample)
    pass


def bi_rrt_star_extend(thing, point_sample, obstacles: list):
    """

    :param thing: Graph/tree object that is currently being expanded.
    :param point_sample: The random sampled point to give a heading to our expansion direction.
    :param obstacles: The list of obstacles created with the 'create_obstacles' function
    :return: Graph/tree object prime
    """
    # vertex_prime, edge_prime = thing
    # vertex_nearest = nearest(thing, point_sample)
    # vertex_new = steer(vertex_nearest, point_sample)
    # if obstacle_free(points_along_line(vertex_nearest, vertex_new)):
    #     vertex_prime.append(vertex_new)
    #     vertex_min = vertex_nearest
    #     vertices_near = near(thing, vertex_new, vertex_magnitude)
    #     for vertex_near in vertices_near:
    #         if obstacle_free(obstacles, points_along_line(vertex_near, vertex_new)):
    #             cost_prime = cost(vertex_near) + c(dist_between_points(vertex_near, vertex_new))
    #             if cost_prime < cost(vertex_new):
    #                 vertex_min = vertex_near
    #     edge_prime.append([vertex_min, vertex_new])
    #     for vertex_near in vertices_near.remove(vertex_min):
    #         if (obstacle_free(obstacles, points_along_line(vertex_new, vertex_near))) and
    #             (cost(vertex_near) > cost(vertex_new) + c(dist_between_points(vertex_new,vertex_near))):
    #             vertext_parent = parent(vertex_near)
    #             edge_prime = edge_prime.remove([vertext_parent, vertex_near])
    #             edge_prime = edge_prime.append([vertex_new, vertex_near])
    #
    # return thing_prime = (vertex_prim, edge_prime)
    pass


def swap():
    """

    :return: The correct graph/tree object that is currently being expanded. It would change from the Start direction to
    the Goal direction every other vertex/edge expansion.
    """
    pass


def create_obstacles(patches: list, n: int = 5):
    """
    Creates three predefined obstacles, a full circle, a full sector, and a ring sector plus n*2 more random
    obstacles.
    :param patches: The current list of obstacles. The list typically starts empty when getting passed,
    but this could be called again to create more random obstacles
    :param n: The number of random obstacles to be generated divided by two
    :return: A filled list of obstacles that can be placed inside 'matplotlib.collections.PatchCollection' function
    """
    # Fixing random state for reproducibility
    np.random.seed(56589)

    patches += [
        Wedge((.3, .7), .1, 0, 360),             # Full circle
        Wedge((.8, .3), .2, 0, 45),              # Full sector
        Wedge((.8, .3), .2, 45, 90, width=0.10),  # Ring sector
    ]

    x = np.random.rand(n)  # Creates an array of random x values
    y = np.random.rand(n)  # Creates an array of random y values
    radii = 0.1*np.random.rand(n)   # Creates an array of random radii values
    for x1, y1, r in zip(x, y, radii):  # Loops through given arrays that have been parsed/paired with the zip function
        circle = Circle((x1, y1), r)  # Creates a circle patch
        patches.append(circle)  # Adds the newly created patch to the list of patches

    x = np.random.rand(n)  # Creates an array of random x values
    y = np.random.rand(n)  # Creates an array of random y values
    radii = 0.1*np.random.rand(n)   # Creates an array of random radii values
    theta1 = 360.0*np.random.rand(n)   # Creates an array of random theta values
    theta2 = 360.0*np.random.rand(n)   # Creates an array of random theta values
    for x1, y1, r, t1, t2 in zip(x, y, radii, theta1, theta2):  # Loops through given arrays using the zip function
        wedge = Wedge((x1, y1), r, t1, t2)  # Creates a wedge patch
        patches.append(wedge)  # Adds the newly created patch to the list of patches

    return patches


def in_obstacle_detection(point: tuple, obstacles: list):
    """
    Detects if a randomly generated point is within any of the created obstacles.
    :param point: A point in the space in the form of a tuple written (x,y)
    :param obstacles: The list of patches that was generated for the space
    :return: Returns if the point lays within any of the obstacles in the list as True or False
    """
    # point = (0.869, 0.428)  # Test point to verify functionality
    count = 0  # Counter to see if it was iterating through the obstacles correctly
    within = False
    for patch in obstacles:
        count += 1
        if patch.contains_point(point):  # Check if the point is within the current obstacle
            within = True
            break  # Exit for loop as soon as the value it True
    # print(str(within) + " " + str(count))  # Debug print statement
    return within


def plot_search_window(obstacles: list, start: tuple = (0.5, 0.5), goal: tuple = (0.125, 0.05)):
    """
    Plots the search window that the Bi-directional RRT* algorithm will search showing the obstacles, start position,
    and the goal position.
    :param obstacles: The list of obstacles created with the 'create_obstacles' function
    :param start: The desired start position as a tuple written (x,y)
    :param goal: The desired goal position as a tuple written (x,y)
    :return: None. Immediate call to plot
    """
    fig, ax = plt.subplots()
    p = PatchCollection(obstacles, facecolors="black", edgecolors="black")
    ax.add_collection(p)
    ax.scatter(start[0], start[1], color='green', marker='o')
    ax.scatter(goal[0], goal[1], color='red', marker='o')
    ax.set_xlim(0, 1)
    ax.set_ylim(0, 1)
    ax.set_box_aspect(1)
    ax.xaxis.set_ticks([])
    ax.yaxis.set_ticks([])
    plt.show()


# if __name__ == "__main__":
#     assert sys.version_info[0] == 3                                  # require python 3
patches = []
Start = (0.5, 0.5)
Goal = (0.125, 0.05)
# plot_search_window(create_obstacles(patches, 5), Start, Goal)


fig, ax = plt.subplots()
p = PatchCollection(create_obstacles(patches, 5), facecolors="black", edgecolors="black")
ax.add_collection(p)
ax.scatter(Start[0], Start[1], color='green', marker='o')
ax.scatter(Goal[0], Goal[1], color='red', marker='o')
Sample = sample(create_obstacles(patches, 5), 1000)
Test = points_along_line(Start, Goal, 0.025)
ax.scatter(*zip(*Sample), color='blue', marker='.')
# ax.scatter(*zip(*Test), color='cyan', marker='.')
ax.set_xlim(0, 1)
ax.set_ylim(0, 1)
ax.set_box_aspect(1)
ax.xaxis.set_ticks([])
ax.yaxis.set_ticks([])
plt.show()

