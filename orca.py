import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial.distance import pdist
from matplotlib.animation import FuncAnimation
import random

EPSILON = 1e-9

def project_point_onto_line(p, A, B):
    # Points A and B on the line y = mx + b
    A = np.array(A)
    B = np.array(B)

    # Point to be projected
    P1 = np.array(p)

    # Calculating the projection of P1 onto the line
    P2 = A + np.dot(B - A, P1 - A) / np.dot(B - A, B - A) * (B - A)
    return P2

# Get velocity obstacle
def get_vo(A, B, delta):
    center = (B.pos - A.pos)/delta
    u = center[0]
    v = center[1]
    r = (A.r + B.r)/delta

    tangent_dist2 = u**2+v**2-r**2
    # This doesn't have to be an epsilon check, as tangent_dist2 = 0 is valid
    if tangent_dist2 < 0:
        #r = np.linalg.norm(center)
        #tangent_dist2 = 0
        return None
    tangent_dist = math.sqrt(tangent_dist2)

    # Get the theta between a tangent line and the center
    center_dist = np.linalg.norm(center)
    theta = np.arcsin(r/center_dist)
    
    # Compute rotation matrices
    R_anticlockwise = np.array([[np.cos(theta), -np.sin(theta)],
                                [np.sin(theta), np.cos(theta)]])
    R_clockwise = np.array([[np.cos(-theta), -np.sin(-theta)],
                            [np.sin(-theta), np.cos(-theta)]])

    # Rotate the center by +/- theta to get the tangent vectors
    tangent1_unit = np.dot(R_anticlockwise, center) / center_dist
    tangent2_unit = np.dot(R_clockwise, center) / center_dist

    # Return VO_{A|B}^\delta
    return (center, r, tangent1_unit, tangent2_unit, tangent_dist)

def get_boundary(vo, p):
    center, r, tangent1_unit, tangent2_unit, tangent_dist = vo
    # NOTE: tangent{1,2} can be vanishingly small,
    #       so we have to use tangent{1,2}_unit
    tangent1 = tangent1_unit * tangent_dist
    tangent2 = tangent2_unit * tangent_dist
    # Project p onto the tangent boundary, and get the norm associated with p
    p1 = np.dot(p, tangent1_unit) * tangent1_unit
    l1norm = (tangent1 - center)
    l1norm /= np.linalg.norm(l1norm)
    p2 = np.dot(p, tangent2_unit) * tangent2_unit
    l2norm = (tangent2 - center)
    l2norm /= np.linalg.norm(l2norm)
    # Invalidate p if it isn't on the correct side (beyond) the tangent point
    if np.dot(p1 - tangent1, tangent1_unit) < 0:
        p1 = None
    if np.dot(p2 - tangent2, tangent2_unit) < 0:
        p2 = None
    # Set proj to the closer between p1 and p2
    proj = None
    if p1 is not None and p2 is not None:
        proj = (p1, l1norm) if np.linalg.norm(p1-p) < np.linalg.norm(p2-p) else (p2, l2norm)
    elif p1 is not None or p2 is not None:
        proj = (p1, l1norm) if p1 is not None else (p2, l2norm)
    # If there's a valid proj, use the proj
    if proj is not None:
        ret = proj
    else:
        # Else, project onto the circle
        norm = (p - center)
        if np.linalg.norm(norm) < EPSILON:
            norm = np.array([0, 0]) - center
        norm /= np.linalg.norm(norm)
        boundary = center + norm * r
        ret = (boundary, norm)
    return ret


# Get the orca_{A|B}^\delta
def get_orca(A, B, delta):
    vo = get_vo(A, B, delta)
    if vo is None:
        print("NO VO")
        return None
    p = A.vel - B.vel
    boundary, norm = get_boundary(vo, p)
    # NOTE: u can be vanishingly small, we have to use "norm" for the halfplane
    u = boundary - p
    line_point = A.vel + u/2
    return (line_point, norm)

def linear_programming_1d(halfplanes, new_halfplane, radius, pref, result):
    H_p, H_n = new_halfplane
    H_dir = np.dot(np.array([[0, -1], [1, 0]]), H_n)

    # If result is already on the goodside of the new halfplane, nothing to do
    if np.dot(result - H_p, H_n) >= 0:
        return result
    else:
        # Parametrize H as {H_p + t*H_dir}
        t_left = float('-inf')
        t_right = float('inf')

        # Find the two values of t such that ||H_p + t H_dir|| = r
        # (H_p + t H_dir) . (H_p + t H_dir) = r**2 is a quadratic in t
        A = np.dot(H_dir, H_dir)
        B = 2*np.dot(H_p, H_dir)
        C = np.dot(H_p, H_p) - radius**2
        disc = B**2 - 4*A*C
        if disc < 0:
            return None
        t_left = (-B-math.sqrt(disc))/(2*A)
        t_right = (-B+math.sqrt(disc))/(2*A)
        
        # Now, we clamp [t_left, t_right] based on the rest of the half planes
        for halfplane in halfplanes:
            J_p, J_n = halfplane
            # Formula for t_i
            t_i_numerator = np.dot(J_n, J_p - H_p)
            t_i_denominator = np.dot(J_n, H_dir)
            # If t_i is too large, then either [t_left, t_right] is entirely in range,
            # or not in range at all
            if abs(t_i_denominator) < EPSILON:
                seg = H_p + t_left * H_dir
                # If J_n and seg-J_p point in the same direction,
                if np.dot(J_n, seg - J_p) >= 0:
                    continue
                else:
                    # If they don't, it's pointing the wrong way
                    return None
            t_i = t_i_numerator / t_i_denominator
            # Bound t_i*H_dir based on whether or not J_n and H_dir go in the same direction
            if np.dot(J_n, H_dir) > 0:
                t_left = max(t_left, t_i)
            if np.dot(J_n, H_dir) < 0:
                t_right = min(t_right, t_i)

    # Calculate preferred t
    pref_t = np.dot(pref - H_p, H_dir) # / np.linalg.norm(H_dir)**2

    # Clamp based on the existent constraints
    if t_left > t_right:
        return None
    pref_t = min(max(pref_t, t_left), t_right)

    # Return the new result
    return H_p + pref_t * H_dir

def linear_programming_2d(halfplanes, radius, pref):
    # Linear Programming
    processed_halfplanes = []
    result = pref
    for new_halfplane in halfplanes:
        result = linear_programming_1d(processed_halfplanes, new_halfplane, radius, pref, result)
        if result is None:
            print("NONE!")
            return None
        processed_halfplanes.append(new_halfplane)
    # Verify Validity
    for halfplane in halfplanes:
        H_p, H_n = halfplane
        if np.dot(result - H_p, H_n) < 0:
            print("INVALID", np.dot(result - H_p, H_n), halfplane)
    return result

FPS = 60

class Agent:
    def __init__(self, pos, target, radius, max_speed, last_vel=None):
        self.pos = np.array(pos)
        self.target = np.array(target)
        self.max_speed = max_speed
        if last_vel is None:
            last_vel = (self.target - self.pos)
            last_vel /= np.linalg.norm(last_vel)
            last_vel *= self.max_speed
        self.vel = np.array(last_vel)
        self.r = float(radius)

    def constrain(self, agents, delta, added_halfplanes=[]):
        # Calculate the target velocity
        pref_vel = (self.target - self.pos)
        if np.linalg.norm(pref_vel) > 0:
            pref_vel /= np.linalg.norm(pref_vel)
            pref_vel *= min(self.max_speed, np.linalg.norm(self.target - self.pos)/delta)
        
        # Get the halfplanes from all of the agents
        halfplanes = []
        for agent in agents:
            halfplane = get_orca(self, agent, delta)
            halfplanes.append(halfplane)

        # Find the next velocity with Linear Programming
        self.next_vel = linear_programming_2d(halfplanes+added_halfplanes, self.max_speed, pref_vel)

    def move(self, delta):
        self.vel = self.next_vel
        self.pos += self.vel * delta

alice = Agent([30.0, 50.0], [-5.0, 90.0], 8.0, 20.0)
bob = Agent([10.0, 60.0], [20.0, 85.0], 8.0, 20.0)
charlie = Agent([-10.0, 50.0], [40.0, 85.0], 8.0, 20.0)
agents = [alice, bob, charlie]

def generate_n_circle(n, radius):
    agents = []
    offset = np.array([50, 50])
    for i in range(n):
        theta = 2*math.pi*i/n
        R_anticlockwise = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        pos = np.dot(R_anticlockwise, np.array([0, radius]))
        pos += np.array([random.random()*1e-1, random.random()*1e-1])
        agents.append(Agent(offset+pos, offset+[-pos[0], -pos[1]], radius/4, radius/n))
    return agents

#agents = generate_n_circle(10, 50)

fig, ax = plt.subplots()

# Setting the limits for the axes
ax.set_xlim(-20, 100)
ax.set_ylim(-20, 120)

circles = []
for agent in agents:
    circles.append(Circle((agent.pos[0], agent.pos[1]), agent.r, alpha=0.5))
    ax.add_patch(circles[-1])

# Create a container for updated arrows
arrows = []

# Animation update function
print("DIST:", np.min(pdist([agent.pos for agent in agents])))
def update(num):
    if len(arrows) > 0:
        pass#return
    delta = 1 / FPS
    
    # Calculate ORCA over the agents
    print("=======================")
    print("CURRENT DIST:", np.min(pdist([agent.pos for agent in agents])))
    for agent in agents:
        print("Agent:", repr(agent.pos), " | ", repr(agent.vel))
        other_agents = [a for a in agents if a is not agent]
        agent.constrain(other_agents, delta)
        print("New Velocity:", repr(agent.next_vel))
        print("-----------")

    # Move the agents
    for agent in agents:
        #ax.add_patch(Circle((agent.pos[0], agent.pos[1]), agent.r, alpha=0.5))
        agent.move(delta)
    print("CURRENT DIST:", np.min(pdist([agent.pos for agent in agents])))
    
    # Remove old arrows before drawing new ones
    for arrow in arrows:
        arrow.remove()
    arrows[:] = []
    # Move the agents, and render their new locations
    for agent, circle in zip(agents, circles):
        circle.center = agent.pos[0], agent.pos[1]
        arrows.append(ax.arrow(agent.pos[0], agent.pos[1], agent.vel[0], agent.vel[1], head_width=3.0, head_length=5.0, fc='k', ec='k'))

# Creating animation
ani = FuncAnimation(fig, update, frames=range(100), interval=1000/FPS, blit=False)

plt.grid(True)
plt.show()
