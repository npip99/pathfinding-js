import numpy as np
import math
import matplotlib.pyplot as plt
from matplotlib.patches import Circle
from scipy.spatial.distance import pdist
from matplotlib.animation import FuncAnimation
import random

EPSILON = 1e-9

# Get the particular t s.t. v_p + t v_dir intersects the given halfplane
def get_t(halfplane, v_p, v_dir):
    H_p, H_n = halfplane
    # Formula for t_i
    t_i_numerator = np.dot(H_n, H_p - v_p)
    t_i_denominator = np.dot(H_n, v_dir)
    # If t_i is too large, then either [t_left, t_right] is entirely in range,
    # or not in range at all
    if abs(t_i_denominator) < EPSILON:
        # If J_n and H_p-J_p point in the same direction,
        if -t_i_numerator >= 0:
            return (float('-inf'), float('inf'))
        else:
            # If they don't, it's pointing the wrong way
            return None
    t_i = t_i_numerator / t_i_denominator
    # Bound t-range based on whether or not J_n and H_dir go in the same direction
    if t_i_denominator > 0:
        return (t_i, float('inf'))
    if t_i_denominator < 0:
        return (float('-inf'), t_i)


# Get velocity obstacle
def get_vo(A, B, delta):
    center = (B.pos - A.pos)/delta
    u = center[0]
    v = center[1]
    r = (A.r + B.r)/delta

    tangent_dist2 = u**2+v**2-r**2
    # This doesn't have to be an epsilon check, as tangent_dist2 = 0 is valid
    if tangent_dist2 < 0:
        return (center, r, None, None, None)
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
    proj = None
    # If there are no tangent lines, just project onto the circle
    if tangent_dist is not None:
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
        if p1 is not None and p2 is not None:
            proj = (p1, l1norm) if np.linalg.norm(p1-p) < np.linalg.norm(p2-p) else (p2, l2norm)
        elif p1 is not None or p2 is not None:
            proj = (p1, l1norm) if p1 is not None else (p2, l2norm)
        else:
            proj = None
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
    p = A.vel - B.vel
    boundary, norm = get_boundary(vo, p)
    # NOTE: u can be vanishingly small, we have to use "norm" for the halfplane
    u = boundary - p
    line_point = A.vel + u/2
    return (line_point, norm)

def linear_programming_1(halfplanes, new_halfplane, pref, radius, result, use_direction=False):
    H_p, H_n = new_halfplane
    H_dir = np.dot(np.array([[0, -1], [1, 0]]), H_n)

    # If using direction, pick the point at infinity
    if use_direction:
        pref = 1000*pref

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
            t_range = get_t(halfplane, H_p, H_dir)
            t_left = max(t_left, t_range[0])
            t_right = min(t_right, t_range[1])
            if t_left > t_right:
                return None

    # Calculate preferred t
    pref_t = np.dot(pref - H_p, H_dir) # / np.linalg.norm(H_dir)**2

    # Clamp based on the existent constraints
    pref_t = min(max(pref_t, t_left), t_right)

    # Return the new result
    return H_p + pref_t * H_dir

def linear_programming_2(halfplanes, pref, radius, use_direction=False):
    # Linear Programming
    processed_halfplanes = []
    result = pref
    if use_direction or np.linalg.norm(result) >= radius:
        result = (result / np.linalg.norm(result)) * radius
    for i, new_halfplane in enumerate(halfplanes):
        next_result = linear_programming_1(processed_halfplanes, new_halfplane, pref, radius, result, use_direction)
        if next_result is None:
            return result, i
        else:
            result = next_result
        processed_halfplanes.append(new_halfplane)
    # Verify Validity
    for halfplane in halfplanes:
        H_p, H_n = halfplane
        if np.dot(result - H_p, H_n) < -EPSILON:
            print("INVALID", np.dot(result - H_p, H_n), halfplane)
    return result, -1

def solve_lp(halfplanes, pref, radius):
    # Find a solution that is closest to pref
    result, fail_i = linear_programming_2(halfplanes, pref, radius, False)
    if fail_i == -1:
        return result
    else:
        dist = 0.0
        for i in range(fail_i, len(halfplanes)):
            H_p, H_n = halfplanes[i]
            H_dir = np.dot(np.array([[0, -1], [1, 0]]), H_n)
            # If result is within dist of the correct side of H, we're good
            if np.dot(H_n, result - H_p) >= -dist:
                continue
            # Otherwise, get the isohalfplanes
            isohalfplanes = []
            for halfplane in halfplanes[:i]:
                J_p, J_n = halfplane
                iso_p = None

                t_range = get_t(halfplane, H_p, H_dir)
                if t_range[0] > t_range[1]:
                    iso_p = (H_p + J_p)/2
                elif t_range[0] == float('-inf') and t_range[1] == float('inf'):
                    # This doesn't introduce a constraint
                    continue
                else:
                    t = t_range[0] if t_range[0] != float('-inf') else t_range[1]
                    iso_p = H_p + t * H_dir
                iso_n = (J_n - H_n)
                iso_n /= np.linalg.norm(iso_n)
                isohalfplanes.append((iso_p, iso_n))
            # Get the new result
            new_result, new_fail_i = linear_programming_2(isohalfplanes, H_n, radius, True)
            if new_fail_i == -1:
                result = new_result
            # Get the new dist
            dist = np.dot(H_n, H_p - result)

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
        self.next_vel = solve_lp(halfplanes+added_halfplanes, pref_vel, self.max_speed)

    def move(self, delta):
        self.vel = self.next_vel
        self.pos += self.vel * delta

alice = Agent([30.0, 50.0], [-5.0, 90.0], 8.0, 20.0)
bob = Agent([10.0, 60.0], [20.0, 85.0], 8.0, 20.0)
charlie = Agent([-10.0, 50.0], [40.0, 85.0], 8.0, 20.0)
agents = [alice, bob, charlie]

def generate_n_circle(n, radius):
    agents = []
    offset = np.array([40, 40])
    for i in range(n):
        theta = 2*math.pi*i/n
        R_anticlockwise = np.array([[np.cos(theta), -np.sin(theta)],
                                    [np.sin(theta), np.cos(theta)]])
        pos = np.dot(R_anticlockwise, np.array([0, radius]))
        pos += np.array([random.random()*1e-1, random.random()*1e-1])
        agents.append(Agent(offset+pos, offset+[-pos[0], -pos[1]], radius/n, radius/2))
    return agents

agents = generate_n_circle(10, 50)

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
current_time = 0
done = False
def update(num):
    global current_time
    global done
    if done:
        return
    delta = 1 / FPS
    current_time += delta
    
    # Calculate ORCA over the agents
    print("=======================")
    print("CURRENT DIST:", np.min(pdist([agent.pos for agent in agents])), "(vs r={})".format(agents[0].r))
    print("-----------")
    for agent in agents:
        print("Agent:", repr(agent.pos), " | ", repr(agent.vel))
        other_agents = [a for a in agents if a is not agent]
        agent.constrain(other_agents, delta)
        print("New Velocity:", repr(agent.next_vel))
        print("-----------")
    last_fastest_speed = np.max([np.linalg.norm(a.vel) for a in agents])
    this_fastest_speed = np.max([np.linalg.norm(a.next_vel) for a in agents])
    if last_fastest_speed == 0 and this_fastest_speed == 0:
        print("DONE in {} seconds!".format(current_time))
        done = True

    # Move the agents
    for agent in agents:
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
