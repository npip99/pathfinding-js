import { Point, Face, EPSILON, getPointDist, lerp } from "./math";
import { polyanya } from "./polyanya";
import { hungarian } from "./hungarian";

export class Agent {
    // Current position
    position: Point;
    // Current speed
    speed: number;
    // Previous velocity
    prevVel = new Point(0, 0);
    // Waypoints are points along the macro-scale path
    waypoints: (Point | null)[] = [];
    // Path is the current path to waypoints[0]
    path: Point[] = [];
    // curTarget is path[0]
    curTarget: Point | null = null;
    // Cache
    faces: Face[];
    constructor(position: Point, speed: number, faces: Face[]) {
        this.position = position;
        this.speed = speed;
        this.faces = faces;
    }
    pathDistRemaining() {
        let totalPathLength = 0;
        for(let i = 0; i < this.path.length; i++) {
            totalPathLength += getPointDist(i == 0 ? this.position : this.path[i-1], this.path[i]);
        }
        return totalPathLength;
    }
    async setWaypoints(waypoints: (Point | null)[], maxStartDist?: number) {
        this.stop();
        this.waypoints = waypoints;
        // Update the current target
        await this.updateTarget(maxStartDist, true);
    }
    stop() {
        this.waypoints = [];
        this.path = [];
        this.curTarget = null;
    }
    isPathing() {
        return this.curTarget != null;
    }
    // Set Agent's target for this iteration
    async updateTarget(maxPathDist?: number, initializing: boolean = false) {
        // If we're done with our target,
        if (this.curTarget == null) {
            if (!initializing) {
                // Drop path[0]
                this.path.splice(0, 1);
            }
            // If we're done with our path, get a new path or exit if done
            if (this.path.length == 0) {
                if (!initializing) {
                    // Drop waypoints[0]
                    this.waypoints.splice(0, 1);
                }
                // Skip over null waypoints
                while(this.waypoints.length > 0 && this.waypoints[0] == null) {
                    this.waypoints.splice(0, 1);
                }
                // If there are no more waypoints, we're done
                if (this.waypoints.length == 0) {
                    this.stop();
                    return;
                }
                let result = await polyanya(this.faces, this.position, this.waypoints[0]!, maxPathDist);
                if (result == null) {
                    this.stop();
                    return;
                }
                this.path = result.path;
                this.path.splice(0, 1); // Ignore src in Path
            }
            // Use the next path
            this.curTarget = this.path[0];
        }
        // If we've already reached the target, pop it and update target
        if (this.position.minus(this.curTarget).magnitude() < EPSILON) {
            this.position = this.curTarget;
            this.curTarget = null;
            await this.updateTarget();
        }
    }
    // Iterate the vec
    async iterate(deltaTime: number, speed?: number) {
        // Update the target
        await this.updateTarget();
        if (speed === undefined) {
            speed = this.speed;
        }

        // Our preferred velocity
        let prefVel = new Point(0, 0);

        // If we have no target, our pref vel is nothing
        if (this.curTarget == null) {
            prefVel = new Point(0, 0);
        } else {
            // Otherwise, our pref vel is Displacement/Time (Capped at max speed)
            prefVel = this.curTarget.minus(this.position).divide(deltaTime);
            if (prefVel.magnitude() > speed) {
                prefVel = prefVel.normalize().multiply(speed);
            }
        }

        // Iterate using the prefVel
        let deltaPos = prefVel.multiply(deltaTime);
        this.position = this.position.plus(deltaPos);

        // timeUsed is how much it did move, versus how much it moves per second
        let timeUsed = deltaPos.magnitude() / speed;
        let timeRemaining = Math.max(deltaTime - timeUsed, 0);

        // Save the most recent velocity
        this.prevVel = deltaPos.divide(timeUsed);

        // Return any remaining time to the caller
        return timeRemaining;
    }
}

// Intermediate Point in a path
class IPoint {
    p: Point;
    line: Point[];
    constructor(point: Point, line: Point[]) {
        this.p = point;
        this.line = line;
    }
}

function createIPoints(path: Point[], d: number) {
    let ipts: IPoint[] = [];

    // Create ipts every time a distance d is accumulated
    // Start with d so that path[0] is an ipt
    let accumulatedDistance = d;
    for(let i = 0; i < path.length-1; i++) {
        let seg = [path[i], path[i+1]];
        let segLength = getPointDist(seg[0], seg[1]);
        let curLoc = d - accumulatedDistance;
        accumulatedDistance += segLength;
        while(accumulatedDistance >= d) {
            accumulatedDistance -= d;
            ipts.push(new IPoint(lerp(seg[0], seg[1], curLoc/segLength), seg));
            curLoc += d;
        }
    }
    // TODO: Drop ipts[-1] if it's too close to path.lenth-1, but not if its src
    // Satisfies every unique IPoint.line is the same seg array reference
    ipts.push(new IPoint(path[path.length-1], ipts[ipts.length-1].line));

    return ipts;
}

export interface AgentFormationData {
    isInFormation: boolean,
    track: (Point | null)[],
}

export class Formation {
    agents: Agent[] = [];
    position: Point;
    speed: number;
    // Tracks for movement
    mainTrack: IPoint[] | null = null;
    relativePositions: Point[] = [];
    // FormationData for each agent
    agentFormationData = new Map<Agent, AgentFormationData>();
    // Cache
    faces: Face[];
    constructor(faces: Face[]) {
        this.faces = faces;
    }
    addAgents(agents: Agent[]) {
        this.agents.push(...agents);
        for(let agent of agents) {
            this.agentFormationData.set(agent, {
                isInFormation: false,
                track: [],
            })
        }
        this.generateFormation();
    }
    getAgentFormationData(agent: Agent): AgentFormationData | null {
        return this.agentFormationData.get(agent) || null;
    }
    generateFormation() {
        // TODO: Find a better meet-up position for all of the agents
        this.position = this.agents[0].position;
        this.speed = this.agents[0].speed;
        this.relativePositions = [];
        for(let i = 0; i < this.agents.length; i++) {
            this.relativePositions.push(new Point(0, (this.agents.length-1)/2 - i));
            if (this.speed == null || this.agents[i].speed < this.speed) {
                this.speed = this.agents[i].speed;
            }
        }
    }
    stop() {
        if (this.mainTrack != null) {
            this.mainTrack = null;
            for(let agent of this.agents) {
                if (this.getAgentFormationData(agent)!.isInFormation && agent.isPathing()) {
                    agent.stop();
                    this.getAgentFormationData(agent)!.isInFormation = false;
                }
            }
        }
    }
    async pathfind(dst: Point) {
        this.stop();
        const IPOINT_DIST = this.speed*1; // IPOINTS are spaced between 1 Second of movement
        const FORMATION_IPOINT_DIST = this.speed;
        let result = await polyanya(this.faces, this.position, dst);
        // If the result exists, and involves actual movement,
        if (result != null && result.distance > 0) {
            // Create ipts as the mainTrack
            this.mainTrack = createIPoints(result.path, IPOINT_DIST);

            // Create all of the other tracks, as shifted versions of the mainTrack
            // This may include `null`, if it shifts into an obstacle or somewhere too far
            let allTracks: (Point | null)[][] = [];
            for(let i = 0; i < this.relativePositions.length; i++) {
                let offset = this.relativePositions[i];
                // Go through the mainTrack, and create the currentTrack using the offset above
                let currentTrack: (Point | null)[] = [];
                for(let j = 0; j < this.mainTrack.length; j++) {
                    // Get the mainTrack pt
                    let mainPt = this.mainTrack[j].p;
                    // Get the direction along this mainTrack ipt
                    let dir = this.mainTrack[j].line[1].minus(this.mainTrack[j].line[0]).normalize();
                    // Get the actual trackPt, based on the offset rotated by dir
                    let currentTrackPt = mainPt.plus(new Point(dir.x*offset.x - dir.y*offset.y, dir.x*offset.y + dir.y*offset.x));
                    // Point is only valid if obstales delay it no more than FORMATION_IPOINT_DIST from mainTrack
                    let isValid = (await polyanya(this.faces, mainPt, currentTrackPt, offset.magnitude() + FORMATION_IPOINT_DIST)) != null;
                    currentTrack.push(isValid ? currentTrackPt : null);
                }
                // TODO: Make these the closest valid points to "currentTrackPt"
                if (currentTrack[0] == null) {
                    currentTrack[0] = this.mainTrack[0].p;
                }
                if (currentTrack[currentTrack.length-1] == null) {
                    currentTrack[currentTrack.length-1] = this.mainTrack[this.mainTrack.length-1].p;
                }
                allTracks.push(currentTrack);
            }

            // Generate a cost matrix between each track and each agent
            let costMatrix: number[][] = [];
            for(let i = 0; i < this.agents.length; i++) {
                let trackCosts: number[] = [];
                for(let j = 0; j < allTracks.length; j++) {
                    // Cost = Distance^2 between agent's position and track start point
                    trackCosts.push(Math.pow(getPointDist(this.agents[i].position, allTracks[j][0]!), 2));
                }
                costMatrix.push(trackCosts);
            }
            // Permute the tracks so that they match with the best agent
            // This gets the permutation with the minimal Sum(Cost) from costMatrix
            let trackPermutation = hungarian(costMatrix);
            allTracks = allTracks.map((_, i) => allTracks[trackPermutation[i]]);

            // Initialize all of the agents along their track
            for(let i = 0; i < this.agents.length; i++) {
                this.getAgentFormationData(this.agents[i])!.isInFormation = false;
                await this.startTrack(this.agents[i], allTracks[i]);
            }
        } else {
            console.error('Formation could not pathfind!');
        }
    }
    async startTrack(agent: Agent, track: (Point | null)[]) {
        // <= 5 seconds away to join formation
        const MAX_START_FORMATION_DIST = this.speed*5;
        if (!this.getAgentFormationData(agent)!.isInFormation) {
            await agent.setWaypoints(track, MAX_START_FORMATION_DIST);
            if (!agent.isPathing()) {
                // TODO: Handle if Agent cannot join formation
                console.error('Could not join formation!', agent.position, track[0]);
                return;
            }
            this.agentFormationData.set(agent, {
                isInFormation: true,
                track: track,
            })
        }
    }
    // Get the amount of distance remaining for this agent
    // Returns null if Formation isn't pathing, or the Agent isn't in formation
    getDistanceRemaining(agent: Agent) {
        if (this.mainTrack == null) {
            return null;
        }
        if (!this.getAgentFormationData(agent)!.isInFormation) {
            return null;
        }
        if (!agent.isPathing()) {
            return 0;
        }
        // Get the distance for the agents current path
        // TODO: If a detour is faster than mainTrack, make the detouring agent slower instead
        let pathDist = agent.pathDistRemaining();
        // Get the distance between the waypoints, as measured along the mainTrack
        let waypointDist = 0;
        for(let i = this.mainTrack.length-agent.waypoints.length; i < this.mainTrack.length-1; i++) {
            waypointDist += getPointDist(this.mainTrack[i].p, this.mainTrack[i+1].p);
        }
        return pathDist + waypointDist;
    }
    getIdealSpeeds(deltaTime: number) {
        const MAX_SPEEDUP = 0.25; // Max speed-up of 25%
        if (this.mainTrack == null) {
            throw "Invalid Call!";
        }

        // Get distance remaining for each Agent, along with the minimum distance remaining
        let distanceRemainings = Array(this.agents.length);
        let minDistanceRemaining: number | null = null;
        for(let i = 0; i < this.agents.length; i++) {
            let agent = this.agents[i];
            let distanceRemaining = this.getDistanceRemaining(agent);
            if (distanceRemaining == null) {
                continue;
            }
            distanceRemainings[i] = distanceRemaining;
            minDistanceRemaining = minDistanceRemaining == null ? distanceRemaining : Math.min(minDistanceRemaining, distanceRemaining);
        }

        // Generate the ideal speeds for each Agent
        let idealSpeeds = Array(this.agents.length);
        if (minDistanceRemaining == null) {
            // If there's no agents in formation, just return
            return idealSpeeds;
        }
        // Ideal speed is based on each agent's comparison to the agent that's furthest along the track
        for(let i = 0; i < this.agents.length; i++) {
            let agent = this.agents[i];
            if (!this.getAgentFormationData(agent)!.isInFormation) {
                continue;
            }
            let distanceRemaining = distanceRemainings[i];
            // If this agent is behind,
            if (minDistanceRemaining < distanceRemaining - EPSILON) {
                // Check how far the agent is behind
                let deltaDist = (distanceRemaining - minDistanceRemaining);
                // And speed up by the necessary amount to catchup this frame.
                let neededSpeedup = deltaDist / deltaTime;
                // Capped at MAX_SPEEDUP or the agent's natural speed
                let maxSpeed = Math.max(this.speed*(1+MAX_SPEEDUP), agent.speed);

                idealSpeeds[i] = Math.min(maxSpeed, this.speed + neededSpeedup);
            } else {
                // No speed-up needed
                idealSpeeds[i] = this.speed;
            }
        }

        // Return the ideal speeds
        return idealSpeeds;
    }
    async iterate(deltaTime: number) {
        // If the Formation is following a main track, iterate it
        if (this.mainTrack != null) {
            // Get Ideal Speed for all Agents
            let agentSpeeds = this.getIdealSpeeds(deltaTime);

            // Iterate all Agents, with reciprocal collision avoidance
            let remainingTimes = Array(this.agents.length);
            for(let i = 0; i < this.agents.length; i++) {
                let agent = this.agents[i];
                if (!this.getAgentFormationData(agent)!.isInFormation) {
                    continue;
                }
                // Iterate this agent
                remainingTimes[i] = await agent.iterate(deltaTime, agentSpeeds[i]);
            }
            // Iterate again, with any remaining time, using strict collision avoidance
            for(let i = 0; i < this.agents.length; i++) {
                let agent = this.agents[i];
                if (!this.getAgentFormationData(agent)!.isInFormation) {
                    continue;
                }
                // Iterate again with any remaining time
                if (remainingTimes[i] > EPSILON) {
                    await agent.iterate(remainingTimes[i], agentSpeeds[i]);
                }
            }

            // Get the status of agents at end-of-frame
            let bestDistanceRemaining: number | null = null;
            let numAgentsPathing = 0;
            for(let i = 0; i < this.agents.length; i++) {
                let agent = this.agents[i];
                let distanceRemaining = this.getDistanceRemaining(agent);
                if (distanceRemaining == null) {
                    continue;
                }
                // Keep track of the most progress,
                if (agent.isPathing()) {
                    numAgentsPathing++;
                }
                if (bestDistanceRemaining == null || distanceRemaining < bestDistanceRemaining) {
                    // Track the best track progress
                    bestDistanceRemaining = distanceRemaining;
                }
            }

            // Find the point along mainTrack that matches the most progress (bestDistanceRemaining)
            let curIdx = this.mainTrack.length - 1;
            let bestPosition = this.mainTrack[curIdx].p;
            // If a bestDistance exists, use it
            if (bestDistanceRemaining != null) {
                let lastLine: Point[] | null = null;
                while (bestDistanceRemaining > 0 && curIdx >= 0) {
                    let mainTrackipt = this.mainTrack[curIdx];
                    // When processing a new line, walk backwards across it
                    if (lastLine == null || lastLine !== mainTrackipt.line) {
                        lastLine = mainTrackipt.line;
                        let dir = lastLine[1].minus(lastLine[0]);
                        let distTravelled = Math.min(dir.magnitude(), bestDistanceRemaining);
                        bestPosition = lastLine[1].minus(dir.normalize().multiply(distTravelled));
                        bestDistanceRemaining -= distTravelled;
                    }
                    curIdx--;
                }
            }
            // Update position of the formation, to be the mainTrack point
            // measured by the unit that has made the most progress
            this.position = bestPosition;
            // If all agents have finished pathing, then finish
            if (numAgentsPathing == 0) {
                this.stop();
            }
        }
    }
}
