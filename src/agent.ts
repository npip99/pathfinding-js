import { Point, Face, EPSILON, getPointDist, lerp } from "./math";
import { polyanya } from "./polyanya";
import { hungarian } from "./hungarian";
import { ORCAAgent, getORCAVelocity } from "./orca";

const USING_ORCA = true;

export class Agent {
    // Current position
    position: Point;
    // Agent Radius
    radius: number;
    // Agent Max speed
    speed: number;
    // Previous velocity
    prevVel = new Point(0, 0);
    // Current velocity and speed
    curVel = new Point(0, 0);
    curSpeed = 0;
    // Waypoints are points along the macro-scale path
    waypoints: (Point | null)[] = [];
    // Path is the current path to waypoints[0]
    path: Point[] = [];
    // curTarget is path[0]
    curTarget: Point | null = null;
    constructor(position: Point, radius: number, speed: number) {
        this.position = position;
        this.radius = radius;
        this.speed = speed;
    }
    getORCAAgent(): ORCAAgent {
        return {
            position: this.position,
            velOpt: this.prevVel,
            radius: this.radius,
        };
    }
    pathDistRemaining() {
        let totalPathLength = 0;
        for(let i = 0; i < this.path.length; i++) {
            totalPathLength += getPointDist(i == 0 ? this.position : this.path[i-1], this.path[i]);
        }
        return totalPathLength;
    }
    // Set current waypoints, returning true on success
    async setWaypoints(offsetTraversableFaces: Face[], waypoints: (Point | null)[], maxStartDist?: number) {
        this.stop();
        this.waypoints = waypoints;
        // Update the current target
        await this.updateTarget(offsetTraversableFaces, maxStartDist, true);
        return this.curTarget != null;
    }
    stop() {
        this.waypoints = [];
        this.path = [];
        this.curTarget = null;
    }
    isPathing() {
        return this.hasAnotherTarget() || (this.curTarget != null && this.position.minus(this.curTarget).magnitude() > EPSILON);
    }
    hasAnotherTarget() {
        return this.path.length > 1 || this.waypoints.length > 1;
    }
    // Set Agent's target for this iteration
    async updateTarget(offsetTraversableFaces: Face[], maxPathDist?: number, initializing: boolean = false) {
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
                let result = await polyanya(offsetTraversableFaces, this.position, this.waypoints[0]!, maxPathDist);
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
        // If we've already reached the target, and there's more, pop it and update target
        if (this.hasAnotherTarget() && this.position.minus(this.curTarget).magnitude() < this.radius*(1-1/Math.sqrt(2))) {
            this.curTarget = null;
            await this.updateTarget(offsetTraversableFaces);
        }
    }
    // Takes all neighboring Agents and Obstacles into consideration during the calculation of current velocity
    async considerNeighboringAgents(boundingFace: Face, neighboringObstacles: Face[], offsetTraversableFaces: Face[], neighboringAgents: Agent[], deltaTime: number, speed?: number) {
        if (speed === undefined) {
            speed = this.speed;
        }
        if (!USING_ORCA) {
            neighboringObstacles = [];
            neighboringAgents = [];
        }

        // Update the target, if needed
        await this.updateTarget(offsetTraversableFaces);

        // Set our preferred velocity
        let prefVel = new Point(0, 0);
        if (this.curTarget != null) {
            // If there's a curTarget, our prefVelocity is prefDisplacement/Time
            prefVel = this.curTarget.minus(this.position).divide(deltaTime);
        }

        // Get ORCA Agents from all neighboring Agents
        let orcaAgents: ORCAAgent[] = [];
        for(let neighboringAgent of neighboringAgents) {
            orcaAgents.push(neighboringAgent.getORCAAgent());
        }

        // Set the current velocity to the ORCA velocity
        this.curVel = getORCAVelocity(boundingFace, neighboringObstacles, this.getORCAAgent(), orcaAgents, prefVel, speed, deltaTime);
        this.curSpeed = speed;
    }
    // Iterate the Agent
    iterate(deltaTime: number) {
        // Iterate using the curVel
        let deltaPos = this.curVel.multiply(deltaTime);
        this.position = this.position.plus(deltaPos);

        // timeUsed is how much it did move, versus how much it moves per second
        let timeUsed = deltaPos.magnitude() / this.curSpeed;
        let timeRemaining = Math.max(deltaTime - timeUsed, 0);

        // Save the most recent velocity
        this.prevVel = deltaPos.divide(deltaTime);

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
    for(let i = path.length-1; i > 0; i--) {
        let seg = [path[i-1], path[i]];
        let segLength = getPointDist(seg[0], seg[1]);
        let curLoc = d - accumulatedDistance;
        accumulatedDistance += segLength;
        while(accumulatedDistance >= d) {
            accumulatedDistance -= d;
            ipts.push(new IPoint(lerp(seg[1], seg[0], curLoc/segLength), seg));
            curLoc += d;
        }
    }
    // Before we push the last ipt, ensure the start points aren't too close together
    // We check if we can pop the to-be 2nd to last row
    if (ipts.length >= 2 && ipts[ipts.length-2].line == ipts[ipts.length-1].line && getPointDist(ipts[ipts.length-1].p, path[0]) < d) {
        ipts.pop();
        // If possible, center the to-be 2nd to last row
        if (ipts.length >= 2 && ipts[ipts.length-2].line == ipts[ipts.length-1].line && ipts[ipts.length-1].line[0] == path[0]) {
            ipts[ipts.length-1].p = ipts[ipts.length-2].p.plus(path[0]).divide(2);
        }
    }
    // Satisfies every unique IPoint.line is the same seg array reference
    ipts.push(new IPoint(path[0], ipts[ipts.length-1].line));
    ipts.reverse();

    return ipts;
}

class FormationAgent {
    agent: Agent;
    isInFormation: boolean = false;
    trackManager: TrackManager | null = null;
    trackPtIdx: number = -1;
    secondaryTrackPtIdx: number = -1;
    constructor(agent: Agent) {
        this.agent = agent;
    }
    // Dist Remaining, including the number of agents ahead
    getDistRemaining(mainTrack: Point[]): number {
        if (this.trackManager == null) throw "Invalid";
        // Get the distance for the agents current path
        // TODO: If a detour is faster than mainTrack, make the detouring agent slower instead
        let pathDist = this.agent.pathDistRemaining();
        // Get the distance between the waypoints, as measured along the mainTrack
        let secondaryPts = this.trackManager.secondaryPts[this.trackPtIdx]!;
        for(let i = this.secondaryTrackPtIdx; i < secondaryPts.length-1; i++) {
            pathDist += getPointDist(secondaryPts[i], secondaryPts[i+1]);
        }
        for(let i = this.trackPtIdx; i < mainTrack.length-1; i++) {
            pathDist += getPointDist(mainTrack[i], mainTrack[i+1]);
        }
        return pathDist;
    }
    isPathing(): boolean {
        return true;
    }
    // True if the next target can be iterated
    canIterateNextTarget(): boolean {
        if (this.trackManager == null) throw "Invalid";
        if (this.trackManager.isFinished(this)) {
            return false;
        } else {
            return this.trackManager.getNextIndices(this) == null;
        }
    }
    // Try to iterate the target to the next target, returning true if iteration was successful
    tryIterateTarget(): boolean {
        if (this.trackManager == null) throw "Invalid";
        if (this.trackManager.isFinished(this)) {
            return false;
        } else {
            return this.trackManager!.iterateFormationAgent(this);
        }
    }
    // Get the current target and the next target
    getTarget(): [Point, Point | null] {
        if (this.trackManager == null) throw "Invalid";
        let target = this.trackManager.secondaryPts[this.trackPtIdx]![this.secondaryTrackPtIdx];
        let nextIndices = this.trackManager.getNextIndices(this);
        let nextTarget = null;
        if (nextIndices != null) {
            nextTarget = this.trackManager.secondaryPts[nextIndices[0]]![nextIndices[1]];
        }
        return [target, nextTarget];
    }
}

class TrackManager {
    pts: (Point | null)[];
    secondaryPts: (Point[] | null)[];
    lastValidPtIdx: number;
    occupants: (FormationAgent | null)[][];
    iPointDist: number;
    constructor(pts: (Point | null)[], iPointDist: number) {
        this.pts = pts;
        this.secondaryPts = pts.map(() => null);
        this.secondaryPts[0] = [pts[0]!];
        this.lastValidPtIdx = 0;
        this.occupants = pts.map(() => []);
        this.iPointDist = iPointDist;
    }
    // Get num agents ahead
    getNumAgentsAhead(formationAgent: FormationAgent) {
        let trackPtIdx = formationAgent.trackPtIdx;
        let secondaryTrackPtIdx = formationAgent.secondaryTrackPtIdx;
        let numAgentsAhead = 0;
        while(true) {
            secondaryTrackPtIdx++;
            if (secondaryTrackPtIdx == this.secondaryPts[trackPtIdx]!.length) {
                trackPtIdx++;
                secondaryTrackPtIdx = 0;
                if (trackPtIdx == this.pts.length || this.secondaryPts[trackPtIdx] == null) {
                    break;
                }
            }
            if (this.occupants[trackPtIdx][secondaryTrackPtIdx] != null) {
                numAgentsAhead++;
            }
        }
        return numAgentsAhead;
    }
    // Get next indices. Returns null if finished or needs initialization
    getNextIndices(formationAgent: FormationAgent): [number, number] | null {
        let trackPtIdx = formationAgent.trackPtIdx;
        let secondaryTrackPtIdx = formationAgent.secondaryTrackPtIdx;
        // Go the next secondary track pt
        secondaryTrackPtIdx++;
        // If we've gone past the last secondary track pt,
        if (secondaryTrackPtIdx == this.secondaryPts[trackPtIdx]!.length) {
            // Go to the next track pt
            trackPtIdx++;
            while (trackPtIdx < this.pts.length && this.pts[trackPtIdx] == null) {
                trackPtIdx++;
            }
            if (trackPtIdx == this.pts.length) {
                return null;
            }
            secondaryTrackPtIdx = 0;
            // If secondary hasn't been initialize yet, notify that
            if (this.secondaryPts[trackPtIdx] == null) {
                return null;
            }
        }
        // Otherwise, return the new indices
        return [trackPtIdx, secondaryTrackPtIdx];
    }
    // Get next update
    getNextSecondary(): [Point, Point] | null {
        if (this.lastValidPtIdx == this.pts.length-1) {
            return null;
        } else {
            let nextPtIdx = this.lastValidPtIdx + 1;
            while(this.pts[nextPtIdx] == null) {
                nextPtIdx++;
            }
            return [this.pts[this.lastValidPtIdx]!, this.pts[nextPtIdx]!];
        }
    }
    // Set next update, getting the index of this point in return
    // Requires getNextSecondary(createIPoints) != null
    setNextSecondary(secondaryPath: Point[]): number {
        let nextPtIdx = this.lastValidPtIdx + 1;
        while(this.pts[nextPtIdx] == null) {
            nextPtIdx++;
        }
        secondaryPath = createIPoints(secondaryPath, this.iPointDist*1.01).map(ipt => ipt.p).slice(1);
        this.secondaryPts[nextPtIdx] = secondaryPath;
        this.occupants[nextPtIdx] = [...Array(secondaryPath.length)].map(() => null);
        this.lastValidPtIdx = nextPtIdx;
        return this.lastValidPtIdx;
    }
    // Set formation agent
    setFormationAgent(formationAgent: FormationAgent, ptIdx: number, secondaryPtIdx: number) {
        this.occupants[ptIdx][secondaryPtIdx] = formationAgent;
        formationAgent.trackManager = this;
        formationAgent.trackPtIdx = ptIdx;
        formationAgent.secondaryTrackPtIdx = secondaryPtIdx;
    }
    // Get formation agent target
    // Returns false if secondary must be iterated
    iterateFormationAgent(formationAgent: FormationAgent): boolean {
        if (this.isFinished(formationAgent)) throw 'Invalid';
        let newIndices = this.getNextIndices(formationAgent);
        if (newIndices == null) {
            return false;
        }
        let [trackPtIdx, secondaryTrackPtIdx] = newIndices;
        // Iterate onto the next point, only if it's not occupied
        if (this.occupants[trackPtIdx][secondaryTrackPtIdx] == null) {
            this.occupants[formationAgent.trackPtIdx][formationAgent.secondaryTrackPtIdx] = null;
            formationAgent.trackPtIdx = trackPtIdx;
            formationAgent.secondaryTrackPtIdx = secondaryTrackPtIdx;
            this.occupants[formationAgent.trackPtIdx][formationAgent.secondaryTrackPtIdx] = formationAgent;
        }
        return true;

    }
    // Checks if the index pair is the last one
    isFinished(formationAgent: FormationAgent) {
        return formationAgent.trackPtIdx == this.pts.length-1 && formationAgent.secondaryTrackPtIdx == this.secondaryPts[this.pts.length-1]!.length-1;
    }
}

export class Formation {
    formationAgents: FormationAgent[] = [];
    position: Point;
    speed: number;
    filePitch: number; // Distance between files
    rankPitch: number; // Distance between ranks
    // Tracks for movement
    mainTrack: IPoint[] | null = null;
    trackManagers: TrackManager[] | null = null;
    // Cache
    boundingFace: Face;
    obstacleFaces: Face[];
    offsetTraversableFaces: Face[];
    constructor(boundingFace: Face, obstacleFaces: Face[], offsetTraversableFaces: Face[]) {
        this.boundingFace = boundingFace;
        this.obstacleFaces = obstacleFaces;
        this.offsetTraversableFaces = offsetTraversableFaces;
    }
    addAgents(agents: Agent[]) {
        this.formationAgents.push(...agents.map(agent => new FormationAgent(agent)));
        this.generateFormation();
    }
    generateFormation() {
        // TODO: Find a better meet-up position for all of the agents
        this.position = this.formationAgents[0].agent.position;
        this.speed = this.formationAgents[0].agent.speed;
        for(let i = 0; i < this.formationAgents.length; i++) {
            let formationAgent = this.formationAgents[i];
            if (this.speed == null || formationAgent.agent.speed < this.speed) {
                this.speed = formationAgent.agent.speed;
            }
        }
    }
    stop() {
        if (this.mainTrack != null) {
            this.mainTrack = null;
            for(let formationAgents of this.formationAgents) {
                if (formationAgents.isInFormation) {
                    formationAgents.agent.stop();
                    formationAgents.isInFormation = false;
                }
            }
        }
    }
    async pathfind(dst: Point) {
        this.stop();

        const numAgents = this.formationAgents.length;
        if (numAgents == 0) {
            return;
        }
        this.filePitch = 2.2*this.formationAgents[0].agent.radius;
        this.rankPitch = 2.2*this.formationAgents[0].agent.radius;
        // More ranks than files
        const numRanks = 3;//Math.min(Math.floor(2*Math.sqrt(numAgents)), numAgents);
        const numFiles = Math.ceil(numAgents/numRanks);

        let fileDisplacements = [0];
        while(fileDisplacements.length < numFiles) {
            let prev = fileDisplacements[fileDisplacements.length-1] || 0;
            let prevSign = Math.sign(prev) || -1;
            if (prevSign < 0) {
                fileDisplacements.push(-prev + this.filePitch);
            } else {
                fileDisplacements.push(-prev);
            }
        }

        const IPOINT_DIST = this.rankPitch*0.99;
        const FORMATION_IPOINT_DIST = this.speed*1; // Agents must get into formation within this distance (Speed*Time)
        let result = await polyanya(this.offsetTraversableFaces, this.position, dst, undefined, true);
        // If the result exists, and involves actual movement,
        if (result != null && result.distance > 0) {
            // Create ipts as the mainTrack
            this.mainTrack = createIPoints(result.path, IPOINT_DIST);

            // Create all of the other tracks, as shifted versions of the mainTrack
            // This may include `null`, if it shifts into an obstacle or somewhere too far
            this.trackManagers = [];
            for(let i = 0; i < fileDisplacements.length; i++) {
                let offset = new Point(0, fileDisplacements[i]);
                // Go through the mainTrack, and create the currentTrack using the offset above
                let currentTrack: (Point | null)[] = [];
                for(let j = 0; j < this.mainTrack.length; j++) {
                    // Get the mainTrack pt
                    let mainPt = this.mainTrack[j].p;
                    // Just use it if there's no offset
                    if (offset.magnitude() == 0) {
                        currentTrack.push(mainPt);
                        continue;
                    }
                    // Get the direction along this mainTrack ipt
                    let dir = this.mainTrack[j].line[1].minus(this.mainTrack[j].line[0]).normalize();
                    // Get the actual trackPt, based on the offset rotated by dir
                    let currentTrackPt = mainPt.plus(new Point(dir.x*offset.x - dir.y*offset.y, dir.x*offset.y + dir.y*offset.x));
                    // Point is only valid if obstales delay it no more than FORMATION_IPOINT_DIST from mainTrack
                    let isValid = (await polyanya(this.offsetTraversableFaces, mainPt, currentTrackPt, offset.magnitude() + FORMATION_IPOINT_DIST, true)) != null;
                    currentTrack.push(isValid ? currentTrackPt : null);
                }
                // TODO: Make these nulls the closest valid points to "currentTrackPt",
                // Still ensuring that the chosen start and end points don't overlap
                for(let j = 0; j < Math.min(numRanks, this.mainTrack.length-1); j++) {
                    if (currentTrack[j] == null) {
                        currentTrack[j] = this.mainTrack[j].p;
                    }
                }
                if (currentTrack[currentTrack.length-1] == null) {
                    currentTrack[currentTrack.length-1] = this.mainTrack[this.mainTrack.length-1].p;
                }
                this.trackManagers.push(new TrackManager(currentTrack, IPOINT_DIST));
            }

            let startPoints: [number, number][] = [];
            for(let j = 0; j < numRanks; j++) {
                for(let i = 0; i < numFiles; i++) {
                    if (j == 0) {
                        startPoints.push([i, 0]);
                        continue;
                    }
                    let nextSecondary = this.trackManagers[i].getNextSecondary();
                    if (nextSecondary != null) {
                        let path = await polyanya(this.offsetTraversableFaces, nextSecondary[0], nextSecondary[1], 3*FORMATION_IPOINT_DIST);
                        if (path != null) {
                            let result = this.trackManagers[i].setNextSecondary(path.path);
                            startPoints.push([i, result]);
                        } else {
                            // TODO: Find closest acceptable point, or keep getting the next secondary
                            console.error('Invalid Starting Point!');
                        }
                    }
                }
            }

            // Generate a cost matrix between each track and each agent
            let costMatrix: number[][] = [];
            for(let i = 0; i < this.formationAgents.length; i++) {
                let trackCosts: number[] = [];
                for(let j = 0; j < startPoints.length; j++) {
                    let [file, rankIdx] = startPoints[j];
                    let pt = this.trackManagers[file].pts[rankIdx]!;
                    // Cost = Distance^2 between agent's position and track start point
                    // TODO: Make this use actual distance rather than Euclidean Distance, up to a limit
                    // (Cost=inf beyond that limit point, which would cause the unit to fall out of formation)
                    let extraFormationWalking = rankIdx*this.rankPitch;
                    trackCosts.push(Math.pow(getPointDist(this.formationAgents[i].agent.position, pt) + extraFormationWalking, 3));
                }
                costMatrix.push(trackCosts);
            }
            // Permute the tracks so that they match with the best agent
            // This gets the permutation with the minimal Sum(Cost) from costMatrix
            let startPointPermutation = hungarian(costMatrix);
            let permutedStartPoints = startPointPermutation.map(i => i == null ? null : startPoints[i]);

            // Initialize all of the agents along their track
            const MAX_START_FORMATION_DIST = this.speed*5;
            for(let i = 0; i < this.formationAgents.length; i++) {
                let formationAgent = this.formationAgents[i];
                let startPoint = permutedStartPoints[i];
                if (startPoint == null) {
                    console.error('Could not hungarian into formation!', formationAgent, formationAgent.agent.position.copy());
                    formationAgent.isInFormation = false;
                    continue;
                }
                let [file, rankIdx] = startPoint;
                let pt = this.trackManagers[file].secondaryPts[rankIdx]![0];
                let success = await formationAgent.agent.setWaypoints(this.offsetTraversableFaces, [pt], MAX_START_FORMATION_DIST);
                if (success) {
                    this.trackManagers[file].setFormationAgent(formationAgent, rankIdx, 0);
                    formationAgent.isInFormation = true;
                } else {
                    console.error('Could not pathfind to formation!', formationAgent, formationAgent.agent.position.copy(), pt);
                    formationAgent.isInFormation = false;
                }
            }
        } else {
            console.error('Formation could not pathfind!');
        }
    }
    getIdealSpeeds(deltaTime: number) {
        const MAX_SPEEDUP = 0.25; // Max speed-up of 25%
        if (this.mainTrack == null) {
            throw "Invalid Call!";
        }
        let mainTrackPts = this.mainTrack.map(ipt => ipt.p);

        // Get distance remaining for each Agent, along with the minimum distance remaining
        let distanceRemainings = Array(this.formationAgents.length);
        let minDistanceRemaining: number | null = null;
        for(let i = 0; i < this.formationAgents.length; i++) {
            let formationAgent = this.formationAgents[i];
            if (!formationAgent.isInFormation) {
                continue;
            }
            let distanceRemaining = this.formationAgents[i].getDistRemaining(mainTrackPts);
            let numAgentsAhead = formationAgent.trackManager!.getNumAgentsAhead(formationAgent);
            distanceRemainings[i] = Math.max(distanceRemaining - numAgentsAhead*this.rankPitch, 0);
            minDistanceRemaining = minDistanceRemaining == null ? distanceRemaining : Math.min(minDistanceRemaining, distanceRemaining);
        }

        // Generate the ideal speeds for each Agent
        let idealSpeeds = Array(this.formationAgents.length);
        // Ideal speed is based on each agent's comparison to the agent that's furthest along the track
        for(let i = 0; i < this.formationAgents.length; i++) {
            let formationAgent = this.formationAgents[i];
            if (!formationAgent.isInFormation) {
                idealSpeeds[i] = this.speed;
                continue;
            }
            let distanceRemaining = distanceRemainings[i];
            // If this agent is behind the agent that has made the most process,
            if (minDistanceRemaining != null && minDistanceRemaining < distanceRemaining - EPSILON) {
                // Check how far the agent is behind
                let deltaDist = (distanceRemaining - minDistanceRemaining);
                // And speed up by the necessary amount to catchup this frame.
                let neededSpeedup = deltaDist / deltaTime;
                // Capped at MAX_SPEEDUP or the agent's natural speed
                let maxSpeed = Math.max(this.speed*(1+MAX_SPEEDUP), formationAgent.agent.speed);

                idealSpeeds[i] = Math.min(maxSpeed, this.speed + neededSpeedup);
            } else {
                // No speed-up needed
                idealSpeeds[i] = this.speed;
            }
        }

        // Return the ideal speeds
        return idealSpeeds;
    }
    async considerNeighboringAgents(otherAgents: Agent[], deltaTime: number) {
        // If the Formation is following a main track, iterate it
        if (this.mainTrack != null) {
            // Get Ideal Speed for all Agents
            let agentSpeeds = this.getIdealSpeeds(deltaTime);

            // Iterate targets
            for(let i = 0; i < this.formationAgents.length; i++) {
                let formationAgent = this.formationAgents[i];
                if (!formationAgent.isInFormation) {
                    continue;
                }
                // Keep iterating the target while we still can
                for (let i = 0; i < 10; i++) {
                    // If we can iterate the next target, do so in advance
                    if (formationAgent.canIterateNextTarget()) {
                        let trackManager = formationAgent.trackManager!;
                        let [start, end] = trackManager.getNextSecondary()!;
                        let path = await polyanya(this.offsetTraversableFaces, start, end);
                        if (path == null) {
                            console.error('Track Broken!', start, end);
                            throw 'Track Broken!';
                        } else {
                            trackManager.setNextSecondary(path.path);
                        }
                    }
                    // Get the target and next target
                    let [curTarget, nextPoint] = formationAgent.getTarget();
                    // If we should iterate if we're within radius of the current target,
                    let tryIterate = false;
                    if (getPointDist(formationAgent.agent.position, curTarget) < formationAgent.agent.radius) {
                        tryIterate = true;
                    } else if (nextPoint != null) {
                        // Or we're going towards the target in the wrong direction
                        let dir = nextPoint.minus(curTarget);
                        if (curTarget.minus(formationAgent.agent.position).dot(dir) < 0) {
                            tryIterate = true;
                        }
                    }
                    // Check otherAgents if they block the waypoint, and we're as close as we can get
                    if (!tryIterate) {
                        for(let otherAgent of otherAgents) {
                            if (getPointDist(otherAgent.position, curTarget) < otherAgent.radius &&
                            getPointDist(formationAgent.agent.position, curTarget) < 1.1*otherAgent.radius + formationAgent.agent.radius) {
                                tryIterate = true;
                            }
                        }
                    }
                    // Try to iterate the target, engaging in iteration logic if succeeded
                    if (tryIterate && formationAgent.tryIterateTarget()) {
                        let [newTarget, _] = formationAgent.getTarget();
                        let success = await formationAgent.agent.setWaypoints(this.offsetTraversableFaces, [newTarget], 20*this.rankPitch);
                        if (!success) {
                            console.error('Could not set Waypoints!', formationAgent.agent.position.copy(), newTarget);
                            formationAgent.isInFormation = false;
                            break;
                        }
                    }
                }
            }
            

            // Have all agents consider collision avoidance
            for(let i = 0; i < this.formationAgents.length; i++) {
                let neighboringAgents = this.formationAgents.map(formationAgent => formationAgent.agent);
                neighboringAgents.splice(i, 1);
                neighboringAgents = neighboringAgents.concat(otherAgents);
                await this.formationAgents[i].agent.considerNeighboringAgents(this.boundingFace, this.obstacleFaces, this.offsetTraversableFaces, neighboringAgents, deltaTime, agentSpeeds[i]);
            }
        }
    }
    async iterate(deltaTime: number) {
        if (this.mainTrack != null) {
            // Iterate all Agents, with reciprocal collision avoidance
            let remainingTimes = Array(this.formationAgents.length);
            for(let i = 0; i < this.formationAgents.length; i++) {
                let formationAgent = this.formationAgents[i];
                if (!formationAgent.isInFormation) {
                    continue;
                }
                // Iterate this agent
                remainingTimes[i] = formationAgent.agent.iterate(deltaTime);
            }
            // Iterate again, with any remaining time, using strict collision avoidance
            // TODO: Implement an ORCA run assuming other agents won't move
            /*for(let i = 0; i < this.formationAgents.length; i++) {
                let formationAgent = this.formationAgents[i];
                if (!agent.isInFormation) {
                    continue;
                }
                // Iterate again with any remaining time
                if (remainingTimes[i] > EPSILON) {
                    await agent.iterate(remainingTimes[i]);
                }
            }*/

            // Get the status of agents at end-of-frame
            let bestDistanceRemaining: number | null = null;
            let numAgentsPathing = 0;
            let mainTrackPts = this.mainTrack.map(ipt => ipt.p);
            for(let i = 0; i < this.formationAgents.length; i++) {
                let formationAgent = this.formationAgents[i];
                if (!formationAgent.isInFormation) {
                    continue;
                }
                let distanceRemaining = formationAgent.getDistRemaining(mainTrackPts);
                // Keep track of the most progress,
                if (formationAgent.isPathing()) {
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
