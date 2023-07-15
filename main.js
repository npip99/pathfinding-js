class Agent {
    position;
    speed;
    path = [];
    isPathing = false;
    constructor(position, speed) {
        this.position = position;
        this.speed = speed;
    }
    pathDistRemaining() {
        let totalPathLength = 0;
        for(let i = 0; i < this.path.length; i++) {
            totalPathLength += getPointDist(i == 0 ? this.position : this.path[i-1], this.path[i]);
        }
        return totalPathLength;
    }
    async pathfind(faces, dst, max_dist=undefined) {
        let result = await polyanya(faces, this.position, dst, max_dist);
        if (result != null) {
            this.path = result['path'];
            this.path.splice(0, 1); // Ignore src in Path
            this.isPathing = true;
            return result['distance'];
        }
        return null;
    }
    stop() {
        this.path = [];
        this.isPathing = false;
    }
    iterate(deltaTime, speed=undefined) {
        let remainingTime = deltaTime;
        if (speed === undefined) {
            speed = this.speed;
        }
        // If there's path left, move along the path
        if (this.path.length > 0) {
            let nextWaypoint = this.path[0];
            let nextWaypointDist = getPointDist(this.position, nextWaypoint);
            let waypointDir = nextWaypoint.minus(this.position).normalize();
            let maxMovementDist = speed * deltaTime;

            // If we can reach the Waypoint, just land right on the Waypoint
            if (nextWaypointDist <= maxMovementDist) {
                this.position = nextWaypoint;
                this.path.splice(0, 1); // TODO: Make a LinkedList for speed
                // Save the remaining time
                remainingTime = (1 - nextWaypointDist/maxMovementDist) * deltaTime;
            } else {
                // Else, get closer by maxMovementDist
                this.position = this.position.plus(waypointDir.multiply(maxMovementDist));
                remainingTime = 0;
            }
        }
        // Mark whether or not this iteration completed the path
        if (this.path.length == 0) {
            this.isPathing = false;
        }
        // If there's still pathing and time remaining, iterate again
        if(this.isPathing == true && remainingTime > 0) {
            return this.iterate(remainingTime, speed);
        } else {
            // Else, pass forward the unused time
            return remainingTime;
        }
    }
}

// Intermediate Point in a path
class IPoint {
    p;
    line;
    constructor(point, line) {
        this.p = point;
        this.line = line;
    }
}

function createIPoints(path, d) {
    let ipts = [];

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
    ipts.push(new IPoint(path[path.length-1], [path[path.length-2], path[path.length-1]]));

    return ipts;
}

class Formation {
    agents = [];
    position;
    speed;
    // Tracks for movement
    mainTrack = null;
    constructor() {
    }
    addAgents(agents) {
        this.agents.push(...agents);
        this.generateFormation();
    }
    generateFormation() {
        // TODO: Find a better meet-up position for all of the agents
        this.position = this.agents[0].position;
        this.speed = null;
        this.relative_positions = [];
        for(let i = 0; i < this.agents.length; i++) {
            this.relative_positions.push(new Point(0, (this.agents.length-1)/2 - i));
            if (this.speed == null || this.agents[i].speed < this.speed) {
                this.speed = this.agents[i].speed;
            }
        }
    }
    stop() {
        if (this.mainTrack != null) {
            this.mainTrack = null;
            for(let agent of this.agents) {
                if (agent.isInFormation) {
                    agent.stop();
                    agent.isInFormation = false;
                }
            }
        }
    }
    async pathfind(faces, dst) {
        this.faces = faces;
        const IPOINT_DIST = this.speed/2;
        const FORMATION_IPOINT_DIST = this.speed;
        let result = await polyanya(faces, this.position, dst);
        // If the result exists, and involves actual movement,
        if (result != null && result['distance'] > 0) {
            // Create ipts as the mainTrack
            this.mainTrack = createIPoints(result['path'], IPOINT_DIST);

            // Create all of the other tracks, as shifted versions of the mainTrack
            // This may include `null`, if it shifts into an obstacle or somewhere too far
            let allTracks = [];
            for(let i = 0; i < this.relative_positions.length; i++) {
                let offset = this.relative_positions[i];
                // Go through the mainTrack, and create the currentTrack using the offset above
                let currentTrack = [];
                for(let j = 0; j < this.mainTrack.length; j++) {
                    // Get the mainTrack pt
                    let mainPt = this.mainTrack[j].p;
                    // Get the direction along this mainTrack ipt
                    let dir = this.mainTrack[j].line[1].minus(this.mainTrack[j].line[0]).normalize();
                    // Get the actual trackPt, based on the offset rotated by dir
                    let currentTrackPt = mainPt.plus(new Point(dir.x*offset.x - dir.y*offset.y, dir.x*offset.y + dir.y*offset.x));
                    // Point is only valid if obstales delay it no more than FORMATION_IPOINT_DIST from mainTrack
                    let isValid = (await polyanya(faces, mainPt, currentTrackPt, offset.magnitude() + FORMATION_IPOINT_DIST)) != null;
                    currentTrack.push(isValid ? currentTrackPt : null);
                }
                allTracks.push(currentTrack);
            }

            // Post-process the tracks to remove any nulls
            let allTrackIndices = [];
            for(let trackIdx = 0; trackIdx < this.relative_positions.length; trackIdx++) {
                // TODO: Make this the closest valid pt "near" the starting location
                let firstTarget = allTracks[trackIdx][0];
                if (firstTarget == null) {
                    firstTarget = this.mainTrack[0].p;
                }
                // The new non-null track, and the mainTrack idx each element refers to
                let track = [firstTarget];
                let trackIndices = [0];
                for(let i = 1; i < this.mainTrack.length; i++) {
                    // Get the next valid track target >= i
                    let idx = i;
                    let trackTarget = null;
                    while(trackTarget == null && idx < this.mainTrack.length) {
                        trackTarget = allTracks[trackIdx][idx];
                        idx++;
                    }
                    // If there's none, get the last mainTrack point
                    // TODO: Make this the closest valid pt "near"
                    // Where the unit is supposed to be
                    if (trackTarget == null) {
                        trackTarget = this.mainTrack[this.mainTrack.length-1].p;
                    }
                    track.push(trackTarget);
                    trackIndices.push(idx-1);
                }
                allTracks[trackIdx] = track;
                allTrackIndices.push(trackIndices);
            }

            // Generate a cost matrix between each track and each agent
            let costMatrix = Array(this.agents.length);
            for(let i = 0; i < this.agents.length; i++) {
                let trackCosts = Array(allTracks.length);
                for(let j = 0; j < allTracks.length; j++) {
                    trackCosts[j] = Math.pow(getPointDist(this.agents[i].position, allTracks[j][0]), 2);
                }
                costMatrix[i] = trackCosts;
            }
            // Permute the tracks so that they match with the best agent
            // This creates the minimal Sum(Distance^2) from costMatrix
            let trackPermutation = hungarian(costMatrix);
            allTracks = allTracks.map((row, i) => allTracks[trackPermutation[i]]);
            allTrackIndices = allTrackIndices.map((row, i) => allTrackIndices[trackPermutation[i]]);

            // Initialize all of the agents along their track
            for(let i = 0; i < this.agents.length; i++) {
                this.agents[i].isInFormation = false;
                await this.startTrack(this.agents[i], allTracks[i], allTrackIndices[i]);
            }
        } else { // result == null
            this.stop();
        }
    }
    async startTrack(agent, track, trackIndices) {
        // <= 5 seconds away to join formation
        const MAX_START_FORMATION_DIST = this.speed*5;
        if (!agent.isInFormation) {
            let dist = await agent.pathfind(this.faces, track[0], MAX_START_FORMATION_DIST);
            if (dist == null) {
                // TODO: Handle if Agent cannot join formation
                console.error('Could not join formation!', agent.position, track[0]);
                return;
            }
            agent.isInFormation = true;
            agent.isJoining = true;
            agent.isTracking = false;
            agent.trackDistance = dist;
            agent.trackProgress = -1;
            agent.track = track;
            agent.trackIndices = trackIndices; // The idx its pointing to
        }
    }
    // Get the amount of distance remaining for this agent
    getDistanceRemaining(agent_index) {
        let agent = this.agents[agent_index];
        if (!agent.isInFormation) {
            return null;
        }
        let trackProgress = agent.trackProgress;
        if (trackProgress == agent.track.length-1) {
            return 0;
        }
        let curTrackIdx = agent.trackIndices[trackProgress+1];
        let distToTrackPoint = agent.pathDistRemaining();
        let distTrackPointToEnd = 0;
        for(let i = curTrackIdx; i < agent.track.length-1; i++) {
            distTrackPointToEnd += getPointDist(this.mainTrack[i].p, this.mainTrack[i+1].p);
        }
        return distToTrackPoint + distTrackPointToEnd;
    }
    getIdealSpeed(agent_index, remainingTimes) {
        const MAX_SPEEDUP = 0.25; // Max speed-up of 25%
        let curTrackRemaining = null;
        let minTrackRemaining = null;
        for(let i = 0; i < this.agents.length; i++) {
            let agent = this.agents[i];
            if (!agent.isInFormation) {
                continue;
            }

            let timeAdjustedDistance = Math.max(0, this.getDistanceRemaining(i) - this.speed*remainingTimes[i]);
            minTrackRemaining = minTrackRemaining == null ? timeAdjustedDistance : Math.min(minTrackRemaining, timeAdjustedDistance);
            if (i == agent_index) {
                curTrackRemaining = timeAdjustedDistance;
            }
        }
        if (curTrackRemaining == null) {
            console.error("Cant get ideal speed, agent not in formation");
            return 0;
        }
        // If this agent is behind,
        if (minTrackRemaining < curTrackRemaining - 1e-13) {
            // Check how far the agent is behind
            let deltaDist = (curTrackRemaining - minTrackRemaining);
            // And speed up by the necessary amount to catchup this frame.
            let neededSpeedup = deltaDist / remainingTimes[agent_index];
            // Capped at MAX_SPEEDUP or the agent's natural speed
            let maxSpeed = Math.max(this.speed*(1.0+MAX_SPEEDUP), this.agents[agent_index].speed);
            
            return Math.min(maxSpeed, this.speed + neededSpeedup);
        } else {
            // No speed-up needed
            return this.speed;
        }
    }
    async iterate(deltaTime) {
        let remainingTimes = Array(this.agents.length);
        for(let i = 0; i < this.agents.length; i++) {
            remainingTimes[i] = deltaTime;
            //if (this.agents[i].isPathing)
            //    console.log(i, this.getDistanceRemaining(i), this.getIdealSpeed(i, deltaTime)/this.speed);
        }
        // If the Formation is following a main track, iterate it
        if (this.mainTrack != null) {
            let bestDistanceRemaining = null;
            let bestPosition = this.mainTrack[0].p;
            for(let i = 0; i < this.agents.length; i++) {
                let agent = this.agents[i];
                if (!agent.isInFormation) {
                    continue;
                }
                // Iterate this agent
                remainingTimes[i] = agent.iterate(deltaTime, this.getIdealSpeed(i, remainingTimes));
                // Check if they're done Joining
                if (agent.isJoining && agent.isPathing == false) {
                    agent.isJoining = false;
                    agent.trackDistance = 0;
                }
                // If they're done joining, iterate them along the track
                if (agent.isJoining == false) {
                    // While the agent is not pathing and there's still more track points,
                    while (agent.isPathing == false && agent.trackProgress < agent.track.length-1) {
                        // Mark the current track point as accomplished,
                        agent.trackProgress++;
                        // And pathfind to the next track point
                        if (agent.trackProgress < agent.track.length-1) {
                            // TODO: Set a pathfind limit here
                            await agent.pathfind(this.faces, agent.track[agent.trackProgress+1]);
                            remainingTimes[i] = agent.iterate(remainingTimes[i], this.getIdealSpeed(i, remainingTimes));
                        }
                    }
                }
                // Keep track of the most progress,
                let distanceRemaining = this.getDistanceRemaining(i);
                if (bestDistanceRemaining == null || distanceRemaining < bestDistanceRemaining) {
                    bestDistanceRemaining = distanceRemaining;
                    let distToTrackPoint = agent.pathDistRemaining();
                    // If the agent is done, the best position is done
                    if (agent.trackProgress == this.mainTrack.length-1) {
                        bestPosition = this.mainTrack[this.mainTrack.length-1].p;
                    } else {
                        // If agent is in the middle,
                        // Find the mainTrack pt that matches it
                        // TODO: Actually walk backwards along the path,
                        //       Rather than just -Dir*dist
                        let mainTrackipt = this.mainTrack[agent.trackIndices[agent.trackProgress+1]];
                        let dir = mainTrackipt.line[0].minus(mainTrackipt.line[1]).normalize();
                        bestPosition = mainTrackipt.p.plus(dir.multiply(distToTrackPoint));
                    }
                }
            }
            // Update position of the formation, to be the mainTrack waypoint
            // Of the unit that has made the most progress
            this.position = bestPosition;
        }
    }
}

async function main() {
    //const faces = LoadPolyData(poly_data_1);

    const MAZE_URL = 'https://api.allorigins.win/raw?url=https://pastebin.com/raw/GWRCSyUp';
    //const maze_poly_data = LoadMaze(await getUrlContents(MAZE_URL));
    //const faces = LoadPolyData(maze_poly_data);

    //const mesh_poly_data = await LoadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena-merged.mesh');
    const mesh_poly_data = await LoadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena.mesh');
    //const mesh_poly_data = await LoadMesh('https://api.allorigins.win/raw?url=https://pastebin.com/raw/DdgmNAT3');
    const faces = LoadPolyData(mesh_poly_data);

    // Simplify the mesh, mark corners, and validate the mesh
    mergeAllFaces(faces);
    markCorners(faces);
    let badFace = findBadFace(faces);
    if (badFace != null) {
        console.log('Bad Face Found!', badFace);
        console.log(badFace.rootEdge, badFace.rootEdge.next, badFace.rootEdge.next.next, badFace.rootEdge.next.next.next, badFace.rootEdge.next.next.next.next)
        return;
    }

    // Set the canvas dimensions to give room for the mesh
    let maxX = 0;
    let minY = 0;
    for(let face of faces) {
        currentEdge = face.rootEdge;
        do {
            maxX = Math.max(maxX, currentEdge.originPoint.x);
            minY = Math.min(minY, currentEdge.originPoint.y);
            currentEdge = currentEdge.next;
        } while (currentEdge != face.rootEdge);
    }
    setCanvasDimensions(Math.ceil(SCALE*(maxX+1)), Math.ceil(SCALE*(-minY+1)));

    // Constants
    let agents = [
        new Agent(new Point(5.5, -1.5), 5),
        new Agent(new Point(8.5, -1.5), 5),
        new Agent(new Point(10.5, -1.5), 5),
        new Agent(new Point(11.5, -1.5), 5),
        new Agent(new Point(12.5, -1.5), 5),
    ];
    let formation = new Formation();
    formation.addAgents(agents);
    agents = [];
    let lastClick = null; // Stores the Point clicked at, if a point was clicked at

    // Main Loop
    const FRAME_DELAY_RATIO = 1;
    let frame_count = 0;
    render = async function() {
        frame_count++;
        if (frame_count % FRAME_DELAY_RATIO != 0) {
            window.requestAnimationFrame(render);
            return;
        }
        // Update path code
        if (lastClick != null) {
            for(let agent of agents) {
                agent.pathfind(faces, lastClick);
            }
            await formation.pathfind(faces, lastClick);
        }
        // Move along path code
        for(let agent of agents) {
            agent.iterate(1/60);
        }
        await formation.iterate(1/60);
        // Render code
        const DRAW_AGENT_PATHING = false;
        const DRAW_DEBUG_TRACKS = false;
        const DRAW_RUNNING_COLOR = false;
        ctx.fillStyle = 'black';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        for(let face of faces) {
            drawFace(face, 'white');
        }
        for(let i = 0; i < formation.agents.length; i++) {
            let agent = formation.agents[i];
            let isRunning = false;
            if (agent.isInFormation) {
                isRunning = formation.getIdealSpeed(i, [...Array(formation.agents.length)].map(() => 1/60)) > formation.speed;
            }
            drawPoint(agent.position, (isRunning && DRAW_RUNNING_COLOR) ? 'lightgreen' : 'green');
            if (DRAW_AGENT_PATHING && agent.path.length > 0) {
                drawPath([agent.position, ...agent.path]);
            }
        }
        if (DRAW_DEBUG_TRACKS && formation.mainTrack != null) {
            drawPoint(formation.position, 'purple');
            let mainTrack = formation.mainTrack;
            for(let i = 0; i < mainTrack.length; i++) {
                for(let j = 0; j < formation.agents.length; j++) {
                    let agent = formation.agents[j];
                    if (agent.isInFormation && agent.track[i] != null) {
                        drawPoint(agent.track[i] , 'green', 1);
                    }
                }
                drawPoint(mainTrack[i].p, 'blue', 1);
            }
        }
        // Queue next Render
        lastClick = null;
        window.requestAnimationFrame(render);
    }
    window.requestAnimationFrame(render);

    // Store click point
    canvas.onclick = async function(event) {
        const rect = canvas.getBoundingClientRect();
        const x = (event.clientX - rect.left)/SCALE + 0.1/SCALE;
        const y = -(event.clientY - rect.top)/SCALE + 0.1/SCALE;
        console.log('Clicked at: ', x, y);
        lastClick = new Point(x, y);
    };
}
main();
