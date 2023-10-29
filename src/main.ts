import { canvas, ctx, SCALE, setCanvasDimensions, getTransformedCoordinates, drawFace, drawPath, drawPoint } from "./graphics";
import { Point, EPSILON, mergeAllFaces, markCorners, findBadFace, faceFromPolyline, offsetFace } from "./math";
import { loadMesh, loadMaze, loadPolyData, traverableFacesFromObstacles } from "./utils";
import { Agent, Formation } from "./agent";
//import "source-map-support/register";

async function main() {
    const AGENT_RADIUS = 0.3;
    const MARGIN = 1; // Margin between the drawing region and the bounding Face
    //const faces = loadPolyData(polyData1);

    //const MAZE_URL = 'https://api.allorigins.win/raw?url=https://pastebin.com/raw/GWRCSyUp';
    //const mazePolyData = loadMaze(await getUrlContents(MAZE_URL));
    //const faces = loadPolyData(mazePolyData);

    // Obstacles as raw polylines
    function createSquareRawObstacle(pt: Point, dist: number) {
        return [
            [pt.x, pt.y - dist],
            [pt.x + dist, pt.y],
            [pt.x, pt.y + dist],
            [pt.x - dist, pt.y],
        ];
    }
    let loc = 25;
    let houseRadius = 2;
    let barracksRadius = 5;
    const rawObstacles: number[][][] = [
        //[[10, -10], [10, -20], [20, -20], [20, -10]],
        //[[30, -30], [30, -40], [60, -40], [60, -30]],
        //[[21, -10], [30, -20], [60, -15], [40, -5]],
        createSquareRawObstacle(new Point(loc+0.5/5*barracksRadius, -loc+0.5/5*barracksRadius), barracksRadius),
        createSquareRawObstacle(new Point(loc+4/5*barracksRadius, -loc+8/5*barracksRadius), barracksRadius),
        createSquareRawObstacle(new Point(loc+(4+3.1)/5*barracksRadius, -loc+(8+7)/5*barracksRadius), barracksRadius),
        createSquareRawObstacle(new Point(loc-3/5*barracksRadius, -loc+(15)/5*barracksRadius), houseRadius),
        createSquareRawObstacle(new Point(loc-9/5*barracksRadius, -loc+(19)/5*barracksRadius), houseRadius),
    ];
    // Convert into obstacle faces

    let minX = 0;
    let maxY = 0;
    let maxX = 45;
    let minY = -45;
    let boundingFace = faceFromPolyline([[minX, maxY], [minX, minY], [maxX, minY], [maxX, maxY]].map(pt => new Point(pt[0], pt[1])));
    let obstacleFaces = rawObstacles.map(polyline => faceFromPolyline(polyline.map(pt => new Point(pt[0], pt[1]))));

    let traversableFaces = traverableFacesFromObstacles(boundingFace, obstacleFaces);
    // TODO: Cut facets into acute vertices during offset, so that 0.99/sqrt(2) is guaranteed to be safe
    const OBSTACLE_OFFSET = AGENT_RADIUS*0.99/Math.sqrt(2);
    let offsetTraversableFaces = traverableFacesFromObstacles(
        offsetFace(boundingFace, -OBSTACLE_OFFSET),
        obstacleFaces.map(face => offsetFace(face, OBSTACLE_OFFSET)),
    );

    //const meshPolyData = await loadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena-merged.mesh');
    //let meshPolyData = await loadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena.mesh');
    //const meshPolyData = await loadMesh('https://api.allorigins.win/raw?url=https://pastebin.com/raw/DdgmNAT3');
    //let traversableFaces = loadPolyData(meshPolyData);

    // Simplify the mesh, mark corners, and validate the mesh
    mergeAllFaces(traversableFaces);
    mergeAllFaces(offsetTraversableFaces);
    markCorners(offsetTraversableFaces);
    let badFace = findBadFace(offsetTraversableFaces);
    if (badFace != null) {
        console.log('Bad Face Found!', badFace);
        console.log(badFace.rootEdge, badFace.rootEdge.next, badFace.rootEdge.next.next, badFace.rootEdge.next.next.next, badFace.rootEdge.next.next.next.next);
        return;
    }

    // Set the canvas dimensions based on the mesh and the MARGIN
    setCanvasDimensions(Math.ceil(SCALE*(maxX+2*MARGIN)), Math.ceil(SCALE*(-minY+2*MARGIN)));

    // Constants
    let agentSpeed = 2;
    let agents = [
        new Agent(new Point(8.5, -1.5), AGENT_RADIUS, agentSpeed),
        new Agent(new Point(9.5, -1.5), AGENT_RADIUS, agentSpeed),
        new Agent(new Point(10.5, -1.5), AGENT_RADIUS, agentSpeed),
        new Agent(new Point(11.5, -1.5), AGENT_RADIUS, agentSpeed),
        new Agent(new Point(12.5, -1.5), AGENT_RADIUS, agentSpeed),
    ];
    if (true) {
        agents.push(...[
            new Agent(new Point(8.5, -2.5), AGENT_RADIUS, agentSpeed),
            new Agent(new Point(9.5, -2.5), AGENT_RADIUS, agentSpeed),
            new Agent(new Point(10.5, -2.5), AGENT_RADIUS, agentSpeed),
            new Agent(new Point(11.5, -2.5), AGENT_RADIUS, agentSpeed),
            //new Agent(new Point(12.5, -2.5), AGENT_RADIUS, agentSpeed),
        ]);
    }
    let formation = new Formation(boundingFace, obstacleFaces, offsetTraversableFaces);
    formation.addAgents(agents);
    agents = [
        new Agent(new Point(10.5, -10.5), AGENT_RADIUS, agentSpeed),
        new Agent(new Point(11.5, -15.5), AGENT_RADIUS, agentSpeed),
    ];
    for(let agent of agents) {
        await agent.setFinalTarget(offsetTraversableFaces, agent.position);
    }
    let lastClick: Point | null = null; // Stores the Point clicked at, if a point was clicked at
    let keysHeld: string[] = []; // Stores the keys pressed
    let lastKeysHeld: string[] = []; // Stores the keys pressed
    function isKeyHeld(key: string) {
        return keysHeld.indexOf(key) > -1;
    }
    function isKeyPressed(key: string) {
        return keysHeld.indexOf(key) > -1 && lastKeysHeld.indexOf(key) == -1;
    }

    // Main Loop
    let DRAW_DEBUG_TRACKS = true;
    let DRAW_AGENT_PATHING = true;
    let DRAW_RUNNING_COLOR = true;
    
    const TARGET_FPS = 60;
    const FRAME_DELAY_RATIO = 1;
    const ISOMETRIC_VIEW = false;
    let frameCount = 0;

    let translate = [MARGIN*SCALE, MARGIN*SCALE];
    let zoom = 1;

    let fps = 1/TARGET_FPS;
    let render = async function() {
        frameCount++;
        if (frameCount % FRAME_DELAY_RATIO != 0) {
            window.requestAnimationFrame(render);
            return;
        }

        // User Input

        let scrollSpeed = 5;
        if (isKeyHeld('ShiftLeft') || isKeyHeld('ShiftRight')) {
            scrollSpeed *= 2;
        }
        if (isKeyHeld('KeyA')) {
            translate[0] += scrollSpeed;
        }
        if (isKeyHeld('KeyD')) {
            translate[0] -= scrollSpeed;
        }
        if (isKeyHeld('KeyW')) {
            translate[1] += scrollSpeed;
        }
        if (isKeyHeld('KeyS')) {
            translate[1] -= scrollSpeed;
        }
        if (isKeyHeld('KeyN')) {
            zoom *= 0.975;
        }
        if (isKeyHeld('KeyM')) {
            zoom /= 0.975;
        }
        zoom = Math.min(Math.max(zoom, 0.5), 6);
        if (isKeyPressed('KeyR')) {
            formation.stop();
        }
        if (isKeyPressed('KeyV')) {
            if (DRAW_DEBUG_TRACKS) {
                DRAW_DEBUG_TRACKS = false;
                DRAW_AGENT_PATHING = false;
                DRAW_RUNNING_COLOR = false;
            } else {
                DRAW_DEBUG_TRACKS = true;
                DRAW_AGENT_PATHING = true;
                DRAW_RUNNING_COLOR = true;
            }
        }
        lastKeysHeld = keysHeld.slice();

        // Initial Rendering Setup

        const rect = canvas.getBoundingClientRect();
        // Reset and Blank screen
        ctx.resetTransform();
        ctx.fillStyle = "black";
        ctx.fillRect(0, 0, rect.width, rect.height);
        // Set the projection matrix
        ctx.translate(rect.width/2, rect.height/2);
        ctx.scale(zoom, zoom);
        ctx.translate(-rect.width/2, -rect.height/2);
        ctx.translate(translate[0], translate[1]);
        if (ISOMETRIC_VIEW) {
            ctx.translate(rect.width/2, rect.height/2);
            ctx.scale(1, Math.sqrt(2)/2);
            ctx.rotate(Math.PI / 4);
            ctx.translate(-rect.width/2, -rect.height/2);
        }
        for(let face of traversableFaces) {
            drawFace(face, 'white');
        }

        // Iteration and Logic
        let startLogicTimer = performance.now();

        // Update path code
        if (lastClick != null) {
            await formation.pathfind(lastClick);
        }
        // Iterate all agents
        let deltaTime = 1/TARGET_FPS;
        let allAgentsInFormation = formation.formationAgents.map(formationAgent => formationAgent.agent);
        for(let i = 0; i < agents.length; i++) {
            let agent = agents[i];
            let otherAgents = agents.slice();
            otherAgents.splice(i, 1);
            otherAgents = otherAgents.concat(allAgentsInFormation);
            await agent.considerNeighboringAgents(boundingFace, obstacleFaces, offsetTraversableFaces, otherAgents, deltaTime);
        }
        await formation.considerNeighboringAgents(agents, deltaTime);
        for(let agent of agents) {
            agent.iterate(deltaTime);
        }
        // TODO: Store formation agents in the same datastructure, so that all agents can ORCA
        await formation.iterate(deltaTime);
        let endLogicTimer = performance.now();

        // Rendering code
        for(let agent of agents) {
            drawPoint(agent.position, 'red', agent.radius);
            if (DRAW_AGENT_PATHING && agent.path.length > 0) {
                drawPath([agent.position, ...agent.path], 'purple');
            }
        }
        for(let i = 0; i < formation.formationAgents.length; i++) {
            let formationAgent = formation.formationAgents[i];
            let agent = formationAgent.agent;
            let isRunning = false;
            if (formationAgent.isInFormation && agent.prevVel !== undefined) {
                isRunning = agent.prevVel.magnitude() > formation.speed + EPSILON;
            }
            drawPoint(agent.position, (isRunning && DRAW_RUNNING_COLOR) ? 'lightgreen' : 'green', agent.radius);
            if (DRAW_AGENT_PATHING && agent.path.length > 0) {
                drawPath([agent.position, ...agent.path], 'purple');
            }
        }
        if (DRAW_DEBUG_TRACKS && formation.mainTrack != null) {
            drawPoint(formation.position, 'purple', 0.2, true);
            let mainTrack = formation.mainTrack;
            // For each point along the main track,
            for(let i = 0; i < mainTrack.length; i++) {
                // Track the agent's track points for that i
                for(let j = 0; j < formation.formationAgents.length; j++) {
                    let formationAgent = formation.formationAgents[j];
                    if (formationAgent.isInFormation) {
                        let secondaryPts = formationAgent.trackManager!.secondaryPts[i];
                        if (secondaryPts != null) {
                            for(let pt of secondaryPts) {
                                drawPoint(pt, 'green', 0.1, true);
                            }
                        } else {
                            let pt = formationAgent.trackManager!.pts[i];
                            if (pt != null) {
                                drawPoint(pt, 'green', 0.1, true);
                            }
                        }
                    }
                }
                // And, draw the main track point
                drawPoint(mainTrack[i].p, 'blue', 0.1, true);
            }
        }

        // Queue next Render
        lastClick = null;
        fps = 0.9*fps + 0.1*Math.min(1000, 1000/(endLogicTimer-startLogicTimer+EPSILON));
        if (frameCount % TARGET_FPS == 0) {
            //console.log("FPS:", fps);
        }
        window.requestAnimationFrame(render);
    }
    window.requestAnimationFrame(render);

    // Store click point
    canvas.onclick = async function(event) {
        const rect = canvas.getBoundingClientRect();
        // Get positive X, Y w.r.t the top-left corner of the canvas
        let originX = event.clientX - rect.left;
        let originY = event.clientY - rect.top;
        // Transform based on canvas projection matrix
        let transformed = getTransformedCoordinates(originX, originY);
        originX = transformed.x;
        originY = transformed.y;
        // Transform into abstract value
        let x = originX/SCALE + 0.1/SCALE;
        let y = -originY/SCALE + 0.1/SCALE;
        console.log('Clicked at: ', x, y);
        lastClick = new Point(x, y);
    };

    // Keypress Logic
    document.addEventListener('keydown', (event) => {
        // Add the key to the keysHeld array if it's not already there
        if (!keysHeld.includes(event.code)) {
            keysHeld.push(event.code);
        }
    });
    document.addEventListener('keyup', (event) => {
        // Remove the key from the keysHeld array
        const index = keysHeld.indexOf(event.code);
        if (index > -1) {
            keysHeld.splice(index, 1);
        }
    });
}

main();
