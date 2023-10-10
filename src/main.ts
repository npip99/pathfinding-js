import { canvas, ctx, SCALE, setCanvasDimensions, getTransformedCoordinates, drawFace, drawPath, drawPoint } from "./graphics";
import { Point, EPSILON, mergeAllFaces, markCorners, findBadFace } from "./math";
import { loadMesh, loadPolyData } from "./utils";
import { Agent, Formation } from "./agent";

async function main() {
    //const faces = loadPolyData(polyData1);

    //const MAZE_URL = 'https://api.allorigins.win/raw?url=https://pastebin.com/raw/GWRCSyUp';
    //const mazePolyData = loadMaze(await getUrlContents(MAZE_URL));
    //const faces = loadPolyData(mazePolyData);

    //const meshPolyData = await loadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena-merged.mesh');
    const meshPolyData = await loadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena.mesh');
    // TODO: Make assert/! cleaner
    console.assert(meshPolyData != null);
    //const meshPolyData = await loadMesh('https://api.allorigins.win/raw?url=https://pastebin.com/raw/DdgmNAT3');
    const faces = loadPolyData(meshPolyData!);

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
        let currentEdge = face.rootEdge;
        do {
            maxX = Math.max(maxX, currentEdge.originPoint.x);
            minY = Math.min(minY, currentEdge.originPoint.y);
            currentEdge = currentEdge.next;
        } while (currentEdge != face.rootEdge);
    }
    setCanvasDimensions(Math.ceil(SCALE*(maxX+1)), Math.ceil(SCALE*(-minY+1)));

    // Constants
    let agentSpeed = 2;
    let agents = [
        new Agent(new Point(5.5, -1.5), agentSpeed, faces),
        new Agent(new Point(8.5, -1.5), agentSpeed, faces),
        new Agent(new Point(10.5, -1.5), agentSpeed, faces),
        new Agent(new Point(11.5, -1.5), agentSpeed, faces),
        new Agent(new Point(12.5, -1.5), agentSpeed, faces),
    ];
    let formation = new Formation(faces);
    formation.addAgents(agents);
    agents = [];
    let lastClick: Point | null = null; // Stores the Point clicked at, if a point was clicked at
    let keysPressed: string[] = []; // Stores the keys pressed
    function isKeyPressed(key: string) {
        return keysPressed.indexOf(key) > -1;
    }

    // Main Loop
    const FRAME_DELAY_RATIO = 1;
    const ISOMETRIC_VIEW = false;
    let frameCount = 0;

    let translate = [0, 0];
    let zoom = 1;

    let render = async function() {
        frameCount++;
        if (frameCount % FRAME_DELAY_RATIO != 0) {
            window.requestAnimationFrame(render);
            return;
        }

        // User Input

        let scrollSpeed = 5;
        if (isKeyPressed('ShiftLeft') || isKeyPressed('ShiftRight')) {
            scrollSpeed *= 2;
        }
        if (isKeyPressed('KeyA')) {
            translate[0] += scrollSpeed;
        }
        if (isKeyPressed('KeyD')) {
            translate[0] -= scrollSpeed;
        }
        if (isKeyPressed('KeyW')) {
            translate[1] += scrollSpeed;
        }
        if (isKeyPressed('KeyS')) {
            translate[1] -= scrollSpeed;
        }
        if (isKeyPressed('KeyN')) {
            zoom *= 0.975;
        }
        if (isKeyPressed('KeyM')) {
            zoom /= 0.975;
        }
        zoom = Math.min(Math.max(zoom, 0.5), 3);
        if (isKeyPressed('KeyR')) {
            formation.stop();
        }

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

        // Iteration and Rendering

        // Update path code
        if (lastClick != null) {
            for(let agent of agents) {
                await agent.setWaypoints([lastClick]);
            }
            await formation.pathfind(lastClick);
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
            if (formation.getAgentFormationData(agent)!.isInFormation && agent.prevVel !== undefined) {
                isRunning = agent.prevVel.magnitude() > formation.speed + EPSILON;
            }
            drawPoint(agent.position, (isRunning && DRAW_RUNNING_COLOR) ? 'lightgreen' : 'green');
            if (DRAW_AGENT_PATHING && agent.path.length > 0) {
                drawPath([agent.position, ...agent.path]);
            }
        }
        if (DRAW_DEBUG_TRACKS && formation.mainTrack != null) {
            drawPoint(formation.position, 'purple');
            let mainTrack = formation.mainTrack;
            // For each point along the main track,
            for(let i = 0; i < mainTrack.length; i++) {
                // Track the agent's track points for that i
                for(let j = 0; j < formation.agents.length; j++) {
                    let agent = formation.agents[j];
                    let agentFormationData = formation.getAgentFormationData(agent)!;
                    if (agentFormationData.isInFormation) {
                        let pt = agentFormationData.track[i];
                        if (pt != null) {
                            drawPoint(pt, 'green', 1);
                        }
                    }
                }
                // And, draw the main track point
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
        // Add the key to the keysPressed array if it's not already there
        if (!keysPressed.includes(event.code)) {
            keysPressed.push(event.code);
        }
    });
    document.addEventListener('keyup', (event) => {
        // Remove the key from the keysPressed array
        const index = keysPressed.indexOf(event.code);
        if (index > -1) {
            keysPressed.splice(index, 1);
        }
    });
}

main();
