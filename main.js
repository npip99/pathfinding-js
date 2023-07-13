class Agent {
    position;
    speed;
    path = [];
    constructor(position, speed) {
        this.position = position;
        this.speed = speed;
    }
    async pathfind(faces, dst) {
        let result = await polyanya(faces, this.position, dst);
        if (result != null) {
            this.path = result['path'];
        }
    }
    iterate(deltaTime, speed=undefined) {
        if (speed === undefined) {
            speed = this.speed;
        }
        if (this.path.length > 0) {
            let nextWaypoint = this.path[0];
            let totalDist = getPointDist(this.position, nextWaypoint);
            let direction = nextWaypoint.minus(this.position);
            let maxMovementDist = speed * deltaTime;
            if (maxMovementDist >= totalDist) {
                this.position = nextWaypoint;
                this.path.splice(0, 1);
            } else {
                this.position = this.position.plus(direction.multiply(maxMovementDist/totalDist));
            }
        }
    }
}

class Formation {
    agents = [];
    relative_positions = [];
    position;
    speed;
    targetPosition;
    targetOrientation;
    constructor() {
    }
    addAgents(agents) {
        this.agents.push(...agents);
        this.generateFormation();
    }
    generateFormation() {
        this.position = this.agents[0].position;
        this.speed = null;
        this.relative_positions = [];
        for(let i = 0; i < this.agents.length; i++) {
            console.log(this.agents[i], this.agents);
            this.relative_positions.push(this.agents[i].position.minus(this.position));
            if (this.speed == null || this.agents[i].speed < this.speed) {
                this.speed = this.agents[i].speed;
            }
        }
    }
    async pathfind(faces, dst) {
        for(let i = 0; i < this.agents.length; i++) {
            await this.agents[i].pathfind(faces, dst.plus(this.relative_positions[i]));
        }
    }
    iterate(deltaTime) {
        for(let i = 0; i < this.agents.length; i++) {
            this.agents[i].iterate(deltaTime, this.speed);
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
    ];
    let formation = new Formation();
    formation.addAgents(agents);
    agents = [];
    let lastClick = null; // Stores the Point clicked at, if a point was clicked at

    // Main Loop
    render = async function() {
        // Update path code
        if (lastClick != null) {
            for(let agent of agents) {
                agent.pathfind(faces, lastClick);
            }
            formation.pathfind(faces, lastClick);
        }
        // Move along path code
        for(let agent of agents) {
            agent.iterate(1/60);
        }
        formation.iterate(1/60);
        // Render code
        ctx.fillStyle = 'black';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        for(let face of faces) {
            drawFace(face, 'white');
        }
        for(let agent of formation.agents) {
            drawPoint(agent.position, 'green');
            if (agent.path.length > 0) {
                drawPath([agent.position, ...agent.path]);
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
