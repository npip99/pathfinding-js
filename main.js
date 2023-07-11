async function main() {
    //const faces = LoadPolyData(poly_data_1);

    const MAZE_URL = 'https://api.allorigins.win/raw?url=https://pastebin.com/raw/GWRCSyUp';
    const maze_poly_data = LoadMaze(await getUrlContents(MAZE_URL));
    const faces = LoadPolyData(maze_poly_data);

    //const mesh_poly_data = await LoadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena-merged.mesh');
    //const mesh_poly_data = await LoadMesh('https://raw.githubusercontent.com/vleue/polyanya/main/meshes/arena.mesh');
    //const mesh_poly_data = await LoadMesh('https://api.allorigins.win/raw?url=https://pastebin.com/raw/DdgmNAT3');
    //const faces = LoadPolyData(mesh_poly_data);

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
    const SPEED = 5;
    let currentLocation = new Point(1.5, -1.5);
    let currentPath = [];
    let lastClick = null; // Stores the Point clicked at, if a point was clicked at

    // Main Loop
    render = async function() {
        // Update path code
        if (lastClick != null) {
            let src = currentLocation;
            let dst = lastClick;
            let result = await polyanya(faces, src, dst);
            if (result != null) {
                currentPath = result['path'];
            }
        }
        // Move along path code
        if (currentPath.length > 0) {
            let nextWaypoint = currentPath[0];
            let totalDist = getPointDist(currentLocation, nextWaypoint);
            let direction = nextWaypoint.subtract(currentLocation);
            let maxMovementDist = SPEED / 60;
            if (maxMovementDist >= totalDist) {
                currentLocation = nextWaypoint;
                currentPath.splice(0, 1);
            } else {
                currentLocation = currentLocation.plus(direction.multiply(maxMovementDist/totalDist));
            }
        }
        // Render code
        ctx.fillStyle = 'black';
        ctx.fillRect(0, 0, canvas.width, canvas.height);
        for(let face of faces) {
            drawFace(face, 'white');
        }
        drawPoint(currentLocation, 'green');
        if (currentPath.length > 0) {
            drawPoint(currentPath[currentPath.length-1], 'purple');
            drawPath([currentLocation, ...currentPath]);
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
