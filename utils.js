
/*
====================
Helper Utilities
====================
*/

async function getUrlContents(url) {
    let result = await fetch(url)
        .then(response => response.text())
        .catch(error => {
            console.error('Error:', error);
        });
    if (url.includes('pastebin')) {
        result = result.replace(/\r/g, '');
    }
    return result;
}

/*
====================
Mesh Loading
====================
*/

// Example Poly Data
const poly_data_1 = [
    [[0, 0], [0, 70], [45, 100], [120, 25], [120, 0]],
    [[120, 25], [45, 100], [45, 120], [90, 170], [190, 120], [290, 25], [190, 25]],
    [[45, 120], [20, 120], [20, 145], [45, 170], [90, 170]],
    [[290, 25], [190, 120], [190, 200], [310, 150], [310, 100]],
    [[290, 25], [310, 100], [360, 100], [340, 25]],
    [[340, 25], [360, 100], [360, 0], [340, 0]],
    [[190, 25], [290, 25], [290, 0], [190, 0]],
    [[190, 200], [340, 200], [340, 150], [310, 150]],
];
const poly_data_2 = [
    [[0, 0], [0, 70], [45, 100], [120, 25], [120, 0]],
    [[120, 25], [45, 100], [45, 120], [90, 170], [190, 120], [290, 25], [190, 25]],
    [[45, 120], [20, 120], [20, 145], [45, 170], [90, 170]],
    [[290, 25], [190, 120], [230, 150], [300, 160], [310, 150], [310, 100]],
    [[290, 25], [310, 100], [360, 100], [340, 25]],
    [[340, 25], [360, 100], [360, 0], [340, 0]],
    [[190, 25], [290, 25], [290, 0], [190, 0]],
    [[300, 160], [340, 200], [340, 150], [310, 150]],
];
const poly_data_3 = [
    [[10, 10], [20, 20], [20, 10]],
    [[10, 30], [30, 20], [25, 20], [20, 20]],
    [[25, 10], [25, 20], [30, 20], [40, 20], [40, 10]],
    [[30, 20], [40, 30], [50, 20], [40, 20]],
    [[50, 10], [50, 20], [60, 10]],
    [[60, 10], [50, 20], [40, 30], [60, 30]],
    [[10, 30], [20, 20], [10, 10]],
];

// Return a list of faces, from raw poly data
function LoadPolyData(poly_data) {
    let faces = [];
    let point_hashes_to_point = {}
    let halfedge_hashes_to_halfedge = {}
    for(let poly_datum of poly_data) {
        let halfedges = [];
        for(let i = 0; i < poly_datum.length; i++) {
            let u = poly_datum[i];
            let p = new Point(u[0], -u[1], false);
            if (point_hashes_to_point[p.hash()] === undefined) {
                point_hashes_to_point[p.hash()] = p;
            }
            u = point_hashes_to_point[p.hash()];

            let halfedge = new HalfEdge(u);
            halfedges.push(halfedge);
        }

        for(let i = 0; i < halfedges.length; i++) {
            // Set next for halfedges[i]
            halfedges[i].next = halfedges[(i+1)%halfedges.length];
            // Setting the HalfEdge prev: halfedges[(i+1)%halfedges.length].prev = halfedges[i];
            // Store this halfedge
            halfedge_hashes_to_halfedge[halfedges[i].hash()] = halfedges[i];

            // Look for opposite twin and set twin if found
            let opposite_hash = halfedges[i].next.originPoint.hash()*1300*1300 + halfedges[i].originPoint.hash();
            if (halfedge_hashes_to_halfedge[opposite_hash] !== undefined) {
                halfedge_hashes_to_halfedge[opposite_hash].twin = halfedges[i];
                halfedges[i].twin = halfedge_hashes_to_halfedge[opposite_hash];
            }
        }

        let face = new Face(halfedges[0], true);
        faces.push(face);
    }

    return faces;
}

function LoadMaze(maze_string, max_size) {
    maze_string = maze_string.trim();
    let currentRowIndex = -1;
    let maze = [];
    let currentRow = [];
    for(let c of maze_string) {
        if (c == '\n') {
            maze.push(currentRow);
            currentRowIndex++;
            currentRow = [];
        } else {
            currentRow.push(c == '.' ? true : false);
        }
    }

    let poly_data = [];
    for(let r = 0; r < maze.length; r++) {
        for(let c = 0; c < maze[r].length; c++) {
            if (maze[r][c] && (max_size === undefined || (r <= max_size && c <= max_size))) {
                poly_data.push([
                    [c, r],
                    [c, r+1],
                    [c+1, r+1],
                    [c+1, r],
                ]);
            }
        }
    }
    return poly_data;
}

// Mesh must be loaded from url, as its too large of a string
async function LoadMesh(url) {
    let result = await getUrlContents(url);
    let lines = result.split('\n');
    if (lines[0] != 'mesh' || lines[1] != '2') {
        return null;
    }
    verts_polys = lines[2].split(' ');
    num_verts = Number(verts_polys[0]);
    num_polys = Number(verts_polys[1]);
    let verts = [];
    for(let i = 0; i < num_verts; i++) {
        let vert_line = lines[3+i];
        let s_verts = vert_line.split(' ');
        verts.push([Number(s_verts[0]), Number(s_verts[1])]);
    }
    let polys = [];
    for(let i = 0; i < num_polys; i++) {
        let poly = [];
        let poly_data = lines[3+num_verts+i].split(' ');
        for(let i = 0; i < poly_data.length; i++) {
            poly_data[i] = Number(poly_data[i]);
        }
        let num_poly_verts = poly_data[0];
        for(let i = 0; i < num_poly_verts; i++) {
            let poly_vert_index = poly_data[1+i];
            poly.push(verts[poly_vert_index]);
        }
        poly.reverse();
        polys.push(poly);
    }
    return polys;
}