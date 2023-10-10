import {Point, HalfEdge, Face} from "./math";

/*
====================
Helper Utilities
====================
*/

async function getUrlContents(url: string) {
    let result = await fetch(url)
        .then(response => response.text())
        .catch(error => {
            console.error('Error:', error);
            return null;
        });
    if (result === null) {
        return null;
    }
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
export const polyData1 = [
    [[0, 0], [0, 70], [45, 100], [120, 25], [120, 0]],
    [[120, 25], [45, 100], [45, 120], [90, 170], [190, 120], [290, 25], [190, 25]],
    [[45, 120], [20, 120], [20, 145], [45, 170], [90, 170]],
    [[290, 25], [190, 120], [190, 200], [310, 150], [310, 100]],
    [[290, 25], [310, 100], [360, 100], [340, 25]],
    [[340, 25], [360, 100], [360, 0], [340, 0]],
    [[190, 25], [290, 25], [290, 0], [190, 0]],
    [[190, 200], [340, 200], [340, 150], [310, 150]],
];
export const polyData2 = [
    [[0, 0], [0, 70], [45, 100], [120, 25], [120, 0]],
    [[120, 25], [45, 100], [45, 120], [90, 170], [190, 120], [290, 25], [190, 25]],
    [[45, 120], [20, 120], [20, 145], [45, 170], [90, 170]],
    [[290, 25], [190, 120], [230, 150], [300, 160], [310, 150], [310, 100]],
    [[290, 25], [310, 100], [360, 100], [340, 25]],
    [[340, 25], [360, 100], [360, 0], [340, 0]],
    [[190, 25], [290, 25], [290, 0], [190, 0]],
    [[300, 160], [340, 200], [340, 150], [310, 150]],
];
export const polyData3 = [
    [[10, 10], [20, 20], [20, 10]],
    [[10, 30], [30, 20], [25, 20], [20, 20]],
    [[25, 10], [25, 20], [30, 20], [40, 20], [40, 10]],
    [[30, 20], [40, 30], [50, 20], [40, 20]],
    [[50, 10], [50, 20], [60, 10]],
    [[60, 10], [50, 20], [40, 30], [60, 30]],
    [[10, 30], [20, 20], [10, 10]],
];

// Return a list of faces, from raw poly data
export function loadPolyData(polyData: number[][][]) {
    let faces: Face[] = [];
    let pointHashes = new Map<number, Point>();
    let halfedgeHashes = new Map<number, HalfEdge>();
    for(let polyDatum of polyData) {
        let halfedges: HalfEdge[] = [];
        for(let i = 0; i < polyDatum.length; i++) {
            let rawU = polyDatum[i];
            let p = new Point(rawU[0], -rawU[1], false);
            let u = pointHashes.get(p.hash());
            if (u === undefined) {
                pointHashes.set(p.hash(), p);
                u = p;
            }

            let halfedge = new HalfEdge(u);
            halfedges.push(halfedge);
        }

        for(let i = 0; i < halfedges.length; i++) {
            // Set next for halfedges[i]
            halfedges[i].next = halfedges[(i+1)%halfedges.length];
            // Setting the HalfEdge prev: halfedges[(i+1)%halfedges.length].prev = halfedges[i];
            // Store this halfedge
            halfedgeHashes.set((new Point(halfedges[i].originPoint.hash(), halfedges[i].next.originPoint.hash())).hash(), halfedges[i]);

            // Look for opposite twin and set twin if found
            let twinHash = (new Point(halfedges[i].next.originPoint.hash(), halfedges[i].originPoint.hash())).hash();
            let twinHalfedge = halfedgeHashes.get(twinHash)
            if (twinHalfedge !== undefined) {
                twinHalfedge.twin = halfedges[i];
                halfedges[i].twin = twinHalfedge;
            }
        }

        let face = new Face(halfedges[0], true);
        faces.push(face);
    }

    return faces;
}

export function loadMaze(mazeString: string, maxSize: number) {
    mazeString = mazeString.trim();
    let currentRowIndex = -1;
    let maze: boolean[][] = [];
    let currentRow: boolean[] = [];
    for(let c of mazeString) {
        if (c == '\n') {
            maze.push(currentRow);
            currentRowIndex++;
            currentRow = [];
        } else {
            currentRow.push(c == '.' ? true : false);
        }
    }

    let polyData: number[][][] = [];
    for(let r = 0; r < maze.length; r++) {
        for(let c = 0; c < maze[r].length; c++) {
            if (maze[r][c] && (maxSize === undefined || (r <= maxSize && c <= maxSize))) {
                polyData.push([
                    [c, r],
                    [c, r+1],
                    [c+1, r+1],
                    [c+1, r],
                ]);
            }
        }
    }
    return polyData;
}

// Mesh must be loaded from url, as its too large of a string
export async function loadMesh(url: string) {
    let result = await getUrlContents(url);
    if (result === null) {
        return result;
    }
    let lines = result.split('\n');
    if (lines[0] != 'mesh' || lines[1] != '2') {
        return null;
    }
    let vertsPolys = lines[2].split(' ');
    let numVerts = Number(vertsPolys[0]);
    let numPolys = Number(vertsPolys[1]);
    let verts: number[][] = [];
    for(let i = 0; i < numVerts; i++) {
        let vertLine = lines[3+i];
        let splitVerts = vertLine.split(' ');
        verts.push([Number(splitVerts[0]), Number(splitVerts[1])]);
    }
    let polys: number[][][] = [];
    for(let i = 0; i < numPolys; i++) {
        let poly: number[][] = [];
        let polyData = lines[3+numVerts+i].split(' ').map(Number);
        let numPolyVerts = polyData[0];
        for(let i = 0; i < numPolyVerts; i++) {
            let polyVertIndex = polyData[1+i];
            poly.push(verts[polyVertIndex]);
        }
        poly.reverse();
        polys.push(poly);
    }
    return polys;
}