import { PriorityQueue } from "data-structure-typed";

/*
====================
Data Structures
====================
*/

export const EPSILON = 1e-13;

// 32-bit FNV-1a hash on an arbitrary buffer
export function FNVHash(buffer: ArrayBuffer) {
    const byteView = new Uint8Array(buffer);

    let h = 2166136261;
    byteView.forEach(byte => {
        h ^= byte;
        h = Math.imul(h, 16777619);
    });
    return h;
}

export class Point {
    x: number;
    y: number;
    isCorner: boolean | undefined;
    constructor(x: number, y: number, isCorner?: boolean) {
        this.x = x;
        this.y = y;
        this.isCorner = isCorner;
    }
    copy() {
        return new Point(this.x, this.y, this.isCorner);
    }
    hash() {
        const buffer = new ArrayBuffer(16);
        const floatView = new Float64Array(buffer);
        floatView[0] = this.x;
        floatView[1] = this.y;
        return FNVHash(buffer);
    }
    plus(other: Point) {
        return new Point(this.x+other.x, this.y+other.y);
    }
    minus(other: Point) {
        return new Point(this.x-other.x, this.y-other.y);
    }
    multiply(factor: number) {
        return new Point(this.x*factor, this.y*factor);
    }
    divide(factor: number) {
        return new Point(this.x/factor, this.y/factor);
    }
    dot(other: Point) {
        return this.x*other.x + this.y*other.y;
    }
    magnitude() {
        return Math.sqrt(this.x*this.x + this.y*this.y);
    }
    normalize() {
        return this.divide(this.magnitude());
    }
}

// f=0 => p1, f=1 => p2
export function lerp(p1: Point, p2: Point, f: number) {
    return p1.multiply(1-f).plus(p2.multiply(f));
}

export class HalfEdge {
    originPoint: Point;
    next: HalfEdge;
    face: Face | null;
    twin: HalfEdge | null;
    constructor(originPoint: Point) {
        this.originPoint = originPoint;
        this.next = this;
        this.face = null;
        this.twin = null;
    }
}

export class Face {
    rootEdge: HalfEdge; // Starting HalfEdge
    isNavigable: boolean; // Whether or not the Face is navigable
    // NOTE: Requires rootEdge to be circular
    constructor(rootEdge: HalfEdge, isNavigable: boolean) {
        this.rootEdge = rootEdge;
        this.isNavigable = isNavigable;
        // Set the face property of all edges
        let currentEdge = rootEdge;
        do {
            currentEdge = currentEdge.next;
            currentEdge.face = this;
        } while(currentEdge != rootEdge);
    }
    // Shoelace theorem
    getArea() {
        let result = 0;

        let currentEdge = this.rootEdge;
        do {
            let a = currentEdge.originPoint;
            let b = currentEdge.next.originPoint;
            result += a.x * b.y - a.y * b.x;
            currentEdge = currentEdge.next;
        } while(currentEdge != this.rootEdge);

        return result/2;
    }
}

/*
====================
Math Functions
====================
*/

export function getPointDist(p1: Point, p2: Point) {
    return p1.minus(p2).magnitude();
}

export function getIntersection(seg1A: Point, seg1B: Point, seg2A: Point, seg2B: Point) {
    const x1 = seg1A.x;
    const y1 = seg1A.y;
    const x2 = seg1B.x;
    const y2 = seg1B.y;
    const x3 = seg2A.x;
    const y3 = seg2A.y;
    const x4 = seg2B.x;
    const y4 = seg2B.y;
  
    const denominator = (x1 - x2) * (y3 - y4) - (y1 - y2) * (x3 - x4);
  
    if (denominator === 0) {
      // Lines are parallel or coincident, no intersection
      return null;
    }
  
    const intersectionX = ((x1 * y2 - y1 * x2) * (x3 - x4) - (x1 - x2) * (x3 * y4 - y3 * x4)) / denominator;
    const intersectionY = ((x1 * y2 - y1 * x2) * (y3 - y4) - (y1 - y2) * (x3 * y4 - y3 * x4)) / denominator;
  
    return new Point(intersectionX, intersectionY);
}

// Positive is CCW, Negative is CW. 0 is Degenerate
export function getTriangleSign(v1: Point, v2: Point, v3: Point) {
    return (v2.x - v1.x) * (v3.y - v1.y) - (v2.y - v1.y) * (v3.x - v1.x);
}

// https://stackoverflow.com/questions/217578/how-can-i-determine-whether-a-2d-point-is-within-a-polygon
// NOTE: This function only works with convex faces
// Return 1 if inside, 0 if on boundary, -1 if on outside
export function isPointInFace(face: Face, p: Point) {
    let count = 0;
    let currentEdge = face.rootEdge;

    let isCollinear = false;
    let isInsideEdge = false;
    do {
        let edgeSegment = [currentEdge.originPoint, currentEdge.next.originPoint];

        // If p is collinear with an Edge,
        // then check if it's on the boundary of at least one edge
        if (getTriangleSign(p, edgeSegment[0], edgeSegment[1]) == 0) {
            isCollinear = true;
            let minX = Math.min(edgeSegment[0].x, edgeSegment[1].x);
            let maxX = Math.max(edgeSegment[0].x, edgeSegment[1].x);
            let minY = Math.min(edgeSegment[0].y, edgeSegment[1].y);
            let maxY = Math.max(edgeSegment[0].y, edgeSegment[1].y);
            if (minX <= p.x && p.x <= maxX && minY <= p.y && p.y <= maxY) {
                isInsideEdge = true;
            }
        }

        if ( (edgeSegment[0].y > p.y) != (edgeSegment[1].y > p.y)
            && p.x < edgeSegment[0].x + (edgeSegment[1].x - edgeSegment[0].x) * (p.y - edgeSegment[0].y) / (edgeSegment[1].y - edgeSegment[0].y) ) {
            count++;
        }

        currentEdge = currentEdge.next;
    } while (currentEdge !== face.rootEdge);

    if (isCollinear) {
        return isInsideEdge ? 0 : -1;
    } else {
        return count % 2 == 1 ? 1 : -1;
    }
}

/*
====================
Mesh Functions
====================
*/

function makeConvex(faces: Face[]) {
    throw "Unimplemented Error";
    let newFaces: Face[] = [];
    for(let face of faces) {
        console.log('Handling Face:', face);
        let currentEdge = face.rootEdge;
        let i = 0;
        do {
            i++;
            if (i == 100) {
                console.error('BAD!');
                return;
            }
            // Do
            console.log(face, currentEdge);
            let e1 = currentEdge;
            let e2 = e1.next;
            let e3 = e2.next;
            if (getTriangleSign(e1.originPoint, e2.originPoint, e3.originPoint) <= 0) {
                let e4 = e3.next;
                // Insert Ein and make a triangle
                let Ein = new HalfEdge(e4.originPoint);
                Ein.next = e2;
                e3.next = Ein;
                let newFace = new Face(e2, true);
                //ctx.drawFace(newFace);
                // Insert Eout into the edgelist
                let Eout = new HalfEdge(e2.originPoint);
                e1.next = Eout;
                Eout.next = e4;
                // Mark the new E twin
                Ein.twin = Eout;
                Eout.twin = Ein;
                // Add the new face
                newFaces.push(newFace);
                // Update the root edge, since we may have just removed it
                face.rootEdge = e1;
            }

            // Iterate Next
            currentEdge = currentEdge.next;
        } while (currentEdge != face.rootEdge);
    }
    // Add all of the new faces into the face list
    for(let face of newFaces) {
        faces.push(face);
    }
}

// Badj     Badj
//   \       /
//    B theirB
//    ^    v
//    A theirA
//   /      \
// Aadj    Aadj
function canMergeTwoFaces(face: Face, edge: HalfEdge) {
    // NOTE: Infinite loop if edge isn't a loop
    function findPrev(edge: HalfEdge) {
        let currentEdge = edge;
        while (currentEdge.next != edge) {
            currentEdge = currentEdge.next;
        }
        return currentEdge;
    }

    // edge refers to <A, B>, where that's the edge that's getting merged
    // Required: edge must be somewhere in the linkedlist of face.rootEdge
    // Required: face.isNavigable == edge.twin.face.isNavigable
    let A = edge;
    // If there's no neighboring polygon, we exit
    if (A.twin == null) {
        return false;
    }
    let B = A.next;
    let theirA = A.twin.next;
    let theirB = A.twin;
    // While B still shares the face with A, iterate to the next B
    while(B.twin != null && A.twin.face == B.twin.face) {
        theirB = B.twin;
        B = B.next;
    }
    // Get A/B adj
    let ourAadj = findPrev(A);
    let ourBadj = B.next;
    let theirAadj = theirA.next;
    let theirBadj = findPrev(theirB);

    // Verify that convexity is held, if these triplets are each CCW
    return getTriangleSign(theirBadj.originPoint, B.originPoint, ourBadj.originPoint) >= 0
        && getTriangleSign(ourAadj.originPoint, A.originPoint, theirAadj.originPoint) >= 0;
}

function mergeTwoFaces(face: Face, edge: HalfEdge): Face {
    // TODO: Remove Debug Check, or make canMergeTwoFaces faster
    if (!canMergeTwoFaces(face, edge)) {
        console.error('Cannot Merge!', face, edge);
        throw 'Cannot Merge!';
    }

    // edge refers to <A, B>, where that's the edge that's getting merged
    // Required: edge must be somewhere in the linkedlist of face.rootEdge
    // Required: face.isNavigable == edge.twin.face.isNavigable
    let A = edge;
    let B = A.next;
    let theirA = A.twin!.next;
    let theirB = A.twin!;
    // While B still shares the face with A, iterate to the next B
    while(B.twin != null && A.twin!.face == B.twin.face) {
        theirB = B.twin;
        B = B.next;
    }
    let removedFace = theirB.face!;

    // Update A to now use the edge of <theirA, theirA.next>
    A.next = theirA.next;
    A.twin = theirA.twin;
    if (A.twin != null)
        A.twin.twin = A;
    // Update theirB to now use the edge of <B, B.next>
    theirB.next = B.next;
    theirB.twin = B.twin;
    if (theirB.twin != null)
        theirB.twin.twin = theirB;
    
    // Set the face of the their edges, to be our face
    let currentEdge = A.next;
    do {
        currentEdge.face = face;
        currentEdge = currentEdge.next;
        // Stop when we hit B.next, since we're back into our face
    } while (currentEdge !== B.next);

    // Make sure face.rootEdge can't be B (Which is dead now)
    face.rootEdge = A;

    // Return the removed face
    return removedFace;
}

function mergeCollinearEdges(faces: Face[]) {
    // Merge consecutive collinear edges
    for(let face of faces) {
        let merged = true;
        while (merged) {
            merged = false;
            // Check each half-edge of the current face
            let currentEdge = face.rootEdge;
            do {
                // If <e1, e2, e3> are collinear, and have the same neighbor,
                // Get rid of e2
                let edge1 = currentEdge;
                let edge2 = edge1.next;
                let edge3 = edge2.next;
                let shareNeighbor = (edge1.twin == null && edge2.twin == null) ||
                (edge1.twin != null && edge2.twin != null && edge1.twin.face == edge2.twin.face); 
                if (shareNeighbor && getTriangleSign(edge1.originPoint, edge2.originPoint, edge3.originPoint) == 0) {
                    edge1.next = edge3;
                    face.rootEdge = edge1;
                    // If the neighboring edge is not null,
                    let theirEdge3 = edge2.twin;
                    if (theirEdge3 != null) {
                        // Also remove otherEdge2 (i.e., theirEdge3.next)
                        theirEdge3.next = theirEdge3.next.next;
                        theirEdge3.face!.rootEdge = theirEdge3;
                        // Connect the twins
                        edge1.twin = theirEdge3;
                        theirEdge3.twin = edge1;
                    }
                    merged = true;
                    break;
                }
                currentEdge = currentEdge.next;
            } while (currentEdge !== face.rootEdge);
        }
    }
}

function mergeAllFacesNaive(faces: Face[]) {
    let merged = true;
    
    // Merge faces while still possible
    let lastI = 0;
    while (merged) {
        merged = false;

        // While not merged, try all the faces
        for (let i = lastI; i < faces.length && !merged; i++) {
            lastI = Math.max(lastI, i);
            let currentFace = faces[i];

            // Check each halfedge of the current face
            let currentEdge = currentFace.rootEdge;
            do {
                let processingEdge = currentEdge.next;
                // If processingEdge is the first edge to this face,
                let otherFaceNow = processingEdge.twin == null ? null : processingEdge.twin.face;
                let otherFacePrev = currentEdge.twin == null ? null : currentEdge.twin.face;
                if (otherFacePrev != otherFaceNow && canMergeTwoFaces(currentFace, processingEdge)) {
                    // Merge the faces and get the face that was removed
                    let removedFace = mergeTwoFaces(currentFace, processingEdge);
                    // Remove the removedFace,
                    faces.splice(faces.indexOf(removedFace), 1);
                    // And then break out
                    merged = true;
                    break;
                }
                currentEdge = currentEdge.next;
            } while (currentEdge !== currentFace.rootEdge);
        }
    }

    mergeCollinearEdges(faces);
}

export function mergeAllFaces(faces: Face[]) {
    // meshmerger.cpp:smart_merge

    // Tracks the last face push into pq,
    // So we can check if a pq element is fresh
    let faceToIndex = new Map();

    // priority-largest queue
    interface PQType {
        face: Face;
        edge: HalfEdge;
        totalArea: number;
        index: number;
    }
    let pq = new PriorityQueue<PQType>({comparator: (a, b) => b.totalArea - a.totalArea});

    function pushFace(face: Face) {
        let currentArea = face.getArea();
        let bestMergeArea = null;
        let bestMergeEdge = null;
        // Check each halfedge of the current face
        let currentEdge = face.rootEdge;
        do {
            let processingEdge = currentEdge.next;

            // If processingEdge is the first edge to this face, and we can merge
            let otherFaceNow = processingEdge.twin == null ? null : processingEdge.twin.face;
            let otherFacePrev = currentEdge.twin == null ? null : currentEdge.twin.face;
            if (otherFacePrev != otherFaceNow && canMergeTwoFaces(face, processingEdge)) {
                let newArea = currentArea + (otherFaceNow == null ? 0 : otherFaceNow.getArea());
                if (bestMergeArea == null || newArea > bestMergeArea) {
                    bestMergeArea = newArea;
                    bestMergeEdge = processingEdge;
                }
            }
            currentEdge = currentEdge.next;
        } while (currentEdge !== face.rootEdge);

        // Get the new index
        let nextFaceIndex = faceToIndex.get(face)+1;
        // Push to the pq, including the index
        faceToIndex.set(face, nextFaceIndex);
        if (bestMergeArea != null) {
            pq.add({
                face: face,
                edge: bestMergeEdge!,
                totalArea: bestMergeArea,
                index: nextFaceIndex,
            });
        }
    }

    // Initialize
    for(let face of faces) {
        faceToIndex.set(face, 0);
        pushFace(face);
    }
    
    // Merge faces while still possible
    while (!pq.isEmpty()) {
        // Get a top that hasn't been marked stale
        let top = pq.poll()!;
        if (top.index != faceToIndex.get(top.face)) {
            continue;
        }

        // get the mergeFace and mergeEdge
        let mergeFace = top.face;
        let mergeEdge = top.edge;

        // Merge
        let removedFace = mergeTwoFaces(mergeFace, mergeEdge);
        // Splice the removedFace out of the list
        faceToIndex.set(removedFace, -1);
        faces.splice(faces.indexOf(removedFace), 1);

        // Check each halfedge of the current face
        let currentEdge = mergeFace.rootEdge;
        do {
            let processingEdge = currentEdge.next;
            // If processingEdge is the first edge to this face,
            let otherFaceNow = processingEdge.twin == null ? null : processingEdge.twin.face;
            let otherFacePrev = currentEdge.twin == null ? null : currentEdge.twin.face;
            if (otherFacePrev != otherFaceNow && otherFaceNow != null) {
                pushFace(otherFaceNow);
            }
            currentEdge = currentEdge.next;
        } while (currentEdge !== mergeFace.rootEdge);

        pushFace(mergeFace);
    }

    mergeCollinearEdges(faces);
}

export function markCorners(faces: Face[]) {
    // Mark Corners
    for(let face of faces) {
        let currentEdge = face.rootEdge;
        do {
            let nextEdge = currentEdge.next;
            if (currentEdge.twin == null) {
                let potentialCorner = nextEdge.originPoint;
                let otherPoint = nextEdge.next.originPoint;
                let cornerEdge = nextEdge.twin;
                while(cornerEdge != null) {
                    cornerEdge = cornerEdge.next;
                    otherPoint = cornerEdge.next.originPoint;
                    cornerEdge = cornerEdge.twin;
                }
                if (getTriangleSign(currentEdge.originPoint, potentialCorner, otherPoint) < 0) {
                    potentialCorner.isCorner = true;
                    //drawPoint(nextEdge.originPoint, 'black');
                }
            }
            currentEdge = nextEdge;
        } while (currentEdge != face.rootEdge);
    }
}

// Find any invalid faces, by checking .twin and <10k edges
export function findBadFace(faces: Face[]) {
    const MAX_EDGES = 10000;
    for(let face of faces) {
        let i = 0;
        let currentEdge = face.rootEdge;
        do {
            i++;
            if (i > MAX_EDGES) {
                return face;
            }
            if (currentEdge.twin != null && currentEdge.twin.twin != currentEdge) {
                return face;
            }
            currentEdge = currentEdge.next;
        } while (currentEdge !== face.rootEdge);
    }
    return null;
}