import { Point, Face, getPointDist, getOrientation, getIntersection, isPointInFace, HalfEdge, EPSILON } from "./math";
import { PriorityQueue } from "data-structure-typed";

class SearchNode {
    startPoint: Point; // Point
    endPoint: Point; // Point
    halfedge: HalfEdge; // HalfEdge
    root: Point; // Point
    g: number; // number
    f: number | null; // number | null
    prevSearchNode: SearchNode | null; // searchNode | null
    intermediatePoint: Point | null; // searchNode | null
    TESTMARKER: number = -1; // number
    constructor(startPoint: Point, endPoint: Point, halfedge: HalfEdge, root: Point, g: number, prevSearchNode: SearchNode | null) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.halfedge = halfedge;
        this.root = root;
        this.g = g;
        this.f = null;
        this.prevSearchNode = prevSearchNode;
        this.intermediatePoint = null;
    }
    getF(dst: Point) {
        if (this.f != null) {
            return this.f;
        }
        let g = this.g;
        let h;
        if (getOrientation(this.root, this.startPoint, this.endPoint) == 0) {
            // If the edge is collinear to the root, just go through the closer point to dst
            let closerPoint = getPointDist(this.root, this.startPoint) < getPointDist(this.root, this.endPoint) ? this.startPoint : this.endPoint;
            h = getPointDist(this.root, closerPoint) + getPointDist(closerPoint, dst);
        } else {
            /*
            // If dst is on the same side as r ([start, end] is CCW), then we flip
            // This is in the Polyanya paper, but is it necessary? TODO: Prove/verify this
            // If the geometry on the other side of [start, end] spins the cone around to shoot it back to dst,
            // Wouldn't it fail the worst-g test? For proof, we can assert() that an edge is never used twice in a path
            if (getOrientation(dst, this.startPoint, this.endPoint) > 0) {
                let dir = this.endPoint.minus(this.startPoint).normalize();
                let dstVec = dst.minus(this.startPoint);
                let dstProj = dir.multiply(dstVec.dot(dir));
                let dstProjDiff = dstProj.minus(dstVec);
                dst = dst.plus(dstProjDiff.multiply(2));
            }
            */
            // Check around the corners
            // We can't check for {start/end}Point.isCorner, since h must be consistent
            let cornerH = null;
            if (getOrientation(this.root, this.startPoint, dst) <= 0) {
                cornerH = Math.min(cornerH || Infinity, getPointDist(this.root, this.startPoint) + getPointDist(this.startPoint, dst));
            }
            if (getOrientation(this.root, this.endPoint, dst) >= 0) {
                cornerH = Math.min(cornerH || Infinity, getPointDist(this.root, this.endPoint) + getPointDist(this.endPoint, dst));
            }
            // If we aren't going around the corners, we just go direct
            if (cornerH == null) {
                h = getPointDist(this.root, dst);
            } else {
                h = cornerH;
            }
        }
        let f = g + h;
        this.f = f;
        return f;
    }
};

// A Polyanya path a sequence of points starting at src and ending at dst,
// Where the path does not intersect the interior of any obstacle
interface PolyanyaPath {
    path: Point[];
    // Total distance of this path
    distance: number;
    // Terminal SearchNode
    searchNode: SearchNode | null;
}

// Get the path on a terminal searchNode
// Requires searchNode.halfedge.twin!.face == dstFace
function getTerminalPath(searchNode: SearchNode, dst: Point): PolyanyaPath | null {
    let startSign = getOrientation(searchNode.root, searchNode.startPoint, dst);
    let endSign = getOrientation(searchNode.root, searchNode.endPoint, dst);
    // Construct Path
    let path: Point[] = [];
    let finalG = null;
    if (startSign >= 0 && endSign <= 0) {
        path.push(dst);
        finalG = searchNode.g + getPointDist(searchNode.root, dst);
    }
    if (searchNode.endPoint.isCorner && endSign > 0) {
        path.push(dst);
        path.push(searchNode.endPoint);
        finalG = searchNode.g + getPointDist(searchNode.root, searchNode.endPoint) + getPointDist(searchNode.endPoint, dst);
    }
    if (searchNode.startPoint.isCorner && startSign < 0) {
        path.push(dst);
        path.push(searchNode.startPoint);
        finalG = searchNode.g + getPointDist(searchNode.root, searchNode.startPoint) + getPointDist(searchNode.startPoint, dst);
    }
    // If there was no valid case, we just return
    if (finalG == null) {
        return null;
    }
    // Reconstruct previous path
    let curSearchNode: SearchNode | null = searchNode;
    while(curSearchNode != null) {
        // If the root has changed, store the new root
        if (curSearchNode.root != path[path.length-1]) {
            path.push(curSearchNode.root);
        }
        // If there was an intermediatePoint, store it
        if(curSearchNode.intermediatePoint !== null) {
            path.push(curSearchNode.intermediatePoint);
        }
        curSearchNode = curSearchNode.prevSearchNode;
    }
    // Reverse Path
    path = path.reverse();
    return {
        path: path,
        distance: finalG,
        searchNode: searchNode,
    };
}

function getSuccessors(searchNode: SearchNode, dst: Point) {
    function getPastSegment(root: Point, other: Point, edgeLine: Point[], checkCW: boolean): Point[] | null {
        // Get orientation of projecting root->other onto the edgeLine
        let startOrientation = getOrientation(root, other, edgeLine[0]);
        let endOrientation = getOrientation(root, other, edgeLine[1]);
        // If the root isn't oriented CCW towards the edge, then we return the whole edge
        if (getOrientation(root, edgeLine[0], edgeLine[1]) <= 0) {
            if (!checkCW) {
                if (startOrientation > 0 && endOrientation >= 0) {
                    return edgeLine;
                } else {
                    return null;
                }
            } else {
                if (endOrientation < 0 && startOrientation <= 0) {
                    return edgeLine;
                } else {
                    return null;
                }
            }
        }
        if (!checkCW) {
            // Check orientation
            // Check [root, other, edgeLine] being CCW
            if (endOrientation > 0 && startOrientation >= 0) {
                return edgeLine;
            }
            if (endOrientation > 0 && startOrientation < 0) {
                // TODO: Remove !
                let intersection = getIntersection(root, other, edgeLine[0], edgeLine[1])!;
                intersection.isCorner = false;
                return [intersection, edgeLine[1]];
            }
            if (endOrientation == 0 && startOrientation < 0) {
                return [edgeLine[1], edgeLine[1]];
            }
            if (endOrientation < 0 && startOrientation < 0) {
                return null;
            }
        } else {
            // Check [root, other, edgeLine] being CW
            if (startOrientation < 0 && endOrientation <= 0) {
                return edgeLine;
            }
            if (startOrientation < 0 && endOrientation > 0) {
                // TODO: Remove !
                let intersection = getIntersection(root, other, edgeLine[0], edgeLine[1])!;
                intersection.isCorner = false;
                return [edgeLine[0], intersection];
            }
            if (startOrientation == 0 && endOrientation > 0) {
                return [edgeLine[0], edgeLine[0]];
            }
            if (startOrientation > 0 && endOrientation > 0) {
                return null;
            }
        }
        console.error('All Cases Covered!', root, other, edgeLine, checkCW, startOrientation, endOrientation, getOrientation(root, edgeLine[0], edgeLine[1]));
        return null;
    }

    // The successors to accumulate
    let successors: SearchNode[] = [];

    // Check if the searchNode is degenerate (root is on [startPoint, endPoint])
    // NOTE: Is this guaranteed true when root is on startPoint/endPoint? Or should we check explicitly
    let isDegenerate = getOrientation(searchNode.root, searchNode.startPoint, searchNode.endPoint) == 0;
    // NOTE: It's sometimes possible for getOrientation(root, searchNode.startPoint, searchNode.endPoint),
    //       to be negative (Implying CW). Handle this better?

    // Otherwise, go through the edges of the opposite face
    let oppositeFace = searchNode.halfedge.twin!.face!;
    let currentEdge = oppositeFace.rootEdge;
    do {
        // Get the processing edge and update the current iteration edge
        let processingEdge = currentEdge;
        currentEdge = currentEdge.next;

        // Exclude projecting onto our twin, since it's the same edge
        if (processingEdge.twin == searchNode.halfedge) {
            continue;
        }

        // Consider the edgeLine, but skip if its collinear
        let edgeLine = [processingEdge.originPoint, processingEdge.next.originPoint];

        // Consider degenerate case
        if (isDegenerate) {
            // If the root is collinear with the successor edge, create a new degenerate successor
            if (getOrientation(searchNode.root, edgeLine[0], edgeLine[1]) == 0) {
                let newRoot = getPointDist(searchNode.root, edgeLine[0]) < getPointDist(searchNode.root, edgeLine[1]) ? edgeLine[0] : edgeLine[1];
                if (newRoot.isCorner) {
                    let newG = searchNode.g + getPointDist(searchNode.root, newRoot);
                    let newSearchNode = new SearchNode(edgeLine[0], edgeLine[1], processingEdge, newRoot, newG, searchNode);
                    newSearchNode.TESTMARKER = 1;
                    successors.push(newSearchNode);
                }
            } else {
                // Else, we have a non-degenerate successor
                let startToRoot = searchNode.root.minus(searchNode.startPoint);
                let endToRoot = searchNode.root.minus(searchNode.endPoint);
                let startToEnd = searchNode.endPoint.minus(searchNode.startPoint);
                let pastEnd = endToRoot.dot(startToEnd) > 0;
                let pastStart = startToRoot.dot(startToRoot.multiply(-1)) > 0;
                if (pastEnd) {
                    // If the root is beyond the endPoint, use the endPoint as the successor root
                    if (searchNode.endPoint.isCorner) {
                        let newSearchNode = new SearchNode(edgeLine[0], edgeLine[1], processingEdge, searchNode.endPoint, searchNode.g, searchNode);
                        newSearchNode.TESTMARKER = 2.2;
                        successors.push(newSearchNode);
                    }
                } else if (pastStart) {
                    // If the root is beyond the startPoint, use the startPoint as the successor root
                    if (searchNode.startPoint.isCorner) {
                        let newSearchNode = new SearchNode(edgeLine[0], edgeLine[1], processingEdge, searchNode.startPoint, searchNode.g, searchNode);
                        newSearchNode.TESTMARKER = 2.1;
                        successors.push(newSearchNode);
                    }
                } else {
                    // If root is on the line, the root is the successor
                    let newSearchNode = new SearchNode(edgeLine[0], edgeLine[1], processingEdge, searchNode.root, searchNode.g, searchNode);
                    newSearchNode.TESTMARKER = 2;
                    successors.push(newSearchNode);
                }
            }
            continue;
        }

        // If the root is collinear with the edge, handle the special case
        if (getOrientation(searchNode.root, edgeLine[0], edgeLine[1]) == 0) {
            // Alternatively, we can just use the closer point as the next root
            if (getOrientation(searchNode.startPoint, edgeLine[0], edgeLine[1]) == 0) {
                let newG = searchNode.g + getPointDist(searchNode.root, edgeLine[0]);
                let newSearchNode = new SearchNode(edgeLine[0], edgeLine[0], processingEdge, edgeLine[0], newG, searchNode);
                newSearchNode.TESTMARKER = 3;
                successors.push(newSearchNode);
            } else if (getOrientation(searchNode.endPoint, edgeLine[0], edgeLine[1]) == 0) {
                let newG = searchNode.g + getPointDist(searchNode.root, edgeLine[1]);
                let newSearchNode = new SearchNode(edgeLine[1], edgeLine[1], processingEdge, edgeLine[1], newG, searchNode);
                newSearchNode.TESTMARKER = 4;
                successors.push(newSearchNode);
            } else {
                //console.error('Impossible Situation');
            }
            continue;
        }

        // Check around the endpoint Corner,
        // Check triangleSign of edgeLine bounds for if cornering is even necessary
        if (searchNode.endPoint.isCorner && (getOrientation(searchNode.root, searchNode.endPoint, edgeLine[0]) > 0 || getOrientation(searchNode.root, searchNode.endPoint, edgeLine[1]) > 0)) {
            let pastEndCCW = getPastSegment(searchNode.root, searchNode.endPoint, edgeLine, false);
            if (pastEndCCW != null) {
                // Set the nextRoot, which might be the seg if it's collinear
                let nextRoot = getOrientation(searchNode.endPoint, pastEndCCW[0], pastEndCCW[1]) == 0 ? pastEndCCW[1] : searchNode.endPoint;
                let newG = searchNode.g + getPointDist(searchNode.root, searchNode.endPoint) + getPointDist(searchNode.endPoint, nextRoot);
                let newSearchNode = new SearchNode(pastEndCCW[0], pastEndCCW[1], processingEdge, nextRoot, newG, searchNode);
                newSearchNode.TESTMARKER = 5;
                if (nextRoot != searchNode.endPoint) {
                    newSearchNode.intermediatePoint = searchNode.endPoint;
                }
                successors.push(newSearchNode);
                //drawSegment(pastEndCCW[0], pastEndCCW[1], 'green');
            }
        }
        // Check around the startpoint Corner
        if (searchNode.startPoint.isCorner && (getOrientation(searchNode.root, searchNode.startPoint, edgeLine[0]) < 0 || getOrientation(searchNode.root, searchNode.startPoint, edgeLine[1]) < 0)) {
            let pastStartCW = getPastSegment(searchNode.root, searchNode.startPoint, edgeLine, true);
            if (pastStartCW != null) {
                let nextRoot = getOrientation(searchNode.startPoint, pastStartCW[0], pastStartCW[1]) == 0 ? pastStartCW[0] : searchNode.startPoint;
                let newG = searchNode.g + getPointDist(searchNode.root, searchNode.startPoint) + getPointDist(searchNode.startPoint, nextRoot);
                let newSearchNode = new SearchNode(pastStartCW[0], pastStartCW[1], processingEdge, nextRoot, newG, searchNode);
                newSearchNode.TESTMARKER = 5.5;
                if (nextRoot != searchNode.startPoint) {
                    newSearchNode.intermediatePoint = searchNode.startPoint;
                }
                successors.push(newSearchNode);
                //drawSegment(pastStartCW[0], pastStartCW[1], 'blue');
            }
        }

        // Consider a searchNode with root updates to searchNode.endPoint

        // Consider the sign from projecting onto the edge
        let CCWFromStart = getPastSegment(searchNode.root, searchNode.startPoint, edgeLine, false);
        let CWFromEnd = getPastSegment(searchNode.root, searchNode.endPoint, edgeLine, true);
        if (CCWFromStart != null && CWFromEnd != null) {
            let subSegment = [CCWFromStart[0], CWFromEnd[1]];
            let newSearchNode = new SearchNode(subSegment[0], subSegment[1], processingEdge, searchNode.root, searchNode.g, searchNode);
            newSearchNode.TESTMARKER = 6;
            successors.push(newSearchNode);
            //drawSegment(subSegment[0], subSegment[1], 'red');
        }
    } while(currentEdge != oppositeFace.rootEdge);
    // Return the successors
    return successors;
}

// if strictDst = false, it's acceptable if dst is within Epsilon of a face
export async function polyanya(faces: Face[], src: Point, dst: Point, maxDist?: number, strictDst: boolean = false): Promise<PolyanyaPath | null> { // Point, Point
    // Get Src + Dst Face
    let dstFace: Face | null = null;
    let srcFace: Face | null = null;
    // TODO: Optimize with R-Tree
    function isPointAlmostInFace(face: Face, p: Point): 0 | 1 | -1 {
        // Check around Epsilon
        // TODO: Replace with checking if p is within epsilon of any edge
        for(let dir of [new Point(0, 1), new Point(0, -1), new Point(1, 0), new Point(-1, 0)]) {
            let ret: 0 | 1 | -1 = isPointInFace(face, p.plus(dir.multiply(EPSILON)));
            if (ret >= 0) {
                return ret;
            }
        }
        return isPointInFace(face, p);
    }
    for(let face of faces) {
        if (srcFace == null && isPointAlmostInFace(face, src) >= 0) {
            srcFace = face;
        }
        if (dstFace == null && isPointAlmostInFace(face, dst) >= 0) {
            if (strictDst && isPointInFace(face, dst) >= 0) {
                dstFace = face;
            }
            if (!strictDst && isPointAlmostInFace(face, dst) >= 0) {
                dstFace = face;
            }
        }
        if (srcFace != null && dstFace != null) {
            break;
        }
    }

    // Check if src or dst aren't traversable
    if (srcFace == null || dstFace == null) {
        return null;
    }

    // Check if a direct path from within the same face is possible
    if (srcFace == dstFace) {
        let dist = getPointDist(src, dst);
        if (maxDist !== undefined && dist > maxDist) {
            return null;
        }
        return {
            path: [src, dst],
            distance: dist,
            searchNode: null,
        };
    }

    // The pq to hold all polyanya search nodes
    let pq = new PriorityQueue<SearchNode>({comparator: (a, b) => a.getF(dst) - b.getF(dst)});
    // Push node into pq, filtering it out if it can't be valid,
    // passthrough=true returns the node rather than putting it into the pq
    function pushNode(newSearchNode: SearchNode, passthrough=false) {
        // If there's a face on the other side to continue exploring, explore it
        // TODO: Don't count cul-de-sacs that don't contain the dst
        if (newSearchNode.halfedge.twin != null) {
            // Push to pq, as long as node isn't dead from maxDist
            let f = newSearchNode.getF(dst);
            if (maxDist === undefined || f <= maxDist) {
                if (passthrough) {
                    return newSearchNode;
                } else {
                    pq.add(newSearchNode);
                }
            }
        }
        return null;
    }

    // Initialize starting search nodes, by going from src to each edge of srcFace
    let currentEdge = srcFace.rootEdge;
    do {
        let newSearchNode = new SearchNode(currentEdge.originPoint, currentEdge.next.originPoint, currentEdge, src, 0, null);
        newSearchNode.TESTMARKER = 0;
        pushNode(newSearchNode);
        currentEdge = currentEdge.next;
    } while (currentEdge != srcFace.rootEdge);

    // Run the search loop
    let i = 0;
    let bestPath: PolyanyaPath | null = null;
    let rootToG = new Map<number, number>();
    let nextSearchNode: SearchNode | null = null;
    const MAX_ITERATIONS = 500000;
    while(!pq.isEmpty()) {
        if (i == MAX_ITERATIONS) {
            console.error('Max Iterations Reached!');
            break;
        }

        // Get the current SearchNode of this iteration
        let curSearchNode;
        if (nextSearchNode == null) {
            curSearchNode = pq.poll()!;
        } else {
            curSearchNode = nextSearchNode;
            nextSearchNode = null;
        }
        // searchNode shouldn't be in the pq if its halfedge doesn't have a twin
        console.assert(curSearchNode.halfedge.twin != null);

        // If this g is > the last recorded g from that root, skip because it's worse
        let rootHash = curSearchNode.root.hash();
        let rootG = rootToG.get(rootHash);
        if (rootG !== undefined && curSearchNode.g > rootG) {
            continue;
        }
        rootToG.set(rootHash, curSearchNode.g);

        // If this path can't possibly be better than the bestPath, just give up
        if (bestPath != null && curSearchNode.getF(dst) >= bestPath.distance) {
            continue;
        }
        // If there's a max dist and this search node exceeds it, give up
        if (maxDist !== undefined && curSearchNode.getF(dst) > maxDist) {
            continue;
        }

        // If the searchNode is Terminal,
        if (curSearchNode.halfedge.twin!.face! == dstFace) {
            // Check that path against the bestPath
            let curPath = getTerminalPath(curSearchNode, dst);
            if (curPath != null && (bestPath == null || curPath.distance < bestPath.distance)) {
                bestPath = curPath;
            }
        } else {
            // Otherwise, Get the successors and save them for next iterations
            let successors = getSuccessors(curSearchNode, dst);
            if (successors.length == 1) {
                // Just immediately process the successor if there's just one
                nextSearchNode = pushNode(successors[0], true);
            } else {
                // Push all of the successors if there are many
                for(let successor of successors) {
                    pushNode(successor);
                }
            }
        }
        i += 1;
    }
    //console.log('Num Iters:', i);

    //console.log('Best Path:', bestPath);
    // Returns null if no path found
    return bestPath;
}