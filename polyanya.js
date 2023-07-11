class SearchNode {
    startPoint; // Point
    endPoint; // Point
    halfedge; // HalfEdge
    root; // Point
    g; // float
    f; // float | null
    constructor(startPoint, endPoint, halfedge, root, g, prevSearchNode) {
        this.startPoint = startPoint;
        this.endPoint = endPoint;
        this.halfedge = halfedge;
        this.root = root;
        this.g = g;
        this.f = null;
        this.prevSearchNode = prevSearchNode;
    }
    getF(dst) {
        if (this.f != null) {
            return this.f;
        }
        let g = this.g;
        let h = null;
        if (getTriangleSign(this.root, this.startPoint, this.endPoint) == 0) {
            // If the edge is collinear to the root, just go through the closer point to dst
            let closerPoint = getPointDist(this.root, this.startPoint) < getPointDist(this.root, this.endPoint) ? this.startPoint : this.endPoint;
            h = getPointDist(this.root, closerPoint) + getPointDist(closerPoint, dst);
        } else {
            // In the worst case, just go directly
            h = getPointDist(this.root, dst);
            // Check rouding the start corner
            if (this.startPoint.isCorner && getTriangleSign(this.root, this.startPoint, dst) <= 0) {
                h = Math.max(h, getPointDist(this.root, this.startPoint) + getPointDist(this.startPoint, dst));
            }
            if (this.endPoint.isCorner && getTriangleSign(this.root, this.endPoint, dst) >= 0) {
                h = Math.max(h, getPointDist(this.root, this.endPoint) + getPointDist(this.endPoint, dst));
            }
        }
        let f = g + h;
        this.f = f;
        return f;
    }
};

function getSuccessors(searchNode, dst, dstFace, bestPath) {
    function getPastSegment(root, other, edgeLine, checkCW) {
        // Get orientation of projecting root->other onto the edgeLine
        let startOrientation = getTriangleSign(root, other, edgeLine[0]);
        let endOrientation = getTriangleSign(root, other, edgeLine[1]);
        // If the root isn't oriented CCW towards the edge, then we return the whole edge
        if (getTriangleSign(root, edgeLine[0], edgeLine[1]) <= 0) {
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
                let intersection = getIntersection([root, other], edgeLine);
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
                let intersection = getIntersection([root, other], edgeLine);
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
        console.error('All Cases Covered!', root, other, edgeLine, checkCW, startOrientation, endOrientation, getTriangleSign(root, edgeLine[0], edgeLine[1]));
    }

    let successors = [];
    // If there's no twin for this halfedge, then there are no successors
    if (searchNode.halfedge.twin == null) {
        return [];
    }
    // Else, loop through all of the edges in the opposite face
    let otherFace = searchNode.halfedge.twin.face;
    // Check straight to destination if that's a possible successor
    if (otherFace == dstFace) {
        let startSign = getTriangleSign(searchNode.root, searchNode.startPoint, dst);
        let endSign = getTriangleSign(searchNode.root, searchNode.endPoint, dst);
        // Construct Path
        let path = [];
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
        if (path.length == 0) {
            return [];
        }
        // Reconstruct previous path
        let curSearchNode = searchNode;
        while(curSearchNode != null) {
            // If the root has changed, store the new root
            if (curSearchNode.root != path[path.length-1]) {
                path.push(curSearchNode.root);
            }
            // If there was an intermediateNode, store it
            if(curSearchNode.intermediateNode !== undefined) {
                path.push(curSearchNode.intermediateNode);
            }
            curSearchNode = curSearchNode.prevSearchNode;
        }
        // Reverse Path
        path = path.reverse();
        if (bestPath['distance'] == null || finalG < bestPath['distance']) {
            bestPath['distance'] = finalG;
            bestPath['path'] = path;
            bestPath['searchNode'] = searchNode;
            //console.log('New Best Path:', searchNode, finalG);
        }
        // No further successors, as we already made it to the dst
        return [];
    }

    // Get if the searchNode is degenerate (root is one of the edge endpoints)
    let isDegenerate = (searchNode.root == searchNode.startPoint || searchNode.root == searchNode.endPoint);
    // NOTE: It's sometimes possible for getTriangleSign(root, searchNode.startPoint, searchNode.endPoint),
    //       to be negative (Implying CW). Handle this better?

    // Otherwise, go through the edges of the opposite face
    let currentEdge = otherFace.rootEdge;
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
            // If the root is collinear with the edge, create a new degenerate successor
            if (getTriangleSign(searchNode.root, edgeLine[0], edgeLine[1]) == 0) {
                // The new degenerate case only matters if the edgeline root is a corner
                let newRoot = getPointDist(searchNode.root, edgeLine[0]) < getPointDist(searchNode.root, edgeLine[1]) ? edgeLine[0] : edgeLine[1];
                if (newRoot.isCorner) {
                    let newG = searchNode.g + getPointDist(searchNode.root, newRoot);
                    let newSearchNode = new SearchNode(edgeLine[0], edgeLine[1], processingEdge, newRoot, newG, searchNode);
                    newSearchNode.TESTMARKER = 1;
                    successors.push(newSearchNode);
                }
            } else {
                // Else, the edge with the current root is the successor
                let newSearchNode = new SearchNode(edgeLine[0], edgeLine[1], processingEdge, searchNode.root, searchNode.g, searchNode);
                newSearchNode.TESTMARKER = 2;
                successors.push(newSearchNode);
            }
            continue;
        }

        // If the root is collinear with the edge, handle the special case
        if (getTriangleSign(searchNode.root, edgeLine[0], edgeLine[1]) == 0) {
            // Alternatively, we can just use the closer point as the next root
            if (getTriangleSign(searchNode.startPoint, edgeLine[0], edgeLine[1]) == 0) {
                let newG = searchNode.g + getPointDist(searchNode.root, edgeLine[0]);
                let newSearchNode = new SearchNode(edgeLine[0], edgeLine[0], processingEdge, edgeLine[0], newG, searchNode);
                newSearchNode.TESTMARKER = 3;
                successors.push(newSearchNode);
            } else if (getTriangleSign(searchNode.endPoint, edgeLine[0], edgeLine[1]) == 0) {
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
        if (searchNode.endPoint.isCorner && (getTriangleSign(searchNode.root, searchNode.endPoint, edgeLine[0]) > 0 || getTriangleSign(searchNode.root, searchNode.endPoint, edgeLine[1]) > 0)) {
            let pastEndCCW = getPastSegment(searchNode.root, searchNode.endPoint, edgeLine, false);
            if (pastEndCCW != null) {
                // Set the nextRoot, which might be the seg if it's collinear
                let nextRoot = getTriangleSign(searchNode.endPoint, pastEndCCW[0], pastEndCCW[1]) == 0 ? pastEndCCW[1] : searchNode.endPoint;
                let newG = searchNode.g + getPointDist(searchNode.root, searchNode.endPoint) + getPointDist(searchNode.endPoint, nextRoot);
                let newSearchNode = new SearchNode(pastEndCCW[0], pastEndCCW[1], processingEdge, nextRoot, newG, searchNode);
                newSearchNode.TESTMARKER = 5;
                if (nextRoot != searchNode.endPoint) {
                    newSearchNode.intermediateNode = searchNode.endPoint;
                }
                successors.push(newSearchNode);
                //drawSegment(pastEndCCW[0], pastEndCCW[1], 'green');
            }
        }
        // Check around the startpoint Corner
        if (searchNode.startPoint.isCorner && (getTriangleSign(searchNode.root, searchNode.startPoint, edgeLine[0]) < 0 || getTriangleSign(searchNode.root, searchNode.startPoint, edgeLine[1]) < 0)) {
            let pastStartCW = getPastSegment(searchNode.root, searchNode.startPoint, edgeLine, true);
            if (pastStartCW != null) {
                let nextRoot = getTriangleSign(searchNode.startPoint, pastStartCW[0], pastStartCW[1]) == 0 ? pastStartCW[0] : searchNode.startPoint;
                let newG = searchNode.g + getPointDist(searchNode.root, searchNode.startPoint) + getPointDist(searchNode.startPoint, nextRoot);
                let newSearchNode = new SearchNode(pastStartCW[0], pastStartCW[1], processingEdge, nextRoot, newG, searchNode);
                newSearchNode.TESTMARKER = 5.5;
                if (nextRoot != searchNode.startPoint) {
                    newSearchNode.intermediateNode = searchNode.startPoint;
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
    } while(currentEdge != otherFace.rootEdge);
    // Return the successors
    return successors;
}

async function polyanya(faces, src, dst) { // Point, Point
    // Get Src + Dst Face
    let dstFace = null;
    let srcFace = null;
    // TODO: Optimize with R-Tree
    for(let face of faces) {
        if (srcFace == null && isPointInFace(face, src) >= 0) {
            srcFace = face;
        }
        if (dstFace == null && isPointInFace(face, dst) >= 0) {
            dstFace = face;
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
        return {
            'distance': getPointDist(src, dst),
            'path': [src, dst],
        };
    }

    // Initialize starting search nodes, by going from src to each edge of srcFace
    let pq = new PriorityQueue((a, b) => a['f'] < b['f']);
    currentEdge = srcFace.rootEdge;
    do {
        let newSearchNode = new SearchNode(currentEdge.originPoint, currentEdge.next.originPoint, currentEdge, src, 0, null);
        newSearchNode.TESTMARKER = 0;
        if (newSearchNode.halfedge.twin != null) {
            //await new Promise(r => setTimeout(r, 1000));
            //drawSearchNode(newSearchNode, 'red');
            pq.push({
                'searchNode': newSearchNode,
                'f': newSearchNode.getF(dst),
            });
        }
        currentEdge = currentEdge.next;
    } while (currentEdge != srcFace.rootEdge);

    // Run the search loop
    i = 0;
    let bestPath = {
        'distance': null,
        'path': null,
    };
    let root_to_g = {};
    let nextSearchNode = null;
    const MAX_ITERATIONS = 500000;
    while(pq.size() > 0) {
        if (i == MAX_ITERATIONS) {
            console.error('Max Iterations Reached!');
            break;
        }
        let curSearchNode = null;
        if (nextSearchNode == null) {
            curSearchNode = pq.pop()['searchNode'];
        } else {
            curSearchNode = nextSearchNode;
            nextSearchNode = null;
        }
        let rootHash = curSearchNode.root.hash();
        // If this g is > the last recorded g from that root, exit
        if (root_to_g[rootHash] !== undefined && curSearchNode.g > root_to_g[rootHash]) {
            //console.log('Invalid');
            continue;
        }
        root_to_g[rootHash] = curSearchNode.g;
        // If this path can't possibly be better than the bestPath, just give up
        if (bestPath['distance'] != null && curSearchNode.getF() >= bestPath['distance']) {
            continue;
        }
        //await new Promise(r => setTimeout(r, 1000));
        //drawSearchNode(curSearchNode, 'red');
        let successors = getSuccessors(curSearchNode, dst, dstFace, bestPath);
        //console.log(i, curSearchNode, successors);
        if (successors.length == 1) {
            // Just immediately process the successor if there's just one
            if (successors[0].halfedge.twin != null) {
                nextSearchNode = successors[0];
            }
        } else {
            for(let successor of successors) {
                // If there's a face on the other side to continue exploring, explore it
                // TODO: Don't count cul-de-sacs that don't contain the dst
                if (successor.halfedge.twin != null) {
                    pq.push({
                        'searchNode': successor,
                        'f': successor.getF(dst),
                    });
                }
            }
        }
        i += 1;
    }
    console.log('Num Iters:', i);

    // Return null if no path found
    console.log('Best Path:', bestPath);
    return bestPath['path'] == null ? null : bestPath;
}