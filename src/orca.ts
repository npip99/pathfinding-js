import { EPSILON, Point, Face, getOrientation, getPointDist } from "./math";

// A Halfplane is represented by a point and a normal
class Halfplane {
    p: Point;
    n: Point;
    constructor(p: Point, n: Point) {
        this.p = p;
        this.n = n;
    }
}

// h = Halfplane
// v = a ray from vP with direction vDir
// Returns [tMin, tMax], s.t. vP + t * vDir \in h, for all t in [tMin, tMax]
// tMax < tMin when there is no such t
function getTRange(h: Halfplane, vP: Point, vDir: Point): [number, number] {
    // Formula for t = tNumerator/tDenominator,
    // Based on solving ((vP + t*vDir) - h.p) \cdot h.n = 0
    let tNumerator = h.n.dot(h.p.minus(vP));
    let tDenominator = h.n.dot(vDir);
    // If v is close to parallel with h,
    if (Math.abs(tDenominator) < EPSILON) {
        // If H_n and H_p-v_p point in opposite directions,
        if (tNumerator <= 0) {
            // All t are valid
            return [-Infinity, Infinity];
        } else {
            // No t are valid
            return [Infinity, -Infinity];
        }
    }
    // Now, calculate t
    let t = tNumerator / tDenominator
    // Bound t-range based on whether or not vDir goes in the same direction as h.n
    // If they do, increasing t*vDir puts the vector deeper in the territory of h.n
    if (tDenominator > 0) {
        return [t, Infinity];
    } else {
        return [-Infinity, t];
    }
}

export class ORCAAgent {
    position: Point;
    velOpt: Point;
    radius: number;
}

interface VO {
    circleCenter: Point;
    circleRadius: number;
    unitTangentCCW: Point | null;
    unitTangentCW: Point | null;
    tangentDist: number | null;
}

function getVO(a: ORCAAgent, b: ORCAAgent, delta: number): VO {
    let center = b.position.minus(a.position).divide(delta);
    let u = center.x;
    let v = center.y;
    let r = (a.radius + b.radius)/delta;

    let tangent_dist2 = center.magnitude2() - Math.pow(r, 2);
    // This doesn't need an epsilon check, as tangent_dist2=0 is acceptable
    if (tangent_dist2 < 0) {
        return {
            circleCenter: center,
            circleRadius: r,
            unitTangentCCW: null,
            unitTangentCW: null,
            tangentDist: null,
        };
    }

    // Get the theta between either tangent line and the center
    let centerDist = center.magnitude();
    let theta = Math.asin(r/centerDist);

    // Get unit vectors in the direction of the two tangent lines
    let unitTangentCCW = center.rotate(theta).normalize();
    let unitTangentCW =  center.rotate(-theta).normalize();

    // Return VO_{A|B}^\delta
    return {
        circleCenter: center,
        circleRadius: r,
        unitTangentCCW: unitTangentCCW,
        unitTangentCW: unitTangentCW,
        tangentDist: Math.sqrt(tangent_dist2),
    };
}

interface VOProjection {
    proj: Point;
    norm: Point;
}

// For a Point p and a VO, we get a point on the boundary of the VO,
// And we get the normal of that boundary point (Pointing outwards from the VO region)
function getVOProjection(vo: VO, p: Point): VOProjection {
    // If there are tangent lines, try to project onto either of the tangent lines
    if (vo.tangentDist != null && vo.unitTangentCCW != null && vo.unitTangentCW != null) {
        // These are the tangent points themselves
        // (But since they can be small, we use unit vectors for all directional calculations)
        let tangentCCW = vo.unitTangentCCW.multiply(vo.tangentDist);
        let tangentCW = vo.unitTangentCW.multiply(vo.tangentDist);
        // Project p onto the tangent boundary
        let projCCW = vo.unitTangentCCW.multiply(p.dot(vo.unitTangentCCW));
        let projCW = vo.unitTangentCW.multiply(p.dot(vo.unitTangentCW));
        // Invalidate p's projection if it isn't at least past the tangent point
        let projCCWValid = projCCW.minus(tangentCCW).dot(vo.unitTangentCCW) >= 0;
        let projCWValid = projCW.minus(tangentCW).dot(vo.unitTangentCW) >= 0;
        // If both projections are valid, invalidate the farther one
        if (projCCWValid && projCWValid) {
            if (projCCW.minus(p).magnitude2() < projCW.minus(p).magnitude2()) {
                projCWValid = false;
            } else {
                projCCWValid = false;
            }
        }
        // Return a valid tangent projection, if any
        if (projCCWValid) {
            return {
                proj: projCCW,
                norm: tangentCCW.minus(vo.circleCenter).normalize(),
            };
        } else if (projCWValid) {
            return {
                proj: projCW,
                norm: tangentCW.minus(vo.circleCenter).normalize(),
            }
        }
    }
    // If projecting onto the tangent line failed, project onto the circle
    let norm = p.minus(vo.circleCenter);
    if (norm.magnitude2() < EPSILON) {
        // Use p=origin if p was too close to the center
        norm = vo.circleCenter.multiply(-1);
        // Use a random direction if even that norm doesn't work
        if (norm.magnitude2() < EPSILON) {
            norm = new Point(0, -1);
        }
    }
    // Return the boundary point and the normal
    norm = norm.normalize();
    let proj = vo.circleCenter.plus(norm.multiply(vo.circleRadius));
    return {
        proj: proj,
        norm: norm,
    };
}

function getORCA(a: ORCAAgent, b: ORCAAgent, delta: number): Halfplane {
    let vo = getVO(a, b, delta);
    let p = a.velOpt.minus(b.velOpt);
    let voProj = getVOProjection(vo, p);
    // u can be small, so we use norm for directional calculations
    let u = voProj.proj.minus(p);
    // Return the halfplane
    return new Halfplane(a.velOpt.plus(u.divide(2)), voProj.norm);
}

// Let's say that we have a currentResult that is in all of the halfplanes and is closest to target,
// We want to then add the contraint h, and select a |newResult| <= radius that satisfies h as well, such that it is closest to target
// If useDirection is true, we optimize for alignment with target's direction.
function linearProgramming1(halfplanes: Halfplane[], h: Halfplane, target: Point, radius: number, currentResult: Point, useDirection: boolean = false): Point | null {
    // Get a direction vector (Which way doesn't matter)
    let hDir = h.n.rotate(Math.PI/2);

    // If using direction, use the point as infinity
    if (useDirection) {
        target = target.multiply(1000);
    }

    // If currentResult is already satisfying h, keep it
    if (h.n.dot(currentResult.minus(h.p)) >= 0) {
        return currentResult;
    } else {
        // Parametrize h as {H_p + t*H_dir}
        let tLeft = -Infinity;
        let tRight = Infinity;

        // Find the two values of t such that ||H_p + t H_dir|| = r
        // (H_p + t H_dir) \cdot (H_p + t H_dir) = r^2 is a quadratic in t
        let a = hDir.dot(hDir);
        let b = 2*h.p.dot(hDir);
        let c = h.p.dot(h.p) - Math.pow(radius, 2);
        let discriminant = Math.pow(b, 2) - 4*a*c;
        if (discriminant < 0) {
            return null;
        }
        // Bound tLeft and tRight by the valid t-values
        tLeft = (-b - Math.sqrt(discriminant)) / (2*a);
        tRight = (-b + Math.sqrt(discriminant)) / (2*a);

        // Now, we clamp [tLeft, tRight] based on the rest of the half planes
        for (let halfplane of halfplanes) {
            let tRange = getTRange(halfplane, h.p, hDir);
            tLeft = Math.max(tLeft, tRange[0]);
            tRight = Math.min(tRight, tRange[1]);
            if (tLeft > tRight) {
                return null;
            }
        }

        // Now, calculate the targetT, from projecting target
        let targetT = hDir.dot(target.minus(h.p));

        // Clamp targetT based on the constraints
        targetT = Math.min(Math.max(targetT, tLeft), tRight);

        // Return the new result
        return h.p.plus(hDir.multiply(targetT));
    }
}

// For a given set of halfplanes and a target
function linearProgramming2(halfplanes: Halfplane[], target: Point, radius: number, useDirection: boolean = false): [Point, number] {
    let processedHalfplanes: Halfplane[] = [];
    // Set the currentResult
    let currentResult = target;
    if (useDirection || currentResult.magnitude() >= radius) {
        currentResult = currentResult.normalize().multiply(radius);
    }
    // Constrain currentResult by each halfplane
    for(let i = 0; i < halfplanes.length; i++) {
        let newHalfplane = halfplanes[i];
        let nextResult = linearProgramming1(processedHalfplanes, newHalfplane, target, radius, currentResult, useDirection);
        // If LP fails, return the failure line and the current result
        if (nextResult == null) {
            return [currentResult, i];
        }
        // Otherwise, continue
        currentResult = nextResult;
        processedHalfplanes.push(newHalfplane);
    }
    // Return success
    return [currentResult, -1];
}

// Solve the LP problem, by returning a |result| <= radius inside of all of the halfplanes,
// Such that result is closest to target
// If no solution exists, we pick the result that minimax's violation distance from the halfplane
function solveLP(mandatoryHalfplanes: Halfplane[], halfplanes: Halfplane[], target: Point, radius: number): Point {
    let [result, failI] = linearProgramming2(mandatoryHalfplanes.concat(halfplanes), target, radius, false);
    if (failI == -1) {
        // If the LP didn't fail, return the result
        return result;
    } else {
        // If the LP failed, we optimize for distance
        let distance = 0; // The current worst distance that we satisfy
        failI = Math.max(failI - mandatoryHalfplanes.length, 0);
        for(let i = failI; i < halfplanes.length; i++) {
            let h = halfplanes[i];
            let hDir = h.n.rotate(Math.PI/2);
            // If result violates this halfplane by better than -distance,
            // We're still good
            if (h.n.dot(result.minus(h.p)) >= -distance) {
                continue;
            }
            // Otherwise, we loosen each halfplane into an isohalfplanes (mandatoryHalfplanes are unmodified)
            // An isohalfplane is equidistant between its respective satisfied halfplanes and the newly violating halfplane
            // We want to pick a result that is on the isohalfplanes and closest to the new halfplane,
            // since that result balances violation between the new halfplane and the existent halfplanes 
            let isoHalfplanes: Halfplane[] = mandatoryHalfplanes.slice();
            for(let j = 0; j < i; j++) {
                let other = halfplanes[j];

                let isoP: Point | null = null;

                // Project h onto j
                let tRange = getTRange(other, h.p, hDir);
                if (tRange[0] > tRange[1]) {
                    isoP = h.p.plus(other.p).divide(2);
                } else if (tRange[0] == -Infinity && tRange[1] == Infinity) {
                    // There's no constraint here
                    continue;
                } else {
                    let t = tRange[0] != -Infinity ? tRange[0] : tRange[1];
                    isoP = h.p.plus(hDir.multiply(t));
                }

                let isoN = other.n.minus(h.n).normalize();
                isoHalfplanes.push(new Halfplane(isoP, isoN));
            }

            // Get the new result that violates these halfplanes the least
            let [newResult, newFailI] = linearProgramming2(isoHalfplanes, h.n, radius, true);
            // This should always succeed, but we check in-case it doesn't because of floating point issues
            if (newFailI == -1) {
                // Set the new result
                result = newResult;
            }
            // Get the new distance into violation territory
            distance = h.n.dot(h.p.minus(result));
        }

        // Return the result
        return result;
    }
}

function getORCAFromObstacle(agent: ORCAAgent, segment: [Point, Point], delta: number): Halfplane | null {
    let a = segment[0].minus(agent.position).divide(delta);
    let b = segment[1].minus(agent.position).divide(delta);
    let r = agent.radius/delta;
    let origin = new Point(0, 0);
    // If O->a->b is CCW, our view is the same as from the interior of the polygon, so it doesn't matter
    if (getOrientation(origin, a, b) >= -EPSILON) {
        return null;
    }
    // TODO: Create ORCA properly even when colliding with segment

    // Get the best aProj and bProj values
    let aNorm = origin.minus(a).normalize();
    let aProj = a.plus(aNorm.multiply(r));
    let bNorm = origin.minus(b).normalize();
    let bProj = b.plus(bNorm.multiply(r));

    // Get the best linesegment Proj values
    let segNorm = b.minus(a).rotate(-Math.PI/2).normalize();
    // The seg is represented by seg[0] + t*segDir, where t ranges from 0 to 1.
    let seg = [a.plus(segNorm.multiply(r)), b.plus(segNorm.multiply(r))];
    let segDir = seg[1].minus(seg[0]).normalize();
    // We get the t that is closest to the origin, and the tRange that bounds the segment
    let tLeft = 0;
    let tRight = getPointDist(seg[0], seg[1]);
    let t = segDir.dot(origin.minus(seg[0]));
    // Clamp t based on the range
    t = Math.min(Math.max(t, tLeft), tRight);
    // Get the segmentProj
    let segProj = seg[0].plus(segDir.multiply(t));

    // Use the closest proj to the origin
    let ret: Halfplane;
    if (segProj.magnitude2() < aProj.magnitude2() && segProj.magnitude2() < bProj.magnitude2()) {
        ret = new Halfplane(segProj, segNorm);
    } else if (aProj.magnitude2() < bProj.magnitude2()) {
        ret = new Halfplane(aProj, aNorm);
    } else {
        ret = new Halfplane(bProj, bNorm);
    }

    /*
    drawSegment(agent.position.plus(seg[0].multiply(delta)), agent.position.plus(seg[1].multiply(delta)), 'green');
    drawPoint(agent.position.plus(a.multiply(delta)), 'green', r*delta);
    drawPoint(agent.position.plus(b.multiply(delta)), 'green', r*delta);
    let proj = agent.position.plus(ret.p.multiply(delta));
    let dir = ret.n.rotate(Math.PI/2).multiply(agent.radius);
    drawSegment(proj.minus(dir), proj.plus(dir), 'orange');
    */

    return ret;
}

// For a given agent, trying to avoid other agents/obstacles, and trying to optimize for a prefVelocity,
// We return the induced ORCAVelocity
export function getORCAVelocity(boundingFace: Face, obstacles: Face[], agent: ORCAAgent, otherAgents: ORCAAgent[], prefVelocity: Point, maxSpeed: number, delta: number): Point {
    // Get the ORCA VO's induced by the obstacle's line segments
    let obstacleHalfplanes: Halfplane[] = [];
    for(let obstacle of obstacles.concat(boundingFace)) {
        let currentEdge = obstacle.rootEdge;
        do {
            currentEdge = currentEdge.next;
            let a = currentEdge.originPoint;
            let b = currentEdge.next.originPoint;

            let seg: [Point, Point] = obstacle == boundingFace ? [b, a] : [a, b];
            let halfplane = getORCAFromObstacle(agent, seg, delta);
            if (halfplane != null) {
                obstacleHalfplanes.push(halfplane);
            }
        } while(currentEdge != obstacle.rootEdge);
    }

    // Get the ORCA VO's induced by the other agents
    let agentHalfplanes: Halfplane[] = [];
    for(let otherAgent of otherAgents) {
        agentHalfplanes.push(getORCA(agent, otherAgent, delta));
    }

    // Return the optimal velocity by solving the LP
    let optimalVelocity = solveLP(obstacleHalfplanes, agentHalfplanes, prefVelocity, maxSpeed);
    return optimalVelocity;
}