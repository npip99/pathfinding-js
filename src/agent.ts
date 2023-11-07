import { Point, Face, EPSILON, getPointDist } from "./math";
import { polyanya } from "./polyanya";
import { ORCAAgent, getORCAVelocity } from "./orca";

const USING_ORCA = true;

export class Agent {
    // Current position
    position: Point;
    // Agent Radius
    radius: number;
    // Agent Max speed
    speed: number;
    // Previous velocity
    prevVel = new Point(0, 0);
    // Current velocity and speed
    curVel = new Point(0, 0);
    curSpeed = 0;
    // The local finalTarget that the Agent is trying to get to
    finalTarget: Point | null;
    // path is the current path to finalTarget
    path: Point[] = [];
    // curTarget is path[0]
    curTarget: Point | null = null;
    constructor(position: Point, radius: number, speed: number) {
        this.position = position;
        this.radius = radius;
        this.speed = speed;
    }
    getORCAAgent(): ORCAAgent {
        return {
            position: this.position,
            velOpt: this.prevVel,
            radius: this.radius,
        };
    }
    pathDistRemaining() {
        let totalPathLength = 0;
        for(let i = 0; i < this.path.length; i++) {
            totalPathLength += getPointDist(i == 0 ? this.position : this.path[i-1], this.path[i]);
        }
        return totalPathLength;
    }
    // Sets current finalTarget, returning true on success
    async setFinalTarget(offsetTraversableFaces: Face[], finalTarget: Point, maxStartDist?: number) {
        this.stop();
        this.finalTarget = finalTarget;
        // Update the current target
        await this.updateTarget(offsetTraversableFaces, maxStartDist);
        return this.curTarget != null;
    }
    stop() {
        this.finalTarget = null;
        this.path = [];
        this.curTarget = null;
    }
    hasAnotherTarget() {
        return this.path.length > 1;
    }
    isPathing() {
        return this.hasAnotherTarget() || (this.curTarget != null && this.position.minus(this.curTarget).magnitude() > EPSILON);
    }
    // Set Agent's target for this iteration
    async updateTarget(offsetTraversableFaces: Face[], maxPathDist?: number) {
        // Repath to target every time
        if (this.finalTarget != null) {
            let result = await polyanya(offsetTraversableFaces, this.position, this.finalTarget, maxPathDist);
            if (result == null) {
                this.stop();
                return;
            }
            this.path = result.path;
            this.path.splice(0, 1); // Ignore src in Path
            this.curTarget = this.path[0];
        }
    }
    // Takes all neighboring Agents and Obstacles into consideration during the calculation of current velocity
    async considerNeighboringAgents(boundingFace: Face, neighboringObstacles: Face[], offsetTraversableFaces: Face[], neighboringAgents: Agent[], deltaTime: number, speed?: number) {
        if (speed === undefined) {
            speed = this.speed;
        }
        if (!USING_ORCA) {
            neighboringObstacles = [];
            neighboringAgents = [];
        }

        // Update the target, if needed
        await this.updateTarget(offsetTraversableFaces);

        // Set our preferred velocity
        let prefVel = new Point(0, 0);
        if (this.curTarget != null) {
            // If there's a curTarget, our prefVelocity is prefDisplacement/Time
            prefVel = this.curTarget.minus(this.position).divide(deltaTime);
        }

        // Get ORCA Agents from all neighboring Agents
        let orcaAgents: ORCAAgent[] = [];
        for(let neighboringAgent of neighboringAgents) {
            orcaAgents.push(neighboringAgent.getORCAAgent());
        }

        // Set the current velocity to the ORCA velocity
        this.curVel = getORCAVelocity(boundingFace, neighboringObstacles, this.getORCAAgent(), orcaAgents, prefVel, speed, deltaTime);
        this.curSpeed = speed;
    }
    // Iterate the Agent
    iterate(deltaTime: number) {
        // Iterate using the curVel
        let deltaPos = this.curVel.multiply(deltaTime);
        this.position = this.position.plus(deltaPos);

        // timeUsed is how much it did move, versus how much it moves per second
        let timeUsed = deltaPos.magnitude() / this.curSpeed;
        let timeRemaining = Math.max(deltaTime - timeUsed, 0);

        // Save the most recent velocity
        this.prevVel = deltaPos.divide(deltaTime);

        // Return any remaining time to the caller
        return timeRemaining;
    }
}
