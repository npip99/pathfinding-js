import { Point, Face, EPSILON, mergeAllFaces, markCorners, findBadFace, faceFromPolyline, offsetFace } from "./math";
import { traverableFacesFromObstacles } from "./utils";
import { Agent } from "./agent";
import { Formation } from "./formation";

const AGENT_RADIUS = 0.3;

class GroupInformation {
    formation: Formation | null;
    agentIDs: number[];
    constructor(agentIDs: number[]) {
        this.agentIDs = agentIDs;
        this.formation = null;
    }
    addAgent(agentID: number) {
        this.agentIDs.push(agentID);
    }
    removeAgent(agentID: number) {
        this.agentIDs.splice(this.agentIDs.indexOf(agentID), 1);
    }
};

export class Pathfinding {
    // Environment Data
    topLeft: Point;
    bottomRight: Point;
    boundingFace: Face;
    obstacles: Map<number, Face>;
    agents: Map<number, Agent>;
    nextObjectID: number;

    // Group Data
    nextGroupID: number;
    groups: Map<number, GroupInformation>;
    agentToGroup: Map<number, number>;

    // Environment Cache
    traversableFaces: Face[];
    offsetTraversableFaces: Face[];

    constructor(topLeft: Point, bottomRight: Point) {
        let minX = topLeft.x;
        let maxY = topLeft.y;
        let maxX = bottomRight.x;
        let minY = bottomRight.y;
        this.topLeft = topLeft;
        this.bottomRight = bottomRight;
        this.boundingFace = faceFromPolyline([[minX, maxY], [minX, minY], [maxX, minY], [maxX, maxY]].map(pt => new Point(pt[0], pt[1])));
        this.obstacles = new Map<number, Face>();
        this.agents = new Map<number, Agent>();
        this.nextObjectID = 1;

        this.groups = new Map<number, GroupInformation>();
        this.agentToGroup = new Map<number, number>();
        this.nextGroupID = 1;

        // Generate Cache
        this.sync();
    }

    addObstacle(obstacleFace: Face): number {
        let obstacleObjectID = this.nextObjectID;
        this.nextObjectID++;
        this.obstacles.set(obstacleObjectID, obstacleFace);
        this.sync();
        return obstacleObjectID;
    }

    removeObstacle(obstacleID: number) {
        if (this.obstacles.delete(obstacleID)) {
            this.sync();
        }
    }

    addAgent(position: Point, radius: number, max_speed: number): number {
        let agentObjectID = this.nextObjectID;
        this.nextObjectID++;
        let agent = new Agent(position, radius, max_speed);
        this.agents.set(agentObjectID, agent);
        return agentObjectID;
    }

    removeAgent(agentID: number) {
        this.agents.delete(agentID);
    }

    getAgent(agentID: number): Agent | null {
        return this.agents.get(agentID) || null;
    }

    groupCreate() {
        let groupID = this.nextGroupID;
        this.nextGroupID++;
        let groupInfo = new GroupInformation([]);
        this.groups.set(groupID, groupInfo);

        let formation = new Formation(this.boundingFace, Array.from(this.obstacles.values()), this.offsetTraversableFaces);
        groupInfo.formation = formation;

        return groupID;
    }

    groupDestroy() {
        throw "TODO: Implement";   
    }

    groupAddAgent(groupID: number, agentID: number) {
        let groupInfo = this.groups.get(groupID)!;
        groupInfo.addAgent(agentID);
        groupInfo.formation!.addAgents([this.agents.get(agentID)!]);
        this.agentToGroup.set(agentID, groupID);
    }

    groupRemoveAgent(groupID: number, agentID: number) {
        this.groups.get(groupID)!.removeAgent(agentID);
        this.agentToGroup.delete(agentID);
    }

    async groupPathfind(groupID: number, position: Point) {
        let groupInfo = this.groups.get(groupID)!;
        await groupInfo.formation!.pathfind(position);
    }

    groupStop(groupID: number) {
        let groupInfo = this.groups.get(groupID)!;
        if (groupInfo.formation) {
            groupInfo.formation.stop();
        }
    }

    freezeAgent(agentID: number) {

    }

    unfreezeAgent(agentID: number) {
        
    }

    getNearbyObjects(position: Point, distance: number): number[] {
        return [];
    }

    isPathing(agentID: number): boolean {
        return false;
    }

    stop(agentID: number) {

    }

    async iterate(deltaTime: number) {
        let obstacleFaces = Array.from(this.obstacles.values());
        let formationIterates: Formation[] = [];

        // agentID -> Ideal Speed
        let idealSpeeds: Map<number, number> = new Map<number, number>();
        // Pre-Iterate all formations, obtaining all ideal speeds as well
        for(let [groupID, groupInfo] of this.groups.entries()) {
            if (groupInfo.formation != null) {
                await groupInfo.formation.preIterate();
                let formationIdealSpeeds = groupInfo.formation.getIdealSpeeds(deltaTime);
                for(let i = 0; i < groupInfo.agentIDs.length; i++) {
                    let idealSpeed = formationIdealSpeeds[i];
                    if (idealSpeed != null) {
                        idealSpeeds.set(groupInfo.agentIDs[i], idealSpeed);
                    }
                }
                formationIterates.push(groupInfo.formation);
            }
        }
        // Iterate all agents to consider all neighboring agents
        for(let [agentID, agent] of this.agents.entries()) {
            // TODO: Use k-d tree for neighboring agents, instead of just using all of them
            let otherAgents = Array.from(this.agents.values()).filter(otherAgent => otherAgent != agent);
            await agent.considerNeighboringAgents(this.boundingFace, obstacleFaces, this.offsetTraversableFaces, otherAgents, deltaTime, idealSpeeds.get(agentID));
        }
        // Iterate all agents
        for(let [agentID, agent] of this.agents.entries()) {
            agent.iterate(deltaTime);
        }
        // Post-Iterate all formations
        for(let formation of formationIterates) {
            formation.postIterate();
        }
    }

    sync() {
        // Create traversableFaces and offsetTraversableFaces, for navigation
        let obstacleFaces = Array.from(this.obstacles.values());
        this.traversableFaces = traverableFacesFromObstacles(this.boundingFace, obstacleFaces);
        // TODO: Cut facets into acute vertices during offset, so that 0.99/sqrt(2) is guaranteed to be safe
        const OBSTACLE_OFFSET = AGENT_RADIUS*0.99/Math.sqrt(2);
        this.offsetTraversableFaces = traverableFacesFromObstacles(
            offsetFace(this.boundingFace, -OBSTACLE_OFFSET),
            obstacleFaces.map(face => offsetFace(face, OBSTACLE_OFFSET)),
        );

        // Simplify the mesh, mark corners, and validate the mesh
        mergeAllFaces(this.traversableFaces);
        mergeAllFaces(this.offsetTraversableFaces);
        markCorners(this.offsetTraversableFaces);
        let badFace = findBadFace(this.offsetTraversableFaces);
        if (badFace != null) {
            console.log('Bad Face Found!', badFace);
            console.log(badFace.rootEdge, badFace.rootEdge.next, badFace.rootEdge.next.next, badFace.rootEdge.next.next.next, badFace.rootEdge.next.next.next.next);
            return;
        }  
    }
};
