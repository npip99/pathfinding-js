// Graphics Code
import { Point, Face } from "./math";

export const canvas = document.getElementById('myCanvas') as HTMLCanvasElement;
export const ctx = canvas.getContext('2d') as CanvasRenderingContext2D;
export const SCALE = 18;

export function setCanvasDimensions(width: number, height: number) {
    canvas.width = width;
    canvas.height = height;
}

// Map x,y from the canvas, to their transformed versions
export function getTransformedCoordinates(x: number, y: number) {
    let transform = ctx.getTransform().inverse();
    let pt = transform.transformPoint(new DOMPoint(x, y));
    return {
        x: pt.x,
        y: pt.y,
    };
}

export function drawSegment(p1: Point, p2: Point, color?: string) {
    if (color !== undefined) {
        ctx.strokeStyle = color;
    }
    ctx.beginPath();
    ctx.moveTo(SCALE*p1.x, -SCALE*p1.y);
    ctx.lineTo(SCALE*p2.x, -SCALE*p2.y);
    ctx.stroke();
}

export function drawPoint(p: Point, color?: string, radius=5) {
    if (color !== undefined) {
        ctx.fillStyle = color;
    }
    ctx.fillRect(SCALE*p.x-Math.floor(radius/2), -SCALE*p.y-Math.floor(radius/2), radius, radius);
}

export function drawFace(face: Face, fillColor: string) {
    ctx.beginPath();
    let currentEdge = face.rootEdge;
    ctx.moveTo(SCALE*currentEdge.originPoint.x, -SCALE*currentEdge.originPoint.y);
    do {
        currentEdge = currentEdge.next;
        ctx.lineTo(SCALE*currentEdge.originPoint.x, -SCALE*currentEdge.originPoint.y);
    } while (currentEdge != face.rootEdge);
    ctx.strokeStyle = fillColor;
    ctx.miterLimit = 2;
    ctx.lineWidth = 3;
    ctx.closePath();
    ctx.fillStyle = fillColor;
    ctx.fill();
    ctx.stroke();
}

export function drawPath(path: Point[], lineWidth?: number) {
    ctx.lineWidth = lineWidth === undefined ? 1 : lineWidth;
    ctx.strokeStyle = 'purple';
    ctx.beginPath();
    ctx.moveTo(SCALE*path[0].x, -SCALE*path[0].y);
    for(let i = 1; i < path.length; i++) {
        ctx.lineTo(SCALE*path[i].x, -SCALE*path[i].y);
    }
    ctx.stroke();
}