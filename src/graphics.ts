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

export function drawSegment(p1: Point, p2: Point, color: string = 'black', lineWidth: number = 1.01) {
    ctx.beginPath();
    ctx.moveTo(SCALE*p1.x, -SCALE*p1.y);
    ctx.lineTo(SCALE*p2.x, -SCALE*p2.y);

    ctx.save();
    ctx.strokeStyle = color;
    ctx.lineWidth = lineWidth;
    ctx.stroke();
    ctx.restore();
}

export function drawPoint(p: Point, color: string = 'black', radius: number = 5, isSquare: boolean = false) {
    radius *= SCALE;

    ctx.save();
    ctx.fillStyle = color;
    if (isSquare) {
        ctx.fillRect(SCALE*p.x-Math.floor(radius/2), -SCALE*p.y-Math.floor(radius/2), radius, radius);
    } else {
        ctx.beginPath();
        ctx.arc(SCALE * p.x, -SCALE * p.y, radius, 0, Math.PI * 2, true);
        ctx.closePath();
        ctx.fill();
    }
    ctx.restore();
}

export function drawFace(face: Face, fillColor: string | null, borderColor: string | null = null) {
    ctx.beginPath();
    let currentEdge = face.rootEdge;
    ctx.moveTo(SCALE*currentEdge.originPoint.x, -SCALE*currentEdge.originPoint.y);
    do {
        currentEdge = currentEdge.next;
        ctx.lineTo(SCALE*currentEdge.originPoint.x, -SCALE*currentEdge.originPoint.y);
    } while (currentEdge != face.rootEdge);
    ctx.closePath();

    ctx.save();
    // Draw interior
    if (fillColor != null) {
        ctx.fillStyle = fillColor;
        ctx.fill();
        if (borderColor == null) {
            ctx.strokeStyle = fillColor;
            ctx.miterLimit = 2;
            ctx.lineWidth = 3;
            ctx.stroke();
        }
    }
    // Draw border
    if (borderColor != null) {
        ctx.strokeStyle = borderColor;
        ctx.miterLimit = 4;
        ctx.lineWidth = 1.01;
        ctx.stroke();
    }
    ctx.restore();
}

export function drawPath(path: Point[], color: string = 'black', lineWidth: number = 1.01) {
    ctx.beginPath();
    ctx.moveTo(SCALE*path[0].x, -SCALE*path[0].y);
    for(let i = 1; i < path.length; i++) {
        ctx.lineTo(SCALE*path[i].x, -SCALE*path[i].y);
    }

    ctx.save();
    ctx.lineWidth = lineWidth;
    ctx.strokeStyle = color;
    ctx.stroke();
    ctx.restore();
}