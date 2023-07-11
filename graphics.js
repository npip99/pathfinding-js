// Graphics Code

const canvas = document.getElementById('myCanvas');
const ctx = canvas.getContext('2d');
const SCALE = 12;

function setCanvasDimensions(width, height) {
    canvas.width = width;
    canvas.height = height;
}

function drawSegment(p1, p2, color) {
    if (color !== undefined) {
        ctx.strokeStyle = color;
    }
    ctx.beginPath();
    ctx.moveTo(SCALE*p1.x, -SCALE*p1.y);
    ctx.lineTo(SCALE*p2.x, -SCALE*p2.y);
    ctx.stroke();
}

function drawPoint(p, color=undefined, radius=5) {
    if (color !== undefined) {
        ctx.fillStyle = color;
    }
    ctx.fillRect(SCALE*p.x-Math.floor(radius/2), -SCALE*p.y-Math.floor(radius/2), radius, radius);
}

function drawSearchNode(searchNode, color) {
    if (color !== undefined) {
        ctx.fillStyle = color;
    }
    ctx.beginPath();
    ctx.moveTo(SCALE*searchNode.root.x, -SCALE*searchNode.root.y);
    ctx.lineTo(SCALE*searchNode.startPoint.x, -SCALE*searchNode.startPoint.y);
    ctx.lineTo(SCALE*searchNode.endPoint.x, -SCALE*searchNode.endPoint.y);
    ctx.lineTo(SCALE*searchNode.root.x, -SCALE*searchNode.root.y);
    ctx.closePath();
    ctx.fill();
}

function drawFace(face, fillColor) {
    ctx.beginPath();
    currentEdge = face.rootEdge;
    ctx.moveTo(SCALE*currentEdge.originPoint.x, -SCALE*currentEdge.originPoint.y);
    do {
        currentEdge = currentEdge.next;
        ctx.lineTo(SCALE*currentEdge.originPoint.x, -SCALE*currentEdge.originPoint.y);
    } while (currentEdge != face.rootEdge);
    if (fillColor !== undefined) {
        ctx.fillStyle = fillColor;
        ctx.strokeStyle = fillColor;
        ctx.fill();
    }
    ctx.stroke();
}

function drawPath(path, lineWidth) {
    ctx.lineWidth = lineWidth === undefined ? 1 : lineWidth;
    ctx.strokeStyle = 'purple';
    ctx.beginPath();
    ctx.moveTo(SCALE*path[0].x, -SCALE*path[0].y);
    for(let i = 1; i < path.length; i++) {
        ctx.lineTo(SCALE*path[i].x, -SCALE*path[i].y);
    }
    ctx.stroke();
}