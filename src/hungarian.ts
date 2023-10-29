export function hungarian(costMatrix: number[][]): (number | null)[] {
    // https://en.wikipedia.org/wiki/Hungarian_algorithm
    const J = costMatrix.length;
    const W = costMatrix[0].length;
    let job: (number | null)[];
    if (J == 0 || W == 0) {
        job = [];
    } else if (J > W) {
        const costMatrixTransposed = costMatrix[0].map((_, colIndex) => costMatrix.map(row => row[colIndex]));
        job = hungarian(costMatrixTransposed);
    } else {
        job = new Array(W + 1).fill(null); // job assigned to w-th worker, or -1 if no job assigned
        let ys: number[] = new Array(J).fill(0);
        let yt: number[] = new Array(W + 1).fill(0);  // potentials

        for (let j_cur = 0; j_cur < J; ++j_cur) {  // assign j_cur-th job
            let w_cur = W;
            job[w_cur] = j_cur;
            let min_to: number[] = new Array(W + 1).fill(Infinity);
            let prv: number[] = new Array(W + 1).fill(-1);  // previous worker on alternating path
            let in_Z: boolean[] = new Array(W + 1).fill(false);  // whether worker is in Z

            while (job[w_cur] !== null) {   // runs at most j_cur + 1 times
                in_Z[w_cur] = true;
                const j = job[w_cur]!;
                let delta = Infinity;
                let w_next;

                for (let w = 0; w < W; w++) {
                    if (!in_Z[w]) {
                        if (min_to[w] > (costMatrix[j][w] - ys[j] - yt[w])) {
                            min_to[w] = costMatrix[j][w] - ys[j] - yt[w];
                            prv[w] = w_cur;
                        }
                        if (delta > min_to[w]) {
                            delta = min_to[w];
                            w_next = w;
                        }
                    }
                }

                for (let w = 0; w <= W; w++) {
                    if (in_Z[w]) {
                        ys[job[w]!] += delta;
                        yt[w] -= delta;
                    } else {
                        min_to[w] -= delta;
                    }
                }
                if (w_next == undefined) {
                    throw "Invalid w_next!";
                }
                w_cur = w_next;
            }

            // update assignments along alternating path
            for (let w = -1; w_cur !== -1; w_cur = w) {
                w = prv[w_cur];
                job[w_cur] = job[w];
            }
        }
        job.splice(job.length-1);
    }
    // Flip the permutation array so that it maps A to B
    let finalP: (number | null)[] = Array(J).fill(null);
    for(let i = 0; i < job.length; i++) {
        if (job[i] !== null) {
            finalP[job[i]!] = i;
        }
    }
    return finalP;
}


function testHungarian() {
    let A = [[36.22, -10.15], [36.175, -8.15], [36.19, -9.15], [36.129, -6.15], [36.152, -7.15], [36.10, -7.16]];
    let B = [[36.03, -6.16], [36.10, -7.16], [36.175, -8.158], [36.246, -9.15], [36.317, -10.15]];
    A = A.concat(A);
    A = A.concat(A);
    A = A.concat(A);
    A = A.concat(A);
    A = A.concat(A);
    A = A.concat(A);
    B = B.concat(B);
    B = B.concat(B);
    B = B.concat(B);
    B = B.concat(B);
    B = B.concat(B);
    let costMatrix = [];
    for (let i = 0; i < A.length; i++) {
        const row = [];
        for (let j = 0; j < B.length; j++) {
            row.push(Math.sqrt(Math.pow(A[i][0] - B[j][0], 2) + Math.pow(A[i][1] - B[j][1], 2)));
            //console.log(A[i], B[j], i, j, row[j]);
        }
        costMatrix.push(row);
    }
    console.log('Cost:', costMatrix);
    console.log('Dimensions:', A.length, 'x', B.length);
    // Array representing the mapping of source points to destination points
    let startTime = performance.now();
    let mapping = hungarian(costMatrix);
    let endTime = performance.now();
    console.log('Hungarian Mapping:', mapping);
    console.log('Time:', endTime-startTime, 'ms');
    for(let i = 0; i < A.length; i++) {
        let j = mapping[i];
        //console.log(A[i], " => ", j == null ? null : B[j]);
    }
}
