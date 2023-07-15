function hungarian(costMatrix) {
    // Helper function to create an array of zeros
    let createArray = (length) => [...Array(length)].map(() => 0);

    // Add a row and a column of zeros at the beginning of the matrix
    let numRows = costMatrix.length;
    let numCols = costMatrix[0].length;
    let matrix = createArray(numRows+1);
    for(let r = 0; r <= numRows; r++) {
        matrix[r] = createArray(numCols+1);
        for(let c = 0; c <= numCols; c++) {
            if (r == 0 || c == 0) {
                matrix[r][c] = 0;
            } else {
                matrix[r][c] = costMatrix[r-1][c-1];
            }
        }
    }

    // Arrays for the dual variables u and v, permutation array p, and auxiliary array way
    let u = createArray(numRows + 1);
    let v = createArray(numCols + 1);
    let p = createArray(numCols + 1);
    let way = createArray(numCols + 1);

    // Step 1: Initialize variables and arrays
    for (let i = 1; i <= numRows; i++) {
        p[0] = i;
        let j0 = 0;
        let minv = createArray(numCols + 1).map(() => Infinity);
        let used = createArray(numCols + 1).map(() => false);

        // Step 2: Find an augmenting path
        do {
            used[j0] = true;
            let i0 = p[j0];
            let delta = Infinity;
            let j1;

            // Step 3: Update the labels and find the minimum non-zero delta
            for (let j = 1; j <= numCols; j++) {
                if (!used[j]) {
                    let cur = matrix[i0][j] - u[i0] - v[j];

                    if (cur < minv[j]) {
                        minv[j] = cur;
                        way[j] = j0;
                    }

                    if (minv[j] < delta) {
                        delta = minv[j];
                        j1 = j;
                    }
                }
            }

            // Step 4: Update the dual variables u and v
            for (let j = 0; j <= numCols; j++) {
                if (used[j]) {
                    u[p[j]] += delta;
                    v[j] -= delta;
                } else {
                    minv[j] -= delta;
                }
            }

            j0 = j1;
        } while (p[j0] !== 0);

        // Step 5: Update the permutation array p
        do {
            let j1 = way[j0];
            p[j0] = p[j1];
            j0 = j1;
        } while (j0);
    }

    // Return the permutation array p, but removing the padded 0's
    let adjustedP = p.slice(1).map((val) => val - 1);
    // Flip the permutation array so that it maps A to B
    let finalP = Array(numRows);
    for(let i = 0; i < numRows; i++) {
        finalP[adjustedP[i]] = i;
    }
    return finalP;
}

function testHungarian() {
    const A = [new Point(36.22, -10.15), new Point(36.175, -8.15), new Point(36.19, -9.15), new Point(36.129, -6.15), new Point(36.152, -7.15)];
    const B = [new Point(36.03, -6.16), new Point(36.10, -7.16), new Point(36.175, -8.158), new Point(36.246, -9.15), new Point(36.317, -10.15)];
    const costMatrix = [];
    for (let i = 0; i < A.length; i++) {
        const row = [];
        for (let j = 0; j < B.length; j++) {
            row.push(Math.pow(getPointDist(A[i], B[j]), 2));
            console.log(A[i], B[j], i, j, row[j]);
        }
        costMatrix.push(row);
    }

    console.log('Cost:', costMatrix);
    const mapping = hungarian(costMatrix);
    console.log('Mapping:', mapping); // Array representing the mapping of source points to destination points
}
