// https://stackoverflow.com/questions/42919469/efficient-way-to-implement-priority-queue-in-javascript
const PQtop = 0;
const PQparent = i => ((i + 1) >>> 1) - 1;
const PQleft = i => (i << 1) + 1;
const PQright = i => (i + 1) << 1;

class PriorityQueue {
  constructor(comparator = (a, b) => a > b) {
    this._heap = [];
    this._comparator = comparator;
  }
  size() {
    return this._heap.length;
  }
  isEmpty() {
    return this.size() == 0;
  }
  peek() {
    return this._heap[PQtop];
  }
  push(...values) {
    values.forEach(value => {
      this._heap.push(value);
      this._siftUp();
    });
    return this.size();
  }
  pop() {
    const poppedValue = this.peek();
    const bottom = this.size() - 1;
    if (bottom > PQtop) {
      this._swap(PQtop, bottom);
    }
    this._heap.pop();
    this._siftDown();
    return poppedValue;
  }
  replace(value) {
    const replacedValue = this.peek();
    this._heap[PQtop] = value;
    this._siftDown();
    return replacedValue;
  }
  _greater(i, j) {
    return this._comparator(this._heap[i], this._heap[j]);
  }
  _swap(i, j) {
    [this._heap[i], this._heap[j]] = [this._heap[j], this._heap[i]];
  }
  _siftUp() {
    let node = this.size() - 1;
    while (node > PQtop && this._greater(node, PQparent(node))) {
      this._swap(node, PQparent(node));
      node = PQparent(node);
    }
  }
  _siftDown() {
    let node = PQtop;
    while (
      (PQleft(node) < this.size() && this._greater(PQleft(node), node)) ||
      (PQright(node) < this.size() && this._greater(PQright(node), node))
    ) {
      let maxChild = (PQright(node) < this.size() && this._greater(PQright(node), PQleft(node))) ? PQright(node) : PQleft(node);
      this._swap(node, maxChild);
      node = maxChild;
    }
  }
}