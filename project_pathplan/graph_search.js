/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    2D Path Planning in HTML5 Canvas | Graph Search Methods

    Stencil methods for student implementation of graph search algorithms.

    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License

    Usage: see search_canvas.html

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

function initSearchGraph() {
  // create the search queue
  visit_queue = [];
  // initialize search graph as 2D array over configuration space
  //   of 2D locations with specified spatial resolution
  G = [];
  for (iind = 0, xpos = -2; xpos < 7; iind++, xpos += eps) {
    G[iind] = [];
    for (jind = 0, ypos = -2; ypos < 7; jind++, ypos += eps) {
      G[iind][jind] = {
        i: iind,
        j: jind, // mapping to graph array
        x: xpos,
        y: ypos, // mapping to map coordinates
        parent: null, // pointer to parent in graph along motion path
        distance: 10000, // distance to start via path through parent
        visited: false, // flag for whether the node has been visited
        priority: null, // visit priority based on fscore
        queued: false, // flag for whether the node has been queued for visiting
      };

      // STENCIL: determine whether this graph node should be the start
      //   point for the search
      if (xpos < 0.01 && xpos > -0.01 && ypos < 0.01 && ypos > -0.01) {
        G[iind][jind].distance = 0;
        G[iind][jind].visited = true;
        visit_queue.push(G[iind][jind]);
      }
    }
  }
}

function iterateGraphSearch() {
  // STENCIL: implement a single iteration of a graph search algorithm
  //   for A-star (or DFS, BFS, Greedy Best-First)
  //   An asynch timing mechanism is used instead of a for loop to avoid
  //   blocking and non-responsiveness in the browser.
  //
  //   Return "failed" if the search fails on this iteration.
  //   Return "succeeded" if the search succeeds on this iteration.
  //   Return "iterating" otherwise.
  //
  //   Provided support functions:
  //
  //   testCollision - returns whether a given configuration is in collision
  //   drawHighlightedPathGraph - draws a path back to the start location
  //   draw_2D_configuration - draws a square at a given location

  current_node = visit_queue.pop();
  //current_node = minheap_extract(visit_queue)
  console.log(
    testCollision([
      G[current_node.i][current_node.j].x,
      G[current_node.i][current_node.j].y,
    ]),
    G[current_node.i][current_node.j]
  );
  G[current_node.i][current_node.j].visited = true;

  adjacent = [
    G[current_node.i - 1][current_node.j],
    G[current_node.i][current_node.j - 1],
    G[current_node.i + 1][current_node.j],
    G[current_node.i][current_node.j + 1],
  ];

  adjacent.forEach((element) => {
    if (!element.queued && !testCollision([element.x, element.y])) {
      visit_queue.push(element);

      element.queued = true;
      if (element.distance > G[current_node.i][current_node.j].distance + eps) {
        element.parent = G[current_node.i][current_node.j];
        element.distance = G[current_node.i][current_node.j].distance + eps;
        element.visited = true;
      }
    }
  });

  if (
    testCollision([
      G[current_node.i][current_node.j].x,
      G[current_node.i][current_node.j].y,
    ])
  ) {
    drawHighlightedPathGraph(current_node);
    search_iterate = false;
    return "failed";
  } else if (G[current_node.i][current_node.j] == G[60][60]) {
    drawHighlightedPathGraph(current_node);
    search_iterate = false;

    return "succeeded";
  } else {
    return "iterating";
  }
}

//////////////////////////////////////////////////
/////     MIN HEAP IMPLEMENTATION FUNCTIONS
//////////////////////////////////////////////////

// STENCIL: implement min heap functions for graph search priority queue.
//   These functions work use the 'priority' field for elements in graph.

function minheap_insert(heap, new_element) {
  // STENCIL: implement your min binary heap insert operation
  var elem_index = heap.length;
  var parent_index = Math.floor((elem_index - 1) / 2);

  heap.push(new_element);

  var heaped = elem_index <= 0 || heap[parent_index] < heap[elem_index];

  while (!heaped) {
    var temp = heap[parent_index];
    heap[parent_index] = heap[elem_index];
    heap[elem_index] = temp;

    elem_index = parent_index;
    parent_index = Math.floor((elem_index - 1) / 2);

    var heaped = elem_index <= 0 || heap[parent_index] < heap[elem_index];
  }
}
function minheap_extract(heap) {
  // STENCIL: implement your min binary heap extract operation
  if (heap.length <= 0) {
    console.log("Heap is empty!");
  }
  if (heap.length == 1) {
    return heap.pop();
  }

  var min_num = heap[0];

  heap[0] = heap.pop();

  //HEAPIFY

  var smallest = 0;

  var elem_index = 0;

  while (true) {
    var left = 2 * smallest + 1;
    var right = 2 * smallest + 2;

    if (left < heap.length && heap[left] < heap[smallest]) {
      smallest = left;
    }
    if (right < heap.length && heap[right] < heap[smallest]) {
      smallest = right;
    }

    if (smallest != elem_index) {
      var temp = heap[smallest];
      heap[smallest] = heap[elem_index];
      heap[elem_index] = temp;
      elem_index = smallest;
    } else {
      break;
    }
  }

  return min_num;
}
