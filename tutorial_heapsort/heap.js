/*|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\

    Heap Sort Stencil | JavaScript support functions

    Quick JavaScript Code-by-Example Tutorial 
     
    @author ohseejay / https://github.com/ohseejay
                     / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Michigan Honor License 

|\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/|
||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/
/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\
\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/||\/*/

// create empty object
minheaper = {};

// define insert function for min binary heap
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

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
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

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
// STENCIL: ensure extract method is within minheaper object
