//////////////////////////////////////////////////
/////     MATRIX ALGEBRA AND GEOMETRIC TRANSFORMS
//////////////////////////////////////////////////

function matrix_copy(m1) {
  // returns 2D array that is a copy of m1

  var mat = [];
  var i, j;

  for (i = 0; i < m1.length; i++) {
    // for each row of m1
    mat[i] = [];
    for (j = 0; j < m1[0].length; j++) {
      // for each column of m1
      mat[i][j] = m1[i][j];
    }
  }
  return mat;
}

// STENCIL: reference matrix code has the following functions:
//   matrix_multiply
//   matrix_transpose
//   matrix_pseudoinverse
//   matrix_invert_affine
//   vector_normalize
//   vector_cross
//   generate_identity
//   generate_translation_matrix
//   generate_rotation_matrix_X
//   generate_rotation_matrix_Y
//   generate_rotation_matrix_Z

// **** Function stencils are provided below, please uncomment and implement them ****//

function matrix_multiply(m1, m2) {
  // returns 2D array that is the result of m1*m2
  let x = m1.length,
    z = m1[0].length,
    y = m2[0].length;

  let productRow = Array.apply(null, new Array(y)).map(
    Number.prototype.valueOf,
    0
  );
  let mat = new Array(x);
  for (let p = 0; p < x; p++) {
    mat[p] = productRow.slice();
  }
  for (let i = 0; i < x; i++) {
    for (let j = 0; j < y; j++) {
      for (let k = 0; k < z; k++) {
        mat[i][j] += m1[i][k] * m2[k][j];
      }
    }
  }
  return mat;
}

function matrix_transpose(m) {
  //     // returns 2D array that is the result of m1*m2
  let mat = matrix_copy(m);

  for (let i = 0; i < m.length; i++) {
    for (let j = 0; j < i; j++) {
      const tmp = mat[i][j];
      mat[i][j] = mat[j][i];
      mat[j][i] = tmp;
    }
  }
  return mat;
}

//function matrix_pseudoinverse(m) {

//}

function matrix_invert_affine(m) {
  //     // returns 2D array that is the invert affine of 4-by-4 matrix m
  return [
    [
      m[0][0],
      m[1][0],
      m[2][0],
      m[0][0] * m[0][3] - m[1][0] * m[1][3] - m[2][0] * m[2][3],
    ],
    [
      m[0][1],
      m[1][1],
      m[2][1],
      m[0][1] * m[0][3] - m[1][1] * m[1][3] - m[2][1] * m[2][3],
    ],
    [
      m[0][2],
      m[1][2],
      m[2][2],
      m[0][2] * m[0][3] - m[1][2] * m[1][3] - m[2][2] * m[2][3],
    ],
    [0, 0, 0, 1],
  ];
}

function vector_normalize(v) {
  //returns normalized vector for v
  let x = v[0];
  let y = v[1];
  let z = v[2];
  let magnitude = Math.sqrt(x * x + y * y + z * z);
  return [x / magnitude, y / magnitude, z / magnitude];
}

function vector_cross(a, b) {
  // return cross product of vector a and b with both has 3 dimensions
  let c = [
    a[1] * b[2] - a[2] * b[1],
    a[2] * b[0] - a[0] * b[2],
    a[0] * b[1] - a[1] * b[0],
  ];

  return c;
}

function generate_identity() {
  // returns 4-by-4 2D array of identity matrix
  return [
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}

function generate_translation_matrix(tx, ty, tz) {
  // returns 4-by-4 matrix as a 2D array
  return [
    [1, 0, 0, tx],
    [0, 1, 0, ty],
    [0, 0, 1, tz],
    [0, 0, 0, 1],
  ];
}

function generate_rotation_matrix_X(angle) {
  //     // returns 4-by-4 matrix as a 2D array, angle is in radians
  return [
    [1, 0, 0, 0],
    [0, Math.cos(angle), -Math.sin(angle), 0],
    [0, Math.sin(angle), Math.cos(angle), 0],
    [0, 0, 0, 1],
  ];
}

function generate_rotation_matrix_Y(angle) {
  //     // returns 4-by-4 matrix as a 2D array, angle is in radians

  return [
    [Math.cos(angle), 0, Math.sin(angle), 0],
    [0, 1, 0, 0],
    [-Math.sin(angle), 0, Math.cos(angle), 0],
    [0, 0, 0, 1],
  ];
}

function generate_rotation_matrix_Z(angle) {
  //     // returns 4-by-4 matrix as a 2D array, angle is in radians

  return [
    [Math.cos(angle), -Math.sin(angle), 0, 0],
    [Math.sin(angle), Math.cos(angle), 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1],
  ];
}
