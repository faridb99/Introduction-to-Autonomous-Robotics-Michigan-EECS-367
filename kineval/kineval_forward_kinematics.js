/*-- |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/|

    KinEval | Kinematic Evaluator | forward kinematics

    Implementation of robot kinematics, control, decision making, and dynamics 
        in HTML5/JavaScript and threejs
     
    @author ohseejay / https://github.com/ohseejay / https://bitbucket.org/ohseejay

    Chad Jenkins
    Laboratory for Perception RObotics and Grounded REasoning Systems
    University of Michigan

    License: Creative Commons 3.0 BY-SA

|\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| |\/| --*/

kineval.robotForwardKinematics = function robotForwardKinematics() {
  if (typeof kineval.buildFKTransforms === "undefined") {
    textbar.innerHTML = "forward kinematics not implemented";
    return;
  }

  kineval.buildFKTransforms();
};

// STENCIL: reference code alternates recursive traversal over
//   links and joints starting from base, using following functions:
//     traverseFKBase
//     traverseFKLink
//     traverseFKJoint
//
// user interface needs the heading (z-axis) and lateral (x-axis) directions
//   of robot base in world coordinates stored as 4x1 matrices in
//   global variables "robot_heading" and "robot_lateral"
//
// if geometries are imported and using ROS coordinates (e.g., fetch),
//   coordinate conversion is needed for kineval/threejs coordinates:
//

kineval.buildFKTransforms = function buildFKTransforms() {
  console.log("BEFORE", robot);
  console.log("//////////////////////////////////////////////////////////////");
  let s = [];
  let mStack = [generate_identity()];
  let visited = {};

  let originTranslationMat = generate_translation_matrix(...robot.origin.xyz);

  let originRotationMatX = generate_rotation_matrix_X(robot.origin.rpy[0]);
  let originRotationMatY = generate_rotation_matrix_Y(robot.origin.rpy[1]);
  let originRotationMatZ = generate_rotation_matrix_Z(robot.origin.rpy[2]);

  robot.origin.xform = matrix_multiply(
    originTranslationMat,
    matrix_multiply(
      originRotationMatZ,
      matrix_multiply(
        originRotationMatY,
        matrix_multiply(originRotationMatX, mStack.pop())
      )
    )
  );

  mStack.push(robot.origin.xform);
  s.push(robot.links["base"]);

  while (s.length > 0) {
    const tempLink = s.pop();
    robot.links[tempLink.name].xform = mStack.pop();

    //console.log("LINK:", tempLink);
    if (visited[tempLink.name] === undefined) {
      visited[tempLink.name] = true;
      const linkChildren = tempLink?.children;
      if (linkChildren !== undefined) {
        linkChildren.forEach((tempChild) => {
          if (visited[robot.joints[tempChild].child] === undefined) {
            //console.log("JOINT:", robot.joints[tempChild]);
            //visited[robot.joints[tempChild].child] = true;
            let originTranslationMat = generate_translation_matrix(
              ...robot.joints[tempChild].origin.xyz
            );

            let originRotationMatX = generate_rotation_matrix_X(
              robot.joints[tempChild].origin.rpy[0]
            );
            let originRotationMatY = generate_rotation_matrix_Y(
              robot.joints[tempChild].origin.rpy[1]
            );
            let originRotationMatZ = generate_rotation_matrix_Z(
              robot.joints[tempChild].origin.rpy[2]
            );

            robot.joints[tempChild].xform = matrix_multiply(
              originTranslationMat,
              matrix_multiply(
                originRotationMatZ,
                matrix_multiply(
                  originRotationMatY,
                  matrix_multiply(
                    originRotationMatX,
                    robot.links[tempLink.name].xform
                  )
                )
              )
            );

            console.log(robot.joints[tempChild].name);
            mStack.push(robot.joints[tempChild].xform);
            var childLinkTempName = robot.joints[tempChild].child;
            s.push(robot.links[childLinkTempName]);
          }
        });
      }
    }
  }
  console.log("//////////////////////////////////////////////////////////////");
  console.log("AFTER", robot);
};
