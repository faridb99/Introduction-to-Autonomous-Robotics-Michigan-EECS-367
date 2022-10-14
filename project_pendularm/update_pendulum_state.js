function update_pendulum_state(numerical_integrator, pendulum, dt, gravity) {
  // integrate pendulum state forward in time by dt
  // please use names 'pendulum.angle', 'pendulum.angle_previous', etc. in else codeblock between line 28-30

  if (typeof numerical_integrator === "undefined")
    numerical_integrator = "none";

  if (numerical_integrator === "euler") {
    // STENCIL: a correct Euler integrator is REQUIRED for assignment
    pendulum.angle_previous = pendulum.angle;
    pendulum.angle = pendulum.angle + pendulum.angle_dot * dt;
    pendulum.angle_dot = pendulum.angle_dot + pendulum.angle_dot_dot * dt;
  } else if (numerical_integrator === "verlet") {
    // STENCIL: basic Verlet integration

    var temp = pendulum.angle_previous;

    pendulum.angle_previous = pendulum.angle;

    pendulum.angle =
      2 * pendulum.angle_previous -
      temp +
      pendulum.angle_dot_dot * Math.pow(dt, 2);
  } else if (numerical_integrator === "velocity verlet") {
    // STENCIL: a correct velocity Verlet integrator is REQUIRED for assignment

    pendulum.angle_previous = pendulum.angle;

    pendulum.angle =
      pendulum.angle +
      pendulum.angle_dot * dt +
      0.5 * pendulum.angle_dot_dot * Math.pow(dt, 2);

    pendulum.angle_dot =
      pendulum.angle_dot +
      0.5 *
        (pendulum.angle_dot_dot + pendulum_acceleration(pendulum, gravity)) *
        dt;
  } else if (numerical_integrator === "runge-kutta") {
    // STENCIL: Runge-Kutta 4 integrator
  } else {
    /*
    pendulum.angle_previous = pendulum.angle;
    pendulum.angle = (pendulum.angle + Math.PI / 180) % (2 * Math.PI);
    pendulum.angle_dot = (pendulum.angle - pendulum.angle_previous) / dt;*/
    numerical_integrator = "none";
  }

  return pendulum;
}

function pendulum_acceleration(pendulum, gravity) {
  // STENCIL: return acceleration(s) system equation(s) of motion
  return (
    -(gravity / pendulum.length) * Math.sin(pendulum.angle) +
    pendulum.control / (pendulum.mass * pendulum.length * pendulum.length)
  );
}

function init_verlet_integrator(pendulum, t, gravity) {
  // STENCIL: for verlet integration, a first step in time is needed
  // return: updated pendulum state and time
  pendulum.angle_previous = pendulum.angle;
  pendulum.angle =
    pendulum.angle_previous +
    dt * pendulum.angle_dot +
    0.5 * Math.pow(dt, 2) * pendulum.angle_dot_dot;
  t = t + dt;
  return [pendulum, t];
}

function set_PID_parameters(pendulum) {
  // STENCIL: change pid parameters
  pendulum.servo = { kp: 0.5, kd: 70, ki: 0.02 }; // no control
  pendulum.servo.error = 0;
  return pendulum;
}

function PID(pendulum, accumulated_error, dt) {
  // STENCIL: implement PID controller
  // return: updated output in pendulum.control and accumulated_error
  var p_term = pendulum.desired - pendulum.angle;
  var d_term = p_term - pendulum.servo.error;

  pendulum.servo.error = d_term;
  accumulated_error = p_term + accumulated_error;
  console.log(pendulum.servo.error);
  pendulum.control =
    pendulum.servo.kp * p_term +
    pendulum.servo.ki * accumulated_error +
    pendulum.servo.kd * d_term;
  return [pendulum, accumulated_error];
}
