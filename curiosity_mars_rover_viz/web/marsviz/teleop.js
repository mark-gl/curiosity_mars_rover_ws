var keepPublishingTeleop;
var speed = 1;
var obstacle = false;

var cmd_vel_service = new ROSLIB.Service({
  ros: ros,
  name: "/curiosity_mars_rover/cmd_vel_obstacle",
  serviceType: "geometry_msgs/Twist",
});

function speedUp() {
  speed = speed * 1.1;
  document.getElementById("speed_state").innerHTML =
    Math.round(speed * 100) / 100;
}

function slowDown() {
  speed = speed * 0.9;
  document.getElementById("speed_state").innerHTML =
    Math.round(speed * 100) / 100;
}

function moveJoy(click) {
  controls = [-click.detail.y, 0, 0, 0, 0, -click.detail.x];
  moveRobot(controls);
}

function moveRobot(arr) {
  let twist = new ROSLIB.Message({
    twist: {
      linear: {
        x: arr[0] * speed,
        y: arr[1],
        z: arr[2],
      },
      angular: {
        x: arr[3],
        y: arr[4],
        z: arr[5] * speed,
      },
    },
  });
  cmd_vel_service.callService(twist, function (result) {
    switch (result.feedback) {
      case "Obstacle in front":
        document.getElementById("tele_state").innerHTML = "Blocked";
        document.getElementById("n").disabled = true;
        document.getElementById("ne").disabled = true;
        document.getElementById("nw").disabled = true;
        clearInterval(keepPublishingTeleop);
        obstacle = true;
        break;
      case "Obstacle in rear":
        document.getElementById("tele_state").innerHTML = "Blocked";
        document.getElementById("s").disabled = true;
        document.getElementById("se").disabled = true;
        document.getElementById("sw").disabled = true;
        clearInterval(keepPublishingTeleop);
        obstacle = true;
        break;
      default:
        if (obstacle) {
          document.getElementById("tele_state").innerHTML = "OK";
          document.getElementById("n").disabled = false;
          document.getElementById("ne").disabled = false;
          document.getElementById("nw").disabled = false;
          document.getElementById("s").disabled = false;
          document.getElementById("se").disabled = false;
          document.getElementById("sw").disabled = false;
        }
        break;
    }
  });
}

function teleop(params) {
  keepPublishingTeleop = setInterval(function () {
    moveRobot(params);
  }, 16);
}

function telestop() {
  clearInterval(keepPublishingTeleop);
}
