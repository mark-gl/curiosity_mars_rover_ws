class Teleop {
  constructor(ros, scene) {
    this.keepPublishingTeleop;
    this.speed = 1;
    this.obstacle = false;

    this.cmd_vel_service = new ROSLIB.Service({
      ros: ros,
      name: "/curiosity_mars_rover/cmd_vel_obstacle",
      serviceType: "geometry_msgs/Twist",
    });


  }

  speedUp() {
    this.speed = this.speed * 1.1;
    document.getElementById("speed_state").innerHTML =
      Math.round(this.speed * 100) / 100;
  }

  slowDown() {
    this.speed = this.speed * 0.9;
    document.getElementById("speed_state").innerHTML =
      Math.round(this.speed * 100) / 100;
  }

  moveJoy(click) {
    var controls = [-click.detail.y, 0, 0, 0, 0, -click.detail.x];
    this.moveRobot(controls);
  }

  moveRobot(arr) {
    let twist = new ROSLIB.Message({
      twist: {
        linear: {
          x: arr[0] * this.speed,
          y: arr[1],
          z: arr[2],
        },
        angular: {
          x: arr[3],
          y: arr[4],
          z: arr[5] * this.speed,
        },
      },
    });
    this.cmd_vel_service.callService(twist, function (result) {
      switch (result.feedback) {
        case "Obstacle in front":
          document.getElementById("tele_state").innerHTML = "Blocked";
          document.getElementById("n").disabled = true;
          document.getElementById("ne").disabled = true;
          document.getElementById("nw").disabled = true;
          clearInterval(this.keepPublishingTeleop);
          this.obstacle = true;
          break;
        case "Obstacle in rear":
          document.getElementById("tele_state").innerHTML = "Blocked";
          document.getElementById("s").disabled = true;
          document.getElementById("se").disabled = true;
          document.getElementById("sw").disabled = true;
          clearInterval(this.keepPublishingTeleop);
          this.obstacle = true;
          break;
        default:
          if (this.obstacle) {
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

  teleop(params) {
    this.keepPublishingTeleop = setInterval(function () {
      this.moveRobot(params);
    }, 16);
  }

  telestop() {
    clearInterval(this.keepPublishingTeleop);
  }
}
