class Teleop {
  constructor(ros, scene) {
    // keepPublishingTeleop is used with setInterval to repeat function calls
    this.keepPublishingTeleop;
    this.speed = 1.0;
    // Whether an obstacle is blocking teleoperation
    this.obstacle = false;

    this.cmdVelService = new ROSLIB.Service({
      ros: ros,
      name: "/curiosity_mars_rover/cmd_vel_obstacle",
      serviceType: "geometry_msgs/Twist",
    });

    const robotMovementEvents = [
      "moveForward",
      "moveBackward",
      "turnLeft",
      "turnRight",
      "stopRobot",
    ];

    // Register movement events to class functions
    scene.addEventListener("moveJoy", this.moveJoy.bind(this));
    scene.addEventListener("speedUp", this.speedUp.bind(this));
    scene.addEventListener("slowDown", this.slowDown.bind(this));
    for (var i = 0; i < robotMovementEvents.length; i++) {
      scene.addEventListener(
        robotMovementEvents[i],
        this.bindControl.bind(this)
      );
    }
  }

  bindControl(event) {
    var type = event.type;
    var currentMappingActions = AFRAME.inputActions[AFRAME.currentInputMapping];
    var parameters = currentMappingActions[type]
      ? currentMappingActions[type].params
      : [0, 0, 0, 0, 0, 0];
    this.moveRobot(parameters);
  }

  moveJoy(click) {
    // Format moveRobot call for VR joysticks
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
    this.cmdVelService.callService(twist, this.updateButtons.bind(this));
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

  teleopStart(params) {
    // Start the teleoperation message publishing loop
    this.keepPublishingTeleop = setInterval(
      this.moveRobot.bind(this, params),
      16
    );
  }

  teleopStop() {
    // Stop loop when key is lifted
    clearInterval(this.keepPublishingTeleop);
  }

  updateButtons(result) {
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
        // Obstacle flag variable means that the changes below only occur once after an obstacle is cleared.
        // Without it, the interface would behave the same, but unnecessary document.getElementById calls would be made
        if (this.obstacle) {
          document.getElementById("tele_state").innerHTML = "OK";
          document.getElementById("n").disabled = false;
          document.getElementById("ne").disabled = false;
          document.getElementById("nw").disabled = false;
          document.getElementById("s").disabled = false;
          document.getElementById("se").disabled = false;
          document.getElementById("sw").disabled = false;
          this.obstacle = false;
        }
        break;
    }
  }
}
