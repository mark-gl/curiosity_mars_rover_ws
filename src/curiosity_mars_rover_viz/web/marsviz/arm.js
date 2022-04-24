class Arm {
  constructor(ros, scene) {
    this.armClient = new ROSLIB.Service({
      ros: ros,
      name: "/curiosity_mars_rover/arm_service",
      serviceType: "curiosity_mars_rover_control/Arm",
    });

    this.armListener = new ROSLIB.Topic({
      ros: ros,
      name: "/curiosity_mars_rover/arm_state",
      messageType: "std_msgs/String",
    });

    this.armListener.subscribe(this.updateArmButtons.bind(this));

    this.armClient.callService(new ROSLIB.ServiceRequest({ mode: "ping" }));

    scene.addEventListener("armToggle", this.armToggle.bind(this));
  }

  armToggle() {
    this.armClient.callService(new ROSLIB.ServiceRequest({ mode: "toggle" }));
  }

  armSet(id, value) {
    // HTML sliders are remapped to the joint rotations (eg, from 1 -> 100 to 0 -> -1.57)
    var req = new ROSLIB.ServiceRequest({});
    switch (id) {
      case "joint1":
        req = new ROSLIB.ServiceRequest({
          mode: "set",
          pos_arm_01: parseFloat((-1.57 * value) / 100),
        });
        break;
      case "joint2":
        req = new ROSLIB.ServiceRequest({
          mode: "set",
          pos_arm_02: parseFloat((-1.57 * value) / 100),
        });
        break;
      case "joint3":
        req = new ROSLIB.ServiceRequest({
          mode: "set",
          pos_arm_03: parseFloat((-0.9 * value) / 100),
        });
        break;
      case "joint4":
        req = new ROSLIB.ServiceRequest({
          mode: "set",
          pos_arm_04: parseFloat((-1.57 * value) / 100),
        });
        break;
      case "effector":
        req = new ROSLIB.ServiceRequest({
          mode: "set",
          pos_arm_tools: parseFloat((-1.57 * value) / 100),
        });
        break;
      default:
        break;
    }
    this.armClient.callService(req);
  }

  updateArmButtons(result) {
    document.getElementById("arm_state").innerHTML = result.data.slice(16);
    switch (result.data.slice(16)) {
      case "Closed":
        document.getElementById("joint1").disabled = true;
        document.getElementById("joint2").disabled = true;
        document.getElementById("joint3").disabled = true;
        document.getElementById("joint4").disabled = true;
        document.getElementById("effector").disabled = true;
        break;
      case "Open":
        document.getElementById("joint1").disabled = false;
        document.getElementById("joint2").disabled = false;
        document.getElementById("joint3").disabled = false;
        document.getElementById("joint4").disabled = false;
        document.getElementById("effector").disabled = false;
        // Reset sliders when arm is opened
        document.getElementById("joint1").value = 0.0;
        document.getElementById("joint2").value = 0.0;
        document.getElementById("joint3").value = 0.0;
        document.getElementById("joint4").value = 0.0;
        document.getElementById("effector").value = 0.0;
        break;
      default:
        break;
    }
  }
}
