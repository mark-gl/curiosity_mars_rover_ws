var armClient = new ROSLIB.Service({
  ros: ros,
  name: "/curiosity_mars_rover/arm_service",
  serviceType: "curiosity_mars_rover_control/Arm",
});

var armListener = new ROSLIB.Topic({
  ros: ros,
  name: "/curiosity_mars_rover/arm_state",
  messageType: "std_msgs/String",
});

function armToggle() {
  armClient.callService(requestToggle);
}

function armSet(id, value) {
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
  armClient.callService(req, function (result) {
    document.getElementById("arm_state").innerHTML =
      result.status_message.slice(16);
  });
}

function updateArmButtons(message) {
  switch (message) {
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

// Initialisation below

armClient.callService(requestPing, function (result) {
  document.getElementById("arm_state").innerHTML =
    result.status_message.slice(16);
  updateArmButtons(result.status_message.slice(16));
});

armListener.subscribe(function (message) {
  document.getElementById("arm_state").innerHTML = message.data;
  updateArmButtons(message.data);
});
