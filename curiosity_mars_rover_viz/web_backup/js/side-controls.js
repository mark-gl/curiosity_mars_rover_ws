var buttonDown = false;
var stitching = false;

function delay(n) {
  return new Promise(function (resolve) {
    setTimeout(resolve, n * 1000);
  });
}

var ros = new ROSLIB.Ros({
  url: "wss://127.0.0.1:9090",
});

ros.on("connection", function () {
  console.log("Connected to ROS!");
});

// Mast and Arm
function mast() {
  mastClient.callService(requestToggle, function (result) {
    document.getElementById("mast_state").innerHTML =
      result.status_message.slice(17);
  });
}
function arm() {
  armClient.callService(requestToggle, function (result) {
    document.getElementById("arm_state").innerHTML =
      result.status_message.slice(16);
  });
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

var cmd_vel_service = new ROSLIB.Service({
  ros: ros,
  name: "/curiosity_mars_rover/cmd_vel_obstacle",
  serviceType: "geometry_msgs/Twist",
});

var mastClient = new ROSLIB.Service({
  ros: ros,
  name: "/curiosity_mars_rover/mast_service",
  serviceType: "curiosity_mars_rover_control/Mast",
});
var armClient = new ROSLIB.Service({
  ros: ros,
  name: "/curiosity_mars_rover/arm_service",
  serviceType: "curiosity_mars_rover_control/Arm",
});

var mastListener = new ROSLIB.Topic({
  ros: ros,
  name: "/curiosity_mars_rover/mast_state",
  messageType: "std_msgs/String",
});

var armListener = new ROSLIB.Topic({
  ros: ros,
  name: "/curiosity_mars_rover/arm_state",
  messageType: "std_msgs/String",
});

var requestPing = new ROSLIB.ServiceRequest({ mode: "ping" });
var requestToggle = new ROSLIB.ServiceRequest({ mode: "toggle" });

mastClient.callService(
  requestPing,
  function (result) {
    document.getElementById("mast_state").innerHTML =
      result.status_message.slice(17);
    updateMastButtons(result.status_message.slice(17));
    document.getElementById("status").innerHTML = "Connected to rover";
    document.getElementById("status").style.color = "rgb(17, 207, 0)";
  },
  function (error) {
    document.getElementById("status").innerHTML = "Couldn't connect.";
    document.getElementById("status").style.color = "rgb(255, 47, 47)";
  }
);
armClient.callService(requestPing, function (result) {
  document.getElementById("arm_state").innerHTML =
    result.status_message.slice(16);
  armControls(result.status_message.slice(16));
});

mastListener.subscribe(function (message) {
  document.getElementById("mast_state").innerHTML = message.data;
  updateMastButtons(message.data);
});

armListener.subscribe(function (message) {
  document.getElementById("arm_state").innerHTML = message.data;
  armControls(message.data);
});

function updateMastButtons(message) {
  if (message == "Raised") {
    document.getElementById("mu").disabled = false;
    document.getElementById("ml").disabled = false;
    document.getElementById("md").disabled = false;
    document.getElementById("mr").disabled = false;
    document.getElementById("panorama_button_text").innerHTML = "Take Panorama";
    if (!stitching) {
      document.getElementById("panorama_button").disabled = false;
    }
    document.getElementById("mast_button").disabled = false;
  }
  if (message == "Lowered") {
    document.getElementById("mu").disabled = true;
    document.getElementById("ml").disabled = true;
    document.getElementById("md").disabled = true;
    document.getElementById("mr").disabled = true;
    document.getElementById("panorama_button").disabled = true;
    document.getElementById("mast_button").disabled = false;
  }
  if (message == "Panorama") {
    document.getElementById("mu").disabled = true;
    document.getElementById("ml").disabled = true;
    document.getElementById("md").disabled = true;
    document.getElementById("mr").disabled = true;
    document.getElementById("panorama_button").disabled = false;
    document.getElementById("panorama_button_text").innerHTML =
      "Cancel Panorama";
    document.getElementById("mast_button").disabled = true;
    //document.getElementById("panorama_button_text")
  }
}

function armControls(message) {
  if (message == "Closed") {
    document.getElementById("joint1").disabled = true;
    document.getElementById("joint2").disabled = true;
    document.getElementById("joint3").disabled = true;
    document.getElementById("joint4").disabled = true;
    document.getElementById("effector").disabled = true;
  } else if (message == "Open") {
    document.getElementById("joint1").disabled = false;
    document.getElementById("joint1").value = 0.0;
    document.getElementById("joint2").disabled = false;
    document.getElementById("joint2").value = 0.0;
    document.getElementById("joint3").disabled = false;
    document.getElementById("joint3").value = 0.0;
    document.getElementById("joint4").disabled = false;
    document.getElementById("joint4").value = 0.0;
    document.getElementById("effector").disabled = false;
    document.getElementById("effector").value = 0.0;
  }
}

// Navigation
var sendingNav = false;

function pickMode() {
  if (
    !orientationPick &&
    !sendingNav &&
    document.getElementById("nav_state").innerHTML != "Navigating"
  ) {
    sendingNav = true;
    document
      .getElementById("camera")
      .setAttribute("look-controls", { enabled: false });
    document.getElementById("send_button").innerHTML =
      "Picking... (Click to cancel)";
  } else if (sendingNav || orientationPick) {
    console.error("ok stop sending nav");
    sendingNav = false;
    orientationPick = false;
    hideMarker();
    document
      .getElementById("camera")
      .setAttribute("look-controls", { enabled: true });
    document.getElementById("send_button").innerHTML = "Send Nav Goal";
  } else {
    move_base_cancel.publish(new ROSLIB.Message({}));
    console.error(document.getElementById("camera"));
    //document.getElementById("nav_state").innerHTML = "Waiting";
    //document.getElementById("send_button").innerHTML = "Send Nav Goal";
  }
}

// document.getElementById("nav_state").innerHTML = "Awaiting Goal";

var move_base = new ROSLIB.Topic({
  ros: ros,
  name: "/move_base_simple/goal",
  messageType: "geometry_msgs/PoseStamped",
});

var move_base_cancel = new ROSLIB.Topic({
  ros: ros,
  name: "/move_base/cancel",
  messageType: "actionlib_msgs/GoalID",
});

var move_base_feedback = new ROSLIB.Topic({
  ros: ros,
  name: "/move_base/feedback",
  messageType: "move_base_msgs/MoveBaseActionFeedback",
});

var move_base_result = new ROSLIB.Topic({
  ros: ros,
  name: "/move_base/result",
  messageType: "move_base_msgs/MoveBaseActionResult",
});

var feedback_counter = 0;

move_base_feedback.subscribe(function (message) {
  feedback_counter += 1;
  if (message.status.status == 1 && navigating) {
    document.getElementById("nav_state").innerHTML = "Navigating";
    document.getElementById("send_button").innerHTML = "Cancel Nav Goal";
    teleportIndicator.setAttribute("visible", "true");
  }
});

var navigating = false;

move_base_result.subscribe(function (message) {
  if (message.status.status == 2) {
    document.getElementById("nav_state").innerHTML = "Cancelled";
  }
  if (message.status.status == 3) {
    document.getElementById("nav_state").innerHTML = "Reached goal!";
    navigating = false;
  }
  if (message.status.status == 4) {
    document.getElementById("nav_state").innerHTML = "Aborted";
  }
  if (!sendingNav) {
    document.getElementById("send_button").innerHTML = "Send Nav Goal";
    teleportIndicator.setAttribute("visible", "false");
  }
});

var keepPublishingTeleop;
var keepPublishingMast;

function teleop(params) {
  keepPublishingTeleop = setInterval(function () {
    moveRobot(params);
  }, 16);
}

function telestop() {
  clearInterval(keepPublishingTeleop);
}

function mastClick(params) {
  keepPublishingMast = setInterval(function () {
    moveMast(params);
  }, 16);
}

function mastStop() {
  clearInterval(keepPublishingMast);
}

var panorama_goal = new ROSLIB.Topic({
  ros: ros,
  name: "/panorama_server_node/goal",
  messageType: "actionlib_msgs/GoalID",
});

var panorama_cancel = new ROSLIB.Topic({
  ros: ros,
  name: "/panorama_server_node/cancel",
  messageType: "actionlib_msgs/GoalID",
});

var panorama_feedback = new ROSLIB.Topic({
  ros: ros,
  name: "/panorama_server_node/feedback",
  messageType: "curiosity_mars_rover_control/PanoramaActionFeedback",
});

var panorama_result = new ROSLIB.Topic({
  ros: ros,
  name: "/panorama_server_node/result",
  messageType: "curiosity_mars_rover_control/PanoramaActionResult",
});

panorama_feedback.subscribe(function (message) {
  document.getElementById("panorama_status").innerHTML = message.feedback.state;
  if (message.feedback.state == "Stitching") {
    stitching = true;
    document.getElementById("panorama_button").disabled = true;
  }
  if (message.feedback.state == "Stitched!") {
    stitching = false;
    document.getElementById("panorama_button").disabled = false;
  }
});

function panorama() {
  if (
    document.getElementById("panorama_button_text").innerHTML == "Take Panorama"
  ) {
    panorama_goal.publish(new ROSLIB.Message({}));
    document.getElementById("panorama_button_text").innerHTML =
      "Cancel Panorama";
  } else {
    panorama_cancel.publish(new ROSLIB.Message({}));
    document.getElementById("panorama_button_text").innerHTML = "Cancelling...";
    document.getElementById("panorama_button").disabled = true;
  }
}
