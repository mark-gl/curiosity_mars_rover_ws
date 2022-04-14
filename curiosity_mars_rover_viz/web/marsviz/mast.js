var keepPublishingMast;
var stitching = false;

var mastClient = new ROSLIB.Service({
  ros: ros,
  name: "/curiosity_mars_rover/mast_service",
  serviceType: "curiosity_mars_rover_control/Mast",
});

var mastListener = new ROSLIB.Topic({
  ros: ros,
  name: "/curiosity_mars_rover/mast_state",
  messageType: "std_msgs/String",
});

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

function mastToggle() {
  mastClient.callService(requestToggle);
}

function mastMove(x, y) {
  request = new ROSLIB.ServiceRequest({
    mode: "rotate",
    rot_x: x,
    rot_y: y,
  });
  mastClient.callService(request);
}

function mastClick(x, y) {
  keepPublishingMast = setInterval(function () {
    mastMove(x, y);
  }, 16);
}

function mastStop() {
  clearInterval(keepPublishingMast);
}

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

function updateMastButtons(message) {
  switch (message) {
    case "Raised":
      document.getElementById("mu").disabled = false;
      document.getElementById("ml").disabled = false;
      document.getElementById("md").disabled = false;
      document.getElementById("mr").disabled = false;
      document.getElementById("panorama_button_text").innerHTML =
        "Take Panorama";
      if (!stitching) {
        document.getElementById("panorama_button").disabled = false;
      }
      document.getElementById("mast_button").disabled = false;
      break;
    case "Lowered":
      document.getElementById("mu").disabled = true;
      document.getElementById("ml").disabled = true;
      document.getElementById("md").disabled = true;
      document.getElementById("mr").disabled = true;
      document.getElementById("panorama_button").disabled = true;
      document.getElementById("mast_button").disabled = false;
      break;
    case "Panorama":
      document.getElementById("mu").disabled = true;
      document.getElementById("ml").disabled = true;
      document.getElementById("md").disabled = true;
      document.getElementById("mr").disabled = true;
      document.getElementById("panorama_button").disabled = false;
      document.getElementById("panorama_button_text").innerHTML =
        "Cancel Panorama";
      document.getElementById("mast_button").disabled = true;
      break;
    default:
      break;
  }
}

// Initialisation below

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

mastListener.subscribe(function (message) {
  document.getElementById("mast_state").innerHTML = message.data;
  updateMastButtons(message.data);
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
