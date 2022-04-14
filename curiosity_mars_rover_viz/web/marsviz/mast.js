class Mast {
  constructor(ros, scene) {
    this.keepPublishingMast;
    this.stitching = false;

    this.mastClient = new ROSLIB.Service({
      ros: ros,
      name: "/curiosity_mars_rover/mast_service",
      serviceType: "curiosity_mars_rover_control/Mast",
    });

    this.mastListener = new ROSLIB.Topic({
      ros: ros,
      name: "/curiosity_mars_rover/mast_state",
      messageType: "std_msgs/String",
    });

    this.panoramaGoal = new ROSLIB.Topic({
      ros: ros,
      name: "/panorama_server_node/goal",
      messageType: "actionlib_msgs/GoalID",
    });

    this.panoramaCancel = new ROSLIB.Topic({
      ros: ros,
      name: "/panorama_server_node/cancel",
      messageType: "actionlib_msgs/GoalID",
    });

    this.panoramaFeedback = new ROSLIB.Topic({
      ros: ros,
      name: "/panorama_server_node/feedback",
      messageType: "curiosity_mars_rover_control/PanoramaActionFeedback",
    });

    this.panoramaResult = new ROSLIB.Topic({
      ros: ros,
      name: "/panorama_server_node/result",
      messageType: "curiosity_mars_rover_control/PanoramaActionResult",
    });
    this.mastClient.callService(
      new ROSLIB.ServiceRequest({ mode: "ping" }),
      this.mastCallback.bind(this),
      function (error) {
        document.getElementById("status").innerHTML = "Couldn't connect.";
        // Can display error?
        document.getElementById("status").style.color = "rgb(255, 47, 47)";
      }
    );

    this.mastListener.subscribe(this.mastCallback.bind(this));

    this.panoramaFeedback.subscribe(this.panoramaCallback.bind(this));

    scene.addEventListener("mastToggle", this.mastToggle.bind(this));

    scene.addEventListener("mastUp", this.mastMove.bind(this, -0.02, 0));
    scene.addEventListener("mastDown",  this.mastMove.bind(this, 0.02, 0));
    scene.addEventListener("mastLeft", this.mastMove.bind(this, 0, 0.02));
    scene.addEventListener("mastRight", this.mastMove.bind(this, 0, -0.02));
  }

  mastCallback(result) {
    document.getElementById("mast_state").innerHTML =
      result.status_message.slice(17);
    this.updateMastButtons(result.status_message.slice(17));
    document.getElementById("status").innerHTML = "Connected to rover";
    document.getElementById("status").style.color = "rgb(17, 207, 0)";
  }

  panoramaCallback(result) {
    document.getElementById("panorama_status").innerHTML =
      result.feedback.state;
    if (result.feedback.state == "Stitching") {
      this.stitching = true;
      document.getElementById("panorama_button").disabled = true;
    }
    if (result.feedback.state == "Stitched!") {
      this.stitching = false;
      document.getElementById("panorama_button").disabled = false;
    }
  }

  mastToggle() {
    this.mastClient.callService(new ROSLIB.ServiceRequest({ mode: "toggle" }));
  }

  mastMove(x, y) {
    var request = new ROSLIB.ServiceRequest({
      mode: "rotate",
      rot_x: x,
      rot_y: y,
    });
    this.mastClient.callService(request);
  }

  // Need changes below here

  mastClick(x, y) {
    this.keepPublishingMast = setInterval(this.mastMove.bind(this, x, y), 16);
  }

  mastStop() {
    clearInterval(this.keepPublishingMast);
  }

  panorama() {
    if (
      document.getElementById("panorama_button_text").innerHTML ==
      "Take Panorama"
    ) {
      this.panoramaGoal.publish(new ROSLIB.Message({}));
      document.getElementById("panorama_button_text").innerHTML =
        "Cancel Panorama";
    } else {
      this.panoramaCancel.publish(new ROSLIB.Message({}));
      document.getElementById("panorama_button_text").innerHTML =
        "Cancelling...";
      document.getElementById("panorama_button").disabled = true;
    }
  }

  updateMastButtons(message) {
    switch (message) {
      case "Raised":
        document.getElementById("mu").disabled = false;
        document.getElementById("ml").disabled = false;
        document.getElementById("md").disabled = false;
        document.getElementById("mr").disabled = false;
        document.getElementById("panorama_button_text").innerHTML =
          "Take Panorama";
        if (!this.stitching) {
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
}
