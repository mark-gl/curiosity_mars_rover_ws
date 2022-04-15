class Mast {
  constructor(ros, scene) {
    // keepPublishingMast is used with setInterval to repeat function calls
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

    this.mastListener.subscribe(this.mastCallback.bind(this));
    this.panoramaFeedback.subscribe(this.panoramaCallback.bind(this));

    // 'Ping' the mastClient with a dummy request, to initialise the user interface.
    this.mastClient.callService(
      new ROSLIB.ServiceRequest({ mode: "ping" }),
      function (result) {
        document.getElementById("status").innerHTML = "Connected to rover";
        document.getElementById("status").style.color = "rgb(17, 207, 0)";
      },
      function (error) {
        document.getElementById("status").innerHTML = "Couldn't connect.";
        document.getElementById("status").style.color = "rgb(255, 47, 47)";
      }
    );
    
    // Register event listeners for class functions.
    scene.addEventListener("mastToggle", this.mastToggle.bind(this));
    scene.addEventListener("mastUp", this.mastMove.bind(this, -0.02, 0));
    scene.addEventListener("mastDown", this.mastMove.bind(this, 0.02, 0));
    scene.addEventListener("mastLeft", this.mastMove.bind(this, 0, 0.02));
    scene.addEventListener("mastRight", this.mastMove.bind(this, 0, -0.02));
  }

  mastCallback(result) {
    // The first 17 characters ('Done! Mast Mode: ') are ignored
    // This isn't very robust, but it works fine
    document.getElementById("mast_state").innerHTML = result.data.slice(17);
    this.updateMastButtons(result.data.slice(17));
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

  mastClick(x, y) {
    // Start sending mast movement messages
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
