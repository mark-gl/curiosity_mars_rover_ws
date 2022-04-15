class Nav {
  constructor(ros) {
    this.navigating = false;
    this.pickPosition = false;
    this.pickOrientation = false;

    this.moveBase = new ROSLIB.Topic({
      ros: ros,
      name: "/move_base_simple/goal",
      messageType: "geometry_msgs/PoseStamped",
    });

    this.moveBaseCancel = new ROSLIB.Topic({
      ros: ros,
      name: "/move_base/cancel",
      messageType: "actionlib_msgs/GoalID",
    });

    this.moveBaseFeedback = new ROSLIB.Topic({
      ros: ros,
      name: "/move_base/feedback",
      messageType: "move_base_msgs/MoveBaseActionFeedback",
    });

    this.moveBaseResult = new ROSLIB.Topic({
      ros: ros,
      name: "/move_base/result",
      messageType: "move_base_msgs/MoveBaseActionResult",
    });

    // The goal cylinder appears once the rover starts navigating and stays there until the goal is reached
    this.goalCylinder = document.createElement("a-cylinder");
    // The goal arrow is the '3D cursor' used for picking destinations
    this.goalArrow = document.createElement("a-entity");
    // Note: VR destination picking is handled in a different file (include/aframe-navigation-controls.js)

    this.goalCylinder.setAttribute("height", 500);
    this.goalCylinder.setAttribute("radius", 0.4);
    this.goalCylinder.setAttribute("color", "cyan");
    this.goalCylinder.setAttribute("visible", "false");
    this.goalCylinder.setAttribute("scale", "0.7 1.2 0.7");
    this.goalCylinder.setAttribute("shadow", { receive: false });
    this.goalCylinder.setAttribute("material", {
      opacity: 0.5,
      transparent: true,
    });
    this.goalArrow.setAttribute("obj-model", { obj: "#arrow" });
    this.goalArrow.setAttribute("color", "cyan");
    this.goalArrow.setAttribute("scale", "1.2 1.2 1.2");
    this.goalArrow.setAttribute("material", {
      opacity: 0.0,
      transparent: true,
      color: "cyan",
    });

    // Add the two (hidden) objects to the world
    document.getElementById("main").appendChild(this.goalCylinder);
    document.getElementById("main").appendChild(this.goalArrow);
    document
      .getElementById("world")
      .addEventListener("click", this.clickedGround.bind(this), false);

    this.moveBaseFeedback.subscribe(this.updateStatus.bind(this));
    this.moveBaseResult.subscribe(this.updateStatus.bind(this));
  }

  async clickedGround(event) {
    // Clicking happens in 2 stages - picking position and picking orientation
    if (this.pickPosition) {
      var touchPoint = event.detail.intersection.point;
      var position =
        touchPoint.x + " " + (touchPoint.y + 0.5) + " " + touchPoint.z;
      this.goalArrow.setAttribute("position", position);
      this.goalArrow.setAttribute("material", {
        opacity: 0.5,
        transparent: true,
      });
      this.pickPosition = false;
      this.pickOrientation = true;
    } else if (this.pickOrientation) {
      this.goalCylinder.setAttribute(
        "position",
        this.goalArrow.object3D.position
      );
      this.goalCylinder.setAttribute("visible", "true");
      this.goalCylinder.setAttribute("material", {
        opacity: 0.2,
        transparent: true,
      });
      this.goalArrow.setAttribute("material", {
        opacity: 0.0,
        transparent: true,
      });
      var currentTime = new Date();
      var mySecs = Math.floor(currentTime.getTime() / 1000);
      var myNsecs = Math.round(
        1000000000 * (currentTime.getTime() / 1000 - mySecs)
      );

      var newRotation = this.goalArrow.object3D.quaternion;
      const quaternion = new THREE.Quaternion();
      quaternion.setFromAxisAngle(new THREE.Vector3(0, -1, 0), Math.PI / 2);
      newRotation.multiplyQuaternions(newRotation, quaternion);

      var nav = new ROSLIB.Message({
        header: {
          seq: 0,
          stamp: {
            secs: mySecs,
            nsecs: myNsecs,
          },
          frame_id: "odom",
        },
        pose: {
          position: {
            x: this.goalArrow.object3D.position.x,
            y: -1 * this.goalArrow.object3D.position.z,
            z: 0.0,
          },
          orientation: {
            x: 0.0,
            y: 0.0,
            z: newRotation.y,
            w: newRotation.w,
          },
        },
      });
      this.moveBase.publish(nav);

      this.pickOrientation = false;
      this.navigating = true;
      document
        .getElementById("camera")
        .setAttribute("look-controls", { enabled: true });
    }
  }

  sendNavButton() {
    // Determine what action to take when button is pressed
    if (!(this.navigating || this.pickPosition || this.pickOrientation)) {
      this.pickPosition = true;
      // Look controls are disable so that user can click world
      document
        .getElementById("camera")
        .setAttribute("look-controls", { enabled: false });
      document.getElementById("send_button").innerHTML =
        "Picking... (Click to cancel)";
    } else if (this.pickPosition || this.pickOrientation) {
      this.pickPosition = false;
      this.pickOrientation = false;
      this.goalArrow.setAttribute("material", {
        opacity: 0.0,
        transparent: true,
      });
      document
        .getElementById("camera")
        .setAttribute("look-controls", { enabled: true });
      document.getElementById("send_button").innerHTML = "Send Nav Goal";
    } else {
      this.moveBaseCancel.publish(new ROSLIB.Message({}));
    }
  }

  updateArrow(intersection) {
    if (this.pickPosition) {
      // Update the position of the arrow in the world
      this.goalArrow.setAttribute("material", {
        opacity: 0.5,
        transparent: true,
      });
      var touchPoint = intersection.point;
      this.goalArrow.object3D.position.set(
        touchPoint.x,
        touchPoint.y + 0.5,
        touchPoint.z
      );
    } else if (this.pickOrientation) {
      // Rotate the arrow
      var quaternion = new THREE.Quaternion();
      var from = this.goalArrow.getAttribute("position");
      quaternion.setFromUnitVectors(from, intersection.point);
      this.goalArrow.object3D.lookAt(
        intersection.point.x,
        from.y,
        intersection.point.z
      );
    }
  }

  updateStatus(message) {
    if (message.status.status == 1 && this.navigating) {
      document.getElementById("nav_state").innerHTML = "Navigating";
      document.getElementById("send_button").innerHTML = "Cancel Nav Goal";
      this.goalCylinder.setAttribute("visible", "true");
    } else if (message.status.status > 1 && message.status.status < 5) {
      if (message.status.status == 2) {
        document.getElementById("nav_state").innerHTML = "Cancelled";
      } else if (message.status.status == 3) {
        document.getElementById("nav_state").innerHTML = "Reached goal!";
      } else if (message.status.status == 4) {
        document.getElementById("nav_state").innerHTML = "Aborted";
      }
      if (!(this.pickPosition || this.pickOrientation)) {
        document.getElementById("send_button").innerHTML = "Send Nav Goal";
        this.goalCylinder.setAttribute("visible", "false");
      }
      this.navigating = false;
    }
  }
}
