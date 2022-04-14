// Needs work

class Nav {
  constructor(ros) {
    this.sendingNav = false;
    this.navigating = false;
    this.orientationPick = false;

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

    this.goalCylinder = document.createElement("a-cylinder");
    this.goalArrow = document.createElement("a-entity");

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

    document.getElementById("main").appendChild(this.goalCylinder);
    document.getElementById("main").appendChild(this.goalArrow);
    document
      .getElementById("world")
      .addEventListener("click", this.nav.bind(this), false);

    this.moveBaseFeedback.subscribe(function (message) {
      if (message.status.status == 1 && navigating) {
        document.getElementById("nav_state").innerHTML = "Navigating";
        document.getElementById("send_button").innerHTML = "Cancel Nav Goal";
        goalCylinder.setAttribute("visible", "true");
      }
    });

    this.moveBaseResult.subscribe(function (message) {
      if (message.status.status == 2) {
        document.getElementById("nav_state").innerHTML = "Cancelled";
      }
      // Need to bind!
      // if (message.status.status == 3) {
      //   document.getElementById("nav_state").innerHTML = "Reached goal!";
      //   navigating = false;
      // }
      if (message.status.status == 4) {
        document.getElementById("nav_state").innerHTML = "Aborted";
      }
      // Re-enable with binded function
      // if (!sendingNav) {
      //   document.getElementById("send_button").innerHTML = "Send Nav Goal";
      //   goalCylinder.setAttribute("visible", "false");
      // }
    });
  }

  async nav(event) {
    if (this.sendingNav) {
      var touchPoint = event.detail.intersection.point;
      var position =
        touchPoint.x + " " + (touchPoint.y + 0.5) + " " + touchPoint.z;
      this.goalArrow.setAttribute("position", position);
      this.goalArrow.setAttribute("material", {
        opacity: 0.5,
        transparent: true,
      });
      this.orientationPick = true;
      this.sendingNav = false;
    } else if (this.orientationPick) {
      this.orientationPick = false;
      this.goalCylinder.setAttribute("position", goalArrow.object3D.position);
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
      var newRotation = goalArrow.object3D.quaternion;
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
            x: goalArrow.object3D.position.x,
            y: -1 * goalArrow.object3D.position.z,
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
      this.navigating = true;
      document
        .getElementById("camera")
        .setAttribute("look-controls", { enabled: true });
    }
  }

  pickLocation() {
    if (
      !this.orientationPick &&
      !this.sendingNav &&
      document.getElementById("nav_state").innerHTML != "Navigating"
    ) {
      this.sendingNav = true;
      document
        .getElementById("camera")
        .setAttribute("look-controls", { enabled: false });
      document.getElementById("send_button").innerHTML =
        "Picking... (Click to cancel)";
    } else if (this.sendingNav || this.orientationPick) {
      this.sendingNav = false;
      this.orientationPick = false;
      this.goalArrow.setAttribute("material", {
        opacity: 0.0,
        transparent: true,
      });
      document
        .getElementById("camera")
        .setAttribute("look-controls", { enabled: true });
      document.getElementById("send_button").innerHTML = "Send Nav Goal";
    } else {
      moveBaseCancel.publish(new ROSLIB.Message({}));
    }
  }
}

// AFRAME.registerComponent("cursor-listener", {
//   init: function () {
//     this.el.addEventListener("raycaster-intersected", (evt) => {
//       this.raycaster = evt.detail.el;
//     });
//     this.el.addEventListener("raycaster-intersected-cleared", (evt) => {
//       this.raycaster = null;
//     });
//   },
//   tick: function () {
//     if (!this.raycaster || !(sendingNav || orientationPick)) {
//       return;
//     } // Not intersecting.
//     let intersection = this.raycaster.components.raycaster.getIntersection(
//       this.el
//     );
//     if (!intersection) {
//       return;
//     } // Not intersecting
//     // intersecting
//     if (!orientationPick) {
//       goalArrow.setAttribute("material", {
//         opacity: 0.5,
//         transparent: true,
//       });
//       var touchPoint = intersection.point;
//       goalArrow.object3D.position.set(
//         touchPoint.x,
//         touchPoint.y + 0.5,
//         touchPoint.z
//       );
//     } else {
//       var quaternion = new THREE.Quaternion(); // create one and reuse it
//       var from = goalArrow.getAttribute("position");
//       quaternion.setFromUnitVectors(from, intersection.point);
//       goalArrow.object3D.lookAt(
//         intersection.point.x,
//         from.y,
//         intersection.point.z
//       );
//     }
//   },
// });
