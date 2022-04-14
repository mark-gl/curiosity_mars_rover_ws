var sendingNav = false;
var navigating = false;
var orientationPick = false;

var teleportIndicator = document.createElement("a-cylinder");
var triangle = document.createElement("a-entity");

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
    sendingNav = false;
    orientationPick = false;
    triangle.setAttribute("material", {
      opacity: 0.0,
      transparent: true,
    });
    document
      .getElementById("camera")
      .setAttribute("look-controls", { enabled: true });
    document.getElementById("send_button").innerHTML = "Send Nav Goal";
  } else {
    move_base_cancel.publish(new ROSLIB.Message({}));
  }
}

async function nav(event) {
  if (sendingNav) {
    var touchPoint = event.detail.intersection.point;
    var position =
      touchPoint.x + " " + (touchPoint.y + 0.5) + " " + touchPoint.z;
    triangle.setAttribute("position", position);
    triangle.setAttribute("material", {
      opacity: 0.5,
      transparent: true,
    });
    orientationPick = true;
    sendingNav = false;
  } else if (orientationPick) {
    orientationPick = false;
    teleportIndicator.setAttribute("position", triangle.object3D.position);
    teleportIndicator.setAttribute("visible", "true");
    teleportIndicator.setAttribute("material", {
      opacity: 0.2,
      transparent: true,
    });
    triangle.setAttribute("material", {
      opacity: 0.0,
      transparent: true,
    });
    var currentTime = new Date();
    var mySecs = Math.floor(currentTime.getTime() / 1000);
    var myNsecs = Math.round(
      1000000000 * (currentTime.getTime() / 1000 - mySecs)
    );
    var newRotation = triangle.object3D.quaternion;
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
          x: triangle.object3D.position.x,
          y: -1 * triangle.object3D.position.z,
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
    move_base.publish(nav);
    navigating = true;
    document
      .getElementById("camera")
      .setAttribute("look-controls", { enabled: true });
  }
}

function initialise_nav() {
  const ground = document.getElementById("world");
  document.getElementById("main").appendChild(teleportIndicator);
  teleportIndicator.setAttribute("height", 500);
  teleportIndicator.setAttribute("radius", 0.4);
  teleportIndicator.setAttribute("color", "cyan");
  teleportIndicator.setAttribute("material", {
    opacity: 0.5,
    transparent: true,
  });
  teleportIndicator.setAttribute("visible", "false");
  teleportIndicator.setAttribute("scale", "0.7 1.2 0.7");
  teleportIndicator.setAttribute("shadow", { receive: false });

  triangle.setAttribute("color", "cyan");
  triangle.setAttribute("scale", "1.2 1.2 1.2");
  triangle.setAttribute("material", {
    opacity: 0.0,
    transparent: true,
    color: "cyan",
  });
  triangle.setAttribute("obj-model", { obj: "#arrow" });
  document.getElementById("main").appendChild(triangle);
  ground.addEventListener("click", (event) => nav(event), false);
}

// Initialisation below

AFRAME.registerComponent("cursor-listener", {
  init: function () {
    this.el.addEventListener("raycaster-intersected", (evt) => {
      this.raycaster = evt.detail.el;
    });
    this.el.addEventListener("raycaster-intersected-cleared", (evt) => {
      this.raycaster = null;
    });
  },
  tick: function () {
    if (!this.raycaster || !(sendingNav || orientationPick)) {
      return;
    } // Not intersecting.
    let intersection = this.raycaster.components.raycaster.getIntersection(
      this.el
    );
    if (!intersection) {
      return;
    } // Not intersecting
    // intersecting
    if (!orientationPick) {
      triangle.setAttribute("material", {
        opacity: 0.5,
        transparent: true,
      });
      var touchPoint = intersection.point;
      triangle.object3D.position.set(
        touchPoint.x,
        touchPoint.y + 0.5,
        touchPoint.z
      );
    } else {
      var quaternion = new THREE.Quaternion(); // create one and reuse it
      var from = triangle.getAttribute("position");
      quaternion.setFromUnitVectors(from, intersection.point);
      triangle.object3D.lookAt(
        intersection.point.x,
        from.y,
        intersection.point.z
      );
    }
  },
});

move_base_feedback.subscribe(function (message) {
  if (message.status.status == 1 && navigating) {
    document.getElementById("nav_state").innerHTML = "Navigating";
    document.getElementById("send_button").innerHTML = "Cancel Nav Goal";
    teleportIndicator.setAttribute("visible", "true");
  }
});

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
