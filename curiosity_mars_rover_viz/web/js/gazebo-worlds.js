var gazeboWorld = new ROSLIB.Service({
  ros: ros,
  name: "/gazebo/get_world_properties",
  serviceType: "gazebo_msgs/GetWorldProperties",
});

var teleportIndicator = document.createElement("a-cylinder");
var triangle = document.createElement("a-entity");
var orientationPick = false;

AFRAME.registerComponent("gazebo-world", {
  schema: {},
  init() {
    gazeboWorld.callService(new ROSLIB.Message({}), function (result) {
      var newElement = document.createElement("a-entity");
      document.getElementById("main").appendChild(newElement);
      newElement.setAttribute("id", "world");
      newElement.setAttribute("visible", "true");
      newElement.setAttribute("shadow", { receive: false });
      newElement.setAttribute("class", "cantap");
      newElement.setAttribute("cursor-listener", "");
      switch (result.model_names[0]) {
        case "world_mars_path":
          newElement.setAttribute("gltf-model", "#path");
          newElement.setAttribute("position", "0 0 0");
          newElement.setAttribute("rotation", "0 0 0");
          newElement.setAttribute("scale", "1 1 1");
          break;
        case "world_mars_photo":
          newElement.setAttribute("gltf-model", "#photo");
          newElement.setAttribute("position", "4.880819 -1.5 0.5");
          newElement.setAttribute("rotation", "0 0 0");
          newElement.setAttribute("scale", "1 1 1");
          break;
        case "world_mars_terrain":
          newElement.setAttribute("gltf-model", "#terrain");
          newElement.setAttribute("position", "0 0 0");
          newElement.setAttribute("rotation", "0 0 0");
          newElement.setAttribute("scale", "1 1 1");
          break;
        default:
          newElement.setAttribute("geometry", {
            primitive: "plane",
            width: 80,
            height: 80,
          });
          newElement.setAttribute("material", { color: "#3b3b3b" });
          newElement.setAttribute("width", "200");
          newElement.setAttribute("height", "200");
          newElement.setAttribute("scale", "4 4 1");
          newElement.setAttribute("rotation", "-90 0 0");
          break;
      }

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
      async function nav(event) {
        if (sendingNav) {
          //   .getElementById("camera")
          //   .setAttribute("look-controls", { enabled: true });
          // sendingNav = false;
          // The raycaster gives a location of the touch in the scene
          var touchPoint = event.detail.intersection.point;
          // teleportIndicator.setAttribute("position", touchPoint);
          // teleportIndicator.setAttribute("material", {
          //   opacity: 0.0,
          //   transparent: true,
          // });
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
          teleportIndicator.setAttribute(
            "position",
            triangle.object3D.position
          );
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
          // var conversion = new THREE.Euler(1.57, 0, 0, "XYZ");
          // var quat = new THREE.Quaternion();
          // quat.setFromEuler(conversion);
          var newRotation = triangle.object3D.quaternion;
          const quaternion = new THREE.Quaternion();
          quaternion.setFromAxisAngle(new THREE.Vector3(0, -1, 0), Math.PI / 2);
          newRotation.multiplyQuaternions(newRotation, quaternion);
          console.error(triangle.object3D.quaternion);
          console.error(newRotation);
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
          // await delay(2);
          // triangle.setAttribute("material", {
          //   opacity: 0.0,
          //   transparent: true,
          // });
          navigating = true;
          document
            .getElementById("camera")
            .setAttribute("look-controls", { enabled: true });
        }
      }
    });
  },
});

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

function hideMarker() {
  triangle.setAttribute("material", {
    opacity: 0.0,
    transparent: true,
  });
}
