var gazeboWorld = new ROSLIB.Service({
  ros: ros,
  name: "/gazebo/get_world_properties",
  serviceType: "gazebo_msgs/GetWorldProperties",
});

AFRAME.registerComponent("gazebo-world", {
  schema: {},
  init() {
    gazeboWorld.callService(new ROSLIB.Message({}), function (result) {
      console.error(result.model_names);
      var newElement = document.createElement("a-entity");
      document.getElementById("main").appendChild(newElement);
      newElement.setAttribute("id", "world");
      newElement.setAttribute("visible", "true");
      newElement.setAttribute("shadow", { receive: false });
      newElement.setAttribute("class", "cantap");
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
            newElement.setAttribute("geometry", { primitive: 'plane', width: 80, height: 80 });
            newElement.setAttribute("material",  { color: '#3b3b3b' });
            newElement.setAttribute("width", "200");
            newElement.setAttribute("height", "200");
            newElement.setAttribute("scale", "4 4 1");
            newElement.setAttribute("rotation", "-90 0 0");
          break;
      }

      const ground = document.getElementById("world");
      var newElement = document.createElement("a-cylinder");
      document.getElementById("main").appendChild(newElement);
      newElement.setAttribute("height", 500);
      newElement.setAttribute("radius", 0.4);
      newElement.setAttribute("color", "cyan");
      newElement.setAttribute("material", { opacity: 0.0, transparent: true });
      newElement.setAttribute("visible", "true");
      newElement.setAttribute("scale", "0.7 1.2 0.7");
      newElement.setAttribute("shadow", { receive: false });
  
      ground.addEventListener("click", (event) => nav(event), false);
      async function nav(event) {
        if (sendingNav) {
          // The raycaster gives a location of the touch in the scene
          var touchPoint = event.detail.intersection.point;
          newElement.setAttribute("position", touchPoint);
          newElement.setAttribute("material", {
            opacity: 0.2,
            transparent: true,
          });
  
          var currentTime = new Date();
          var mySecs = Math.floor(currentTime.getTime() / 1000);
          var myNsecs = Math.round(
            1000000000 * (currentTime.getTime() / 1000 - mySecs)
          );
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
                x: event.detail.intersection.point.x,
                y: -1 * event.detail.intersection.point.z,
                z: 0.0,
              },
              orientation: {
                x: 0.0,
                y: 0.0,
                z: 0.0,
                w: 1.0,
              },
            },
          });
          move_base.publish(nav);
          await delay(2);
          newElement.setAttribute("material", {
            opacity: 0.0,
            transparent: true,
          });
          sendingNav = false;
          navigating = true;
        }
      }

    });
  },
});
