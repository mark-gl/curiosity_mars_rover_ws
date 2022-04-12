
var gazeboWorld = new ROSLIB.Service({
    ros: ros,
    name: "/gazebo/get_world_properties",
    serviceType: "gazebo_msgs/GetWorldProperties",
  });
  
  gazeboWorld.callService(new ROSLIB.Message({}), function (result) {
    console.error(result.model_names);
    switch (result.model_names[0]) {
      case "world_flat":
        document.getElementById("ground_plane").setAttribute("visible", true);
        break;
      case "world_mars_path":
        document.getElementById("mars_path").setAttribute("visible", true);
        break;
      case "world_mars_photo":
        document.getElementById("mars_photo").setAttribute("visible", true);
        break;
      case "world_mars_terrain":
        console.error("Moment");
        var mars_terrain = document.createElement("a-entity");
        mars_terrain.setAttribute("gltf-model", "#terrain");
        mars_terrain.setAttribute("position", "0 0 0");
        mars_terrain.setAttribute("rotation", "0 0 0");
        mars_terrain.setAttribute("scale", "1 1 1");
        mars_terrain.setAttribute("shadow", "receive: false");
        mars_terrain.setAttribute("class", "cantap");
        document.getElementById("a-scene").appendChild(mars_terrain);
        break;
      default:
        document.getElementById("ground_plane").setAttribute("visible", true);
        break;
    }
  });