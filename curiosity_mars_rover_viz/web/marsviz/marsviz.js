// Main class for rover controls (either via the keyboard or UI panel)
// Contains Arm, Mast, Nav and Teleop instances
// Instantiated in main function below class definition
class MarsViz {
  constructor(secsToWait) {
    // ROS is accessible through a web socket on port 9090
    this.ros = new ROSLIB.Ros({ url: "wss://127.0.0.1:9090" });
    this.scene = document.querySelector("a-scene");

    this.teleop = new Teleop(this.ros, this.scene);
    this.mast = new Mast(this.ros, this.scene);
    this.arm = new Arm(this.ros, this.scene);
    this.navigation; // Initialised after world is loaded

    // gazeboWorld is needed to tell us the current models in the Gazebo world
    this.gazeboWorld = new ROSLIB.Service({
      ros: this.ros,
      name: "/gazebo/get_world_properties",
      serviceType: "gazebo_msgs/GetWorldProperties",
    });

    // Wait a bit before spawning in the rover
    this.init_after_seconds(secsToWait);
  }

  async init_after_seconds(secs) {
    await new Promise((r) => setTimeout(r, secs * 1000));

    // Setup a client to listen to rover transform changes
    var tfClient = new ROSLIB.TFClient({
      ros: this.ros,
      angularThres: 0.01,
      transThres: 0.01,
      rate: 60.0,
      fixedFrame: "odom",
    });

    var urdfClient = this.addVisualization(
      document.getElementById("robot-element-parent"),
      ROS3D.UrdfClient,
      {
        ros: this.ros,
        tfClient: tfClient,
        path: "https://127.0.0.1:8080/",
      }
    );
  }

  addVisualization(rootNode, type, options, enabled = true) {
    if (enabled) {
      var node = document.createElement("a-entity");
      rootNode.appendChild(node);
      options.rootObject = node.object3D;
      var visualization = new type(options);
      return visualization;
    }
  }

  loadWorldModel(gazeboMessage) {
    var newElement = document.createElement("a-entity");
    document.getElementById("main").appendChild(newElement);
    newElement.setAttribute("id", "world");
    newElement.setAttribute("visible", "true");
    newElement.setAttribute("shadow", { receive: false });
    newElement.setAttribute("class", "cantap");
    newElement.setAttribute("cursor-listener", "");
    // Set the model attribute depending on the first model in the world
    // (In each Gazebo worlds, these are the names of empty models at the start of each file)
    switch (gazeboMessage.model_names[0]) {
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
        // Make a flat, empty world otherwise
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
    this.navigation = new Nav(this.ros);
  }
}

// Main code - A-Frame components/input mappings are registered outside of the class scopes
// MarsViz class is instantiated in ./index.html using this function
function initialiseMarsviz() {
  AFRAME.registerInputMappings(Controls.mappings);
  AFRAME.registerInputActions(Controls.inputActions, "default");

  // This component listens for mouse movement on the world model for sending navigation goals
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
      if (!this.raycaster) {
        return; // Not intersecting
      }
      let intersection = this.raycaster.components.raycaster.getIntersection(
        this.el
      );
      if (intersection) {
        // Update the position of the navigation goal picker!
        marsviz.navigation.updateArrow(intersection);
      }
    },
  });

  // Runs the world model loading once the A-Frame scene has loaded
  AFRAME.registerComponent("gazebo-world", {
    schema: {},
    init() {
      marsviz.gazeboWorld.callService(
        new ROSLIB.Message({}),
        function (result) {
          marsviz.loadWorldModel(result);
        }
      );
    },
  });

  return new MarsViz(2);
}
