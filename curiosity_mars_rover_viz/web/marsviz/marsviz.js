class MarsViz {
  constructor(secsToWait) {
    this.ros = new ROSLIB.Ros({ url: "wss://127.0.0.1:9090" });
    this.scene = document.querySelector("a-scene");

    this.teleop = new Teleop(this.ros, this.scene);
    this.mast = new Mast(this.ros, this.scene);
    this.arm = new Arm(this.ros, this.scene);

    this.navigation; // Initialised after world loaded

    this.gazeboWorld = new ROSLIB.Service({
      ros: this.ros,
      name: "/gazebo/get_world_properties",
      serviceType: "gazebo_msgs/GetWorldProperties",
    });

    this.init_after_seconds(secsToWait);
  }

  async init_after_seconds(secs) {
    await new Promise((r) => setTimeout(r, secs * 1000));

    // Setup a client to listen to TFs.
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

// Main code - A-Frame input mapping/events are defined outside of the class scopes

function initialiseMarsviz() {
  AFRAME.registerInputMappings(Controls.mappings);
  AFRAME.registerInputActions(Controls.inputActions, "default");

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
      if (!intersection) {
        return; // Not intersecting
      }
      // Intersecting
      marsviz.navigation.updateArrow(intersection);
    },
  });

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
