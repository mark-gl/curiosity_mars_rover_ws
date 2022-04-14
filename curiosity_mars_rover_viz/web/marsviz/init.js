const ros = new ROSLIB.Ros({ url: "wss://127.0.0.1:9090" });
const requestPing = new ROSLIB.ServiceRequest({ mode: "ping" });
const requestToggle = new ROSLIB.ServiceRequest({ mode: "toggle" });

const gazeboWorld = new ROSLIB.Service({
  ros: ros,
  name: "/gazebo/get_world_properties",
  serviceType: "gazebo_msgs/GetWorldProperties",
});

function setupEventListeners(scene) {
  var robotMovementEvents = [
    "moveForward",
    "moveBackward",
    "turnLeft",
    "turnRight",
    "stopRobot",
  ];

  // Teleop functions
  scene.addEventListener("moveJoy", moveJoy);
  scene.addEventListener("speedUp", speedUp);
  scene.addEventListener("slowDown", slowDown);

  // Mast/arm functions
  scene.addEventListener("mastToggle", mastToggle);
  scene.addEventListener("armToggle", armToggle);

  scene.addEventListener("mastUp", function (_) {
    mastMove(-0.02, 0);
  });
  scene.addEventListener("mastDown", function (_) {
    mastMove(0.02, 0);
  });
  scene.addEventListener("mastLeft", function (_) {
    mastMove(0, 0.02);
  });
  scene.addEventListener("mastRight", function (_) {
    mastMove(0, -0.02);
  });

  for (var i = 0; i < robotMovementEvents.length; i++) {
    scene.addEventListener(robotMovementEvents[i], function (event) {
      var type = event.type;
      var currentMappingActions =
        AFRAME.inputActions[AFRAME.currentInputMapping];
      var parameters = currentMappingActions[type]
        ? currentMappingActions[type].params
        : [0, 0, 0, 0, 0, 0];
      moveRobot(parameters);
    });
  }
}

function addVisualization(rootNode, type, options, enabled = true) {
  if (enabled) {
    node = document.createElement("a-entity");
    rootNode.appendChild(node);
    options.rootObject = node.object3D;
    visualization = new type(options);
    return visualization;
  }
}

async function init_env() {
  var scene = document.querySelector("a-scene");
  setupEventListeners(scene);

  // Setup a client to listen to TFs.
  tfClient = new ROSLIB.TFClient({
    ros: ros,
    angularThres: 0.01,
    transThres: 0.01,
    rate: 60.0,
    fixedFrame: "odom",
  });

  urdfClient = addVisualization(
    document.getElementById("robot-element-parent"),
    ROS3D.UrdfClient,
    {
      ros: ros,
      tfClient: tfClient,
      path: "https://127.0.0.1:8080/",
    }
  );
}

async function init_env_after_seconds(seconds) {
  await new Promise((r) => setTimeout(r, seconds * 1000));
  init_env();
}

// Initialisation below

AFRAME.registerInputMappings(mappings);
AFRAME.registerInputActions(inputActions, "default");

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

      initialise_nav(); // pass over to nav.js
    });
  },
});
