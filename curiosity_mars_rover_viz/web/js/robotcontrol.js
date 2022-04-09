
function colormap(x){
    _floatColor = new Float32Array(1);
    _rgb_lut = new Float32Array(256);
    for (var i = 0; i < 256; i++) {
        _rgb_lut[i] = i / 255.0
    }
    _floatColor[0] = x;
        const intColorArray = new Int32Array(_floatColor.buffer);
        const intColor = intColorArray[0];

        return {
            r: _rgb_lut[(intColor >> 16) & 0xff],
            g: _rgb_lut[(intColor >> 8) & 0xff],
            b: _rgb_lut[(intColor) & 0xff],
            a: 1.0
        };
}

function moveRobo(click){
    controls = [-click.detail.y,0,0,0,0,-click.detail.x]
          
    moveRobot(controls)
          
}

async function init_env_2(){
    var scene = document.querySelector('a-scene');
    setupEventListeners(scene);
    rootObjectNode = document.getElementById('robot-element-parent')
    // Connect to ROS.
    var ros = new ROSLIB.Ros({
        url: 'wss://127.0.0.1:9090'
    });
    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
        ros: ros,
        angularThres: 0.01,
        transThres: 0.01,
        rate: 60.0,
        fixedFrame: 'odom'
    });

    var gridClient =  addVisualization(rootObjectNode,'gridClient',ROS3D.OccupancyGridClient,{
        ros: ros,
        topic: '/map',
    });

    var urdfClient =   addVisualization(rootObjectNode,'urdfClient',ROS3D.UrdfClient, {
        ros: ros,
        tfClient: tfClient,
        path: 'https://127.0.0.1:8080/',
    });
    
    var cmdVel = addControls('cmd_vel', ROSLIB.Topic,{
        ros : ros,
        name : '/curiosity_mars_rover/ackermann_drive_controller/cmd_vel',
        messageType : 'geometry_msgs/Twist'
    });

    var  cloudClient = addVisualization(rootObjectNode,'cloudClient', ROS3D.PointCloud2,{
        ros: ros,
        tfClient: tfClient,
        topic: '/colored_cloud',
        material: { size: 0.05 },
        colormap: colormap,
        max_pts: 30000000,
        decay:5
    })


    var menu = generateMenuOfVisualizations();
    scene.appendChild(menu)

}

function setupEventListeners(scene){
    var robotMovementEvents = ['moveForward','moveBackward','turnLeft','turnRight','stopRobot']

    scene.addEventListener('changeMode',changeMode)
    scene.addEventListener('moveRobo',moveRobo)


    scene.addEventListener('openMenu', openMenu);

    for (var i = 0; i < robotMovementEvents.length; i++) {
        scene.addEventListener(robotMovementEvents[i], function (event) {
            var type = event.type;
            var currentMappingActions = AFRAME.inputActions[AFRAME.currentInputMapping];
            var parameters = currentMappingActions[type] ? currentMappingActions[type].params : [0,0,0,0,0,0];
            //moveRobot(event);
            moveRobot( parameters)
        });
    }

}
/**
 * Setup all visualization elements when the page is loaded.
 */
async function init_env() {

    var scene = document.querySelector('a-scene');


    // these are the events of common/mappings.js
    // slowly we should replace them by our events
    // common.js -> inputActions 
    // var events = ['dpadleft', 'dpadrightlong', 'dpad', 'logtask1', 'logtask2', 'logdefault', 'righthand', 'lefthand', 'doubletouch', 'doublepress', 'longpress'];
    // if(leftHanded){
        // replace left and right in the strings and register the event listeners
        //  TODO: good idea?
    // }

    rootObjectNode = document.getElementById('robot-element-parent')
    setupEventListeners(scene)


    // Connect to ROS.
    var ros = new ROSLIB.Ros({
        url: 'wss://127.0.0.1:9090'
    });

    // the topic we will publish controls to
    var cmdVel = new ROSLIB.Topic({
        ros : ros,
        name : '/curiosity_mars_rover/ackermann_drive_controller/cmd_vel',
        messageType : 'geometry_msgs/Twist'
      });

    // Setup a client to listen to TFs.
    var tfClient = new ROSLIB.TFClient({
        ros: ros,
        angularThres: 0.01,
        transThres: 0.01,
        rate: 60.0,
        fixedFrame: 'odom'
    });


    var node = document.createElement("a-entity");
    rootObjectNode.appendChild(node)
    var cloudClient = getCloudClient(ros,tfClient,'/colored_cloud',node.object3D,5)


    // node = document.createElement("a-entity");
    // rootObjectNode.appendChild(node)

    // markerClient = new ROS3D.MarkerArrayClient(
    //     {
    //         ros: ros,
    //         topic: '/worldmodel_main/occupied_cells_vis_array',
    //         tfClient: tfClient,
    //         rootObject: node.object3D
    //     }
    // )

    // // Point cloud of the rotating laser scanner
    // var cloudClientOfLaserScanner = new ROS3D.PointCloud2({
    //     ros: ros,
    //     tfClient: tfClient,
    //     topic: '/scan_matched_points2', 
    //     rootObject :  node.object3D,
    //     material: { size: 0.02, color: "rgb(123,45,125)"},

    // });

    node = document.createElement("a-entity");
    rootObjectNode.appendChild(node)
    //Occupancy grid map used for navigation
    var gridClient = new ROS3D.OccupancyGridClient({
        ros: ros,
        topic: '/map',
        rootObject: node.object3D,
    });

    node = document.createElement("a-entity");
    rootObjectNode.appendChild(node)

    var urdfClient = load_urdf(ros, tfClient, node.object3D)

    used_visualisations["urdf"] = urdfClient;
    used_visualisations["cloudClient"] = cloudClient;
    used_visualisations["gridClient"] = gridClient;
    // used_visualisations["markerClient"] = markerClient;
    used_controls["cmd_vel"]  = cmdVel;
}

async function init_env_after_seconds(seconds) {
    await new Promise(r => setTimeout(r, seconds * 1000));
    init_env_2()

}


//controls

/**
 * this needs to be configured to the specific robot
 */
function teleportToRobot(){
    scale = rootObjectNode.getAttribute("scale")
    position = used_visualisations["urdf"].tfClient.frameInfos.base_link.transform.translation //TODO: make configurable
    position.x *=scale.x
    position.y *=scale.y
    position.z *=scale.z

    rig.setAttribute("position",position)
}


/**
 * opening the toggle menu
 */
function openMenu() {
    menuOpen = !menuOpen

    var menugui = document.getElementById('menu-gui')
    
    position = document.getElementById("camera").object3D.getWorldPosition()
    position.x -= document.getElementById("camera").object3D.getWorldDirection().x * 4
    position.z -= document.getElementById("camera").object3D.getWorldDirection().z * 4
    position.y = document.getElementById("camera").object3D.getWorldPosition().y 
    console.log(position)
    menugui.object3D.position = position
    var menuguirotation = new THREE.Vector3( );

    menuguirotation.copy(menugui.object3D.rotation)
    menugui.setAttribute('position',position)

    menugui.object3D.lookAt(document.getElementById('rig').object3D.position)
    // menugui.object3D.rotation.x = menuguirotation.x;
    // menugui.object3D.rotation.z = menuguirotation.z;

    console.log(menugui.object3D.position)
    menugui.setAttribute("visible", menuOpen);

}
