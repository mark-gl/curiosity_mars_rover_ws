var buttonDown = false;

function delay(n){
    return new Promise(function(resolve){
        setTimeout(resolve,n*1000);
    });
}

var ros = new ROSLIB.Ros({
    url: 'wss://127.0.0.1:9090'
});

ros.on('connection', function() {
    console.log('Connected to ROS!');
  });

// Mast and Arm
function mast() {
    mastClient.callService(requestToggle, function(result) {
        document.getElementById("mast_state").innerHTML = result.status_message.slice(17);
    });
}
function arm() {
    armClient.callService(requestToggle, function(result) {
        document.getElementById("arm_state").innerHTML = result.status_message.slice(16);
    });
}

function armSet(id, value) {
    var req = new ROSLIB.ServiceRequest({})
    switch (id)
    {
        case 'joint1':
            req = new ROSLIB.ServiceRequest({ mode: 'set', pos_arm_01: parseFloat(-1.57 * value/100) })
            break;
        case 'joint2':
            req = new ROSLIB.ServiceRequest({ mode: 'set', pos_arm_02: parseFloat(-1.57 * value/100) })
            break;
        case 'joint3':
            req = new ROSLIB.ServiceRequest({ mode: 'set', pos_arm_03: parseFloat(-0.9 * value/100) })
            break;
        case 'joint4':
            req = new ROSLIB.ServiceRequest({ mode: 'set', pos_arm_04: parseFloat(-1.57 * value/100) })
            break;
        case 'effector':
            req = new ROSLIB.ServiceRequest({ mode: 'set', pos_arm_tools: parseFloat(-1.57 * value/100) })
            break;
        default:
            break;
}
    armClient.callService(req, function(result) {
        document.getElementById("arm_state").innerHTML = result.status_message.slice(16);
    });
}



var mastClient = new ROSLIB.Service({
    ros : ros,
    name : '/curiosity_mars_rover/mast_service',
    serviceType : 'curiosity_mars_rover_control/Mast'
});
var armClient = new ROSLIB.Service({
    ros : ros,
    name : '/curiosity_mars_rover/arm_service',
    serviceType : 'curiosity_mars_rover_control/Arm'
});

var mastListener = new ROSLIB.Topic({
    ros : ros,
    name : '/curiosity_mars_rover/mast_state',
    messageType : 'std_msgs/String'
});

var armListener = new ROSLIB.Topic({
    ros : ros,
    name : '/curiosity_mars_rover/arm_state',
    messageType : 'std_msgs/String'
});

var requestPing = new ROSLIB.ServiceRequest({ mode: 'ping' });
var requestToggle = new ROSLIB.ServiceRequest({ mode: 'toggle' });

// console.error(urdfClient)
// if (ros) {
//     document.getElementById('status').innerHTML = "Connected to rover"
//     document.getElementById('status').style.color = "rgb(17, 207, 0)";
// }

mastClient.callService(requestPing, function(result) {
    document.getElementById("mast_state").innerHTML = result.status_message.slice(17); 
    document.getElementById('status').innerHTML = "Connected to rover";
    document.getElementById('status').style.color = "rgb(17, 207, 0)";
}, function(error) {
    document.getElementById('status').innerHTML = "Couldn't connect.";
    document.getElementById('status').style.color = "rgb(255, 47, 47)";
});
armClient.callService(requestPing, function(result) {
    document.getElementById("arm_state").innerHTML = result.status_message.slice(16);
    armControls(result.status_message.slice(16));
});

mastListener.subscribe(function(message) {
    document.getElementById("mast_state").innerHTML = message.data;
});

armListener.subscribe(function(message) {
    document.getElementById("arm_state").innerHTML = message.data;
    armControls(message.data)
});

function armControls(message) {
    if (message == "Closed") {
        document.getElementById("joint1").disabled = true;
        document.getElementById("joint2").disabled = true;
        document.getElementById("joint3").disabled = true;
        document.getElementById("joint4").disabled = true;
        document.getElementById("effector").disabled = true;
    }
    else if (message == "Open") {
        document.getElementById("joint1").disabled = false;
        document.getElementById("joint1").value = 0.0;
        document.getElementById("joint2").disabled = false;
        document.getElementById("joint2").value = 0.0;
        document.getElementById("joint3").disabled = false;
        document.getElementById("joint3").value = 0.0;
        document.getElementById("joint4").disabled = false;
        document.getElementById("joint4").value = 0.0;
        document.getElementById("effector").disabled = false;
        document.getElementById("effector").value = 0.0;
    }
}

// Navigation
var sendingNav = false;

function pickMode() {
    sendingNav = true;
}

// document.getElementById("nav_state").innerHTML = "Awaiting Goal";

var move_base = new ROSLIB.Topic({
    ros : ros,
    name : '/move_base_simple/goal',
    messageType : 'geometry_msgs/PoseStamped'
});

var move_base_feedback = new ROSLIB.Topic({
    ros : ros,
    name : '/move_base/feedback',
    messageType : 'move_base_msgs/MoveBaseActionFeedback'
});

var move_base_result = new ROSLIB.Topic({
    ros : ros,
    name : '/move_base/result',
    messageType : 'move_base_msgs/MoveBaseActionResult'
});

move_base_feedback.subscribe(function(message) {
    document.getElementById("nav_state").innerHTML = "Navigating to goal";
});

move_base_result.subscribe(function(message) {
    document.getElementById("nav_state").innerHTML = "Reached goal!";
});


AFRAME.registerComponent('tap-place', {
    schema: {
        min: {default: 6},
        max: {default: 10},
    },
    init() {
        const ground = document.getElementById('worldterrain')
        var newElement = document.createElement('a-cylinder')
        document.getElementById('main').appendChild(newElement)
        newElement.setAttribute('height', 500)
        newElement.setAttribute('radius', 0.4)
        newElement.setAttribute('color', "cyan")
        newElement.setAttribute('material', {opacity: 0.0, transparent: true})
        newElement.setAttribute('visible', 'true')
        newElement.setAttribute('scale', '0.7 1.2 0.7')
        newElement.setAttribute('shadow', {receive: false})

        ground.addEventListener('click', (event) => nav(event), false)
        async function nav(event) {
            if (sendingNav) {
                // The raycaster gives a location of the touch in the scene
                var touchPoint = event.detail.intersection.point
                newElement.setAttribute('position', touchPoint)
                newElement.setAttribute('material', {opacity: 0.2, transparent: true})

                var currentTime = new Date();
                var mySecs = Math.floor(currentTime.getTime()/1000);
                var myNsecs = Math.round(1000000000*(currentTime.getTime()/1000-mySecs));
                var nav = new ROSLIB.Message({
                header: {
                    seq: 0,
                    stamp: {
                        secs: mySecs,
                        nsecs: myNsecs
                    },
                    frame_id: 'odom'
                },
                pose: {
                    position: {
                    x: event.detail.intersection.point.x,
                    y: -1 * event.detail.intersection.point.z,
                    z: 0.0, },
                    orientation: {
                    x: 0.0,
                    y: 0.0,
                    z: 0.0,
                    w: 1.0}
                    }
                });
                move_base.publish(nav);
                await delay(2);
                newElement.setAttribute('material', {opacity: 0.0, transparent: true})
                sendingNav = false;
            }
            }    
    },

});


var keepPublishingTeleop;
var keepPublishingMast;

function teleop(params) {
    keepPublishingTeleop = setInterval(function() {
        moveRobot(params)
      }, 16);
}

function telestop() {
    clearInterval(keepPublishingTeleop)
}

function mastClick(params) {
    keepPublishingMast = setInterval(function() {
        moveMast(params)
      }, 16);
}

function mastStop() {
    clearInterval(keepPublishingMast)
}


