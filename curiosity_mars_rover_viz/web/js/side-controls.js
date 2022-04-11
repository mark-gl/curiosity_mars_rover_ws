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
console.error(mastClient)

mastClient.callService(requestPing, function(result) {
    document.getElementById("mast_state").innerHTML = result.status_message.slice(17); 
    document.getElementById('status').innerHTML = "Connected to rover";
    document.getElementById('status').style.color = "rgb(17, 207, 0)";
}, function(error) {
    document.getElementById('status').innerHTML = "Couldn't connect.";
    document.getElementById('status').style.color = "rgb(255, 47, 47)";
});
armClient.callService(requestPing, function(result) {
    document.getElementById("arm_state").innerHTML = result.status_message.slice(16); });

mastListener.subscribe(function(message) {
    document.getElementById("mast_state").innerHTML = message.data;
});

armListener.subscribe(function(message) {
    document.getElementById("arm_state").innerHTML = message.data;
});


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
        newElement.setAttribute('height', 1)
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
