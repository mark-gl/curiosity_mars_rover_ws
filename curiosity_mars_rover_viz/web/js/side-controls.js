

// Calling a service
// -----------------
var ros = new ROSLIB.Ros({
    url: 'wss://127.0.0.1:9090'
});

var mastClient = new ROSLIB.Service({
    ros : ros,
    name : '/curiosity_mars_rover/mast_service',
    serviceType : 'gazebo_msgs/DeleteModel'
});
var armClient = new ROSLIB.Service({
    ros : ros,
    name : '/curiosity_mars_rover/arm_service',
    serviceType : 'gazebo_msgs/DeleteModel'
});

var requestPing = new ROSLIB.ServiceRequest({ model_name: 'ping' });
var requestToggle = new ROSLIB.ServiceRequest({ model_name: 'toggle' });

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

mastClient.callService(requestPing, function(result) {
    document.getElementById("mast_state").innerHTML = result.status_message.slice(17); });
armClient.callService(requestPing, function(result) {
    document.getElementById("arm_state").innerHTML = result.status_message.slice(16); });

    
// Subscribing to a Topic
// ----------------------

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

mastListener.subscribe(function(message) {
    document.getElementById("mast_state").innerHTML = message.data;
});

armListener.subscribe(function(message) {
    document.getElementById("arm_state").innerHTML = message.data;
});