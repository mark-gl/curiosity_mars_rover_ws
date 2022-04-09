// This code is used for development purposes
function logEvent(event) {
    var scene = document.querySelector('a-scene');
  
    var type = event.type;
    var currentMappingActions = AFRAME.inputActions[AFRAME.currentInputMapping];
    var text = currentMappingActions[type] ? currentMappingActions[type].label : type;
  
    console.log(text);
  }
  