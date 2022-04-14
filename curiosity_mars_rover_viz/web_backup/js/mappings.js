/**
 *   for info https://aframe.io/docs/1.0.0/components/vive-controls.html#events
 *  this file contains our mappings for the controls
*   https://blog.mozvr.com/input-mapping/ - read this to understand whats happening here
**/

var leftHanded = false;
var mappings = {
  mappings: {
    default: {
    common: {
      triggerdown: {left: 'lefthand', right: 'righthand'}
    },
    'vive-controls': {
      'trackpad.up': 'navigationend',
      'trackpad.down': 'navigationstart',
    },
    'oculus-touch-controls': {
      'trigger.down': 'navigationstart',
      'trigger.up': 'navigationend',
      'abutton.down': 'navigationstart',
      'abutton.up': 'navigationend',
    },
    'windows-motion-controls': {
      'grip.down': 'changeTask'
    },
    keyboard: {
      't_down': 'navigationstart',
      't_up': 'navigationend',
      'c_up': 'changeTask',
      'o_down': 'logtask1',
      'r_down':'changeMode',
    }
  },
  roboControls:{
    'vive-controls': {      
      'trackpad.up': 'navigationend',
      'trackpad.down': 'navigationstart',
      'trackpadmoved' : 'moveRobo',
    },
    'oculus-touch-controls': {
      'trigger.down': 'navigationstart',
      'trigger.up': 'navigationend',
      'ybutton.down': 'mastToggle',
      'bbutton.down' : 'armToggle',
      'thumbstickmoved':'moveRobo'
    },
    keyboard: {
      'i_down': 'moveForward',      
      'i_up' : 'stopRobot',

      'j_down': 'turnLeft',
      'j_up' : 'stopRobot',

      'k_down': 'moveBackward',
      'k_up' : 'stopRobot',

      'l_down': 'turnRight',
      'l_up' : 'stopRobot',

      't_down': 'mastUp',
      'g_down': 'mastDown',
      'f_down': 'mastLeft',
      'h_down': 'mastRight',

      'q_down':'speedUp',
      'z_down':'slowDown',

      'm_down':'mastToggle',
      'n_down':'armToggle',

    }
  }
  }
};


/**
 * There is no "common"  inputAction like there is in mapping
 * what should actually happen after the events are fired - depending on which 'mode' we are in
 */
var inputActions = {
  default: {
    changeMode: { currentMode: 'default' },
    scaleSmall : { percent : 20},
    rescalegrip : { start: 'no parameter needed?'},
    endrescalegrip : { start: 'no parameter needed?'}

  },
  controls: {
    logtask2: { label: 'Test Log Task 2' }
  },
  roboControls:{ // eg. mode 2 - control the robot
    moveForward: { params: [1,0,0,0,0,0]},
    moveBackward: { params: [-1,0,0,0,0,0] }, 
    turnLeft:{ params: [0,0,0,0,0,1]},
    turnRight:{ params: [0,0,0,0,0,-1]},
    stopRobot: {params : [0,0,0,0,0,0]},
    moveRobo:{ params: 'keine'},
    changeMode: { currentMode: 'roboControls' },
    logtask1: { label: 'Test Log RoboMode' }, 
    speedUp: {params: ''},
    slowDown: {params: ''},
    mastToggle: {params: ''},
    armToggle: {params: ''},
    mastUp: {params: 'up'},
    mastDown: {params: 'down'},
    mastLeft: {params: 'left'},
    mastRight: {params: 'right'},
  }
}


// we need to register the inputactions whenever we want to use them
// we need to always register mappings and the actions
AFRAME.registerInputMappings(mappings);
// to change the mode, we register other input mappings 
AFRAME.registerInputActions(inputActions, 'roboControls'); 

var modes = [{'name': 'default',color:'#FF0000'},{'name':'roboControls',color:'#0000FF'}]
var currentMode = 1
/**
 * changes the effect of the mappings
 * the possible modes are:
 * default
 * roboControls
 * 
 * 
 * 
 * @param {*} mode one of the defined inputActions as shown above
 */
function changeMode(mode){
  AFRAME.registerInputActions(inputActions, mode);
}

function changeMode(){
  currentMode = (currentMode+1)%modes.length
  console.log(currentMode)
  document.getElementById('mode-indicator').setAttribute('color',modes[currentMode]['color'])
  document.getElementById('mode-text').setAttribute('text',"value: " + modes[currentMode]['name']+";")

  AFRAME.registerInputActions(inputActions, modes[currentMode]['name']);
}