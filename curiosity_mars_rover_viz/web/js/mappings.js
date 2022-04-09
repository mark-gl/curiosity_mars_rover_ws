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
      'grip.down': 'rescalegrip',
      'grip.up': 'endrescalegrip',
      'menu.down': { right:  'changeMode'},
      'trackpad.up': 'teleportend',
      'trackpad.down': 'teleportstart',
      'trackpad.doubletouch': 'openMenu',
      'trackpad.doublepress': 'doublepress',
      // Activators for down, up, touchstart and touchend are optionals you can just write the event without the .
      'trackpaddpadleftdown': 'dpadleft',
      'trackpaddpadright.longpress': 'dpadrightlong'
    },
    'oculus-touch-controls': {
      'abutton.down': 'teleportstart',
      'abutton.up': 'teleportend',
      'bbutton.down' : 'changeMode',
      'xbutton.up': 'openMenu',

    },
    'windows-motion-controls': {
      'grip.down': 'changeTask'
    },
    keyboard: {
      't_down': 'teleportstart',
      't_up': 'teleportend',
      'c_up': 'changeTask',
      'm_up': 'openMenu',
      'o_down': 'logtask1',
      'r_down':'changeMode',
      'q_down':'speedUp',
      'z_down':'slowDown',
    }
  },
  roboControls:{ // sadly this needs to be duplicated TODO: add it dynamically with sth like mappings.robocontrols = x and mappings.default= x
    'vive-controls': {      
      'menu.down': { right:  'changeMode'},
      'trackpad.up': 'teleportend',
      'trackpad.down': 'teleportstart',
      'trackpad.doubletouch': 'openMenu',
      'trackpad.doublepress': 'doublepress',
      'trackpadmoved' : 'moveRobo',
      // Activators for down, up, touchstart and touchend are optionals you can just write the event without the .
      'trackpaddpadleftdown': 'dpadleft',
      'trackpaddpadright.longpress': 'dpadrightlong'
    },
    'oculus-touch-controls': {
      'abutton.down': 'teleportstart',
      'abutton.up': 'teleportend',
      'bbutton.down' : 'changeMode',
      'xbutton.up': 'openMenu',
      'thumbstickmoved':'moveRobo'

    },
    keyboard: {
      'm_up': 'openMenu',

      'i_down': 'moveForward',      
      'i_up' : 'stopRobot',

      'j_down': 'turnLeft',
      'j_up' : 'stopRobot',

      'k_down': 'moveBackward',
      'k_up' : 'stopRobot',

      'l_down': 'turnRight',
      'l_up' : 'stopRobot',

      'o_down':'logtask1'


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
    openMenu: { label : 'Open Menu'},
    changeMode: { currentMode: 'default' },
    scaleSmall : { percent : 20},
    rescalegrip : { start: 'no parameter needed?'},
    endrescalegrip : { start: 'no parameter needed?'}

  },
  controls: {
    openMenu: { label : 'openMenu'},
    logtask2: { label: 'Test Log Task 2' }
  },
  roboControls:{ // eg. mode 2 - control the robot
    openMenu: { label : 'Open Menu'},
    moveForward: { params: [1,0,0,0,0,0]},
    moveBackward: { params: [-1,0,0,0,0,0] }, 
    turnLeft:{ params: [0,0,0,0,0,1]},
    turnRight:{ params: [0,0,0,0,0,-1]},
    stopRobot: {params : [0,0,0,0,0,0]},
    moveRobo:{ params: 'keine'},
    changeMode: { currentMode: 'roboControls' },
    logtask1: { label: 'Test Log RoboMode' }, 
    rescalegrip : { start: 'no parameter needed?'},
    endrescalegrip : { start: 'no parameter needed?'}
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
