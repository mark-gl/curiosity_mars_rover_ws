/**
 *   for info https://aframe.io/docs/1.0.0/components/vive-controls.html#events
 *  this file contains our mappings for the controls
 *   https://blog.mozvr.com/input-mapping/ - read this to understand whats happening here
 **/
class Controls {
  static mappings = {
    mappings: {
      default: {
        "vive-controls": {
          "trackpad.up": "navigationend",
          "trackpad.down": "navigationstart",
          trackpadmoved: "moveJoy",
        },
        "oculus-touch-controls": {
          "trigger.down": "navigationstart",
          "trigger.up": "navigationend",
          "ybutton.down": "mastToggle",
          "bbutton.down": "armToggle",
          thumbstickmoved: "moveJoy",
        },
        keyboard: {
          i_down: "moveForward",
          j_down: "turnLeft",
          k_down: "moveBackward",
          l_down: "turnRight",

          i_up: "stopRobot",
          j_up: "stopRobot",
          k_up: "stopRobot",
          l_up: "stopRobot",

          t_down: "mastUp",
          g_down: "mastDown",
          f_down: "mastLeft",
          h_down: "mastRight",

          q_down: "speedUp",
          z_down: "slowDown",

          m_down: "mastToggle",
          n_down: "armToggle",
        },
      },
    },
  };

  static inputActions = {
    default: {
      moveForward: { params: [1, 0, 0, 0, 0, 0] },
      moveBackward: { params: [-1, 0, 0, 0, 0, 0] },
      turnLeft: { params: [0, 0, 0, 0, 0, 1] },
      turnRight: { params: [0, 0, 0, 0, 0, -1] },
      stopRobot: { params: [0, 0, 0, 0, 0, 0] },
      moveJoy: { params: "" },
      speedUp: { params: "" },
      slowDown: { params: "" },
      mastToggle: { params: "" },
      armToggle: { params: "" },
      mastUp: { params: "" },
      mastDown: { params: "" },
      mastLeft: { params: "" },
      mastRight: { params: "" },
    },
  };
}
