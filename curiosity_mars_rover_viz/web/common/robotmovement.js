/**
 * Instead of taking the parameters, we also want to be able to work if someone moves diagonally
 * so instead of just taking the movement and publishing it - which only publishes one of two commands, we will try to
 * publish one at a time
 * 
 * @param {*} x linear x 
 * @param {*} y linear y
 * @param {*} z  linear z
 * @param {*} a angular x
 * @param {*} b angular y
 * @param {*} c angular z
 */
function moveRobot(x,y,z,a,b,c){
    console.log(x)

      var twist = new ROSLIB.Message({
        linear : {
          x : x,
          y : y,
          z : z
        },
        angular : {
          x : a,
          y : b,
          z : c
        }
      });
      used_controls['cmd_vel'].publish(twist);
}