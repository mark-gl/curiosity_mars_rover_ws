var end = false
var lastScale = 1

function distanceBetweenLeftAndRight(){
    var righthand = document.getElementById("rightHand");
    var lefthand = document.getElementById("leftHand");
    distance = righthand.object3D.position.distanceTo(lefthand.object3D.position)
    return distance
}
async function rescalegrip(){
    end = false;
    var righthand = document.getElementById("rightHand");
    var lefthand = document.getElementById("leftHand");
    
    var initialdistance = distanceBetweenLeftAndRight() // this is our "1" scale - the rest will be scaled relative to this
    // our scale will be calculated as ((newdistance-initialdistance)/initialdistance)+1
    var i = 0;
    while(!end){
        var newdistance = distanceBetweenLeftAndRight()
        console.log(newdistance)
        await new Promise(r => setTimeout(r, 10)); // "sleep"for x ms
        newscale = (newdistance/initialdistance) * lastScale
        console.log(newscale)
        rescaleaction(null,newscale)
        i++;
        if (i== 1000){
            end = true;
            console.log("i=1000 the scaling is being stopped. This is a failsafe incase the end of the scaling has not been caught")
        }

    }
    lastScale = newscale

}

function endrescalegrip(){
    end = true

}
