'use strict'

console.log('Begin');

var module_num = 2;         // Replace this with the module number.
var lines_of_text = 45;      // Replace this with the number of lines of text TeachBot must read.
var wifi = true       
if(wifi){
    var ipaddress = '18.21.181.74';
}
else{
    var ipaddress = 'localhost'
}
var dir = 'https://' + ipaddress + ':8000/';
//var dir = 'https://localhost:3000/';

/*******************************
 *   Setup ROS communication   *
 *******************************/
var ros = new ROSLIB.Ros({
    url: 'wss://' + ipaddress + ':9090'
});
ros.on('connection', function() { console.log('Connected to websocket server.');});
ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });
ros.on('close', function() { console.log('Connection to websocket server closed.');});

// Publishing topic

// var joy_cmd_topic = new ROSLIB.Topic({
//     ros: ros,
//     name: '/joy/cmd',
//     messageType: 'user_input/JoyCmd'
// })
// Subscribing topics

var image_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/color/image_raw/compressed',
    messageType: 'sensor_msgs/CompressedImage'
})
var voxels2browser = new ROSLIB.Topic({
    ros: ros,
    name: '/images/voxels',
    messageType: 'user_input/Voxel'
})
/************************
 *   Setup Text Files   *
 ************************/
// var text_url = dir + 'text/' + module_num + '.txt';
// var textArray = new Array(lines_of_text);
// jQuery.get(text_url, function(data) {
//     textArray = data.split('\n');
// },'text');
// text.innerHTML = 'Loading...';

/*************************
 *   Setup Image Files   *
 *************************/
// Example:
var image_changer = document.getElementById('image_changer')
var frame_counter = 0;
/*************************
 *   Setup HTML Elements   *
 *************************/
var canvas = document.getElementById('canvas')
var ctx = canvas.getContext('2d');


/************************
 *   Helper Functions   *
 ************************/
// Sleep for some milliseconds. Usage:
// await sleep(ms);
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}


function save_info(string) {
    //String should be "correct", "incorrect", "time_start", "time_end"
    //"correct" and "incorrect" logs 
    let msg = new ROSLIB.Message({
        data: string
    });
    save_info_topic.publish(msg);
}


function clear_canvas(){
    ctx.clearRect(0,0, canvas.width, canvas.height);
}


function draw_voxels(depths,lengths){
    var corners = [];
    // scale 1 pixel to 10 mm
    for(var i = 0; i < depths.length; i++){
        depths[i] = depths[i]/10.0;
    };
    ctx.beginPath();
    var curr_posx = 0;
    ctx.moveTo(curr_posx,0);
    for(var i=0;i < depths.length; i ++){
        curr_posx += lengths[i]
        ctx.lineTo(curr_posx, Math.round(depths[i]));
    }
    ctx.lineTo(curr_posx,0);
    ctx.lineTo(0,0);
    ctx.fillStyle = '#3577D8';  
    ctx.fill();
    // console.log('drawn')
}
main()

// ***************** Main Function************************************ //
async function main() {
    canvas_container.style.display = 'initial';
    // draw_ball(ctx,ballA.x,ballA.y,40,ballA.fillStyle);
    // draw_ball(ctx,ballB.x,ballB.y,40,ballB.fillStyle); 
    // draw_ball(ctx,ballC.x,ballC.y,40,ballC.fillStyle); 
    // draw_bar(ctx,0.4,Math.PI,barA.axisLeft,barA.axisRight,barA.maxHeight,barA.antiWidth,ballA.fillStyle,'A')
    // draw_bar(ctx,0.8,Math.PI,barB.axisLeft,barB.axisRight,barB.maxHeight,barB.antiWidth,ballB.fillStyle,'B')
    // draw_bar(ctx,1.2,Math.PI,barC.axisLeft,barC.axisRight,barC.maxHeight,barC.antiWidth,ballC.fillStyle,'C')

    // Subscribe to Python Commands
    voxels2browser.subscribe(async function(message) {
        clear_canvas();
        var depths = message.depths;
        var lengths = message.lengths;
        draw_voxels(depths, lengths);
        await sleep(1);
        // console.log('message received');
    });
}