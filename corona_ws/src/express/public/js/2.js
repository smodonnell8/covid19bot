'use strict'

console.log('Begin');

var module_num = 2;         // Replace this with the module number.
var lines_of_text = 45;      // Replace this with the number of lines of text TeachBot must read.
var wifi = true        
if(wifi){
    var ipaddress = '10.31.204.240';
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
var iframe_sequence_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/bot_sequence',
    messageType: 'std_msgs/Int32'
});
var audio_duration_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/audio_duration',
    messageType: 'std_msgs/Float64'
});
var save_info_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/save_info',
    messageType: 'std_msgs/String'
})
var joy_connected_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/joy/connected',
    messageType: 'std_msgs/Bool'
})
var joy_cmd_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/joy/cmd',
    messageType: 'user_input/JoyCmd'
})
// Subscribing topics
var cmd2browser = new ROSLIB.Topic({ //main_seq_receiver
    ros: ros,
    name: '/cmd2browser',
    messageType: 'std_msgs/Int32'
})
var iframe_receiver_seq = new ROSLIB.Topic({
    ros: ros,
    name: '/iframe_sequence',
    messageType: 'std_msgs/Int32'
})
var unique_input_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/unique_input',
    messageType: 'std_msgs/String'
})

var image_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/camera/color/image_raw/compressed',
    messageType: 'sensor_msgs/CompressedImage'
})
/************************
 *   Setup Text Files   *
 ************************/

/*************************
 *   Setup Image Files   *
 *************************/
// Example:
var image_changer = document.getElementById('image_changer')
// image_changer.src = 'http://0.0.0.0:8080/stream_viewer?topic=/camera/color/image_raw';
var frame_counter = 0;
/*************************
 *   Setup HTML Elements   *
 *************************/
var ctx = canvas.getContext('2d');

/************************
 *   Helper Functions   *
 ************************/
// Sleep for some milliseconds. Usage:
// await sleep(ms);
function sleep(ms) {
    return new Promise(resolve => setTimeout(resolve, ms));
}

// Publish message to ROSTopic to be received by Python
function pub_msg(index) {
    let int = new ROSLIB.Message({
        data: index
    });
    iframe_sequence_topic.publish(int);
    console.log('Sent: ' + index);
}
function pub_dur(audio_duration) {
    let msg = new ROSLIB.Message({
        data: audio_duration
    });
    //audio_duration_topic.publish(msg);
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

function draw_rectangle(cx,cy, width, length, color){
    //ctx.clearRect(0,0,canvas.width,canvas.height);
    var corner1x = cx-width/2;
    var corner2x = cx+width/2;

    var corner1y = cy-length/2;
    var corner2y = cy+length/2;

    ctx.beginPath();    
    ctx.moveTo(corner1x,corner1y);
    ctx.lineTo(corner1x,corner2y);
    ctx.lineTo(corner2x,corner2y);
    ctx.lineTo(corner2x,corner1y);
    ctx.lineTo(corner1x,corner1y);

    ctx.lineWidth = 7;
    if(color == 'blue'){
        ctx.strokeStyle = '#3577D8';  
    }
    else if(color == 'red'){
        ctx.strokeStyle = '#C01414';
    }
    else if(color == 'green'){
        ctx.strokeStyle = '#20C014'

    }
    ctx.stroke()
    
}
var connected = false;
window.addEventListener("gamepadconnected", function(e) {
    console.log("Gamepad connected at index %d: %s. %d buttons, %d axes.",
    e.gamepad.index, e.gamepad.id,
    e.gamepad.buttons.length, e.gamepad.axes.length);

    let joy_msg = new ROSLIB.Message({
        data: true
    });
    joy_connected_topic.publish(joy_msg)

});

window.addEventListener("gamepaddisconnected", function(e) {
    console.log("Gamepad disconnected from index %d: %s",
        e.gamepad.index, e.gamepad.id);
    let joy_msg = new ROSLIB.Message({
        data: false
    });
    joy_connected_topic.publish(joy_msg)
    console.log('published connection')

});

function pollGamepads() {
    connected = true
    var gamepads = navigator.getGamepads ? navigator.getGamepads() : (navigator.webkitGetGamepads ? navigator.webkitGetGamepads : []);
    if(gamepads[0]){
        var gp = gamepads[0];
        // let gp_msg = new ROSLIB.Message({
        //     axis1: gp.axes[0],
        //     axis2: gp.axes[1],
        //     axis3: gp.axes[2],
        //     btn1: gp.buttons[0].value,
        //     btn2: gp.buttons[1].value,
        //     btn3: gp.buttons[2].value,
        // });
        var gp_msg = new ROSLIB.Message({
            axis1: gp.axes[0],
            axis2: gp.axes[1],
            axis3: gp.axes[2],
            btn1: gp.buttons[0].value,
            btn2: gp.buttons[1].value,
            btn3: gp.buttons[2].value
        });
        joy_cmd_topic.publish(gp_msg);

        // console.log("ID: " + gp.id)
        // console.log("Buttons: " + gp.buttons[0].value + "," + gp.buttons[1].value + "," + gp.buttons[2].value)
        // console.log("Axes: " + gp.axes[0] + "," + gp.axes[1] + "," + gp.axes[2])
    }
}

image_topic.subscribe(function(message){
    var imagedata = "data:image/jpg;base64," + message.data;
    image_changer.setAttribute('src', imagedata);
})
main()

// ***************** Main Function************************************ //
async function main() {
    // canvas_container.style.display = 'initial';
    // draw_ball(ctx,ballA.x,ballA.y,40,ballA.fillStyle);
    // draw_ball(ctx,ballB.x,ballB.y,40,ballB.fillStyle); 
    // draw_ball(ctx,ballC.x,ballC.y,40,ballC.fillStyle); 
    // draw_bar(ctx,0.4,Math.PI,barA.axisLeft,barA.axisRight,barA.maxHeight,barA.antiWidth,ballA.fillStyle,'A')
    // draw_bar(ctx,0.8,Math.PI,barB.axisLeft,barB.axisRight,barB.maxHeight,barB.antiWidth,ballB.fillStyle,'B')
    // draw_bar(ctx,1.2,Math.PI,barC.axisLeft,barC.axisRight,barC.maxHeight,barC.antiWidth,ballC.fillStyle,'C')

    // Subscribe to Python Commands
    cmd2browser.subscribe(async function(message) {
        switch (message.data) {

            default:
                pollGamepads();
                await sleep(10);
                var joy_msg = new ROSLIB.Message({
                    data: connected
                });
                joy_connected_topic.publish(joy_msg)
                await sleep(5);
                break;
        }
    });
}