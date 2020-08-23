'use strict'

console.log('Begin');

var module_num = 2;         // Replace this with the module number.
var lines_of_text = 5;      // Replace this with the number of lines of text TeachBot must read.
var dir = 'https://localhost:8000/';
/*******************************
 *   Setup ROS communication   *
 *******************************/
var ros = new ROSLIB.Ros({
    url: 'wss://localhost:9090'
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
// Subscribing topics
var iframe_receiver_nav = new ROSLIB.Topic({
    ros: ros,
    name: '/right_navigator_button',
    messageType: 'std_msgs/String'
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


/*************************
 *   Setup Image Files   *
 *************************/
// Example:
var welcome_url = 'https://localhost:8000/images/Welcome.png';
var title_url = 'https://localhost:8000/images/module2_title.jpg'

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

// Play audio corresponding to index (e.g. smartsparrowaudio/module1/line1.mp3)
var player = document.getElementById('player');
async function play(index) {
    player.src = audiofiles_mp3[index];
    player.play();
    text.innerHTML = textArray[index];
    pub_dur(audio_duration[index]/1000.0);
    console.log('Playing audio #' + index);
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


/**************************
 *   Main Functionality   *
 **************************/
 //testing waypoints pub_msg 25, counter = 30
 //obstacle course pub_msg 28 counter = 34
var counter = 0;
init();
async function init() {

    await sleep(2000);
    //Today we will be learning...
    //play(counter);      // Play opening audio
    draw_rectangle(725, 440, 110, 40, 'blue'); //box1
    draw_rectangle(610, 440, 40, 110, 'blue'); //box2
    draw_rectangle(700, 275, 80, 150,'green'); //bin1
    draw_rectangle(480, 450, 110, 40, 'blue'); //box3
    draw_rectangle(550, 300, 150, 80, 'green'); //bin2
    draw_rectangle(322, 315, 40, 110, 'blue'); //box4
    //draw_obstacle_course(1,99)
    textdiv.style.display = 'initial';
    image_changer.style.display = 'none'
    canvas_container.style.display = 'initial';
    iframe_receiver_nav.subscribe(async function(message){ 

        if(message.data === 'Button \'OK\': OFF'){
            pub_msg(counter);
            counter++;
        }else if(message.data==='Button \'OK\': OFF'){
            counter--;
            pub_msg(counter);
            counter++;
        }
        if(counter == 5){
            iframe_receiver_nav.unsubscribe();
            iframe_receiver_nav.removeAllListeners();
        }
    })
}