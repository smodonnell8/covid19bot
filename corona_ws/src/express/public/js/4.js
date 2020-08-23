'use strict'

console.log('Begin');

var module_num = 4;         // Replace this with the module number.
var lines_of_text = 9;      // Replace this with the number of lines of text TeachBot must read.
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
var interaction_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/interaction_topic',
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

/**********************************
 *   Setup SmartSparrow SimCapi   *
 **********************************/
var simModel = new simcapi.CapiAdapter.CapiModel({
    last_interacted_obj: 'none',
    what_action: 'none'
});

/************************
 *   Setup Text Files   *
 ************************/
var text_url = dir + 'text/' + module_num + '.txt';
var textArray = new Array(lines_of_text);
jQuery.get(text_url, function(data) {
    textArray = data.split('\n');
},'text');
text.innerHTML = 'Loading...';

/*************************
 *   Setup Image Files   *
 *************************/
// Example:
var welcome_url = 'https://localhost:8000/images/Welcome.png';
var title_url = 'https://localhost:8000/images/module2_title.jpg'
var cnc_url = 'https://localhost:8000/images/cnc.jpg'

/*************************
 *   Setup HTML Elements   *
 *************************/
var ctx = canvas.getContext('2d');
/*************************
 *   Setup Audio Files   *
 *************************/
var audiofiles_mp3 = new Array(textArray.length);
var audio_duration = new Array(textArray.length);
var loaded = 0;
for(let i=0; i<audiofiles_mp3.length;i++){
    audiofiles_mp3[i] = dir + 'audio/module' + module_num + '/line' + i.toString() + '.mp3';
}

// Preload all the audio files
for (let i in audiofiles_mp3) {
    preloadAudio(audiofiles_mp3[i], i);
}
function preloadAudio(url, i) {
    let audio = new Audio();
    audio.addEventListener('canplaythrough', function() {
        audio_duration[i] = audio.duration*1000;
        loaded++;
        if (loaded == audiofiles_mp3.length){
            // All have loaded. Begin the module.
            init();
        }
    }, false);
    audio.src = url;
}

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
var interaction_counter = 1;
async function init() {
    //Welcome to the last lesson of the gateway module.
    play(counter);      // Play opening audio
    await sleep(audio_duration[counter]);

    counter++;
    //This last lesson will be about tying everything together from the past three lessons and applying what you've learned to real life
    play(counter);
    await sleep(audio_duration[counter]);
    counter++;
    //The first thing we'll talk about is machine tending.
    play(counter);
    await sleep(audio_duration[counter]);

    counter++;
    //There are a ton of machines at machine shops that are really useful for manufacturing, such as mills and lathes.
    play(counter);
    await sleep(audio_duration[counter]);

    counter++;
    //With that in mind, the first thing I'm going to do is apply the concepts we learned about pick and place to a mill. 
    play(counter);
    await sleep(audio_duration[counter]);
    image_changer.src = cnc_url;
    image_changer.style.display = 'initial';


    counter++;
    //As this box comes down the conveyor, I stop it with the event based programming we learned in module 3 as it passes into the pick up location.
    play(counter);
    console.log(audio_duration[counter]);
    await sleep(audio_duration[counter]);
    pub_msg(0);   // Advance in Py

    // Subscribe to Python Commands
    iframe_receiver_seq.subscribe(async function(message){
        console.log('Received: ' + message.data);
    	switch(message.data){
    		case 0:
                // Check to make sure the counter is synced with the message data
    			counter++;
                //Using my knowledge of waypoints and kinematics, I plan my motion to pick up the box.
                play(counter);
                await sleep(audio_duration[counter]);

                // Advance in Py
				pub_msg(1);
                break;

            case 1:
                counter++;
                //I know where I want to place the box according to my encoder values, so I move the box into the CNC mill. 
                play(counter);
                await sleep(audio_duration[counter]);

                pub_msg(2);
                break;

            case 2:
                counter++;
                //Lastly, I place the box with a straight motion in the correct orientation and release my gripper.
                play(counter);
                await sleep(audio_duration[counter]);

                pub_msg(3);
                break;

    	}
    });
}