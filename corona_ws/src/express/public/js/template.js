'use strict'

console.log('Begin');

var module_num = #;                     // Replace this with the module number.
var lines_of_text = #;                  // Replace this with the number of lines of text TeachBot must read.
var dir = 'https://localhost:8000/';    // Replace this with the directory of resources for TeachBot

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

/**********************************
 *   Setup SmartSparrow SimCapi   *
 **********************************/
var simModel = new simcapi.CapiAdapter.CapiModel({
    last_interacted_obj: 'none',
    what_action: 'none'
});

/*********************
 *   HTML Elements   *
 *********************/
var choices = [choice_back,choice_rethink,choice_circle,choice_square,choice_x];
var choice_text = [choice_desc_back,choice_desc_rethink,choice_desc_circle,choice_desc_square,choice_desc_x]
var ctx = canvas.getContext('2d');

/************************
 *   Setup Text Files   *
 ************************/
var text_url = dir + 'module_text/module' + module_num + '.txt';
var textArray = new Array(lines_of_text);
jQuery.get(text_url, function(data) {
    textArray = data.split('\n');
},'text');
var text = 'Loading...';

/*******************************
 *   Setup Image/Video Files   *
 *******************************/
// Example:
// var sawyer_image_url = dir + 'images/Sawyer.jpg';

/*************************
 *   Setup Audio Files   *
 *************************/
var audiofiles_mp3 = new Array(textArray.length);
var audio_duration = new Array(textArray.length);
var loaded = 0;
for(var i=0; i<audiofiles_mp3.length;i++){
    audiofiles_mp3[i] = dir + 'smartsparrowaudio/module' + module_num + '/line' + i.toString() + '.mp3';
}

// Preload all the audio files
for (var i in audiofiles_mp3) {
    preloadAudio(audiofiles_mp3[i], i);
}
function preloadAudio(url, i) {
    var audio = new Audio();
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
}

// Publish message to ROSTopic to be received by Python
function pub_msg(counter) {
    var int = new ROSLIB.Message({
        data: counter;
    });
    iframe_sequence_topic.publish(int);
}

// Display multiple choice gallery with answers
function display_choices(choices_arr) {
    for (let c=0; c<choices.length; c++) {
        choices[c].style.display = 'initial';
        choice_text[c].innerHTML = choices_arr[c];
    }
}

// Hide multiple choice gallery
function hide_choices() {
    for (let c=0; c<choices.length; c++) {
        choices[c].style.display = 'none';
    }
}

/**************************
 *   Main Functionality   *
 **************************/
var seq_ind = 0;
var audio_ind = 0;
async function init() {
    play(audio_ind);                        // Play opening audio
    await sleep(audio_duration[seq_ind]);   // Wait for audio to finish
    audio_ind++;                            // Increment index
    pub_msg(seq_ind);                       // Advance in Py

    // Subscribe to Python Commands
    iframe_receiver_seq.subscribe(async function(message) {
        console.log('Received: ' + message.data);

        // Check to make sure the counter is synced with the message data
        if (seq_ind!=message.data) {
            let err_msg = 'Error: message.data = ' + message.data + ' but seq_ind = ' + seq_ind;
            console.log(err_msg);
            throw new Error(err_msg);
        }

    	switch (message.data) {
    		case 0:
    			// TODO: Play audio, change images, etc.

                pub_msg(++seq_ind);         // Advance in Py
                break;
            // TODO: More Cases

    	}
    });
}