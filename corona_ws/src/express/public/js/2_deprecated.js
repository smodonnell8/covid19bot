'use strict'

console.log('Begin');

var module_num = 22;         // Replace this with the module number.
var lines_of_text = 34;      // Replace this with the number of lines of text TeachBot must read.
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
async function init() {

    await sleep(2000);
    //Today we will be learning...
    play(counter);      // Play opening audio
    draw_rectangle(730, 450, 88, 38, 'blue'); //box1
    draw_rectangle(610, 448, 38, 88, 'blue'); //box2
    draw_rectangle(700, 275, 80, 150,'green'); //bin1
    draw_rectangle(500, 450, 88, 38, 'blue'); //box3
    draw_rectangle(550, 300, 150, 80, 'green'); //bin2
    draw_rectangle(350, 450, 88, 38, 'blue'); //box4
    //draw_obstacle_course(1,99)
    await sleep(750);
    textdiv.style.display = 'initial';
    image_changer.style.display = 'none'
    canvas_container.style.display = 'initial';
    await sleep(audio_duration[counter]);
    pub_msg(0);   // Advance in Py
    // Subscribe to Python Commands
    iframe_receiver_seq.subscribe(async function(message){
        console.log('Received: ' + message.data);
    	switch(message.data){
    		case 0:
                // Check to make sure the counter is synced with the message data
                if(counter!=0) {
                    var err_msg = 'Error: message.data = ' + message.data + ' but counter = ' + counter;
                    console.log(err_msg);
                    throw new Error(err_msg);
                }

    			counter++;
                //To begin with, let me try to grab it with my hands
                clear_canvas();
                draw_rectangle(730, 450, 88, 38, 'blue'); //box1
                play(counter);
                await sleep(audio_duration[counter]);

                // Advance in Py
				pub_msg(1);
                break;

            case 1:
                counter++;
                //Oh no, I've seemed to make a mistake.
                play(counter);
                await sleep(audio_duration[counter]);       
                iframe_receiver_nav.subscribe(async function(message){
                    if(message.data==='Button \'Square\': OFF'){
                        iframe_receiver_nav.unsubscribe();
                        iframe_receiver_nav.removeAllListeners();
                        counter++;
                        //The first lesson of the day will be about orientation.
                        play(counter);
                        await sleep(audio_duration[counter]);
                        pub_msg(2);
                    }
                    else{
                        await sleep(10);
                    }

                })

                break;

            case 2:
                counter++;
                //...if I change the orientation of my hand.
                play(counter);
                await sleep(audio_duration[counter]);
                counter++;
                //Let's try another example. I want to package box2
                play(counter);
                clear_canvas();
                draw_rectangle(610, 448, 38, 88, 'blue'); //box2
                await sleep(audio_duration[counter]);
                save_info('time_start')
                pub_msg(3);
                break;

            case 3:
                counter = 6;
                //Now can you rotate my hand to the correct orientation?
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(4);
                break;
            
            case 4:

                save_info('time_end')
                counter++;
                //Alright, let's see.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(5);
                break;


            case 5:
                //incorrect. can't pick up the shoe
                save_info('incorrect')
                counter=8;
                //Oops, I couldn't pick it up like that. 
                play(counter);
                await sleep(audio_duration[counter]);
                save_info('time_start')
                pub_msg(3);

                break;

            case 6:
                //correct. picked up the shoe
                save_info('correct')
                counter=9;
                //Great! In addition to orienting my hand correctly.
                play(counter);
                await sleep(audio_duration[counter]);
                counter++;
                //For example, now that I've picked up the shoe...
                clear_canvas();
                draw_rectangle(700, 275, 80, 150,'green'); //bin1
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(6);
                break;

            case 7:
                counter++;
                //it won't fit. So before I can put the shoe in the shoebox...
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(7);
                break;

            case 8:
                counter++;
                //Now I can package the shoe. 
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(8);
                break;

            case 9:
                counter = 13;
                //Now let's try moving this 3rd object over into the 2nd bin
                play(counter);
                clear_canvas();
                draw_rectangle(500, 450, 88, 38, 'blue'); //box3
                await sleep(audio_duration[counter]);
                save_info('time_start')
                pub_msg(9);
                break;

            case 10:
                counter = 14;
                //Now can you rotate my hand to the correct orientation so that I can grasp it?
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(10);
                break;

            case 11:
                save_info('time_end')
                counter++;
                //Alright, let's see 
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(11);
                break;

            case 12:
                save_info('incorrect')
                counter++;
                //Oops, I couldn't pick it up like that.
                play(counter);
                await sleep(audio_duration[counter]);
                save_info('time_start')
                pub_msg(12);
                break;

            case 13:
                save_info('correct')
                counter=17;
                //Now that I've grasped the object, do you think I can move it into the shoebox directly?
                clear_canvas();
                draw_rectangle(550, 300, 150, 80, 'green'); //bin2
                play(counter);
                await sleep(audio_duration[counter]);
                iframe_receiver_nav.subscribe(async function(message){ 
                    if(message.data == 'Button \'Square\': OFF'){
                        //Correct
                        pub_msg(13);
                        iframe_receiver_nav.unsubscribe()
                        iframe_receiver_nav.removeAllListeners();

                    } else if(message.data == 'Button \'X\': OFF'){
                        //Incorrect
                        pub_msg(14);
                        iframe_receiver_nav.unsubscribe()
                        iframe_receiver_nav.removeAllListeners();
                    }
                })
                break;

            case 14:
                save_info('correct')
                //correct answer
                counter=18;
                //That's correct. I can't just move directly to this mark...
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(15);
                break;  

            case 15:
                //after dropping it with the correct answer
                counter=19;
                //Well. Let's try again.
                play(counter);
                await sleep(audio_duration[counter]);
                iframe_receiver_nav.subscribe(async function(message){ 
                    if(message.data =='Button \'Square\': OFF'){
                        //Wait for object to be placed
                        pub_msg(16);
                        iframe_receiver_nav.removeAllListeners();
                    }
                })
                break;

            case 16:
                //demonstration of what needed to happen
                counter=20;
                //If I want to move it onto the mark, I'll have to do something different. First...
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(17);
                break;

            case 17:
                counter++;
                //I can move it above the point
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(18);
                break;

            case 18:
                counter++;
                //Lower it and release it
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(19);
                break;

            case 19:
                save_info('incorrect')
                //incorrect answer
                counter=23;
                //Well, let's find out!
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(20);
                break;

            case 20:
                //incorrect answer continued
                counter++;
                //Oh no! Turns out I can't move it directly to the mark. Waiting on the square button
                play(counter);
                await sleep(audio_duration[counter]);
                iframe_receiver_nav.subscribe(async function(message){ 
                    if(message.data =='Button \'Square\': OFF'){
                        //Wait for object to be placed
                        pub_msg(21);
                        iframe_receiver_nav.removeAllListeners();
                    }
                })
                //go to case 16
                break;

            case 21:
                counter=25;
                //This method of moving my arm around used a thing called waypoints.
                play(counter);
                await sleep(audio_duration[counter]);
                counter++;
                //Waypoints are like little stops I make on my way to my destination
                play(counter);
                await sleep(audio_duration[counter]);
                counter++;
                //Now let's say I want to move the object back.
                play(counter);
                await sleep(audio_duration[counter]);
                clear_canvas();
                draw_rectangle(500, 450, 88, 38, 'blue'); //box3
                counter++;
                pub_msg(22);
                break;

            case 22:
                counter = 28;
                //To do so, I'd have to go through a different path considering I'm starting from the end position rather than the start.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(23); 
                break;

            case 23:
                counter++;
                //Before we start the last lesson, could you move my arm over the 4th box?
                play(counter);
                await sleep(audio_duration[counter]);
                clear_canvas();
                draw_rectangle(350, 450, 88, 38, 'blue'); //box4
                save_info('time_start');
                pub_msg(24);
                break;

            case 24:
                counter=30;
                //Now can you rotate my hand to the correct orientation so that I can grasp it?
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(25);
                break;

            case 25:
                save_info('time_end');
                counter = 31;
                //Alright, let's see.
                play(counter);
                await sleep(audio_duration[counter]);
                pub_msg(26);
                break;

            case 26:
                save_info('incorrect');
                counter = 32;
                //Oops, I couldn't pick it up like that. Can you 
                play(counter);
                await sleep(audio_duration[counter]);
                save_info('time_start');
                pub_msg(24);
                break;

            case 27:
                save_info('correct');
                counter =  33;
                //Now that I've grabbed the 4th box, let's talk about collision avoidance.
                play(counter);
                await sleep(audio_duration[counter]);
                clear_canvas();
                //draw_obstacle_course(1,99);
                pub_msg(99);
                break;

            default:
                await sleep(10);
                break;
            // TODO: More Cases

    	}
    });
}