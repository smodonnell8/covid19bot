'use strict'

console.log('Begin');

const module_num = 1;                     // Module number
const lines_of_text = 66;                 // Number of lines of text TeachBot must read
const dir = 'https://localhost:8000/';    // Directory containing resources
const joints = 7;                         // Numer of joints in Sawyer arm
const verbose = true;

/*******************************
 *   Setup ROS communication   *
 *******************************/
var ros = new ROSLIB.Ros({
    url: 'wss://localhost:9090'
});
ros.on('connection', function() { console.log('Connected to websocket server.');});
ros.on('error', function(error) { console.log('Error connecting to websocket server: ', error); window.alert('Error connecting to websocket server'); });
ros.on('close', function() { console.log('Connection to websocket server closed.');});

// Publishing topics
var cmd2shell = new ROSLIB.Topic({ //bot_sequence_topic
    ros: ros,
    name: '/cmd2shell',
    messageType: 'std_msgs/Int32'
});
var audio_duration_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/audio_duration',
    messageType: 'std_msgs/Float64'
});

// Subscribing topics
var nav_receiver = new ROSLIB.Topic({
    ros: ros,
    name: '/right_navigator_button',
    messageType: 'std_msgs/String'
})
var cmd2browser = new ROSLIB.Topic({ //main_seq_receiver
    ros: ros,
    name: '/cmd2browser',
    messageType: 'std_msgs/Int32'
})
var numeric_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/numeric_topic',
    messageType: 'std_msgs/Float64'
})
var array_topic = new ROSLIB.Topic({
    ros: ros,
    name: '/array_topic',
    messageType: 'std_msgs/Float64MultiArray'
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
var ctx = canvas.getContext('2d');
var bar_ctx = [];
for(let j=0; j<joints; j++) {
    bar_ctx.push(document.getElementById('bar' + (j+1)).getContext('2d'));
}
var sawyer_red = getComputedStyle(document.body).getPropertyValue('--sawyer-red')

//percentages for the blank slots on the png
var choice_x_pos = [191/canvas.width,584/canvas.width,191/canvas.width,584/canvas.width,191/canvas.width] 
var choice_y_pos = [105/canvas.height,105/canvas.height,241/canvas.height,241/canvas.height,340/canvas.height]
// var choice_x_pos = [191/odometer.width,584/odometer.width,191/odometer.width,584/odometer.width,191/odometer.width] 
// var choice_y_pos = [105/odometer.height,105/odometer.height,241/odometer.height,241/odometer.height,340/odometer.height]

/************************
 *   Setup Text Files   *
 ************************/
var text_url = dir + 'text/' + module_num + '.txt';
var textArray = new Array(lines_of_text);
jQuery.get(text_url, function(data) {
    textArray = data.split('\n');
},'text');

/*******************************
 *   Setup Image/Video Files   *
 *******************************/
var welcome_image_url = dir + 'images/Welcome.png';
var encoder_img_url = dir + 'images/encoder.jpg';
var motor_animation_url = dir + 'videos/motor_animation.mp4';
var multi_choice_url = dir + 'images/sized_cuff.png';

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

/*******************
*     Setup Dial    *
********************/
var wheel_val = 5; //how far apart numbers are
var dial_spacing = Math.PI/5; //the rotation we want
var desired_val = wheel_val*dial_spacing-Math.PI; //this way, desired points to the bottom of the wheel
var dial_pos = 0;
var dial_speed = 200;
var num_notches= 5

var back_plate = new Image();
var center_dial = new Image();
var indicator = new Image();
var shadow = new Image();
var mask = new Image();

//attempt at preloading images
back_plate.src = dir + 'images/back_plate.png';
indicator.src = dir + 'images/Indicator.png';
shadow.src = dir +  'images/shadow.png';
mask.src = dir + 'images/mask.png';
center_dial.src = dir + 'images/center_dial.png';
var run_odometer = false;

function init_odometer() {
  window.requestAnimationFrame(drawOdometer);  
}

/************************
 *   Helper Functions   *
 ************************/
// Play audio corresponding to index
var player = document.getElementById('player');
async function play(index) {
    player.src = audiofiles_mp3[index];
    player.play();
    modifiable_text.innerHTML = textArray[index];
    pub_dur(audio_duration[index]/1000.0);
    if (verbose) console.log('Playing audio #' + index);
    font_size_fix();
}

// Publish message to ROSTopic to be received by Python
function pub_cmd(counter) {
    let int = new ROSLIB.Message({
        data: counter
    });
    cmd2shell.publish(int);
    if (verbose) console.log('Sent: ' + counter);
}
function pub_dur(audio_duration) {
    let msg = new ROSLIB.Message({
        data: audio_duration
    });
    audio_duration_topic.publish(msg);
}

function drawOdometer() {
    if (run_odometer===true){
      //var ctx = document.getElementById('canvas').getContext('2d');
      //the goal numbers
      var desired_val = -wheel_val*dial_spacing + Math.PI;
      console.log("animation wheel_val", wheel_val)
      var offset = dial_pos%(2*Math.PI);
      var small_offset = dial_pos%(dial_spacing);
      if (dial_pos<0){
        var generated_min = Math.floor(-dial_pos/dial_spacing);
      } else {
        var generated_min = Math.ceil(-dial_pos/dial_spacing);
      }
      //var generated_max =  Math.floor((2*Math.PI-offset)/dial_spacing);
      //the number of numbers that need to be displayed
      if (dial_pos<desired_val-.01){
        dial_pos = dial_pos+2*Math.PI/dial_speed;
      }else if (dial_pos>desired_val+.01){
        dial_pos = dial_pos - 2*Math.PI/dial_speed;
      }else{
        //dial_pos = desired_val;
        generated_min = wheel_val-5;
        small_offset = 0;
      }
      ctx.globalCompositeOperation = 'destination-over';
      ctx.clearRect(0, 0, canvas.width, canvas.width); // clear canvas
      ctx.fillStyle = 'rgba(0, 0, 0, 0.4)';
      ctx.strokeStyle = 'rgba(0, 153, 255, 0.4)';
      ctx.save()
      ctx.drawImage(indicator,0,0);
      ctx.translate(canvas.width/2,canvas.height/2);
      ctx.rotate(offset)
      ctx.drawImage(center_dial,-canvas.width/2,-canvas.height/2);
      ctx.restore();
      ctx.drawImage(mask,0,0);
      ctx.save();
      ctx.translate(canvas.width/2,canvas.height/2);
      ctx.rotate(small_offset);
      ctx.font = "38px Courier New";
      ctx.textAlign="center";
      //drawing the numbers in
      var i;
      for (i=0;i<=(8);i++){
          ctx.save()
          ctx.translate(0,98)
          ctx.rotate(Math.PI);
          ctx.fillText(generated_min+i,0, 0);
          ctx.restore()
          ctx.rotate(dial_spacing)
      }

      ctx.restore();
      ctx.drawImage(shadow,0,0);
      ctx.drawImage(back_plate,0,0);
      window.requestAnimationFrame(drawOdometer);
  } else {
    ctx.clearRect(0, 0, canvas.width, canvas.width); 
  }
}


// Display multiple choice answers
function display_choices(choices_arr, ctx_in=ctx) {
    canvas_container.style.display = 'initial';
    ctx_in.font = canvas.width*.02 +  "px Raleway";
    ctx_in.clearRect(0,0,canvas.width,canvas.height);
    var imgb = new Image();
    ctx_in.fillStyle = "#373737"
    imgb.onload = function(){
        ctx_in.drawImage(imgb, 0,0);
        for (let c=0; c<choices_arr.length; c++) {
            ctx_in.fillText(choices_arr[c],canvas.width*choice_x_pos[c], canvas.height*choice_y_pos[c]);
        }
    }
    imgb.src = multi_choice_url;
    ctx_in.fill()
}

//Update font size
function font_size_fix() {
    $('#adjustable').textfill({
        debug: false,
        maxFontPixels: 0
    });
}

function draw_goal(goal, current, input_text = "Current") {
    //current is the current position of the arm (usually from a rostopic)
    //goal is the static position, which the user is aiming for 
    console.log("drawing goal")
    ctx.clearRect(0,0,canvas.width,canvas.height); 
    var bottom_margin = .9*canvas.height;
    var bar_width = .3*canvas.width;
    var error_margin = 3;
    var cluster = .07;
    //function for hatching

    //arrow
    ctx.textAlign= "left" 
    ctx.strokeStyle = "#373737";
    ctx.fillStyle = "#373737"
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.font = canvas.width*.03 +  "px Raleway";
    ctx.moveTo(canvas.width*.055, canvas.height*.208);
    ctx.fillText("Error",canvas.width*.05, canvas.height*.2);
    ctx.lineTo(canvas.width*.105, canvas.height*.208);
    //line to the center of the error bar 
    ctx.lineTo(canvas.width*(.25+cluster)-.5*bar_width, (bottom_margin - (current*bottom_margin/180)) -(goal-current)*bottom_margin/180/2 )
    ctx.stroke();
    ctx.closePath();
    //comparison rectangle
    ctx.beginPath();
    ctx.rect(canvas.width*(.75-cluster)-.5*bar_width, bottom_margin, bar_width, -goal*bottom_margin/180 )
    ctx.fillStyle = "#373737"
    ctx.fill();
    ctx.strokeStyle = "#555555";
    ctx.lineWidth = 3;
    ctx.stroke();
    ctx.closePath();
    ctx.beginPath();
    //base rectangle
    ctx.rect(canvas.width*(.25+cluster)-.5*bar_width, bottom_margin, bar_width, -((current*bottom_margin/180)));
    ctx.fillStyle="#7C2629";
    ctx.fill();
    ctx.closePath();
    ctx.beginPath();
    //diff rectangle
    ctx.rect(canvas.width*(.25+cluster)-.5*bar_width, (bottom_margin - (current*bottom_margin/180)), bar_width, -((goal-current)*bottom_margin/180));    
    if (current<goal){
        //yellow and red
        ctx.fillStyle="#9d781b";
    } else {
        //yellow
        ctx.fillStyle="#b79f00"
    }
    ctx.fill();
    //text labels
    ctx.fillStyle = "#373737"
    ctx.font = canvas.width*.05 +  "px Raleway";
    ctx.textAlign= "center" 
    ctx.fillText(input_text, canvas.width*(.25+cluster), canvas.height);
    ctx.fillText("Desired", canvas.width*(.75-cluster), canvas.height); 
    //ctx.rect(20,20,100,150);
    //ctx.stroke();
    ctx.closePath();
   
}

function toRadians(deg) {
    return deg * Math.PI / 180
}

// Graphic objects
var ballA = {x:520, y:350, fillStyle: 'BlueViolet'};
var ballB = {x:500, y:240, fillStyle: sawyer_red};
var ballC = {x:495, y:95, fillStyle: 'DarkGreen'};

var barL = {axisLeft: 590, axisRight: 670, maxHeight: 150, antiWidth: 10, fillStyle: sawyer_red};
var barR = {axisLeft: 680, axisRight: 760, maxHeight: 150, antiWidth: 10, fillStyle: 'BlueViolet'};
var barA = {axisLeft: 110, axisRight: 170, maxHeight: 350, antiWidth: 7};
var barB = {axisLeft: 180, axisRight: 240, maxHeight: 350, antiWidth: 7};
var barC = {axisLeft: 250, axisRight: 310, maxHeight: 350, antiWidth: 7};

/**************************
 *   Main Functionality   *
 **************************/
var seq_ind = 0;   // 30
var audio_ind = 0; // 35
async function init() {
    // canvas_container.style.display = 'initial';
    // draw_ball(ctx,ballA.x,ballA.y,40,ballA.fillStyle);
    // draw_ball(ctx,ballB.x,ballB.y,40,ballB.fillStyle); 
    // draw_ball(ctx,ballC.x,ballC.y,40,ballC.fillStyle); 
    // draw_bar(ctx,0.4,Math.PI,barA.axisLeft,barA.axisRight,barA.maxHeight,barA.antiWidth,ballA.fillStyle,'A')
    // draw_bar(ctx,0.8,Math.PI,barB.axisLeft,barB.axisRight,barB.maxHeight,barB.antiWidth,ballB.fillStyle,'B')
    // draw_bar(ctx,1.2,Math.PI,barC.axisLeft,barC.axisRight,barC.maxHeight,barC.antiWidth,ballC.fillStyle,'C')
    if (seq_ind==0) {
        // 0: Hello, my name is Teachbot. It's nice to meet you. Please stand back so I can show you something.
        play(audio_ind);                            // Play opening audio
        await sleep(audio_duration[audio_ind]);     // Wait for audio to finish
        audio_ind++;                                // Increment audio index
        image.style.display = 'none';            // Display welcome image
    }
    pub_cmd(seq_ind);                               // Advance in Py

    // Subscribe to Python Commands
    cmd2browser.subscribe(async function(message) {
        if (verbose) console.log('Received: ' + message.data);

        // Check to make sure the counter is synced with the message data
        if (seq_ind!=message.data) {
            let err_msg = 'Error: message.data = ' + message.data + ' but seq_ind = ' + seq_ind;
            console.log(err_msg);
            throw new Error(err_msg);
        }

        switch (message.data) {
            case 0:
                // 1: Want to know how I did that? Inside my arm I have a bunch of electronic things called electric motors. They let me move my arm, like how muscles move yours. Here, take a look.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                image.style.display = 'none';
                animator.style.display = 'initial';
                animator.play();

                pub_cmd(++seq_ind);
                break;

            case 1:
                animator.style.display = 'none';
                canvas_container.style.display = 'initial';
                run_odometer = true;
                init_odometer();
                // 2: Just by looking, how many motors do you think I have in my arm? To input your answer, wait for me to move my arm. Next, turn the scroll wheel on my arm to scroll through numbers. Then press on the scroll wheel to select your answer.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 2:
                nav_receiver.subscribe(async function(message) {
                    if (message.data.startsWith('Wheel value: ')) {
                        wheel_val = parseInt(message.data.substring(13))%21;
                        console.log('Wheel value', wheel_val, message.data==="Button 'OK': CLICK");
                    } else if (message.data==="Button 'OK': CLICK") {
                        nav_receiver.unsubscribe();
                        nav_receiver.removeAllListeners();
                        run_odometer = false;
                        canvas_container.style.display = 'none';
                        // 3: Step back and we'll count them together.
                        play(audio_ind); await sleep(audio_duration[audio_ind++]);
                        pub_cmd(++seq_ind);
                    }
                });
                break;
            
            case 3:
                image.src = welcome_image_url;
                image.style.display = 'initial';
                // 4: Pay close attention while I move each motor individually. Try to count how many motors I have.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 4:
                // 5: How many motors did you count?
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                image.style.display = 'none';
                canvas_container.style.display = 'initial';
                run_odometer = true;
                init_odometer();

                pub_cmd(++seq_ind);
                break;

            case 5:
                wheel_val = 0;
                nav_receiver.subscribe(async function(message) {
                    if (message.data.startsWith('Wheel value: ')) {
                        wheel_val = parseInt(message.data.substring(13))%21;
                        console.log('Wheel value', wheel_val, message.data==="Button 'OK': CLICK");
                    } else if (message.data==="Button 'OK': CLICK") {
                        console.log(wheel_val);
                        if (wheel_val===joints) {
                            nav_receiver.unsubscribe();
                            nav_receiver.removeAllListeners();
                            run_odometer = false;
                            // 6: Good job! Now, please stand back so I can show you something else.
                            play(audio_ind); await sleep(audio_duration[audio_ind]);audio_ind+=2;
                            canvas_container.style.display = 'none';
                            image.style.display = 'initial';
                            pub_cmd(++seq_ind);
                        } else {
                            // 7: Not quite. Remember that each motor can move in two directions: clockwise and counterclockwise.
                            play(++audio_ind); await sleep(audio_duration[audio_ind--]);
                        }
                    }
                });
                break;

            case 6:
                // 8: Like most of my human coworkers, I too have a torso and an arm. Unlike most of my human coworkers' arms, which only have shoulder, elbow, and wrist joints, my arm has six joints! For simplicity, I'll call this one my shoulder,
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 7:
                // 9: this one my elbow,
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 8:
                // 10: and this one my wrist,
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 9:
                // 11: and lastly, we'll call this one my torso.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 10:
                image.style.display = 'none';
                
                // 12: But how do I tell where my arm is? How do I know how far I've rotated each joint? I'm going to unlock my torso joint. Try pushing my arm. I'll display how much I think you've turned it on the screen. When you're finished, press the scroll wheel button to let me know.
                play(audio_ind++);

                canvas_container.style.display = 'initial';
                draw_bar(ctx, 0, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth);
                numeric_topic.subscribe(async function(message) {
                    if (verbose) console.log('Received: ' + message.data);
                    ctx.clearRect(0,0,canvas.width,canvas.height);
                    draw_bar(ctx, message.data, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth);
                });

                pub_cmd(++seq_ind);
                break;

            case 11:
                numeric_topic.unsubscribe();
                numeric_topic.removeAllListeners();

                // 13: Let me show you how I know. Please stand back.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                ctx.clearRect(0,0,canvas.width,canvas.height);
                canvas_container.style.display = 'none';
                image.src = encoder_img_url;
                image.style.display = 'initial';

                pub_cmd(++seq_ind);
                break;

            case 12:
                // 14: This device is called an encoder. Every motor in my arm has one. These encoders can measure the angles of each of my joints.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 13:
                // 15: Now I'm going to lock my torso and unlock my elbow. Try pushing the tip of my arm again. When you're done, press the scroll wheel button.
                play(audio_ind++);
                image.style.display = 'none';
                canvas_container.style.display = 'initial';
                draw_bar(ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth);
                numeric_topic.subscribe(async function(message) {
                    if (verbose) console.log('Received: ' + message.data);
                    ctx.clearRect(0,0,canvas.width,canvas.height);
                    draw_bar(ctx, message.data, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                });

                pub_cmd(++seq_ind);
                break;

            case 14:
                numeric_topic.unsubscribe();
                numeric_topic.removeAllListeners();

                // 16: So far I've only unlocked one joint at a time. Let's try unlocking both my torso and elbow joints. Push on my arm again and see what happens. When you're done, press the scroll wheel button.
                play(audio_ind++);
                ctx.clearRect(0,0,canvas.width,canvas.height);
                draw_bar(ctx, 0, 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                draw_bar(ctx, 0, 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                array_topic.subscribe(async function(message) {
                    if (verbose) console.log('Received: ' + message.data);
                    ctx.clearRect(0,0,canvas.width,canvas.height);
                    draw_bar(ctx, message.data[0], 120, barL.axisLeft, barL.axisRight, barL.maxHeight, barL.antiWidth, barL.fillStyle);
                    draw_bar(ctx, message.data[1], 120, barR.axisLeft, barR.axisRight, barR.maxHeight, barR.antiWidth, barR.fillStyle);
                });

                pub_cmd(++seq_ind);
                break;

            case 15:
                ctx.clearRect(0,0,canvas.width,canvas.height);

                // 17: Good! Now, before we go further, how many encoders do you think I have in my arm? Step back so I can move my arm for you to use the scroll wheel.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                run_odometer = true;
                init_odometer();

                pub_cmd(++seq_ind);
                break;

            case 16:
                wheel_val = 0;
                nav_receiver.subscribe(async function(message) {
                    if (message.data.startsWith('Wheel value: ')) {
                        wheel_val = parseInt(message.data.substring(13))%21;
                    } else if (message.data==="Button 'OK': CLICK") {
                        console.log(wheel_val);
                        if (wheel_val===joints) {
                            nav_receiver.unsubscribe();
                            nav_receiver.removeAllListeners();
                            run_odometer = false;
                            // 18: Good job!
                            play(audio_ind); await sleep(audio_duration[audio_ind]);audio_ind+=2;
                            ctx.clearRect(0,0,canvas.width,canvas.height);
                            canvas_container.style.display = 'none';
                            image.style.display = 'initial';
                            pub_cmd(++seq_ind);
                        } else {
                            // 19: Not quite. If I have one encoder for every motor, how many encoders do I have?
                            play(++audio_ind); await sleep(audio_duration[audio_ind--]);
                        }
                    }
                });
                break;

            case 17:
                image.style.display = 'none';

                // 20: Now grab and move my arm again. I've unlocked all of my joints, so my arm might drift around a little. Go ahead and grab it anyway while watching the projection to see what I sense as you move each joint. When you're done, press the scroll wheel button.
                play(audio_ind++);
                protractor_table.style.display = 'initial';
                array_topic.subscribe(async function(message) {
                    if (verbose) console.log('Received: ' + message.data);
                    for (let p=1; p<=message.data.length; p++) {
                        document.getElementById('protractor' + p).value = '' + (100*message.data[p-1]);
                        bar_ctx[p-1].clearRect(0,0,canvas.width,canvas.height);
                        draw_bar(bar_ctx[p-1], message.data[p-1], 3.15,30,305,75,20);
                        document.getElementById('bar' + p).value = '' + (100*message.data[p-1]);
                    }
                });

                pub_cmd(++seq_ind);
                break;

            case 18:
                array_topic.unsubscribe();
                array_topic.removeAllListeners();
                
                // 21: Using my encoders, I can measure the angles of all seven of my joints. Please stand back so I can reset my position.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                protractor_table.style.display = 'none';
                canvas_container.style.display = 'initial';
                ctx.clearRect(0,0,canvas.width,canvas.height);
                draw_ball(ctx,ballA.x,ballA.y,40,ballA.fillStyle);  // A
                ctx.font = '30px Arial';
                ctx.fillText('A',ballA.x-45,ballA.y-10);
                draw_ball(ctx,ballB.x,ballB.y,40,ballB.fillStyle);  // B
                ctx.fillText('B',ballB.x-45,ballB.y-10);

                pub_cmd(++seq_ind);
                break;

            case 19:
                // 22: Now let's talk about a new concept, feedback. I've projected some points onto the table. Push my arm so that my shadow is on point B.
                play(audio_ind++);

                pub_cmd(++seq_ind);
                break;

            case 20:
                // 23: Great! Thanks. Now I'm going to move to point B on my own.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 21:
                // 24: Oops! This time I overshot to a new point, C.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                draw_ball(ctx,ballC.x,ballC.y,40,ballC.fillStyle);
                ctx.fillText('C',ballC.x-45,ballC.y-10);

                // 25: Can you correct my position back to point B?
                play(audio_ind++);

                pub_cmd(++seq_ind);
                break;

            case 22:
                // 26: Thank you! At any point, I know the position of my arm because of my encoder. Here is position A,
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 23:
                draw_bar(ctx,0.6,Math.PI,barA.axisLeft,barA.axisRight,barA.maxHeight,barA.antiWidth,ballA.fillStyle,'A')
                // 27: here is position B,
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 24:
                draw_bar(ctx,1,Math.PI,barB.axisLeft,barB.axisRight,barB.maxHeight,barB.antiWidth,ballB.fillStyle,'B')
                // 28: and here is position C.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                pub_cmd(++seq_ind);
                break;

            case 25:
                draw_bar(ctx,1.2,Math.PI,barC.axisLeft,barC.axisRight,barC.maxHeight,barC.antiWidth,ballC.fillStyle,'C')
                // 29: I compare my encoder reading at my current position, C, against the destination encoder position, B. If C is smaller than B, I push left. If C is longer than B, I push right.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                ctx.clearRect(0,0,canvas.width,canvas.height);
                canvas_container.style.display = 'none';
                image.src = welcome_image_url;
                image.style.display = 'initial';

                pub_cmd(++seq_ind);
                break;

            case 26:
                // 30: But that's only one demonstration of feedback. Let me show you another one. Try gently pushing down on my arm. Can you notice the difference?
                play(audio_ind++);

                pub_cmd(++seq_ind);
                break;

            case 27:
                // 31: Watch the screen as you push my arm. I've put up a little display showing how far you've pushed my arm and how much force I'm using to push back. When you're done, press the scroll wheel button.
                play(audio_ind++);

                image.style.display = 'none';
                canvas_container.style.display = 'initial';

                array_topic.subscribe(async function(message) {
                    if (verbose) console.log(message.data);
                    draw_goal(100, message.data[0]*400+100)
                });

                pub_cmd(++seq_ind);
                break;

            case 28:
                array_topic.unsubscribe();
                array_topic.removeAllListeners();
                ctx.clearRect(0,0,canvas.width,canvas.height);
                canvas_container.style.display = 'none';
                image.style.display = 'initial';

                // 32: Before, my motors were turned off, but right now, I'm creating force with my motors to keep my arm in place. What am I using to know when my arm is moving?
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                image.style.display = 'none';
                display_choices(['Motors','Buttons','Cameras','Encoders','Wheels']);

                pub_cmd(++seq_ind);
                break;

            case 29:
                nav_receiver.subscribe(async function(message) {
                    console.log(message.data);
                    if (message.data==='Button \'Square\': CLICK') {
                        nav_receiver.unsubscribe();
                        nav_receiver.removeAllListeners();
                        // 33: That's right! I measured my arm positions, and created a force to push my arm back.
                        play(audio_ind); await sleep(audio_duration[audio_ind]);audio_ind+=2;
                        pub_cmd(++seq_ind);
                    } else if (!message.data.endsWith('OFF')) {
                        // 34: Do you remember pushing my arm and I knew how far I moved?
                        play(++audio_ind); await sleep(audio_duration[audio_ind--]);
                    }
                });
                break;

            case 30:
                ctx.clearRect(0,0,canvas.width,canvas.height);
                image.style.display = 'none'; // XXX DON'T NEED
                canvas_container.style.display = 'initial';
                // 35: Look at the projection again to see how I decide how hard to push back. The black line indicates where I want my arm to be, called the "reference" position. The further you push my arm away from the black line, the harder I tell my motors to push back. It should feel a little bit like a rubber band: the more you displace it, the harder it will want to spring back into its original position.
                play(audio_ind++);

                draw_spring(canvas,100,155,0,50);

                numeric_topic.subscribe(async function(message) {
                    if (verbose) console.log(message.data);
                    draw_spring(canvas,100,155,8*Math.abs(message.data),50);
                });

                pub_cmd(++seq_ind);
                break;

            case 31:
                image.style.display = 'initial'
                canvas_container.style.display = 'none'
                // 36: Now let's talk about why all of this is possible. 
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                // 37: The reason I'm able to teach and interact with you is because I have something called memory. 
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                // 38: I don't actually have a brain like you, so everything you see was put into my memory before we met. This is just like how your cell phone is able to remember phone numbers or store pictures. 
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                // 39: In this last demonstration, I invite you to test my memory and how good it can be. 
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                // 40: I'm going to make my arm go completely limp and you can move it however you'd like. Whenever you want me to memorize a position, press my scroll wheel. After five memorized positions, I will execute each of them in order. 
                play(audio_ind++); 
                pub_cmd(++seq_ind)
                break;

            case 32:
                var saved_locations = 0
                nav_receiver.subscribe(async function(message) {
                    console.log(message.data);
                    if(message.data==='Button \'OK\': OFF'){
                        saved_locations++;
                        pub_cmd(100);
                        if(saved_locations == 5){
                            pub_cmd(++seq_ind);
                            nav_receiver.unsubscribe();
                            nav_receiver.removeAllListeners();
                        }
                    }
                })
                break;

            case 33:
                // 41: Ok, here we go!
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                pub_cmd(++seq_ind);
                break;

            case 34:
                // 42: The first position was here.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                pub_cmd(++seq_ind);
                break;

            case 35:
                // 43: The second position was here.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                pub_cmd(++seq_ind);
                break;

            case 36:
                // 44: The third position was here.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                pub_cmd(++seq_ind);
                break;

            case 37:
                // 45: The fourth position was here.
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                pub_cmd(++seq_ind);
                break;

            case 38:
                // 46: And here was the last position!
                play(audio_ind); await sleep(audio_duration[audio_ind++]);

                // 47: If I wanted to, I could memorize dozens of positions with no problem! In future lessons, I'll demonstrate reasons why this is so useful. But until then, have a good day!
                play(audio_ind); await sleep(audio_duration[audio_ind++]);
                break;


            // case 31:
            //     numeric_topic.unsubscribe();
            //     numeric_topic.removeAllListeners();
            //     ctx.clearRect(0,0,canvas.width,canvas.height);
            //     // 36: Now let's talk about why I have so many joints.
            //     play(audio_ind++);

            //     pub_cmd(++seq_ind);
            //     break;

            // case 32:
            //     // 37: Having so many joints is really important. If I can only move my torso,
            //     play(audio_ind); await sleep(audio_duration[audio_ind++]);

            //     arc3pt(ctx,531,46,520,201,491,295,false);

            //     pub_cmd(++seq_ind);
            //     break;

            // case 33:
            //     // 38: I can only reach points along this curve. So, if I want to reach this point,
            //     play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;

            //     pub_cmd(++seq_ind);
            //     break;
/*
            case 34:
                // 39: I'll need to move another motor, too!
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                pub_cmd(++seq_ind);
                break;
            case 35:
                // 40: I'm going to move my hand back to where it was and use the projector to show the same arc as before.
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                arc3pt(ctx,531,46,520,201,491,295,false);
                pub_cmd(++seq_ind);
                break;
            case 36:
                canvas_container.style.display = 'initial';image.style.display='none';// NICK:DEBUGGING
                // 41: If I want to reach that point, I only need to move one motor, my elbow.
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                ctx.clearRect(0,0,canvas.width,canvas.height);
                draw_star(100,340,5,100,40);
                // 42: How many motors will I need to use to reach this new point?
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                pub_cmd(++seq_ind);
                break;
            case 37:
                // 43: Watch closely and see if you're right. 
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                arc3pt(ctx,542,396,582,175,574,55,true);
                pub_cmd(++seq_ind);
                break;
            case 38:
                // 44: I only needed to use one motor, but it was my shoulder, not my elbow. 
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                pub_cmd(++seq_ind);
                break;
            case 39:
                // 45: With my elbow motor, I could move along this curve, 
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                pub_cmd(++seq_ind);
                break;
            case 40:
                nav_receiver.subscribe(async function(message) {
                    if (verbose) console.log(message.data);
                    nav_receiver.unsubscribe();
                    nav_receiver.removeAllListeners();
                    // 46: ut with my shoulder motor, I can move along this curve. 
                    play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                    pub_cmd(++seq_ind);
                });
                break;
            case 41:
                // 47: Since I have six motors, let me show you everywhere I can move by just moving one of them.
                play(audio_ind); await sleep(audio_duration[audio_ind]); audio_ind++;
                
                break;
*/          
            default:
                await sleep(10);
                break;
        }
    });
}