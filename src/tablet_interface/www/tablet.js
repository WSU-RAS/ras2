// Calling an action
// -----------------
tabletResponse = new ROSLIB.ActionClient({
    ros : ros,
    serverName: '/tablet_response',
    actionName: 'ras_msgs/TabletAction'
});

function sendROSResponse(msg) {
    var timeout = 5000; // Milliseconds

    var goal = new ROSLIB.Goal({
        actionClient: tabletResponse,
        goalMessage: {
            response: msg
        }
    });

    goal.on('feedback', function(feedback) {
        console.log('Feedback: ' + feedback.text);
    });

    goal.on('result', function(result) {
        console.log('Result for '+msg+': ' + result.success);
    });

    // Retry on timeout
    goal.on('timeout', function(result) {
        console.log('Timeout, trying again');
        goal.send(timeout);
    });

    // Try to send the goal
    goal.send(timeout);
    console.log('Sending: '+msg)
}

// Commands for Web UI
// -------------------

function showOne(screen) {
    var screens = ['init','default','choice','options','video'];
    
    // Stop audio if currently playing
    var sound = document.getElementById("audio-wrapper");
    var asource = document.getElementById("audio-source");
    if (asource.getAttribute('src') != "") {
        sound.pause();
        asource.setAttribute('src', "");
    }

    // Stop video if currently playing
    var vid = document.getElementById("video-wrapper");
    var vsource = document.getElementById("video-source");
    if (vsource.getAttribute('src') != "") {
        vid.pause();
        vsource.setAttribute('src', "");
    }

    // Hide all others
    for (var i = 0; i < screens.length; ++i)
        if (screens[i] != screen)
            document.getElementById(screens[i]).style.display = 'none';

    // Make sure the one we want is showing
    if (screen !== undefined)
        document.getElementById(screen).style.display = 'inline';
}


// Multimedia
function playSound(audioURL) {
    var sound = document.getElementById("audio-wrapper");
    var source = document.getElementById("audio-source");
    source.setAttribute('src', audioURL);
    sound.onended = function() {
        respondAudioDone();
    };
    sound.load();
    sound.play();
    //sendROSResponse("audioplay");
}

function getBasename() {
    // Kyoto connection too slow, so we host them on a local server there
    var basename;
    if (window.location.hostname == "wsu-ras-joule.kyoto.local") {
        basename = 'http://kyoto.kyoto.local/'
    } else {
        basename = 'http://casas.wsu.edu/smarthomestats/'
    }
    return basename;
}

function playVideo(url) {
    showOne('video');

    if (url == "") {
        // We're done since apparently we don't have a video for this error
        respondVideoDone();
    } else {
        basename = getBasename();

        var vid = document.getElementById("video-wrapper");
        var source = document.getElementById("video-source");
        source.setAttribute('src', basename + 'videos/' + url);
        vid.onended = function() {
            respondVideoDone();
        };
        vid.addEventListener('error', function() {
            // Not really... but we'd rather not have everything die at this point
            respondVideoDone();
        });
        vid.load();
        vid.play();
        //sendROSResponse("videoplay");
    }
}

// Show screen
function showDefault(happy) {
    showOne('default');
    basename = getBasename();
    if (happy == true)
        document.getElementById("default-face").src = basename + 'pictures/black_happy_with_mouth.jpg';
    else
        document.getElementById("default-face").src = basename + 'pictures/black_happy_without_mouth.jpg';
}
function showChoice() {
    showOne('choice');
    playSound('resources/help-you.mp3');
    basename = getBasename();
    document.getElementById("face").src = basename + 'pictures/black_surprised_with_mouth.jpg';
}
function showOptions() {
    showOne('options');
    playSound('resources/select-options.mp3');

    // Only play if we have just navigated to an object
    if (state.objectName == "done") {
        playSound('resources/here-you-go.mp3');
        document.getElementById("buttonGoTo").style.display = 'none';
    } else {
        // Only show the "go to object" button if there is an object for this error
        if (state.objectName.length > 0) {
            document.getElementById("buttonGoTo").style.display = 'inline';
            document.getElementById("object").innerHTML = state.objectName;
        } else {
            document.getElementById("buttonGoTo").style.display = 'none';
        }
    }
}

// User response
function respondChoice(choice) {
    var request;
    if (choice == true) {
        sendROSResponse("yes");
        showOptions();
    } else {
        sendROSResponse("no");
        showDefault(false);
    }
}
function respondOptions(option) {
    switch (option) {
        case "watchfull":
            sendROSResponse(option);
            playVideo(state.videoFullURL);
            break;
        case "watchstep":
            sendROSResponse(option);
            playVideo(state.videoStepURL);
            break;
        case "goto":
            showDefault(false);
            sendROSResponse(option);
            break;
        case "complete":
            showDefault();
            sendROSResponse(option);
            break;
        default:
            console.log("Unknown option selection");
            break;
    }
}
function respondVideoDone() {
    sendROSResponse("videodone");
    showDefault(false);
}
function respondAudioDone() {
    //sendROSResponse("audiodone");
}

// Selecting buttons
document.getElementById("buttonYes").onclick = function() {
    respondChoice(true);
}
document.getElementById("buttonNo").onclick = function() {
    respondChoice(false);
}
document.getElementById("buttonVideoFull").onclick = function() {
    respondOptions("watchfull");
}
document.getElementById("buttonVideoStep").onclick = function() {
    respondOptions("watchstep");
}
document.getElementById("buttonGoTo").onclick = function() {
    respondOptions("goto");
    playSound('resources/follow-me.mp3');
}
document.getElementById("buttonComplete").onclick = function() {
    respondOptions("complete");
    showDefault(true); // Show happy face
    playSound('resources/okay-thank-you.mp3');
}
document.getElementById("buttonInit").onclick = function() {
    showDefault(false);
    
    // This appears to fix the not playing on first call problem
    playSound("");
}

// Connect to ROS and automatically reconnect, creating the tablet service each time
autoReconnect(function() { }, function () {
    // Advertising a Service
    // ---------------------
    // The Service object does double duty for both calling and advertising services
    var tablet = new ROSLIB.Service({
        ros : ros,
        name : '/tablet',
        serviceType : 'tablet_interface/Tablet'
    });

    // Use the advertise() method to indicate that we want to provide this service
    tablet.advertise(function(request, response) {
        var screen = request["screen"];

        // Update global state
        state = State.clone(request);

        // Show desired screen
        switch (screen) {
            case "default": showDefault(false); break;
            case "choice":  showChoice();  break;
            case "options": showOptions(); break;
            default:
                console.log("Unknown screen");
                break;
        }

        response['success'] = true;
        return true;
    });
});
