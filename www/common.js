// Query parameters
// From: https://stackoverflow.com/a/2880929/2698494
urlParams = {};
(window.onpopstate = function () {
    var match,
        pl     = /\+/g,  // Regex for replacing addition symbol with a space
        search = /([^&=]+)=?([^&]*)/g,
        decode = function (s) { return decodeURIComponent(s.replace(pl, " ")); },
        query  = window.location.search.substring(1);

    urlParams = {};
    while (match = search.exec(query))
       urlParams[decode(match[1])] = decode(match[2]);
})();

// State
//
// I've never done Javascript objects before, so this may not be the best way
// to do it.
//
// https://gist.github.com/hallettj/64478
if (typeof Object.create !== 'function') {
    Object.create = function(o) {
        var F = function() {};
        F.prototype = o;
        return new F();
    };
}

var State = {
    clone: function(d) {
        var newState = Object.create(this);

        if (d !== undefined) {
            newState.screen = d["screen"];
            newState.objectName = d["object"];
            newState.faceURL = d["face_url"];
            newState.videoStepURL = d["video_step_url"];
            newState.videoFullURL = d["video_full_url"];
        } else {
            newState.screen = "default";
            newState.objectName = "";
            newState.faceURL = "";
            newState.videoStepURL = "";
            newState.videoFullURL = "";
        }
        return newState;
    }
};

state = State.clone();

// Connecting to ROS
// -----------------
ros = new ROSLIB.Ros();

// Create a connection to the rosbridge WebSocket server.
//
function reconnect(f) {
    if (ros.isConnected != true) {
        console.log("Reconnecting")
		ros.connect('ws://'+window.location.hostname+':9090');

        // Do something after reconnecting
        f();
    }
}

// See if we're connected every once in a while and try to connect if we're not
// Allow passing a function that does something after connecting, e.g. setting
// up services
function autoReconnect(reconFunc, onConFunc) {
    // If there is an error on the backend, an 'error' emit will be emitted.
    ros.on('error', function(error) {
        document.getElementById('connecting').style.display = 'none';
        document.getElementById('connected').style.display = 'none';
        document.getElementById('closed').style.display = 'none';
        document.getElementById('error').style.display = 'inline';
        console.log(error);
    });

    ros.on('connection', function() {
        if (typeof onConFunc !== 'undefined') onConFunc();

        console.log('Connection made!');
        document.getElementById('connecting').style.display = 'none';
        document.getElementById('error').style.display = 'none';
        document.getElementById('closed').style.display = 'none';
        document.getElementById('connected').style.display = 'inline';
    });

    ros.on('close', function() {
        console.log('Connection closed.');
        document.getElementById('connecting').style.display = 'none';
        document.getElementById('connected').style.display = 'none';
        document.getElementById('closed').style.display = 'inline';
    });

    reconnect(reconFunc);
    setInterval(function() { reconnect(reconFunc); }, 10000);
}
