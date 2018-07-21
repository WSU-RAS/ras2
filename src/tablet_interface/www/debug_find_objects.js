//Subscribing to a Topic
//----------------------

// Like when publishing a topic, we first create a Topic object with details of the topic's name
// and message type. Note that we can call publish or subscribe on the same topic object.
var listener = new ROSLIB.Topic({
    ros : ros,
    name : '/find_objects',
    messageType : 'object_detection_msgs/Object'
});

// Then we add a callback to be called every time a message is published on this topic.
listener.subscribe(function(message) {
    //console.log('Received message on ' + listener.name + ': ' + message.data);
    // listener.name
    elem = document.getElementById("msg");

    // Max of this many lines in history
    maxLines = 50;
    oldmsgs = elem.innerHTML.split("\n");
    oldmsg = oldmsgs.slice(0,maxLines).join("\n");

    // Update
    elem.innerHTML = message.name + ": " +
        "{ " +
            message.x + ", " +
            message.y + ", " +
            message.z +
        " }\n" + oldmsg

    // If desired, we can unsubscribe from the topic as well.
    //listener.unsubscribe();
});

// Connect to ROS
autoReconnect(function () {});
