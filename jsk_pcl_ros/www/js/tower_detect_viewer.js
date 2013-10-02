$(function() {
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    var debugp = $("body").attr("data-debug") == "true";

    var CANVAS_DIV_ID = "mjpeg";

    var aspect_ratio = 640 / 480.0;
    var window_height = $(window).height();
    var window_width = $(window).width();
    var target_width = window_width;
    var target_height = window_height;
    if ((window_width / window_height) > aspect_ratio) { // too long width
        var height_should_be = (1.0 / aspect_ratio) * window_width;
        target_height = height_should_be;
        // setting offset
        $("#" + CANVAS_DIV_ID).css("margin-top", "-" + (target_height - window_height) / 2.0 + "px");
    }
    else {
        var width_should_be = aspect_ratio * window_height;
        target_width = width_should_be;
        $("#" + CANVAS_DIV_ID).css("margin-left", "-" + (target_width - window_width) / 2.0 + "px");
    }
    var mjpeg_viewer = new MJPEGCANVAS.Viewer({
        divID : CANVAS_DIV_ID,
        host : location.hostname,
        width : target_width,
        height : target_height,
        topic : '/image_marked'
    });

    // broadcast click event to ros
    $(function() {
        var click_topic = new ROSLIB.Topic({
            ros: ros,
            name: "/browser/click",
            messageType: "geometry_msgs/Point"
        });
        $("#mjpeg canvas").click(function(e) {
            var offset = $(this).offset();
            var x = e.clientX - offset.left;
            var y = e.clientY - offset.top;
            //console.log(x + ", " + y);
            var point = new ROSLIB.Message({
                x: x / $(this).width(),
                y: y / $(this).height(),
                z: 0
            });
            
            click_topic.publish(point);
        });
    });

    // draw /browser/message
    $(function() {
        var listener = new ROSLIB.Topic({
            ros: ros,
            name: "/browser/message",
            messageType: "std_msgs/String"
        });
        listener.subscribe(function(msg) {
            $("#message").html(msg.data);
        });
    });

    // draw /browser/state
    $(function() {
        var listener = new ROSLIB.Topic({
            ros: ros,
            name: "/browser/state",
            messageType: "std_msgs/String"
        });
        listener.subscribe(function(msg) {
            $("#state").html("state: " + msg.data);
        });
    });
});
