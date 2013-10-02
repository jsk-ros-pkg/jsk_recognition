$(function() {
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    var debugp = $("body").attr("data-debug") == "true";

    var CANVAS_DIV_ID = "mjpeg";
    
    var mjpeg_viewer = new MJPEGCANVAS.Viewer({
        divID : CANVAS_DIV_ID,
        host : location.hostname,
        width : 640,
        height : 480,
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
            $("#coordinates").html("(" + x + ", " + y + ")");
            click_topic.publish(point);
        });
    });
});
