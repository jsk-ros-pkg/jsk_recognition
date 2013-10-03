$(function() {
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    var debugp = false;

    var CANVAS_DIV_ID = "mjpeg";

    function calcCanvasSize() {
        var aspect_ratio = 640 / 480.0;
        var window_height = $(window).height();
        var window_width = $(window).width();
        var target_width = window_width;
        var target_height = window_height;
        if ((window_width / window_height) > aspect_ratio) { // too long width
            var height_should_be =( 1.0 / aspect_ratio) * window_width;
            target_height = height_should_be;
        }
        else {
            var width_should_be = aspect_ratio * window_height;
            target_width = width_should_be;
        }
        return {
            window_height: window_height,
            window_width: window_width,
            canvas_width: target_width,
            canvas_height: target_height,
            aspect_ratio: aspect_ratio
        };
    }

    function updateCanvasSize(viewer) {
        var size = calcCanvasSize();
        viewer.width = size.canvas_width;
        viewer.height = size.canvas_height;
        $("#" + CANVAS_DIV_ID)
            .css("width", size.window_width + "px")
            .css("height", size.window_height + "px")
            .find("canvas")
            .attr("width", size.canvas_width + "px")
            .attr("height", size.canvas_height + "px");
        
        if ((size.window_width / size.window_height) > size.aspect_ratio) { // too long width
            $("#" + CANVAS_DIV_ID)
                .find("canvas")
                .css("margin-top", "-" + ((size.canvas_height - size.window_height) / 2.0) + "px");
        }
        else {
            $("#" + CANVAS_DIV_ID).find("canvas")
                .css("margin-left", "-" + ((size.canvas_width - size.window_width) / 2.0) + "px");
        }
    }

    // initialize mjpeg viewer
    var mjpeg_viewer = new MJPEGCANVAS.Viewer({
        divID : CANVAS_DIV_ID,
        host : location.hostname,
        topic : '/image_marked',
    });
    
    updateCanvasSize(mjpeg_viewer);
    // when the window size changes, change the
    // size and offset of the canvas
    $(window).resize(function() {
        updateCanvasSize(mjpeg_viewer);
    });

    // $(window).bind("orientationchange",function(){
    //     updateCanvasSize(mjpeg_viewer);
    // });

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

    // debug button
    $("#debug-button").click(function() {
        if ($(this).hasClass("btn-default")) {
            $(this).addClass("btn-danger")
                .removeClass("btn-default")
                .html("有効");
            debugp = true;
            $("#overlay").css("display", "block");
        }
        else {
            $(this).addClass("btn-default")
                .removeClass("btn-danger")
                .html("無効");
            $("#overlay").css("display", "none");
            debugp = false;
        }
    });
    
    // question button
    // $(function() {
    //     $("#question-button").click(function() {
    //         return false;
    //     });
    // });

    $("#fullscreen-button").click(function() {
        if (this.webkitRequestFullScreen) {
            this.webkitRequestFullScreen();
        }
        else if (this. mozRequestFullScreen) {
            this. mozRequestFullScreen();
        }
        else {
            alert("not found")
        }
    });
    
});
