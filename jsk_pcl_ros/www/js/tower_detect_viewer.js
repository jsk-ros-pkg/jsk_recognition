$(function() {
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    var debugp = false;

    var waiting_server = false;
    var state = -1;
    var INITIALSTATE = 0;
    var CONFIRMING_STATE = 1
    
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

    function enableLoading() {
        $("#loader").css("display", "block");
    };

    function disableLoading() {
        // $("#loader").css("display", "none");
    };
    
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

    // broadcast click event to ros
    $(function() {
        var click_topic = new ROSLIB.Topic({
            ros: ros,
            name: "/browser/click",
            messageType: "geometry_msgs/Point"
        });
        var check_circle = new ROSLIB.Service({
            ros: ros,
            name: "/browser/check_circle",
            serviceType: "jsk_pcl_ros/CheckCircle"
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

            var req = new ROSLIB.ServiceRequest({
                point: point
            });
            enableLoading();
            waiting_server = true;
            check_circle.callService(req, function(result) {
                waiting_server = false;
                disableLoading();
                if (result.clicked) {
                    // showing modal
                    alert("you clicked " + result.index);
                }
            });
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
            
            //state = msg.data;
            
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

    // setup loader
    $(function() {
        var param = {
            width: 100,
            height: 50,
            stepsPerFrame: 1,
            trailLength: 1,
            pointDistance: .1,
            fps: 10,
            padding: 10,
            //step: 'fader',
            fillColor: '#ff0000',
            setup: function() {
                this._.lineWidth = 20;
            },
            path: [
                ['line', 0, 0, 100, 0],
                ['line', 100, 0, 0, 0]
                ]
        };
        var a = new Sonic(param);
        var d = document.getElementById("loader");
        d.appendChild(a.canvas);
        a.play();
    });
});
