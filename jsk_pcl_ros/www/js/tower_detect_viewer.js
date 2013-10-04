$(function() {
    var ros = new ROSLIB.Ros({
        url: "ws://" + location.hostname + ":9090"
    });
    var waiting_server = false;
    var state = -1;             // state is always updated by /browser/state
    
    var STATE = {
        INITIAL: 1,
        SELECT_TOWER: 2,
        CONFIRM: 3,
        START_TASK: 4,
        INITIALIZE_PROBLEM: 5,
        MOVE_LARGE_S_G: 6,
        MOVE_MIDDLE_S_I: 7,
        MOVE_LARGE_G_I: 8,
        MOVE_SMALL_S_G: 9,
        MOVE_LARGE_I_S: 10,
        MOVE_MIDDLE_I_G: 11,
        MOVE_LARGE_S_G: 12,
    };

    function stateToString(state) {
        for (var key in STATE) {
            if (state == STATE[key])
                return key;
        }
        return "NONE";
    }
    
    
    var CANVAS_DIV_ID = "mjpeg";
    var clicked_index = -1;

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
        $("#loader").css("display", "none");
    };

    ///////////////////////////////////////////////////////
    // mjpeg viewer
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
    
    ///////////////////////////////////////////////////////
    // click event
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
            if (state != STATE.INITIAL) {
                return;
            }
            var req = new ROSLIB.ServiceRequest({
                point: point
            });
            enableLoading();
            waiting_server = true;
            check_circle.callService(req, function(result) {
                waiting_server = false;
                disableLoading();
                if (result.clicked) {
                    clicked_index = result.index;
                    // showing modal
                    //alert("you clicked " + result.index);
                    var color_name = "";
                    var color_class = "";
                    if (result.index == 0) {
                        color_name = "赤";
                        color_class = "label-danger";
                    }
                    else if (result.index == 1) {
                        color_name = "緑";
                        color_class = "label-success";
                    }
                    else if (result.index == 2) {
                        color_name = "青";
                        color_class = "label-primary";
                    }
                    $(".clicked-tower-color")
                        .removeClass("label-danger")
                        .removeClass("label-success")
                        .removeClass("label-primary")
                        .addClass(color_class)
                        .html(color_name);
                    $("#confirm-modal").modal();
                }
            });
        });
    });

    // modal yes click
    $(function() {
        var pickup = new ROSLIB.Service({
            ros: ros,
            name: "/browser/pickup",
            serviceType: "jsk_pcl_ros/TowerPickUp"
        });
        
        $("#confirm-modal-yes-button").click(function() {
            var req = new ROSLIB.ServiceRequest({
                index: clicked_index
            });
            enableLoading();
            $("#confirm-modal").modal("hide");
            pickup.callService(req, function(result) {
                disableLoading();
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
            messageType: "std_msgs/Int16"
        });
        listener.subscribe(function(msg) {
            state = msg.data;
            $("#state").html("state: " + stateToString(state));
        });
    });

    ///////////////////////////////////////////////////////
    // help modal
    // 
    // debug button
    // toggle #debug visibility when debug button is clicked
    $("#debug-button").click(function() {
        if ($(this).hasClass("btn-default")) {
            $(this).addClass("btn-danger")
                .removeClass("btn-default")
                .html("有効");
            $("#debug").css("display", "block");
        }
        else {
            $(this).addClass("btn-default")
                .removeClass("btn-danger")
                .html("無効");
            $("#debug").css("display", "none");
        }
    });
    // enable fullscreen button
    $("#fullscreen-button").click(function() {
        if (this.webkitRequestFullScreen) {
            this.webkitRequestFullScreen();
            updateCanvasSize(mjpeg_viewer);
        }
        else if (this. mozRequestFullScreen) {
            this. mozRequestFullScreen();
            updateCanvasSize(mjpeg_viewer);
        }
        else {
            alert("全画面モードはサポートされていません");
        }
    });

    // calling updateCanvasSize when modal view is shown
    $(".modal").on("show.bs.modal", function() {
        setTimeout(function() {
            updateCanvasSize(mjpeg_viewer);
            console.log("foooo");
        }, 1000);
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
