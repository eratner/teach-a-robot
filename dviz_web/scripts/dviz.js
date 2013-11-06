/** 
 * @author Ellis Ratner - eratner@bowdoin.edu
 */
var DVIZ = DVIZ || {};

/**
 * Creates a simple moving average (SMA) filter of size n.
 */
var createSMAFilter = function(n) {
  var pointer = 0;
  var buffer = [];

  // Initialize the buffer to all zeros.
  for(var i = 0; i < n; ++i) {
    buffer.push(0);
  }

  return {
    getValue : function() {
      var mean = 0;
      for(var i = 0; i < n; ++i) {
	mean += buffer[i];
      }
      return (mean/n);
    },

    push : function(item) {
      buffer[pointer] = item;
      pointer = (pointer + 1) % n;
    },

    get : function(key) {
      return buffer[key];
    }
  };
};

/**
 * A camera manager for DViz.
 *
 * @constructor
 * @param options - object with the following keys:
 *  * ros - a handle to the ROS connection
 *  * tfClient - the TF client handle to use
 *  * viewer - a handle to the ROS3D Viewer
 *  * width - the width of the Viewer's canvas
 *  * height - the height of the Viewer's canvas
 *  * id - the ID of the associated DVizUser on the server
 *  * filter (optional) - filter the TF messages for smoother camera
 *  * filterBufferSize (optional) - how many TF messages to average over
 */
DVIZ.CameraManager = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.viewer = options.viewer;
  this.canvasWidth = options.width;
  this.canvasHeight = options.height;
  this.id = options.id;
  this.filterTf = options.filter || false;
  this.filterBufferSize = options.filterBufferSize || 50;

  this.cameraMode = 0;
  this.lastCameraMode = 1;

  this.zMode = false;

  // Use SMA filters to make the camera smoother.
  this.xFilter = createSMAFilter(this.filterBufferSize);
  this.yFilter = createSMAFilter(this.filterBufferSize);

  console.log('[CameraManager] user_id = ' + this.id + ' topic: /dviz_user_' + this.id + '/base_footprint');

  this.tfClient.subscribe('/dviz_user_' + this.id + '/base_footprint', function(message) {
    var tf = new ROSLIB.Transform(message);

    that.xFilter.push(tf.translation.x);
    that.yFilter.push(tf.translation.y);

    // Center the camera at the base of the robot.
    if(that.cameraMode === 0) {
      if(that.filterTf) {
	that.viewer.cameraControls.center.x = that.xFilter.getValue();
	that.viewer.cameraControls.center.y = that.yFilter.getValue();
      } else {
	that.viewer.cameraControls.center.x = tf.translation.x;
	that.viewer.cameraControls.center.y = tf.translation.y;
	//that.viewer.cameraControls.center.z = tf.translation.z;
      }
      that.viewer.cameraControls.update();
    }
  });
};

/**
 * Sets the camera mode. @todo
 *
 * @param mode
 */
DVIZ.CameraManager.prototype.setCamera = function(mode) {
  if(mode !== this.cameraMode) {
    console.log('[DVizClient] Changing from camera mode ' + this.cameraMode
                + ' to ' + mode + '.');
  }
  this.lastCameraMode = this.cameraMode;
  this.cameraMode = mode;
};

DVIZ.CameraManager.prototype.setFilterTf = function(filter) {
  this.filterTf = filter;
}

/**
 * A web-based demonstration visualizer client.
 * 
 * @constructor
 * @param options - object with the following keys:
 *  * ros - a handle to the ROS connection
 *  * tfClient - the TF client handle to use
 *  * viewer - a handle to the ROS3D Viewer
 *  * width - the width of the Viewer's canvas
 *  * height - the height of the Viewer's canvas
 *  * id - the ID of the associated DVizUser on the server
 */
DVIZ.DemonstrationVisualizerClient = function(options) {
  var that = this;
  var ros = options.ros;
  var tfClient = options.tfClient;
  var viewer = options.viewer;
  var width = options.viewerWidth;
  var height = options.viewerHeight;
  this.goals = [];
  this.currentGoalNumber = -1;
  this.id = options.id;

  console.log('id = ' + this.id);
  
  this.cameraManager = new DVIZ.CameraManager({
    ros : ros,
    tfClient : tfClient,
    viewer : viewer,
    width : width,
    height : height,
    id : this.id
  });
  
  this.dvizCommandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_command',
    serviceType : 'dviz_core/Command'
  });

  var taskTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/dviz_user_' + this.id + '/dviz_task',
    messageType : 'dviz_core/Task',
    compression : 'png'
  });
  taskTopic.subscribe(function(message) {
    if(message.goals.length === 0) {
      document.getElementById('task').innerHTML = 'No task loaded.';
      return;
    }
    that.goals = message.goals;
    var currentGoal = message.current_goal;
    // @todo find a better way to decide when to update the task list...
    //       maybe there should be a flag: has the task changed?
    if(that.currentGoalNumber != currentGoal) {
      that.currentGoalNumber = currentGoal;
      var html = '';
      for(var i = 0; i < message.goals.length; ++i) {
	html = html + '<a href="#" onclick="dvizClient.changeGoal(' +
	  message.goals[i].number + ')" class="list-group-item' + 
	  (message.goals[i].number == currentGoal ? ' active' : '') + 
	  '">' + message.goals[i].description + '</a>';
      }
      document.getElementById('task').innerHTML = html;

      // Check if we need to change the state of the camera (e.g. if the 
      // next goal is a pick up goal, we need to focus the camera at the 
      // grasp pose).
      if(that.goals[that.currentGoalNumber].type === 0) { // Pick up goal
	// @todo switch camera to focus on that gripper pose
	console.log('[DVizClient] Current goal is of type pick-up.');
	that.showInteractiveGripper(that.currentGoal);
      } else {
	// @todo switch camera back to original pose
	
      }
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.play = function() {
  console.log('[DVizClient] Playing simulator...');
  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'play',
    args : [this.id.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.pause = function() {
  console.log('[DVizClient] Pausing simulator...');
  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'pause_now',
    args : [this.id.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.resetRobot = function() {
  console.log('[DVizClient] Resetting robot...');
  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'reset_robot',
    args : [this.id.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.loadScene = function() {
  // @todo for now, just load the kitchen mesh.
  console.log('[DVizClient] Loading kitchen...');

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_scene',
    args : [this.id.toString(), 
	    //'/home/eratner/ros/teach-a-robot/dviz_core/scenes/kitchen.xml']
	    '/home/eratner/ros/teach-a-robot/dviz_core/scenes/kitchen_lite.xml']
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + res.response);
    } else {
      console.log('[DVizClient] Loaded kitchen mesh.');
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.loadTask = function() {
  // @todo for now, just load the brownie task.
  console.log('[DVizClient] Loading brownie task...');

  // Load the task.
  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_task',
    args : [this.id.toString(),
	    'package://dviz_core/tasks/brownie_recipe.xml']
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + res.response);
    } else {
      console.log('[DVizClient] Loaded brownie task.');
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.handleKeyPress = function(e) {
  //console.log('[DVizClient] Handling key press event: ' + e.which + '.');

  // Filter certain keys depending on the client state.
  if(e.which === 90) {
    if(!this.zMode) {
      console.log('[DVizClient] Enabling z-mode.');
      this.zMode = true;
    } else {
      return;
    }
  }

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'process_key',
    args : [this.id.toString(),
	    e.which.toString(),
	    '6'] // Qt code for KeyPress, @todo make a constant
  }), function(response) {
    // @todo report errors
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.handleKeyRelease = function(e) {
  //console.log('[DVizClient] Handling key release event: ' + e.which + '.');

  if(e.which === 90) {
    if(this.zMode) {
        console.log('[DVizClient] Disabling z-mode.');
        this.zMode = false;
      } else {
	return;
      }
  }

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'process_key',
    args : [this.id.toString(),
	    e.which.toString(),
	    '7'] // Qt code for KeyRelease, @todo make a constant
  }), function(response) {
    // @todo report errors
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.changeGoal = function(num) {
  // @todo first alert the user, and confirm that they want to switch goals.

  console.log('[DVizClient] Changing goal to ' + num + '.');
  // @todo check if valid goal number?
  this.currentGoal = num;

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'change_goal',
    args : [this.id.toString(),
	    num.toString()]
  }), function(response) {
    // @todo report errors
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.showInteractiveGripper = function(goalNumber) {
  console.log('[DVizClient] Showing interactive gripper for goal ' + goalNumber.toString() + '.');

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'show_interactive_gripper',
    args : [this.id.toString(),
	    goalNumber.toString()]
  }), function(response) {
    // @todo report errors
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.acceptGrasp = function() {
  console.log('[DVizClient] Accepting grasp.');

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'accept_grasp',
    args : [this.id.toString()]
  }), function(response) {
    // @todo report errors
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.beginRecording = function() {
  console.log('[DVizClient] Begin recording.');

  $('#rrButton').html(
    '<button type="button" class="btn btn-default" onclick="dvizClient.endRecording()">End Recording</button>');

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'begin_recording',
    args : [this.id.toString()]
  }), function(response) {
    // @todo report errors
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.endRecording = function() {
  console.log('[DVizClient] End recording.');

  $('#rrButton').html(
    '<button type="button" class="btn btn-default" onclick="dvizClient.beginRecording()">Begin Recording</button>');

  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'end_recording',
    args : [this.id.toString()]
  }), function(response) {
    // @todo report errors
  });
}

var ros = null;
var dvizCommandClient = null;
var dvizClient = null;

function init() {
  //openIntroDialog();

  ros = new ROSLIB.Ros({
    url: 'ws://sbpl.net:21891'
  });

  var viewer = new ROS3D.Viewer({
    divID : 'dviz',
    width : 800,
    height : 600,
    antialias : true
  });

  // Request a new DVizUser from the server.
  dvizCommandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_command',
    serviceType : 'dviz_core/Command'
  });

  dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'add_user',
    args : []
  }), function(response) {
    var userId = parseInt(response.response);
    console.log('[DVizClient] Assigned DVizUser ID ' + userId + '.');

    var tfClient = new ROSLIB.TFClient({
      ros : ros,
      angularThres : 0.01,
      transThres : 0.01,
      rate : 30.0,
      fixedFrame : '/dviz_user_' + userId + '/map'
    });
    
    var meshClient = new ROS3D.MultiMarkerClient({
      ros : ros,
      topic : '/dviz_user_' + userId + '/visualization_marker',
      tfClient : tfClient,
      rootObject : viewer.scene
    });

    var imClient = new ROS3D.InteractiveMarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/dviz_user_' + userId + '/interactive_markers',
      path : 'http://resources.robotwebtools.org/',
      camera : viewer.camera,
      rootObject : viewer.selectableObjects
    });

    var rmClient = new ROS3D.RobotMarkerArrayClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/dviz_user_' + userId + '/visualization_marker_array',
      path : 'http://resources.robotwebtools.org/',
      rootObject : viewer.scene
    });

    dvizClient = new DVIZ.DemonstrationVisualizerClient({
      ros : ros,
      tfClient : tfClient,
      viewer : viewer,
      viewerWidth : 800,
      viewerHeight : 600,
      id : userId
    });

    // Add keyboard bindings.
    $(window).bind('keydown', function(e) {
      dvizClient.handleKeyPress(e);
    });

    $(window).bind('keyup', function(e) {
      dvizClient.handleKeyRelease(e);
    });
  });
}

// Kill the DVizUser when the user exits the page
window.onbeforeunload = function removeUser() {
  dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'kill_user',
    args : [dvizClient.id.toString()]
  }), function(response) {
    console.log('[DVizClient] Killed user ' + dvizClient.id.toString() + '.');
  });
}

// Front-end specific functions.
function openIntroDialog() {
  $('#tarIntroDialog').modal('show');
}

// Functions to interface with the user interface front-end.
function setCameraFollowing() {
  var follow = document.getElementById('cameraFollowing').checked;
  if(follow) {
    console.log('[DVizClient] Enabling camera following.');
    dvizClient.cameraManager.setCamera(0);
  } else {
    console.log('[DVizClient] Disabling camera following.');
    dvizClient.cameraManager.setCamera(1);
  }
}

function setCameraFilter() {
  var filter = document.getElementById('cameraTfFilter').checked;
  if(filter) {
    console.log('[DVizClient] Enabling camera TF filtering.');
    dvizClient.cameraManager.setFilterTf(filter);
  } else {
    console.log('[DVizClient] Disabling camera TF filtering.');
    dvizClient.cameraManager.setFilterTf(filter);
  }
}

function setFrameBufferSize() {
  var frameBufferSize = parseInt(document.getElementById('frameBuffer').value);
  if(isNaN(frameBufferSize)) {
    showAlert('\'' + document.getElementById('frameBuffer').value
	      + '\' is not a valid integer value!');
    console.log('[DVizClient] \'' + document.getElementById('frameBuffer').value
		+ '\' is not a valid integer value!');
  } else {
    console.log('[DVizClient] Set frame buffer size to ' + frameBufferSize + '.');
    dvizClient.cameraManager.xFilter = createSMAFilter(frameBufferSize);
    dvizClient.cameraManager.yFilter = createSMAFilter(frameBufferSize);
  }
}

function setFrameRate() {
  var frameRate = parseFloat(document.getElementById('frameRate').value);
  if(isNaN(frameRate)) {
    showAlert('\'' + document.getElementById('frameRate').value
	      + '\' is not a valid number!');
    console.log('[DVizClient] \'' + document.getElementById('frameRate').value
		+ '\' is not a valid number!');
  } else {
    console.log('[DVizClient] Set frame rate to ' + frameRate + '.');
    dvizClient.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_frame_rate',
      args : [dvizClient.id.toString(),
	      frameRate.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      } else {
	// @todo
      }
    });
  }
}

function showAlert(message) {
  var alertDialog = document.getElementById('alertDialog');
  if(alertDialog === null) {
    document.getElementById('alerts').innerHTML = '<div id="alertDialog" class="alert alert-warning"></div>';
  }

  document.getElementById('alertDialog').innerHTML = '<a class="close" data-dismiss="alert" href="#" aria-hidden="true">&times;</a><p>' 
    + message + '</p>';
}

function setBaseLinearSpeed() {
  var linearSpeed = parseFloat(document.getElementById('baseLinearSpeed').value);
  if(isNaN(linearSpeed)) {
    showAlert('Not a valid speed!');
  } else {
    dvizClient.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_base_speed',
      args : [dvizClient.id.toString(),
	      linearSpeed.toString(),
	      "0.0"]
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      } else {
	// @todo
      }
    });
  }
}

function setBaseAngularSpeed() {
  var angularSpeed = parseFloat(document.getElementById('baseAngularSpeed').value);
  if(isNaN(angularSpeed)) {
    showAlert('Not a valid speed!');
  } else {
    dvizClient.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_base_speed',
      args : [dvizClient.id.toString(),
	      "0.0",
	      angularSpeed.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      } else {
	// @todo
      }
    });
  }
}

function setEndEffectorSpeed() {
  var speed = parseFloat(document.getElementById('endEffectorSpeed').value);
  if(isNaN(speed)) {
    showAlert('Not a valid speed!');
  } else {
    dvizClient.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_arm_speed',
      args : [dvizClient.id.toString(),
	      speed.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      } else {
	// @todo
      }
    });
  }
}

function numUsers() {
  dvizClient.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'num_users',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      var numUsers = parseInt(response.response);
      showAlert('There are currently ' + numUsers.toString() + ' DViz users.');
    } else {
      console.log('[DVizClient] Error getting the number of DViz users.');
    }
  });
}
