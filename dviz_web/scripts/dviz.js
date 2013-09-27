/**
 * @author Ellis Ratner - eratner@bowdoin.edu
 */

var DVIZ = DVIZ || {};

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

  this.cameraMode = 0;
  this.lastCameraMode = 1;

  console.log('[CameraManager] user_id = ' + this.id + ' topic: /dviz_user_' + this.id + '/base_footprint');

  this.tfClient.subscribe('/dviz_user_' + this.id + '/base_footprint', function(message) {
    var tf = new ROSLIB.Transform(message);

    // Center the camera at the base of the robot.
    if(that.cameraMode === 0) {
      that.viewer.cameraControls.center.x = tf.translation.x;
      that.viewer.cameraControls.center.y = tf.translation.y;
      that.viewer.cameraControls.center.z = tf.translation.z;
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
    console.log('changing from camera mode ' + this.cameraMode
                + ' to ' + mode + '.');
  }
  this.lastCameraMode = this.cameraMode;
  this.cameraMode = mode;
};

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
    var html = '';
    for(var i = 0; i < message.goals.length; ++i) {
      html = html + '<a href="#" class="list-group-item' + 
	(message.goals[i].number == currentGoal ? ' active' : '') + 
	'">' + message.goals[i].description + '</a>';
    }
    document.getElementById('task').innerHTML = html;
  });
};

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

  // Load the kitchen scene to start with.
  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_mesh',
    args : [this.id.toString(), 
	    'package://dviz_core/meshes/max_webgl_kitchen/max_webgl_kitchen.dae',
	    'false', 
	    'kitchen']
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
	    'package://dviz_core/tasks/brownie_mix.xml']
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + res.response);
    } else {
      console.log('[DVizClient] Loaded brownie task.');
    }
  });
}

var ros = null;
var dvizCommandClient = null;
var dvizClient = null;

function init() {
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
    
    var meshClient = new ROS3D.MarkerClient({
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
