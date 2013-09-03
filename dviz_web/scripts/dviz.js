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
 */
DVIZ.CameraManager = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  this.tfClient = options.tfClient;
  this.viewer = options.viewer;
  this.canvasWidth = options.width;
  this.canvasHeight = options.height;

  this.cameraMode = 0;
  this.lastCameraMode = 1;

  this.tfClient.subscribe('/base_footprint', function(message) {
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
 */
DVIZ.DemonstrationVisualizerClient = function(options) {
  var ros = options.ros;
  var tfClient = options.tfClient;
  var viewer = options.viewer;
  var width = options.viewerWidth;
  var height = options.viewerHeight;
  
  this.cameraManager = new DVIZ.CameraManager({
    ros : ros,
    tfClient : tfClient,
    viewer : viewer,
    width : width,
    height : height
  });
  
  this.dvizCommandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_command',
    serviceType : 'dviz_core/Command'
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.resetRobot = function() {
  this.dvizCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'reset_robot',
    args : []
  }), function(response) {
    console.log(response.response);
  });
};

var dvizClient = null;

function init() {
  var ros = new ROSLIB.Ros({
    url: 'ws://sbpl.net:21891'
  });

  var viewer = new ROS3D.Viewer({
    divID : 'dviz',
    width : 800,
    height : 600,
    antialias : true
  });

  var tfClient = new ROSLIB.TFClient({
    ros : ros,
    angularThres : 0.01,
    transThres : 0.01,
    rate : 10.0,
    fixedFrame : 'map'
  });
  
  var meshClient = new ROS3D.MarkerClient({
    ros : ros,
    topic : '/visualization_marker',
    tfClient : tfClient,
    rootObject : viewer.scene
  });

  var imClient = new ROS3D.InteractiveMarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/dviz_interactive_markers',
    path : 'http://resources.robotwebtools.org/',
    camera : viewer.camera,
    rootObject : viewer.selectableObjects
  });

  var rmClient = new ROS3D.RobotMarkerArrayClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/visualization_marker_array',
    path : 'http://resources.robotwebtools.org/',
    rootObject : viewer.scene
  });

  dvizClient = new DVIZ.DemonstrationVisualizerClient({
    ros : ros,
    tfClient : tfClient,
    viewer : viewer,
    viewerWidth : 800,
    viewerHeight : 600
  });
}
