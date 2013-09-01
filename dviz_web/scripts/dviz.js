/**
 * @author Ellis Ratner - ellis.ratner@gmail.com
 */

var DVIZ = DVIZ || {};

/**
 * A camera manager for DViz.
 *
 * @constructor
 * @param options - object with the following keys:
 *  * ros - a handle to the ROS connection
 *  * viewer - a handle to the ROS3D Viewer
 *  * width - the width of the Viewer's canvas
 *  * height - the height of the Viewer's canvas
 */
DVIZ.CameraManager = function(options) {
  var that = this;
  options = options || {};
  this.ros = options.ros;
  this.viewer = options.viewer;
  this.canvasWidth = options.width;
  this.canvasHeight = options.height;

  this.cameraMode = 1;
  this.lastCameraMode = 0;

  var updateTopic = new ROSLIB.Topic({
    ros : this.ros,
    name : '/dviz_camera_update',
    messageType : 'demonstration_visualizer/CameraUpdate',
    compression : 'png'
  });
  updateTopic.subscribe(this.processCameraUpdate.bind(this));
};

/**
 * Updates the camera each time a CameraUpdate is received from DViz.
 *
 * @param message - the CameraUpdate message to use
 */
DVIZ.CameraManager.prototype.processCameraUpdate = function(message) {
  var basePose = new ROSLIB.Pose(message.base_pose);
  var endEffectorPose = new ROSLIB.Pose(message.end_effector_pose);
  var objectPose = new ROSLIB.Pose(message.object_pose);

  this.viewer.cameraControls.center.x = basePose.position.x;
  this.viewer.cameraControls.center.y = basePose.position.y;
  this.viewer.cameraControls.center.z = basePose.position.z;
  this.viewer.cameraControls.update();
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
}

var cameraManager = null;

function init() {
  var ros = new ROSLIB.Ros({
    url : 'ws://localhost:9090'
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

  cameraManager = new DVIZ.CameraManager({
    ros : ros,
    viewer : viewer,
    width : 800,
    height : 600
  });
}

function setCamera(mode) {
  if(cameraManager === null) {
    console.warn('camera manager not initialized!');
    return;
  } 

  cameraManager.setCamera(mode);
}