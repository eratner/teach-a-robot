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
}