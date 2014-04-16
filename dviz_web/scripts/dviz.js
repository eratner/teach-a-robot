/** 
 * @author Ellis Ratner - eratner@bowdoin.edu
 */
var DVIZ = DVIZ || {};

// This flag toggles output to the console that may be useful when debugging
DVIZ.debug = false;

/**
 * Creates a simple moving average (SMA) filter of size n
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

    back : function() {
      return buffer[n-1];
    },

    get : function(key) {
      return buffer[key];
    },

    getLatest : function() {
      var latest = (pointer - 1 < 0 ? n + (pointer - 1) : pointer - 1);
      return buffer[latest];
    }
  };
};

/**
 * A camera manager for DViz
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

  // Use SMA filters to make the camera smoother
  this.baseXFilter = createSMAFilter(this.filterBufferSize);
  this.baseYFilter = createSMAFilter(this.filterBufferSize);

  this.rEndEffectorXFilter = createSMAFilter(this.filterBufferSize);
  this.rEndEffectorYFilter = createSMAFilter(this.filterBufferSize);
  this.rEndEffectorZFilter = createSMAFilter(this.filterBufferSize);

  DVIZ.debug && console.log('[CameraManager] user_id = ' + this.id + ' topic: /dviz_user_' + this.id + '/base_footprint');

  this.tfClient.subscribe('/dviz_user_' + this.id + '/base_footprint', function(message) {
    var tf = new ROSLIB.Transform(message);

    that.baseXFilter.push(tf.translation.x);
    that.baseYFilter.push(tf.translation.y);

    // Center the camera at the base of the robot
    if(that.cameraMode === 0) {
      if(that.filterTf) {
	that.viewer.cameraControls.center.x = that.baseXFilter.getValue();
	that.viewer.cameraControls.center.y = that.baseYFilter.getValue();
	that.viewer.cameraControls.center.z = 0.0;
      } else {
	that.viewer.cameraControls.center.x = tf.translation.x;
	that.viewer.cameraControls.center.y = tf.translation.y;
	that.viewer.cameraControls.center.z = tf.translation.z;
      }
      that.viewer.cameraControls.update();
    }
  });

  this.tfClient.subscribe('/dviz_user_' + this.id + '/right_end_effector', function(message) {
    var tf = new ROSLIB.Transform(message);

    // console.log('r-ee x = ' + tf.translation.x.toString()
    // 		+ ' y = ' + tf.translation.y.toString()
    // 		+ ' z = ' + tf.translation.z.toString());
    that.rEndEffectorXFilter.push(tf.translation.x);
    that.rEndEffectorYFilter.push(tf.translation.y);
    that.rEndEffectorZFilter.push(tf.translation.z);

    if(that.cameraMode === 2) {
      if(that.filterTf) {
	that.viewer.cameraControls.center.x = that.rEndEffectorXFilter.getValue();
	that.viewer.cameraControls.center.y = that.rEndEffectorYFilter.getValue();
	that.viewer.cameraControls.center.z = that.rEndEffectorZFilter.getValue();
      } else {
	that.viewer.cameraControls.center.x = tf.translation.x;
	that.viewer.cameraControls.center.y = tf.translation.y;
	that.viewer.cameraControls.center.z = tf.translation.z;
      }

      that.viewer.cameraControls.update();
    }
  });

};

/**
 * Sets the camera mode
 *
 * @param mode
 *
 * Note: Currently:
 *   0 -- Follows the base of the robot
 *   1 -- Does not follow the base of the robot (used for 
 *        focusing the camera at a particular position)
 *   2 -- Follows the right end-effector of the robot
 */
DVIZ.CameraManager.prototype.setCamera = function(mode) {
  if(mode !== this.cameraMode) {
    DVIZ.debug && console.log('[DVizClient] Changing from camera mode ' + this.cameraMode
			 + ' to ' + mode);

    this.lastCameraMode = this.cameraMode;
    this.cameraMode = mode;

    // Update the camera controls
    //this.viewer.cameraControls.update();
  }
};

DVIZ.CameraManager.prototype.getCameraMode = function() {
  return this.cameraMode;
}

DVIZ.CameraManager.prototype.getLastCameraMode = function() {
  return this.lastCameraMode;
}

DVIZ.CameraManager.prototype.setFilterTf = function(filter) {
  this.filterTf = filter;
}

/**
 * Focus the center of the camera to a the point (x, y, z) in space
 */
DVIZ.CameraManager.prototype.centerCameraAt = function(x, y, z) {
  // @todo needs further testing
  this.viewer.cameraControls.center.x = x;
  this.viewer.cameraControls.center.y = y;
  this.viewer.cameraControls.center.z = z;
}

/**
 * Specify the camera offset from the center position in spherical
 * coordinates, where phi is the angle from the z-axis, theta is
 * the angle from the x-axis about the z-axis, and rho is the radial
 * distance from the center
 */
DVIZ.CameraManager.prototype.setCameraOffset = function(phi, theta, rho) {
  var offset = new THREE.Vector3();

  offset.z = rho * Math.cos(phi);
  offset.y = rho * Math.sin(phi) * Math.sin(theta);
  offset.x = rho * Math.sin(phi) * Math.cos(theta);

  // Note that x->y, y->z, z->x, so
  var transformedOffset = new THREE.Vector3(offset.z, offset.x, offset.y);

  transformedOffset.add(this.viewer.cameraControls.center);
  this.viewer.cameraControls.camera.position = transformedOffset;
}

/* @todo fix this
DVIZ.CameraManager.prototype.setCameraDistance = function(dist) {
  var position = this.viewer.cameraControls.camera.position;
  var offset = position.clone().sub(this.viewer.cameraControls.center);
  
  var theta = Math.atan2(offset.y, offset.x);
  var phi = Math.atan2(Math.sqrt(offset.y * offset.y + offset.x * offset.x), offset.z);
  
  console.log('setting camera to phi = ' + phi.toString() + 
	      ', theta = ' + theta.toString() + 
	      ', rho = ' + dist.toString());

  this.setCameraOffset(theta, phi, dist);
}
*/

/**
 * Sets the zoom speed of the camera
 */
DVIZ.CameraManager.prototype.setZoomSpeed = function(speed) {
  DVIZ.debug && console.log('[DVizClient] Setting zoom speed to ' + speed.toString());

  // @todo this needs to be tested further
  this.viewer.cameraControls.userZoomSpeed = speed;
  this.viewer.cameraControls.update();
}

/**
 * Sets the actual zoom of the camera to a particular scale
 */
DVIZ.CameraManager.prototype.setZoom = function(scale) {
  DVIZ.debug && console.log('[DVizClient] Setting zoom scale to ' + scale.toString());

  this.viewer.cameraControls.scale = scale;
  this.viewer.cameraControls.update();
}

/**
 * A web-based demonstration visualizer client
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

  this.basicGripperControls = true;
  this.playing = true;
  this.gameStarted = false;
  this.replayMode = false;
  this.replayBagfile = null;
  this.acceptedGrasp = false;
  //console.log('ACCEPTED GRASP -> FALSE');

  DVIZ.debug && console.log('[DVizClient] Assigned id ' + this.id);
  
  this.cameraManager = new DVIZ.CameraManager({
    ros : ros,
    tfClient : tfClient,
    viewer : viewer,
    width : width,
    height : height,
    id : this.id
  });

  this.cameraManager.setZoomSpeed(1.7);
  
  this.coreCommandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_command',
    serviceType : 'dviz_core/Command'
  });

  this.commandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_user_' + this.id + '/dviz_command',
    serviceType : 'dviz_core/Command'
  });

  var pingResponseTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/dviz_user_' + this.id + '/ping_response',
    messageType : 'std_msgs/Empty',
    compression : 'png'
  });
  var pingTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/dviz_user_' + this.id + '/ping',
    messageType : 'std_msgs/Empty',
    compression : 'png'
  });
  pingTopic.subscribe(function(message) {
    // Respond to the ping so that the server does not kill the client
    var response = new ROSLIB.Message({});
    pingResponseTopic.publish(response);
    DVIZ.debug && console.log('[DVizClient] Responded to a ping');
  });

  var taskTopic = new ROSLIB.Topic({
    ros : ros,
    name : '/dviz_user_' + this.id + '/dviz_task',
    messageType : 'dviz_core/Task',
    compression : 'png'
  });
  taskTopic.subscribe(function(message) {
    if(message.goals.length === 0) {
      //document.getElementById('task').innerHTML = 'No task loaded.';
      $('#task').html('No task loaded.');
      return;
    }
    that.goals = message.goals;
    var currentGoal = message.current_goal;
    // @todo find a better way to decide when to update the task list...
    //       maybe there should be a flag: has the task changed?

    if(that.currentGoalNumber != currentGoal) {
      // When changing from goal -1 (i.e. no goals) to goal 0
      // (i.e. the first goal in the task), do not display a goal
      // completed dialog
      if(currentGoal > 0) {
  	that.goalCompleted(that.currentGoalNumber);
      }

      DVIZ.debug && console.log('[DVizClient] Changing from goal ' + that.currentGoalNumber.toString() + ' to goal ' + currentGoal.toString());

      that.currentGoalNumber = currentGoal;

      if(that.currentGoalNumber >= that.goals.length) {
  	// All goals complete; notify the user
  	that.displayStatusText('All goals completed!');
  	return;
      }

      $('#currentGoal').empty().append('Current goal: <strong>' + 
  				       message.goals[that.currentGoalNumber].description +
				       '</strong>');

      var html = '';
      for(var i = 0; i < message.goals.length; ++i) {
  	var color = '000000';
  	if(i < that.currentGoalNumber) {
  	  color = '999999';
  	} else if(i === that.currentGoalNumber) {
  	  color = '0088CC';
  	}
  	html = html + '<a href="#"' + 
  	  (DVIZ.debug ? 
  	   (' onclick="dvizClient.changeGoal(' + message.goals[i].number + ')" ') :
  	   ' ') + 'class="list-group-item">' + 
  	  '<font color="' + color + '">' + 
  	  message.goals[i].description + '</font></a>';
      }
      $('#task').empty().append(html);

      // Check if we need to change the state of the camera (e.g. if the 
      // next goal is a pick up goal, we need to focus the camera at the 
      // grasp pose)
      that.displayStatusText('Next goal: ' + that.goals[that.currentGoalNumber].description);
      if(that.goals[that.currentGoalNumber].type === 0) { // Pick up goal
  	// Show an interactive gripper marker at the current goal
  	DVIZ.debug && console.log('[DVizClient] Current goal is of type pick-up');
  	that.acceptedGrasp = false;
  	$('#acceptChangeGrasp').empty().append('<img src="images/accept_grasp.png" width="65" height="65" />');
  	$('#gripperJointAngle').slider('option', 'value', 
  				       that.goals[that.currentGoalNumber].gripper_joint_position);
  	that.showInteractiveGripper(that.currentGoalNumber);
  	$('#playPause').prop('disabled', true);
  	$('#acceptChangeGrasp').prop('disabled', false);
  	$('#acceptChangeGrasp').tooltip('show');
  	$('#freeFollowingCamera').prop('disabled', true);
  	$('#baseHandCamera').prop('disabled', true);
  	$('#rotateHandControls').prop('disabled', true);
	that.changeCamera('none');
      } else {
  	// The goal is not of type pick up
  	$('#acceptChangeGrasp').prop('disabled', true);
      }
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.play = function() {
  DVIZ.debug && console.log('[DVizClient] Playing simulator...');
  this.playing = true;

  $('#playPause').html('<img src="images/pause.png" width="65" height="65" />');
  $('#playPause').tooltip('hide')
    .attr('data-original-title', 'Click here to pause the game.')
    .tooltip('fixTitle')
    .tooltip('show');
  

  if(!this.gameStarted) {
    if(this.replayMode) {
      // If in replay mode, load the bagfile and begin replaying
      dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
	command : 'begin_replay',
	// @todo we should have some way of specifying the path
	args : ['/home/eratner/demonstrations/' + this.replayBagfile]
      }), function(response) {
	if(response.response.length > 0) {
	  DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
	} else {
	  DVIZ.debug && console.log('[DVizClient] Loaded bagfile');
	  dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
	    command : 'num_frames_in_recording',
	    args : []
	  }), function(response) {
	    var frames = parseInt(response.response);
	    if(isNaN(frames)) {
	      DVIZ.debug && console.log('[DVizClient] Error: recieved NaN as frame count');
	    } else {
	      $('#fastForwardReplay').slider({
		value : 0,
		min : 0,
		max : frames-1,
		step : 1,
		slide : function(event, ui) {
		  var index = parseInt(ui.value);
		  DVIZ.debug && console.log('[DVizClient] Fast-forwarding to frame '
					    + index.toString() + ' of ' 
					    + frames.toString() + ' frames');
		  var duration = ((frames/10)/60).toFixed(2);
		  var framesHtml = index.toString() + '/' + frames.toString()
		    + ' frames (approximately ' + duration.toString() + ' minutes)';
		  $('#fastForwardFrames').html(framesHtml);
		  if(!isNaN(index)) {
		    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
		      command : 'fast_forward_replay',
		      args : [index.toString()]
		    }), function(response) {
		      
		    });
		  }
		}
	      });
	      
	      var duration = ((frames/10)/60).toFixed(2);
	      var framesHtml = '0/' + frames.toString()
		+ ' frames (approximately ' + duration.toString() + ' minutes)';
	      $('#fastForwardFrames').html(framesHtml);
	      // @todo We need the slider to move along with the replay
	      // setInterval(function() {
	      // 	var currentFrame = parseInt(
	      // 	  $('#fastForwardReplay').slider('getValue'));
	      // 	$('#fastForwardReplay').slider('setValue', currentFrame+3);
	      // }, 100);
	      // Note: 30 fps => trigger every 33.33333... ms => 3 frames every 100 ms
	    }
	  });
	}
      });
      this.gameStarted = true;
    } else {
      // If the user has not started the game yet, load the task
      this.loadTask();
      this.robotMarkerControl(true);
      // Start recording
      if(workerId !== null && assignmentId !== null) {
	var bagfileName = 'user_' + workerId + '_demonstration_' 
	  + assignmentId + '_' + this.id.toString() + '.bag';
	this.beginRecording(bagfileName, assignmentId);
      } else {
	this.beginRecording();
      }
      this.gameStarted = true;
      $('#done').prop('disabled', false);
    }
  }

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'play',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.pause = function(now) {
  if(!this.playing) {
    return;
  }

  DVIZ.debug && console.log('[DVizClient] Pausing simulator...');
  this.playing = false;

  $('#playPause').html('<img src="images/play.png" width="65" height="65" />');
  $('#playPause').tooltip('hide')
    .attr('data-original-title', 'Click here to continue the game.')
    .tooltip('fixTitle')
    .tooltip('show');

  var pauseNow = now || true;

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : (pauseNow ? 'pause_now' : 'pause_later'),
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.isPlaying = function() {
  return this.playing;
}

DVIZ.DemonstrationVisualizerClient.prototype.changeCamera = function(camera) {
  if(camera === 'base') {
    DVIZ.debug && console.log('[DVizClient] Changing camera to follow the base');
    this.cameraManager.setCamera(0);
    this.cameraManager.viewer.cameraControls.center.x =
      this.cameraManager.baseXFilter.back();
    this.cameraManager.viewer.cameraControls.center.y =
      this.cameraManager.baseYFilter.back();
    this.cameraManager.viewer.cameraControls.center.z = 0.0;
    // @todo set a default zoom
  } else if(camera === 'gripper') {
    DVIZ.debug && console.log('[DVizClient] Changing camera to follow the gripper');
    this.cameraManager.setCamera(2);
    // console.log('setting camera to r-ee x = ' + 
    // 		this.cameraManager.rEndEffectorXFilter.getLatest().toString()
    // 		+ ' y = ' + this.cameraManager.rEndEffectorYFilter.getLatest().toString() + ' z = ' + this.cameraManager.rEndEffectorZFilter.getLatest().toString());
    this.cameraManager.viewer.cameraControls.center.x =
      this.cameraManager.rEndEffectorXFilter.getLatest();
    this.cameraManager.viewer.cameraControls.center.y =
      this.cameraManager.rEndEffectorYFilter.getLatest();
    this.cameraManager.viewer.cameraControls.center.z =
      this.cameraManager.rEndEffectorZFilter.getLatest();
    // @todo set a default zoom
  } else if(camera === 'none') {
    DVIZ.debug && console.log('[DVizClient] Changing camera to free mode');
    this.cameraManager.setCamera(1);
  } else if(camera === 'last') {
    DVIZ.debug && console.log('[DVizClient] Changing camera to last mode');
    this.cameraManager.setCamera(this.cameraManager.getLastCameraMode());
  } else {
    DVIZ.debug && console.log('[DVizClient] Unknown camera mode "' + camera + '"');
  }
}

DVIZ.DemonstrationVisualizerClient.prototype.toggleGripperControls = function() {
  DVIZ.debug && console.log('[DVizClient] Toggling gripper controls to ' +
	      (!this.basicGripperControls).toString() + '.');
  this.basicGripperControls = !this.basicGripperControls;
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'gripper_controls',
    args : [this.basicGripperControls.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.resetRobot = function() {
  DVIZ.debug && console.log('[DVizClient] Resetting robot...');
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'reset_robot',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.resetTask = function() {
  DVIZ.debug && console.log('[DVizClient] Resetting task...');
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'reset_task',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.loadScene = function(scene) {
  //showAlertModal('Loading the kitchen...', 'This might take a minute or two!');
  var sceneName = scene || 'kitchen_lite.xml';

  DVIZ.debug && console.log('[DVizClient] Loading scene ' + sceneName);

  // Pause the game while the scene loads
  //this.pause();

  // @todo bring up a loading message until the scene has fully loaded into the browser.
  // then, need to figure out how to wait until the collada meshes have been fully rendered.

  this.coreCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_scene',
    args : [this.id.toString(), 
	    '/home/eratner/ros/teach-a-robot/dviz_core/scenes/' + sceneName]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + res.response);
    } else {
      DVIZ.debug && console.log('[DVizClient] Loaded the scene ' + sceneName);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.loadTask = function(task) {
  var taskName = task || 'brownie_recipe.xml';

  DVIZ.debug && console.log('[DVizClient] Loading task ' + taskName);

  // Load the task
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_task',
    args : ['package://dviz_core/tasks/' + taskName]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + res.response);
    } else {
      DVIZ.debug && console.log('[DVizClient] Loaded the task ' + taskName);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.handleKeyPress = function(e) {
  //console.log('[DVizClient] Handling key press event: ' + e.which + '.');

  // Filter certain keys depending on the client state.
  if(e.which === 90) {
    if(!this.zMode) {
      DVIZ.debug && console.log('[DVizClient] Enabling z-mode');
      this.zMode = true;
    } else {
      return;
    }
  }

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'process_key',
    args : [e.which.toString(),
	    '6'] // Qt code for KeyPress, @todo make a constant
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.handleKeyRelease = function(e) {
  if(e.which === 90) {
    if(this.zMode) {
        DVIZ.debug && console.log('[DVizClient] Disabling z-mode');
        this.zMode = false;
      } else {
	return;
      }
  }

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'process_key',
    args : [e.which.toString(),
	    '7'] // Qt code for KeyRelease, @todo make a constant
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.changeGoal = function(num) {
  if(num === this.currentGoalNumber) {
    DVIZ.debug && console.log('[DVizClient] Attemping to change to goal ' + num + ': already on this goal');
    return;
  }
  // @todo first alert the user, and confirm that they want to switch goals
  this.hideInteractiveGripper(this.currentGoalNumber);

  DVIZ.debug && console.log('[DVizClient] Changing goal from ' + this.currentGoalNumber + ' to ' + num);
  // @todo check if valid goal number?
  //this.currentGoalNumber = num;

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'change_goal',
    args : [num.toString()]
  }), function(response) {
    DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.showInteractiveGripper = function(goalNumber) {
  DVIZ.debug && console.log('[DVizClient] Showing interactive gripper for goal ' + goalNumber.toString());

  // Disable marker control of the robot (so the user cannot move 
  // the robot while selecting a grasp)
  this.robotMarkerControl(false);
  this.pause();

  this.changeCamera('none');
  $('#baseHandCamera').prop('disabled', true);

  var objectX = this.goals[goalNumber].initial_object_pose.position.x;
  var objectY = this.goals[goalNumber].initial_object_pose.position.y;
  var objectZ = this.goals[goalNumber].initial_object_pose.position.z;

  var cameraPhi = this.goals[goalNumber].camera_phi;
  var cameraTheta = this.goals[goalNumber].camera_theta;
  var cameraRadius = this.goals[goalNumber].camera_radius;

  this.cameraManager.centerCameraAt(objectX, objectY, objectZ);
  this.cameraManager.setCameraOffset(cameraPhi, cameraTheta, cameraRadius);

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'show_interactive_gripper',
    args : [goalNumber.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.hideInteractiveGripper = function(goalNumber) {
  DVIZ.debug && console.log('[DVizClient] Hiding interactive gripper for goal ' + goalNumber.toString());

  this.robotMarkerControl(true);
  
  // Return to the last camera mode
  var b = $('#baseHandCamera');
  var f = $('#freeFollowingCamera');
  if(this.cameraManager.getLastCameraMode() === 0) {
    // Base-following camera
    b.html('Hand View');
    b.tooltip('hide')
      .attr('data-original-title', 'Switch to a view that follows the robot\'s hand.')
      .tooltip('fixTitle')
      .tooltip('show');
    $('#viewImage').attr('src', 'images/base_view.png');
    f.html('Free View');
    f.tooltip('hide')
      .attr('data-original-title', 'You can move your view freely.')
      .tooltip('fixTitle')
      .tooltip('show');
    b.prop('disabled', false);

    this.changeCamera('base');
  } else if(this.cameraManager.getLastCameraMode() === 2) {
    // Gripper-following camera
    b.html('Base View');
    b.tooltip('hide')
      .attr('data-original-title', 'Switch to a view that follows the robot\'s base.')
      .tooltip('fixTitle')
      .tooltip('show');
    $('#viewImage').attr('src', 'images/hand_view.png');
    b.prop('disabled', false);
    f.html('Free View');
    f.tooltip('hide')
      .attr('data-original-title', 'You can move your view freely.')
      .tooltip('fixTitle')
      .tooltip('show');

    this.changeCamera('gripper');
  } else if(this.cameraManager.getLastCameraMode() === 1) {
    // @todo set the buttons appropriately
    f.html('Following View');
    f.tooltip('hide')
     .attr('data-original-title', 'Your view will follow the position of the robot automatically.')
     .tooltip('fixTitle')
     .tooltip('show');
    b.prop('disabled', true);

    // Free-camera (focus the camera intially at the base of the robot)
    this.cameraManager.viewer.cameraControls.center.x =
      this.cameraManager.baseXFilter.back();
    this.cameraManager.viewer.cameraControls.center.y =
      this.cameraManager.baseYFilter.back();
    this.cameraManager.viewer.cameraControls.center.z = 0.0;

    this.changeCamera('none');
  } else {
    DVIZ.debug && console.log('[DVizClient] Error switching the camera back');
  }

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'hide_interactive_gripper',
    args : [goalNumber.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.acceptGrasp = function() {
  if(this.acceptedGrasp === true) {
    DVIZ.debug && console.log('[DVizClient] Changing/choosing grasp');
    this.pause();
    $('#playPause').attr('disabled', true);
    $('#freeFollowingCamera').attr('disabled', true);
    $('#baseHandCamera').attr('disabled', true);
    $('#rotateHandControls').attr('disabled', true);
    this.acceptedGrasp = false;

    $('#acceptChangeGrasp').html('<img src="images/accept_grasp.png" width="65" height="65" />');
    $('#acceptChangeGrasp').tooltip('hide')
      .attr('data-original-title', 'Click here when you\'re done choosing a good grasp for the object.')
      .tooltip('fixTitle')
      .tooltip('show');

    this.showInteractiveGripper(this.currentGoalNumber);
  } else {
    DVIZ.debug && console.log('[DVizClient] Accepting grasp');
    this.play();
    $('#playPause').attr('disabled', false);
    $('#freeFollowingCamera').attr('disabled', false);
    if(this.cameraManager.cameraMode !== 0) {
      $('#baseHandCamera').attr('disabled', false);
    }
    $('#rotateHandControls').attr('disabled', false);
    this.acceptedGrasp = true;

    // Hide the interactive gripper of the current goal
    this.hideInteractiveGripper(this.currentGoalNumber);

    this.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'accept_grasp',
      args : []
    }), function(response) {
      if(response.response.length > 0) {
	DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
      }
    });
    
    $('#acceptChangeGrasp').html('<img src="images/change_grasp.png" width="65" height="65" />');
    $('#acceptChangeGrasp').tooltip('hide')
      .attr('data-original-title', 'Click here if you want to change the grasp that you\'ve already chosen.')
      .tooltip('fixTitle')
      .tooltip('show');
  }
}

DVIZ.DemonstrationVisualizerClient.prototype.beginRecording = function(name, demo) {
  var bagfileName = name || '';
  var userName = workerId || this.id.toString();
  var demoName = demo || '';

  DVIZ.debug && console.log('[DVizClient] Begin recording to '
			    + (bagfileName.length > 0 ? bagfileName : 'default')
			    + ' with user name ' + userName + 
			   (demoName.length > 0 ? (' and demo name ' + demoName) : ''));

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'begin_recording',
    args : [userName, demoName, bagfileName]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.endRecording = function() {
  DVIZ.debug && console.log('[DVizClient] End recording');

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'end_recording',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.beginReplay = function() {
  DVIZ.debug && console.log('[DVizClient] Begin replay');

  // Reset the robot before replaying a user demonstration
  this.resetRobot();

  // @todo query a database of user demonstrations to replay the 
  // desired one

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'begin_replay',
    args : ['/home/eratner/demonstrations/user' + this.id.toString() + '_demonstration0.bag']
    // @todo the user should somehow input which demonstration to replay
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.goalCompleted = function(goalNumber) {
  DVIZ.debug && console.log('[DVizClient] Goal ' + goalNumber.toString() + ' completed!');

  goalsCompleted += 1;

  // Notify the user that a goal has been completed, and give a 
  // description of the next goal in the task
  var message = 'Goal ' + goalNumber.toString() + ' completed! ';
  if(goalNumber + 1 >= this.goals.length) {
    message += 'Congratulations, the task is complete! Close this window and your demonstration '
    + ' will automatically be saved. Don\'t forget to submit the Mechanical Turk HIT results.';
    this.endDemonstration(true);
  } else {
    message += ('Next goal: ' + 
		this.goals[goalNumber + 1].description);
  }

  message = message + ' <br />You have now completed <strong>' + goalsCompleted.toString() 
    + ' out of 6 goals</strong>. You are now eligible for <strong>$'
    + ((goalsCompleted * 0.50).toFixed(2)).toString() 
    + '</strong>! <br />If you\'re done, remember to submit your <strong>Teach-A-Robot'
    + ' Demonstration ID (' + assignmentId + ')</strong> on the Mechanical Turk HIT page'
    + ' so that we can pay you.';

  // Save all demonstration progress after each goal is completed
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'save_recording',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });

  $('#goalCompleteMessage').html(message);
  $('#goalCompleteModal').modal('show');
}

DVIZ.DemonstrationVisualizerClient.prototype.endDemonstration = function(complete) {
  DVIZ.debug && console.log('[DVizClient] Ending demonstration');

  var taskComplete = complete || false;

  if(this.gameStarted) {
    this.gameStarted = false;
    this.endRecording();
  }
}

DVIZ.DemonstrationVisualizerClient.prototype.robotMarkerControl = function(enabled) {
  DVIZ.debug && console.log('[DVizClient] Robot marker control is ' +
	      (enabled ? 'enabled' : 'disabled'));

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'robot_marker_control',
    args : [enabled.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.setGripperJointAngle = function(angle) {
  DVIZ.debug && console.log('[DVizClient] Setting gripper joint angle to ' + angle.toString());

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'set_gripper_joint',
    args : [angle.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.displayStatusText = function(text) {
  DVIZ.debug && console.log('[DVizClient] Adding status: ' + text);
  if(this.statusText == null) {
    this.statusText = document.createElement('div');
    this.statusText.style.position = 'absolute';
    this.statusText.style.width = 100;
    this.statusText.height = 100;
    this.statusText.style.backgroundColor = 'black';
    this.statusText.style.color = 'white';
    this.statusText.style.top = $('#dviz').offset().top + 'px';
    this.statusText.style.left = $('#dviz').offset().left + 'px';
    this.statusText.innerHTML = '...';
    document.body.appendChild(this.statusText);
  }
  this.statusText.innerHTML = text;
}

var ros = null;
var dvizCoreCommandClient = null;
var dvizClient = null;
var watching = false;
var replayMode = false;
var replayBagfile = null;

// Mechanical Turk information
var hitId = null;
var workerId = null;
var assignmentId = null;
var previewMode = false;
var development = false;

var goalsCompleted = 0;

function init() {
  $('#debugControls').hide();

  var variables = getUrlVariables();


  // Get any variables that are associated with a Mechanical Turk HIT
  for(var i = 0; i < variables.length; i++) {
    if(variables[i][0] === 'hitId') {
      hitId = variables[i][1];
    } else if(variables[i][0] === 'assignmentId') {
      assignmentId = variables[i][1];
      previewMode = (assignmentId === 'ASSIGNMENT_ID_NOT_AVAILABLE');
    } else if(variables[i][0] === 'workerId') {
      workerId = variables[i][1];
    } else if(variables[i][0] === 'replayBagfile') {
      replayBagfile = variables[i][1];
      replayMode = true;
    } else if(variables[i][0] == 'dev') {
      development = true;
    }
  }

  //SERVER DOWN MESSAGE
  // if(!development) {
  //   $('#serverDown').modal('show');
  //   return;
  // }


  if(hitId !== null && assignmentId !== null) {
    DVIZ.debug && console.log('[DVizClient] Processing HIT with hitId ' + hitId.toString()
			 + ' and assignmentId ' + assignmentId.toString());
  }

  ros = new ROSLIB.Ros({
    url : 'ws://sbpl.net:21891'
    //url : 'ws://localhost:9090'
  });

  // Width and height of the viewer, in pixels
  var W = 800;
  var H = Math.min(600, 
		   Math.max(400, window.innerHeight - 200));
  DVIZ.debug && console.log('[DVizClient] Instantiating the viewer with width '
		       + W.toString() + ' and height ' + H.toString());

  // The frame rate limit on the viewer (frames/second)
  var fps = 15.0;
  DVIZ.debug && console.log('[DVizClient] Capping frame rate at ' + fps.toString() + ' frames/second');

  var viewer = new ROS3D.Viewer({
    divID : 'dviz',
    width : W,
    height : H,
    antialias : true,
    fps : fps
  });

  // Request a new DVizUser from the core
  dvizCoreCommandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_command',
    serviceType : 'dviz_core/Command'
  });

  var userId = 0;
  for(var i = 0; i < variables.length; i++) {
    if(variables[i][0] === 'watchUserId') {
      userId = variables[i][1];
      break;
    }
  }

  // Potential Mechanical Turk users can preview the task, and we do not 
  // want to load the entire game when this happens
  if(previewMode) {
    DVIZ.debug && console.log('[DVizClient] Preview mode');
    $('#settings').prop('disabled', true);
    return;
  }
  
  // Either watch an existing user (referenced by ID), or request
  // a new user from the DVizCore server
  if(userId > 0) {
    initializeDemonstration(userId, W, H, ros, viewer);
    watching = true;
    dvizClient.displayStatusText('Watching user ' + userId.toString());
  } else {
    // First check if there are too many users online
    var usersOnline = 0;
    dvizCoreCommandClient.callService(new ROSLIB.ServiceRequest({
      command : 'num_users',
      args : []
    }), function(response) {
      if(response.response.length > 0) {
	usersOnline = parseInt(response.response);

	DVIZ.debug && console.log('[DVizClient] There are ' + usersOnline.toString()
				  + ' users online');

	if(usersOnline > 5) {
	  DVIZ.debug && console.log('[DVizClient] Too many users online (' +
				    usersOnline.toString() + ')');
	  // @todo show modal dialog
	  $('#tooManyUsers').modal('show');
	} else {
	  dvizCoreCommandClient.callService(new ROSLIB.ServiceRequest({
	    command : 'add_user',
	    args : []
	  }), function(response) {
	    var userId = parseInt(response.response);
	    console.log('[DVizClient] Starting DVizUser ID ' + userId);
	    initializeDemonstration(userId, W, H, ros, viewer);

	    dvizClient.loadScene();
	    $('#playPause').prop('disabled', false);
	    $('#playPause').tooltip('show');

	    $('#rotateHandControls').prop('disabled', false);
	    $('#freeFollowingCamera').prop('disabled', false);
	    $('#baseHandCamera').prop('disabled', false);
	    $('#endDemonstration').prop('disabled', false);

	    dvizClient.displayStatusText('Connected to the server!');

	    // Show the instructions and prompt the user to input their id
	    if(!replayMode) {
	      // TODO: Prompt the user for their worker ID and give them a randomly 
	      // generated completion string
	      assignmentId = generateRandomString(8);
	      DVIZ.debug && console.log('[DVizClient] Random assignment ID: ' + assignmentId);
	      $('#assignmentId').html(assignmentId);		
	      $('#mturkInfo').modal('show');
	      $('#confirmInfo').on('click', function() {
		// TODO: Grab worker ID
		workerId = $('#workerId').val();
		if(workerId.length > 0) {
		  DVIZ.debug && console.log('[DVizClient] Worker ID: ' + workerId);
		  $('#mturkInfo').modal('hide');
		  $('#infoModal').modal('show');
		} else {
		  $('#mturkInfoError').html('Please enter your worker ID!');
		}
	      });
	    }

	    dvizClient.coreCommandClient.callService(new ROSLIB.ServiceRequest({
      	      command : 'pass_user_command_threaded',
      	      args : [dvizClient.id.toString(), 
      		      'robot_marker_control',
	              'false']
	    }), function(response) {
      	      if(response.response.length > 0) {
      		DVIZ.debug && console.log('[DVizClient] Error response: ' + res.response);
      	      } 
	    });
	  });
	}
      }
    });
  }

  // Initialize all tooltips
  $('.tip').tooltip();

  // Initialize gameplay settings
  initializeGameplaySettings();
  hideGameplaySettings();

  // Defer the loading of the gifs on the help modal for
  // when the modal is shown, otherwise the entire page 
  // loads too slowly
  $('#helpModal').on('show.bs.modal', function(e) {
    var panelGroup = $(this).find('.panel-group').hide();
    var deferreds = [];
    var imgs = $('.panel-group', this).find('img');
    // loop over each img
    imgs.each(function() {
      var self = $(this);
      var datasrc = self.attr('data-src');
      console.log('deferring ' + datasrc);
      if(datasrc) {
        var d = $.Deferred();
        self.one('load', d.resolve)
          .attr("src", datasrc)
          .attr('data-src', '');
        deferreds.push(d.promise());
      }
    });

    $.when.apply($, deferreds).done(function() {
      $('#helpLoader').hide();
      panelGroup.fadeIn(1000);
    });
  });

  $('#help').on('click', function() {
    $('#helpModal').modal('show');
  });
}

// Kill the DVizUser when the user exits the page
window.onbeforeunload = removeUser;
function removeUser() {
  if(!watching) {
    // Importantly, ending the demonstration explicitly flushes the progress made in the 
    // demonstration to the appropriate bagfile
    dvizClient.endDemonstration();

    dvizCoreCommandClient.callService(new ROSLIB.ServiceRequest({
      command : 'kill_user',
      args : [dvizClient.id.toString()]
    }), function(response) {
      DVIZ.debug && console.log('[DVizClient] Killed user ' + dvizClient.id.toString());
    });
  }
}

function initializeDemonstration(id, width, height, ros, viewer) {
  var tfClient = new ROSLIB.TFClient({
    ros : ros,
    angularThres : 0.01,
    transThres : 0.01,
    rate : 30.0,
    fixedFrame : '/dviz_user_' + id + '/map'
  });
  
  var meshClient = new ROS3D.MarkerClient({
    ros : ros,
    topic : '/dviz_user_' + id + '/visualization_marker',
    tfClient : tfClient,
    rootObject : viewer.scene
  });

  // Client to handle interactive markers
  var imClient = new ROS3D.InteractiveMarkerClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/dviz_user_' + id + '/interactive_markers',
    path : 'http://resources.robotwebtools.org/',
    camera : viewer.camera,
    rootObject : viewer.selectableObjects
  });

  var rmClient = new ROS3D.MarkerArrayClient({
    ros : ros,
    tfClient : tfClient,
    topic : '/dviz_user_' + id + '/visualization_marker_array',
    path : 'http://resources.robotwebtools.org/',
    rootObject : viewer.scene
  });

  // Client to display spheres to indicate robot collisions
  var rcClient = new ROS3D.MarkerClient({
    ros : ros,
    topic : '/dviz_user_' + id + '/collisions/visualization_marker',
    tfClient : tfClient,
    rootObject : viewer.scene
  });

  dvizClient = new DVIZ.DemonstrationVisualizerClient({
    ros : ros,
    tfClient : tfClient,
    viewer : viewer,
    viewerWidth : width,
    viewerHeight : height,
    id : id
  });

  // If in replay mode, set up 
  if(replayMode) {
    DVIZ.debug && console.log('[DVizClient] Replaying from bagfile "' + replayBagfile + '"');
    dvizClient.replayMode = true;
    dvizClient.replayBagfile = replayBagfile;
  }

  // Initialize button click callbacks
  $('#playPause').on('click', function() {
    if(dvizClient.isPlaying() && dvizClient.gameStarted) {
      dvizClient.pause();
    } else {
      dvizClient.play();
    }
  });

  $('#acceptChangeGrasp').on('click', function() {
    dvizClient.acceptGrasp();
  });

  $('#baseHandCamera').on('click', function() {
    var b = $('#baseHandCamera');

    if(dvizClient.cameraManager.getCameraMode() === 0) {
      // Base-following camera
      b.html('Base View');
      b.tooltip('hide')
	.attr('data-original-title', 'Switch to a view that follows the robot\'s base.')
	.tooltip('fixTitle')
	.tooltip('show');
      $('#viewImage').attr('src', 'images/hand_view.png');

      dvizClient.changeCamera('gripper');
    } else if(dvizClient.cameraManager.getCameraMode() === 2) {
      // Gripper-following camera
      b.html('Hand View');
      b.tooltip('hide')
	.attr('data-original-title', 'Switch to a view that follows the robot\'s hand.')
	.tooltip('fixTitle')
	.tooltip('show');
      $('#viewImage').attr('src', 'images/base_view.png');

      dvizClient.changeCamera('base');
    } else {
      console.log('[DVizClient] Error in base/hand view handler');
    }
  });

  $('#freeFollowingCamera').on('click', function() {
    var b = $('#freeFollowingCamera');

    if(dvizClient.cameraManager.getCameraMode() !== 1) {
      // Camera is not in free mode
      b.html('Following View');
      b.tooltip('hide')
	.attr('data-original-title', 'Your view will follow the position of the robot automatically.')
	.tooltip('fixTitle')
	.tooltip('show');
      $('#viewImage').attr('src', 'images/free_view.png');
      
      dvizClient.changeCamera('none');
      $('#baseHandCamera').prop('disabled', true);
    } else {
      // Camera is in free mode
      b.html('Free View');
      b.tooltip('hide')
	.attr('data-original-title', 'You can move your view freely.')
	.tooltip('fixTitle')
	.tooltip('show');

      if(dvizClient.cameraManager.lastCameraMode === 0) {
	$('#viewImage').attr('src', 'images/base_view.png');
	dvizClient.changeCamera('base');
      } else if(dvizClient.cameraManager.lastCameraMode === 2) {
	$('#viewImage').attr('src', 'images/hand_view.png');
	dvizClient.changeCamera('gripper');
      } else if(dvizClient.cameraManager.lastCameraMode === 1) {
	dvizClient.changeCamera('none');
	$('#baseHandCamera').prop('disabled', true);
      } else {
	DVIZ.debug && console.log('[DVizClient] Error in free/following camera handler (lastCameraMode = ' + dvizClient.cameraManager.lastCameraMode.toString() + ')');
      }
      $('#baseHandCamera').prop('disabled', false);
    }
  });

  $('#rotateHandControls').on('click', function() {
    var b = $('#rotateHandControls');

    if(dvizClient.basicGripperControls) {
      b.html('<img src="images/simple_control.png" width="65" height="65" />');
      b.tooltip('hide')
	.attr('data-original-title', 'Show controls for only moving the position of the arm.')
	.tooltip('fixTitle')
	.tooltip('show');
    } else {
      b.html('<img src="images/full_control.png" width="65" height="65" />');
      b.tooltip('hide')
	.attr('data-original-title', 'Full control gives you more freedom to move the elbow and rotate the hand of the robot.')
	.tooltip('fixTitle')
	.tooltip('show');
    }

    dvizClient.toggleGripperControls();
  })

  $('#endDemonstration').on('click', function() {
    dvizClient.endDemonstration();
  });

  // Add keyboard bindings.
  $(window).bind('keydown', function(e) {
    dvizClient.handleKeyPress(e);
  });

  $(window).bind('keyup', function(e) {
    dvizClient.handleKeyRelease(e);
  });
}

function setCameraFilter() {
  var filter = document.getElementById('cameraTfFilter').checked;
  if(filter) {
    DVIZ.debug && console.log('[DVizClient] Enabling camera TF filtering');
    dvizClient.cameraManager.setFilterTf(filter);
  } else {
    DVIZ.debug && console.log('[DVizClient] Disabling camera TF filtering');
    dvizClient.cameraManager.setFilterTf(filter);
  }
}

function setFrameBufferSize() {
  var frameBufferSize = parseInt(document.getElementById('frameBuffer').value);
  if(isNaN(frameBufferSize)) {
    showAlert('\'' + document.getElementById('frameBuffer').value
	      + '\' is not a valid integer value!');
    DVIZ.debug && console.log('[DVizClient] \'' + document.getElementById('frameBuffer').value
			 + '\' is not a valid integer value!');
  } else {
    DVIZ.debug && console.log('[DVizClient] Set frame buffer size to ' + frameBufferSize);
    dvizClient.cameraManager.baseXFilter = createSMAFilter(frameBufferSize);
    dvizClient.cameraManager.baseYFilter = createSMAFilter(frameBufferSize);
  }
}

function setFrameRate(rate) {
  var frameRate = rate ||
    parseFloat($('#frameRate').value);
  if(isNaN(frameRate)) {
    showAlert('\'' + document.getElementById('frameRate').value
	      + '\' is not a valid number!');
    DVIZ.debug && console.log('[DVizClient] \'' + document.getElementById('frameRate').value
			 + '\' is not a valid number!');
  } else {
    console.log('[DVizClient] Set frame rate to ' + frameRate);
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_frame_rate',
      args : [frameRate.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
      }
    });
  }
}

function showAlert(message, timeout) {
  // Default argument for timeout is 2 s (2000 ms)
  timeout = (typeof timeout === 'undefined') ? 2000 : timeout;

  var alertDialog = $('#alertDialog');
  if(alertDialog === null) {
    //$('#alerts').innerHTML = '<div id="alertDialog" class="alert alert-warning"></div>';
    $('#alerts').html('<div id="alertDialog" class="alert alert-warning"></div>');
  }

  //$('#alertDialog').innerHTML = '<a class="close" data-dismiss="alert" href="#" aria-hidden="true">&times;</a><p>' 
  //  + message + '</p>';
  $('#alertDialog').empty().append('<a class="close" data-dismiss="alert" href="#" aria-hidden="true">&times;</a><p>'
				   + message + '</p>');

  // Set automatic timeout
  window.setTimeout(function() {
    $('#alertDialog').alert('close'); 
  }, timeout);
}

function setBaseLinearSpeed(speed) {
  var linearSpeed = speed || parseFloat($('#baseLinearSpeed').value);
  if(isNaN(linearSpeed)) {
    showAlert('Not a valid speed!');
  } else {
    DVIZ.debug && console.log('[DVizClient] Setting base linear speed to ' + linearSpeed.toString());
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_base_speed',
      args : [linearSpeed.toString(),
	      '0.0']
    }), function(response) {
      if(response.response.length > 0) {
	DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
      } 
    });
  }
}

function setBaseAngularSpeed(speed) {
  var angularSpeed = speed || parseFloat($('#baseAngularSpeed').value);
  if(isNaN(angularSpeed)) {
    showAlert('Not a valid speed!');
  } else {
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_base_speed',
      args : ['0.0',
	      angularSpeed.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
      } else {
	// @todo
      }
    });
  }
}

function setEndEffectorSpeed(s) {
  var speed = s || parseFloat($('#endEffectorSpeed').value);
  if(isNaN(speed)) {
    showAlert('Not a valid speed!');
  } else {
    DVIZ.debug && console.log('[DVizClient] Setting end-effector speed to ' + speed.toString());
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_arm_speed',
      args : [speed.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	DVIZ.debug && console.log('[DVizClient] Error response: ' + response.response);
      }
    });
  }
}

function setZoomSpeed(speed) {
  var zoomSpeed = speed || parseFloat($('#zoomSpeed').value);
  if(isNaN(speed)) {
    showAlert('Not a valid speed');
  } else if(dvizClient === null) {
    DVIZ.debug && console.log('[DVizClient] Not connected to the server');
  } else {
    dvizClient.cameraManager.setZoomSpeed(zoomSpeed);
  }
}

function numUsers() {
  dvizCoreCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'num_users',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      var numUsers = parseInt(response.response);
      showAlert('There are currently ' + numUsers.toString() + ' DViz users');
    } else {
      DVIZ.debug && console.log('[DVizClient] Error getting the number of DViz users');
    }
  });
}

function showGameplaySettings() {
  DVIZ.debug && console.log('[DVizClient] Showing gameplay settings');
  $('#gameplaySettings').show();
  $('#settings').tooltip('hide')
    .attr('data-original-title', 'Close gameplay settings.')
    .tooltip('fixTitle')
    .tooltip('show');
  $('#settings').unbind('click');
  $('#settings').on('click', hideGameplaySettings);
}

function hideGameplaySettings() {
  DVIZ.debug && console.log('[DVizClient] Hiding gameplay settings');
  $('#gameplaySettings').hide();
  $('#settings').tooltip('hide')
    .attr('data-original-title', 'View and modify gameplay settings.')
    .tooltip('fixTitle')
    .tooltip('show');
  $('#settings').unbind('click');
  $('#settings').on('click', showGameplaySettings);
}

function initializeGameplaySettings() {
  $('#baseLinearSpeed').slider({
    value : 0.5,
    min : 0.01,
    max : 1.0,
    step : 0.03,
    slide : function(event, ui) {
      setBaseLinearSpeed(ui.value);
    }
  });
  $('#baseAngularSpeed').slider({
    value : 0.5,
    min : 0.01,
    max : 1.0,
    step : 0.03,
    slide : function(event, ui) {
      setBaseAngularSpeed(ui.value);
    }
  });
  $('#endEffectorSpeed').slider({
    value : 0.06,
    min : 0.02,
    max : 0.24,
    step : 0.02,
    slide : function(event, ui) {
      setEndEffectorSpeed(ui.value);
    }
  });
  $('#zoomSpeed').slider({
    value : 1.7,
    min : 0.5,
    max : 3.0,
    step : 0.1,
    slide : function(event, ui) {
      setZoomSpeed(ui.value);
    }
  });
}

function getUrlVariables() {
  var info = location.search.replace('?', '').split('&').map(function(x) {
    return x.split('=');
  });

  if(DVIZ.debug) {
    console.log('url variables: ');
    for(var i = 0; i < info.length; i++) {
      console.log(info[i][0] + ' = ' + info[i][1]);
    }
  }
  
  return info;
}

function generateRandomString(size) {
  var chars = 'ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789';
  var res = '';
  for(var i = 0; i < size; ++i) {
    res += chars[Math.round(Math.random() * (chars.length - 1))];
  }
  return res;
}
