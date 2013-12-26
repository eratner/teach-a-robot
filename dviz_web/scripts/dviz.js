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

    back : function() {
      return buffer[n-1];
    },

    get : function(key) {
      return buffer[key];
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

  console.log('[CameraManager] user_id = ' + this.id + ' topic: /dviz_user_' + this.id + '/base_footprint');

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
    console.log('[DVizClient] Changing from camera mode ' + this.cameraMode
                + ' to ' + mode + '.');
  }
  this.lastCameraMode = this.cameraMode;
  this.cameraMode = mode;

  // Update the camera controls
  this.viewer.cameraControls.update();
};

DVIZ.CameraManager.prototype.getCameraMode = function() {
  return this.cameraMode;
}

DVIZ.CameraManager.prototype.setFilterTf = function(filter) {
  this.filterTf = filter;
}

/**
 * Focus the center of the camera to a the point (x, y, z) in space
 */
DVIZ.CameraManager.prototype.centerCameraAt = function(x, y, z) {
  // @todo
  this.cameraManager.viewer.cameraControls.center.x = x;
  this.cameraManager.viewer.cameraControls.center.y = y;
  this.cameraManager.viewer.cameraControls.center.z = z;
  this.cameraManager.viewer.cameraControls.update();
}

/**
 * Sets the zoom speed of the camera
 */
DVIZ.CameraManager.prototype.setZoomSpeed = function(speed) {
  console.log('[DVizClient] Setting zoom speed to ' + speed.toString());

  // @todo this needs to be tested further
  this.cameraManager.viewer.cameraControls.userZoomSpeed = speed;
  this.cameraManager.viewer.cameraControls.update();
}

/**
 * Sets the actual zoom of the camera to a particular scale
 */
DVIZ.CameraManager.prototype.setZoom = function(scale) {
  console.log('[DVizClient] Setting zoom scale to ' + scale.toString());

  this.cameraManager.viewer.cameraControls.scale = scale;
  this.cameraManager.viewer.cameraControls.update();
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
  this.acceptedGrasp = false;

  console.log('[DVizClient] Assigned id ' + this.id);
  
  this.cameraManager = new DVIZ.CameraManager({
    ros : ros,
    tfClient : tfClient,
    viewer : viewer,
    width : width,
    height : height,
    id : this.id
  });
  
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
      // When changing from goal -1 (i.e. no goals) to goal 0
      // (i.e. the first goal in the task), do not display a goal
      // completed dialog
      if(currentGoal > 0) {
	that.goalCompleted(that.currentGoalNumber);
      }

      console.log('[DVizClient] Changing from goal ' + that.currentGoalNumber.toString() + ' to goal ' + currentGoal.toString());

      that.currentGoalNumber = currentGoal;

      if(that.currentGoalNumber >= that.goals.length) {
	// All goals complete; notify the user
	that.displayStatusText('All goals completed!');
	return;
      }

      var html = '';
      for(var i = 0; i < message.goals.length; ++i) {
	html = html + '<a href="#" onclick="dvizClient.changeGoal(' +
	  message.goals[i].number + ')" class="list-group-item' + 
	  (message.goals[i].number === currentGoal ? ' active' : '') + 
	  '">' + message.goals[i].description + '</a>';
      }
      $('#task').html(html);

      // Check if we need to change the state of the camera (e.g. if the 
      // next goal is a pick up goal, we need to focus the camera at the 
      // grasp pose)
      that.displayStatusText('Next goal: ' + that.goals[that.currentGoalNumber].description);
      if(that.goals[that.currentGoalNumber].type === 0) { // Pick up goal
	// Show an interactive gripper marker at the current goal
	console.log('[DVizClient] Current goal is of type pick-up');
	this.acceptedGrasp = false;
	$('#acceptChangeGrasp').html('<img src="images/accept_grasp.png" width="65" height="65" />');
	$('#gripperJointAngle').slider('option', 'value', 
				       that.goals[that.currentGoalNumber].gripper_joint_position);
	that.showInteractiveGripper(that.currentGoalNumber);
	$('#acceptChangeGrasp').prop('disabled', false);
	$('#acceptChangeGrasp').tooltip('show');
	// Switch the camera back to follow the base of the robot
	that.cameraManager.setCamera(0);
      } else {
	// The goal is not of type pick up
	$('#acceptChangeGrasp').prop('disabled', true);
      }
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.play = function() {
  console.log('[DVizClient] Playing simulator...');
  this.playing = true;

  $('#playPause').html('<img src="images/pause.png" width="65" height="65" />');
  
  // If the user has not started the game yet, load the task
  if(!this.gameStarted) {
    this.loadTask();
    this.gameStarted = true;
  }

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'play',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.pause = function(now) {
  console.log('[DVizClient] Pausing simulator...');
  this.playing = false;

  $('#playPause').html('<img src="images/play.png" width="65" height="65" />');

  var pauseNow = now || true;

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : (pauseNow ? 'pause_now' : 'pause_later'),
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.isPlaying = function() {
  return this.playing;
}

DVIZ.DemonstrationVisualizerClient.prototype.changeCamera = function(follow) {
  if(follow === 'base') {
    console.log('[DVizClient] Changing camera to follow the base');
    this.cameraManager.setCamera(0);
    this.cameraManager.viewer.cameraControls.center.x =
      this.cameraManager.baseXFilter.back();
    this.cameraManager.viewer.cameraControls.center.y =
      this.cameraManager.baseYFilter.back();
    this.cameraManager.viewer.cameraControls.center.z = 0.0;
  } else if(follow === 'gripper') {
    console.log('[DVizClient] Changing camera to follow the gripper');
    this.cameraManager.setCamera(2);
    this.cameraManager.viewer.cameraControls.center.x =
      this.cameraManager.rEndEffectorXFilter.back();
    this.cameraManager.viewer.cameraControls.center.y =
      this.cameraManager.rEndEffectorYFilter.back();
    this.cameraManager.viewer.cameraControls.center.z =
      this.cameraManager.rEndEffectorZFilter.back();
  } else if(follow === 'none') {
    console.log('[DVizClient] Changing camera to free mode');
    this.cameraManager.setCamera(1);
  } else {
    console.log('[DVizClient] Unknown camera mode "' + follow + '"');
  }
}

DVIZ.DemonstrationVisualizerClient.prototype.toggleGripperControls = function() {
  console.log('[DVizClient] Toggling gripper controls to ' +
	      (!this.basicGripperControls).toString() + '.');
  this.basicGripperControls = !this.basicGripperControls;
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'gripper_controls',
    args : [this.basicGripperControls.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.resetRobot = function() {
  console.log('[DVizClient] Resetting robot...');
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'reset_robot',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.resetTask = function() {
  console.log('[DVizClient] Resetting task...');
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'reset_task',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
};

DVIZ.DemonstrationVisualizerClient.prototype.loadScene = function(scene) {
  //showAlertModal('Loading the kitchen...', 'This might take a minute or two!');
  var sceneName = scene || 'kitchen_lite.xml';

  console.log('[DVizClient] Loading scene ' + sceneName);

  // Pause the game while the scene loads
  this.pause();

  // @todo bring up a loading message until the scene has fully loaded into the browser.
  // then, need to figure out how to wait until the collada meshes have been fully rendered.

  this.coreCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_scene',
    args : [this.id.toString(), 
	    '/home/eratner/ros/teach-a-robot/dviz_core/scenes/' + sceneName]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + res.response);
    } else {
      console.log('[DVizClient] Loaded the scene ' + sceneName);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.loadTask = function(task) {
  var taskName = task || 'brownie_recipe.xml';

  console.log('[DVizClient] Loading task ' + taskName);

  // Load the task
  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'load_task',
    args : ['package://dviz_core/tasks/' + taskName]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + res.response);
    } else {
      console.log('[DVizClient] Loaded the task ' + taskName);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.handleKeyPress = function(e) {
  //console.log('[DVizClient] Handling key press event: ' + e.which + '.');

  // Filter certain keys depending on the client state.
  if(e.which === 90) {
    if(!this.zMode) {
      console.log('[DVizClient] Enabling z-mode');
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
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.handleKeyRelease = function(e) {
  //console.log('[DVizClient] Handling key release event: ' + e.which + '.');

  if(e.which === 90) {
    if(this.zMode) {
        console.log('[DVizClient] Disabling z-mode');
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
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.changeGoal = function(num) {
  // @todo first alert the user, and confirm that they want to switch goals

  console.log('[DVizClient] Changing goal to ' + num);
  // @todo check if valid goal number?
  this.currentGoal = num;

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'change_goal',
    args : [num.toString()]
  }), function(response) {
    console.log('[DVizClient] Error response: ' + response.response);
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.showInteractiveGripper = function(goalNumber) {
  console.log('[DVizClient] Showing interactive gripper for goal ' + goalNumber.toString());

  // Disable marker control of the robot (so the user cannot move 
  // the robot while selecting a grasp)
  this.robotMarkerControl(false);
  this.pause();

  $('#gripperJointAngle').slider('option', 'disabled', false);

  this.changeCamera('none');
  // @todo there should be a centerCameraAt(x,y,z) method in DVIZ.CameraManager
  this.cameraManager.viewer.cameraControls.center.x = 
    this.goals[this.currentGoalNumber].initial_object_pose.position.x;
  this.cameraManager.viewer.cameraControls.center.y = 
    this.goals[this.currentGoalNumber].initial_object_pose.position.y;
  this.cameraManager.viewer.cameraControls.center.z = 
    this.goals[this.currentGoalNumber].initial_object_pose.position.z;
  this.cameraManager.viewer.cameraControls.update();

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'show_interactive_gripper',
    args : [goalNumber.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.hideInteractiveGripper = function(goalNumber) {
  console.log('[DVizClient] Hiding interactive gripper for goal ' + goalNumber.toString());

  $('#gripperJointAngle').slider('option', 'disabled', true);

  this.robotMarkerControl(true);
  this.changeCamera('none');
  this.cameraManager.viewer.cameraControls.center.x =
    this.cameraManager.baseXFilter.back();
  this.cameraManager.viewer.cameraControls.center.y =
    this.cameraManager.baseYFilter.back();
  this.cameraManager.viewer.cameraControls.center.z = 0.0;

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'hide_interactive_gripper',
    args : [goalNumber.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.acceptGrasp = function() {
  if(this.acceptedGrasp) {
    console.log('[DVizClient] Changing grasp');
    this.pause();
    this.acceptedGrasp = false;

    $('#acceptChangeGrasp').html('<img src="images/accept_grasp.png" width="65" height="65" />');
    $('#acceptChangeGrasp').tooltip('destroy');
    $('#acceptChangeGrasp').tooltip({
      title : 'Click here when you\'re done choosing a good grasp for the object.'
    });
    $('#acceptChangeGrasp').tooltip('show');

    this.showInteractiveGripper(this.currentGoalNumber);
  } else {
    console.log('[DVizClient] Accepting grasp');
    this.play();
    this.acceptedGrasp = true;

    // Hide the interactive gripper of the current goal
    this.hideInteractiveGripper(this.currentGoalNumber);

    this.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'accept_grasp',
      args : []
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      }
    });
    
    $('#acceptChangeGrasp').html('<img src="images/change_grasp.png" width="65" height="65" />');
    $('#acceptChangeGrasp').tooltip('destroy');
    $('#acceptChangeGrasp').tooltip({
      title : 'Click here if you want to change the grasp that you\'ve already chosen.'
    });
    $('#acceptChangeGrasp').tooltip('show');
  }
}

DVIZ.DemonstrationVisualizerClient.prototype.beginRecording = function() {
  console.log('[DVizClient] Begin recording');

  $('#rrButton').html(
    '<button type="button" class="btn btn-default" onclick="dvizClient.endRecording()">End Recording</button>');

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'begin_recording',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.endRecording = function() {
  console.log('[DVizClient] End recording');

  $('#rrButton').html(
    '<button type="button" class="btn btn-default" onclick="dvizClient.beginRecording()">Begin Recording</button>');

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'end_recording',
    args : []
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.beginReplay = function() {
  console.log('[DVizClient] Begin replay');

  // Reset the robot before replaying a user demonstration
  this.resetRobot();

  // @todo query a database of user demonstrations to replay the 
  // desired one

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'begin_replay',
    args : ['/home/eratner/demonstrations/last_demonstration_'
	    + this.id.toString() + '.bag']
    // hack! the user should somehow input which demonstration to replay
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.goalCompleted = function(goalNumber) {
  console.log('[DVizClient] Goal ' + goalNumber.toString() + ' completed!');

  // Notify the user that a goal has been completed, and give a 
  // description of the next goal in the task
  var message = 'Goal ' + goalNumber.toString() + ' completed! ';
  if(goalNumber + 1 >= this.goals.length) {
    message += 'The task is complete.';
  } else {
    message += ('Next goal: ' + 
		this.goals[goalNumber + 1].description);
  }
  $('#goalCompleteMessage').text(message);
  $('#goalCompleteModal').modal('show');
}

DVIZ.DemonstrationVisualizerClient.prototype.endDemonstration = function() {
  console.log('[DVizClient] Ending demonstration.');

  if(this.gameStarted) {
    this.gameStarted = false;
    // @todo do all the post-game 'clean up': save demonstration files,
    // etc. 
    // this can be called when a game is prematurely terminated

  }
}

DVIZ.DemonstrationVisualizerClient.prototype.robotMarkerControl = function(enabled) {
  console.log('[DVizClient] Robot marker control is ' +
	      (enabled ? 'enabled' : 'disabled'));

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'robot_marker_control',
    args : [enabled.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.setGripperJointAngle = function(angle) {
  console.log('[DVizClient] Setting gripper joint angle to ' + angle.toString());

  this.commandClient.callService(new ROSLIB.ServiceRequest({
    command : 'set_gripper_joint',
    args : [angle.toString()]
  }), function(response) {
    if(response.response.length > 0) {
      console.log('[DVizClient] Error response: ' + response.response);
    }
  });
}

DVIZ.DemonstrationVisualizerClient.prototype.displayStatusText = function(text) {
  console.log('adding status: ' + text);
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

function init() {
  $('#debugControls').hide();

  ros = new ROSLIB.Ros({
    url: 'ws://sbpl.net:21891'
  });

  var viewer = new ROS3D.Viewer({
    divID : 'dviz',
    width : 800,
    height : 600,
    antialias : true
  });

  // Request a new DVizUser from the core
  dvizCoreCommandClient = new ROSLIB.Service({
    ros : ros,
    name : '/dviz_command',
    serviceType : 'dviz_core/Command'
  });

  dvizCoreCommandClient.callService(new ROSLIB.ServiceRequest({
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

    // Client to handle interactive markers
    var imClient = new ROS3D.InteractiveMarkerClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/dviz_user_' + userId + '/interactive_markers',
      path : 'http://resources.robotwebtools.org/',
      camera : viewer.camera,
      rootObject : viewer.selectableObjects
    });

    // Client to handle the robot visualization markers
    var rmClient = new ROS3D.RobotMarkerArrayClient({
      ros : ros,
      tfClient : tfClient,
      topic : '/dviz_user_' + userId + '/visualization_marker_array',
      path : 'http://resources.robotwebtools.org/',
      rootObject : viewer.scene
    });

    // Client to display spheres to indicate robot collisions
    var rcClient = new ROS3D.MultiMarkerClient({
      ros : ros,
      topic : '/dviz_user_' + userId + '/collisions/visualization_marker',
      tfClient : tfClient,
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

    // Initialize button click callbacks
    $('#playPause').on('click', function() {
      if(dvizClient.isPlaying()) {
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
	b.html('Base Camera<br /><img src="images/black.jpg" width="65" height="65" />');
	dvizClient.changeCamera('gripper');
      } else if(dvizClient.cameraManager.getCameraMode() === 2) {
	// Gripper-following camera
	b.html('Hand Camera<br /><img src="images/black.jpg" width="65" height="65" />');
	dvizClient.changeCamera('base');
      } else {
	console.log('[DVizClient] Error in base/hand camera handler');
      }
    });

    $('#freeFollowingCamera').on('click', function() {
      var b = $('#freeFollowingCamera');

      if(dvizClient.cameraManager.getCameraMode() !== 1) {
	// Camera is not in free mode
	b.html('Lock Camera<br /><img src="images/black.jpg" width="65" height="65" />');
	dvizClient.changeCamera('none');
	$('#baseHandCamera').prop('disabled', true);
      } else {
	// Camera is in free mode
	b.html('Unlock Camera<br /><img src="images/black.jpg" width="65" height="65" />');
	dvizClient.cameraManager.setCamera(
	  dvizClient.cameraManager.lastCameraMode
	);
	$('#baseHandCamera').prop('disabled', false);
      }
    });

    $('#rotateHandControls').on('click', function() {
      var b = $('#rotateHandControls');

      if(dvizClient.basicGripperControls) {
	b.html('<img src="images/simple_control.png" width="65" height="65" />');
      } else {
	b.html('<img src="images/full_control.png" width="65" height="65" />');
      }

      dvizClient.toggleGripperControls();
    })

    $('#gripperJointAngle').slider({
      value : 0.548,
      min : 0.0,
      max : 0.548,
      step : 0.02,
      slide : function(event, ui) {
	dvizClient.setGripperJointAngle(ui.value);
      },
      disabled : true
    });

    dvizClient.loadScene();
    $('#playPause').prop('disabled', false);
    $('#playPause').tooltip('show');

    dvizClient.displayStatusText('Connected to DVizServer!');

    // Add keyboard bindings.
    $(window).bind('keydown', function(e) {
      dvizClient.handleKeyPress(e);
    });

    $(window).bind('keyup', function(e) {
      dvizClient.handleKeyRelease(e);
    });
  });

  // Initialize all tooltips
  $('.tip').tooltip();

  // Show the instructions
  //$('#infoModal').modal('show');

  // Initialize gameplay settings
  initializeGameplaySettings();
  hideGameplaySettings();

  //$(window).scroll(function() {
  //console.log('scrolling');
  //});
}

// Kill the DVizUser when the user exits the page
window.onbeforeunload = function removeUser() {
  dvizCoreCommandClient.callService(new ROSLIB.ServiceRequest({
    command : 'kill_user',
    args : [dvizClient.id.toString()]
  }), function(response) {
    console.log('[DVizClient] Killed user ' + dvizClient.id.toString());
  });
}

// @todo these need to change with the new user interface
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
    console.log('[DVizClient] \'' + document.getElementById('frameRate').value
		+ '\' is not a valid number!');
  } else {
    console.log('[DVizClient] Set frame rate to ' + frameRate);
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_frame_rate',
      args : [frameRate.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      }
    });
  }
}

function showAlert(message, timeout) {
  // Default argument for timeout is 2 s (2000 ms)
  timeout = (typeof timeout === 'undefined') ? 2000 : timeout;

  var alertDialog = $('#alertDialog');
  if(alertDialog === null) {
    $('#alerts').innerHTML = '<div id="alertDialog" class="alert alert-warning"></div>';
  }

  $('#alertDialog').innerHTML = '<a class="close" data-dismiss="alert" href="#" aria-hidden="true">&times;</a><p>' 
    + message + '</p>';

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
    console.log('[DVizClient] Setting base linear speed to ' + linearSpeed.toString());
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_base_speed',
      args : [linearSpeed.toString(),
	      '0.0']
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
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
	console.log('[DVizClient] Error response: ' + response.response);
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
    dvizClient.commandClient.callService(new ROSLIB.ServiceRequest({
      command : 'set_arm_speed',
      args : [speed.toString()]
    }), function(response) {
      if(response.response.length > 0) {
	console.log('[DVizClient] Error response: ' + response.response);
      }
    });
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
      console.log('[DVizClient] Error getting the number of DViz users');
    }
  });
}

function showGameplaySettings() {
  console.log('[DVizClient] Showing gameplay settings');
  $('#gameplaySettings').show();
  $('#settings').html('Hide Gameplay Settings<br /><img src="images/black.jpg" width="65" height="45" />');
  $('#settings').unbind('click');
  $('#settings').on('click', hideGameplaySettings);
}

function hideGameplaySettings() {
  console.log('[DVizClient] Hiding gameplay settings');
  $('#gameplaySettings').hide();
  $('#settings').html('Show Gameplay Settings<br /><img src="images/black.jpg" width="65" height="45" />');
  $('#settings').unbind('click');
  $('#settings').on('click', showGameplaySettings);
}

function initializeGameplaySettings() {
  $('#baseLinearSpeed').slider({
    value : 0.4,
    min : 0.01,
    max : 1.0,
    step : 0.03,
    slide : function(event, ui) {
      setBaseLinearSpeed(ui.value);
    }
  });
  $('#baseAngularSpeed').slider({
    value : 0.4,
    min : 0.01,
    max : 1.0,
    step : 0.03,
    slide : function(event, ui) {
      setBaseAngularSpeed(ui.value);
    }
  });
  $('#endEffectorSpeed').slider({
    value : 0.4,
    min : 0.01,
    max : 1.0,
    step : 0.03,
    slide : function(event, ui) {
      setEndEffectorSpeed(ui.value);
    }
  });
}