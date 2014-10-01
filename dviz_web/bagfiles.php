<html>
  <head>
    <title>Teach-a-Robot Demonstrations</title>
  </head>

  <body>
  <h2>Teach-a-Robot Demonstration Repository</h2>

  <p>The following is a list of demonstrations of a mobile manipulation task in a kitchen environment using a PR2. There are currently <b>1154</b> demonstrations from <b>116</b> different demonstrators.</p>

  <p>The demonstrated trajectories have been serialized into ROS bagfiles. The bagfile message types can be downloaded <a href="demo_msgs.tar">here</a>.</p>

  <p>Demonstration bagfiles can be downloaded individually from the list below, or as a single file <a href="all_demos.tar">here</a>.</p>
<?php

// User requested to download a bagfile
if(isset($_GET['dl']))
{
  // Download the requested bagfile
  $file = '/home/eratner/demonstrations/good/' . $_GET['dl'];

  $dlname = 'demo.bag';
  if(preg_match('/demonstration_(.*)_/', $file, $info))
  {
    $dlname = $info[1] . '.bag';
  }

  header('Content-Description: File Transfer');
  header('Content-Type: application/octet-stream');
  header('Content-Disposition: attachment; filename=' . $dlname);
  header('Content-Transfer-Encoding: binary');
  header('Expires: 0');
  header('Cache-Control: must-revalidate');
  header('Pragma: public');
  header('Content-Length: ' . filesize($file));
  ob_clean();
  flush();
  readfile($file);
  exit;
}
// List all bagfiles in the folder
else
{
  $directory = dir('/home/eratner/demonstrations');

  //echo('<p><strong>Path: ' . $directory->path . '</p></strong>');

  echo('<ol>');

  while(false !== ($entry = $directory->read()))
  {
    if(preg_match('/.+\.(bag)/', $entry, $output))
    {
      //echo($directory->path . '/' . $entry);
      $filesize = filesize($directory->path . '/' . $entry);
      $filesizeMb = $filesize / 1000000.0;
      if(preg_match('/demonstration_(.*)_/', $entry, $info))
      {    
      	 echo('<li><a href=\'http://sbpl.net:21890/?replayBagfile=' . $entry . '\'>' 
              . 'Replay' . '</a> <a href=\'http://sbpl.net:21890/bagfiles.php?dl=' . $entry . '\'>'
	      . 'Download</a>' . ' (' . $filesizeMb . ' MB)</li>');

	 $description = array(
	   0 => array('pipe', 'r'), // stdin
	   1 => array('pipe', 'w'), // stdout
	   2 => array('pipe', 'w'), // stderr
	 );

	 $cwd = '/home/eratner/demonstrations/';
	 
	 $appliction = '. /home/eratner/demonstrations/inspect_demo ' . $entry;
	 $pipes = array();
	 
	 $proc = proc_open($application, $description, $pipes, $cwd);  
	 
	 if(is_resource($proc))
	 {
	   //echo('stdout: ' . stream_get_contents($pipes[1]));
	   //echo('stderr: ' . stream_get_contents($pipes[2]));
	   fclose($pipes[0]);
	   fclose($pipes[1]);
	   fclose($pipes[2]);

	   //$returnval = proc_close($proc);
	   //echo('return value = $returnval');
	 }
      }		 
    }
  }

  echo('</ol>');

  $directory->close();
}
?>
  </body>
</html>
