<?php

// User requested to download a bagfile
if(isset($_GET['dl']))
{
  // TODO: Download the requested bagfile
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
      if(preg_match('/demonstration_(.*)_/', $entry, $info))
      {    
      	 echo('<li><a href=\'http://sbpl.net:21890/?replayBagfile=' . $entry . '\'>' 
              . $info[1] . '</a> (' . $filesize . ' bytes)</li>');

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