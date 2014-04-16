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

  echo('<p><strong>Path: ' . $directory->path . '</p></strong>');

  echo('<ol>');

  while(false !== ($entry = $directory->read()))
  {
    if(preg_match('/.+\.(bag)/', $entry, $output))
    {
      //echo($directory->path . '/' . $entry);
      $filesize = filesize($directory->path . '/' . $entry);
      echo('<li><a href=\'http://sbpl.net:21890/?replayBagfile=' . $entry . '\'>' 
      . $entry . '</a> (' . $filesize . ' bytes)</li>');
    }
  }

  echo('</ol>');

  $directory->close();
}

?>