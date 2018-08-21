<!DOCTYPE html>
<html>
  <head>
    <meta http-equiv="refresh" content="5">
    <style>
      h1 {text-align:center;}
      h2 {text-align:center;}
    </style>
  </head>
  <body>
    <h1>raspi4 monitor</h1>
    <pre><?php
        printf ( "<h2>%s</h2>\n", date ( DATE_RFC822 ) );

	printf ( "<h2>" );
	echo shell_exec('cat /tmp/workfile');
        printf ( "</h2>\n" );

        //echo shell_exec('tail -n 20 /var/log/piupsmon.log');
    ?></pre>
  </body>
</html>

