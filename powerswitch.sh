#! /bin/sh
### BEGIN INIT INFO
# Provides:          powerswitch
# Required-Start:    
# Required-Stop:     
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Starts & Stops powerswitch
# Description:       Starts & Stops powerswitch
### END INIT INFO
 
#Switch case fuer den ersten Parameter
case "$1" in
    start)
 #Aktion wenn start uebergeben wird
        echo "Starte powerswitch"
        ;;
 
    stop)
 #Aktion wenn stop uebergeben wird
        echo "Stoppe powerswitch"
        ;;
 
    restart)
 #Aktion wenn restart uebergeben wird
        echo "Restarte powerswitch"
        ;;
 *)
 #Standard Aktion wenn start|stop|restart nicht passen
 echo "(start|stop|restart)"
 ;;
esac
 
exit 0
