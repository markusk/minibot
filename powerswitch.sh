#! /bin/sh
### BEGIN INIT INFO
# Provides:          powerswitch
# Required-Start:
# Required-Stop:
# Default-Start:     2 3 4 5
# Default-Stop:      0 1 6
# Short-Description: Starts & Stops powerswitch monitor
# Description:       Starts & Stops powerswitch monitor
### END INIT INFO

#Switch case fuer den ersten Parameter
case "$1" in
    start)
 #Aktion wenn start uebergeben wird
        echo "Starte powerswitch monitor"
        # Starte mein Programm, was den powerswitch überwacht
        /home/pi/minibot/powerswitchmonitor.sh
        ;;

    stop)
 #Aktion wenn stop uebergeben wird
        echo "Stoppe powerswitch monitor"
        ;;

    restart)
 #Aktion wenn restart uebergeben wird
        echo "Restarte powerswitch monitor"
        ;;
 *)
 #Standard Aktion wenn start|stop|restart nicht passen
 echo "(start|stop|restart)"
 ;;
esac

exit 0
