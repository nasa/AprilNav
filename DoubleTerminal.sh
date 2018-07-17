!#bash/bin

##### Constants
TITLE="System Information for $HOSTNAME:"
RIGHT_NOW=$(date +"%x %r %Z")
TIME_STAMP="Updated on $RIGHT_NOW by $USER"

COMMAND="tail -f output.txt"

echo $TITLE
echo $RIGHT_NOW
echo $TIME_STAMP
echo
echo -ne "\033]0;APRIL TAGS MAIN\007"
doubleterminal=false


if [ $# -gt 0 ]; then
    echo -e "Your command line contains $# arguments"
else
    echo "Your command line contains no arguments"
fi

echo
echo "You start with $# positional parameters"
echo

# Loop until all parameters are used up
while [ "$1" != "" ]; do
    echo "Parameter 1 equals $1"
    echo "You now have $# positional parameters"
    if [ "$1" == "-h" ]; then
      echo "    -h : help"
      echo "    -d : double terminal"
    elif [ "$1" == "-d" ]; then
      doubleterminal=true
      echo -e "\e[96m Double Terminal Activated \e[97m"
      if [ "$doubleterminal" = true ]; then
        #gnome-terminal -e 'tail -f output.txt'
        #gnome-terminal -e 'rainbow --cyan="Time*" --yellow="Tags detected:*" --blink="Vel*" tail -f output.txt'
        nohup xterm -hold -e 'rainbow --cyan="Time*" --yellow="Tags detected:*" --blink="Vel*" tail -f output.txt' &
        nohup xterm -hold -e './build/bin/ThreadTest' &
        #gnome-terminal -e './build/bin/StarTrackerInput'
        ./build/bin/AprilNav -S .401 -X Calibration/BlackCamera.txt -s

      fi
    else
      echo -e "\033[31m $1 is Not an accepted argument, -h for accepted options \e[97m"
      break
    fi

    # Shift all the parameters down by one
    shift
done




#tail -f output.txt | sed --unbuffered     -e "s/\(.*Time:*\)/\o033[96m\1\o033[39m/"     -e "s/\(.*Tags detected.*\)/\o033[93m\1\o033[39m/"
