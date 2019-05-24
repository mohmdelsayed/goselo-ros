#!/bin/sh

if [ -n "$DESTDIR" ] ; then
    case $DESTDIR in
        /*) # ok
            ;;
        *)
            /bin/echo "DESTDIR argument must be absolute... "
            /bin/echo "otherwise python's distutils will bork things."
            exit 1
    esac
    DESTDIR_ARG="--root=$DESTDIR"
fi

echo_and_run() { echo "+ $@" ; "$@" ; }

echo_and_run cd "/home/mohamed/catkin_ws/src/rosbot/src/rosbot_description"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/mohamed/catkin_ws/src/rosbot/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/mohamed/catkin_ws/src/rosbot/install/lib/python2.7/dist-packages:/home/mohamed/catkin_ws/src/rosbot/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/mohamed/catkin_ws/src/rosbot/build" \
    "/usr/bin/python" \
    "/home/mohamed/catkin_ws/src/rosbot/src/rosbot_description/setup.py" \
    build --build-base "/home/mohamed/catkin_ws/src/rosbot/build/rosbot_description" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/mohamed/catkin_ws/src/rosbot/install" --install-scripts="/home/mohamed/catkin_ws/src/rosbot/install/bin"
