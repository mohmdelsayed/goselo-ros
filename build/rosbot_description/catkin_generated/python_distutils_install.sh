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

echo_and_run cd "/home/ros/goselo-ros/src/rosbot_description"

# ensure that Python install destination exists
echo_and_run mkdir -p "$DESTDIR/home/ros/goselo-ros/install/lib/python2.7/dist-packages"

# Note that PYTHONPATH is pulled from the environment to support installing
# into one location when some dependencies were installed in another
# location, #123.
echo_and_run /usr/bin/env \
    PYTHONPATH="/home/ros/goselo-ros/install/lib/python2.7/dist-packages:/home/ros/goselo-ros/build/lib/python2.7/dist-packages:$PYTHONPATH" \
    CATKIN_BINARY_DIR="/home/ros/goselo-ros/build" \
    "/usr/bin/python" \
    "/home/ros/goselo-ros/src/rosbot_description/setup.py" \
    build --build-base "/home/ros/goselo-ros/build/rosbot_description" \
    install \
    $DESTDIR_ARG \
    --install-layout=deb --prefix="/home/ros/goselo-ros/install" --install-scripts="/home/ros/goselo-ros/install/bin"
