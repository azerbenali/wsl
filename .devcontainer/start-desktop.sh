#!/bin/bash
# Virtual 1920x1080 desktop on DISPLAY :1, shown in a browser through noVNC:
#   http://localhost:6080/vnc.html?autoconnect=1&resize=scale
# VNC listens on localhost only; noVNC is reached through the forwarded port.
pgrep -x Xvfb > /dev/null || { Xvfb :1 -screen 0 1920x1080x24 > /tmp/xvfb.log 2>&1 & sleep 1; }
pgrep -x fluxbox > /dev/null || { DISPLAY=:1 fluxbox > /tmp/fluxbox.log 2>&1 & }
pgrep -x x11vnc > /dev/null || { x11vnc -display :1 -localhost -forever -shared -nopw -rfbport 5901 -quiet > /tmp/x11vnc.log 2>&1 & }
pgrep -f "websockify.*6080" > /dev/null || { websockify --web /usr/share/novnc 6080 localhost:5901 > /tmp/novnc.log 2>&1 & }
echo "Desktop: http://localhost:6080/vnc.html?autoconnect=1&resize=scale"
