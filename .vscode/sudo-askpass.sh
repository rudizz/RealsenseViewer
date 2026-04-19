#!/bin/sh

/usr/bin/osascript \
    -e 'display dialog "Password required to debug RealsenseViewer with sudo:" default answer "" with hidden answer buttons {"OK", "Cancel"} default button "OK" cancel button "Cancel"' \
    -e 'text returned of result'
