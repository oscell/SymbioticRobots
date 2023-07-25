@echo off

REM The front_laser configuration of Jackal is sufficient for
REM basic gmapping and navigation. It is mostly the default
REM config, but with a SICK LMS100 series LIDAR on the front,
REM pointing forward.

set JACKAL_LASER=1
set JACKAL_URDF_EXTRAS=/home/oscell/jackal_ws/src/jackal_description/urdf/realsense.urdf.xacro
