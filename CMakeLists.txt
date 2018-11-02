cmake_minimum_required(VERSION 2.8.3)

option(STANDALONE "Use MY_LIB instead of THEIR_LIB" OFF) #OFF by default
if(STANDALONE)
    MESSAGE( "Stanalone" )
    include( CMakeLists.standalone.txt)
else()
    MESSAGE( "Catkin" )
    include( CMakeLists.rospkg.txt)
endif(STANDALONE)
unset(STANDALONE CACHE)
