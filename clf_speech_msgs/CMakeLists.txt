cmake_minimum_required(VERSION 3.2)
project(clf_speech_msgs)

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
)

add_message_files(
	FILES
	NLU.msg
	Entity.msg
)

add_service_files(
	FILES
	SetFloat32.srv
)

generate_messages(
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)