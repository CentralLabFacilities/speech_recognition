cmake_minimum_required(VERSION 3.2)
project(clf_speech_msgs)

find_package(catkin REQUIRED
	COMPONENTS
	message_generation
	actionlib
	actionlib_msgs
)

add_action_files(
	FILES
	SpeechRecognition.action
)

add_message_files(
	FILES
		ASR.msg
		NLU.msg
		Entity.msg
)

add_service_files(
	FILES
	SetFloat32.srv
	Understand.srv
)

generate_messages(
	DEPENDENCIES actionlib_msgs
)

catkin_package(
	CATKIN_DEPENDS
	message_runtime
)