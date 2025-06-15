#!/usr/bin/env python3
import os
import rospkg
import rospy
import threading
from .NLUTableModel import NLUTableModel

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QTimer
from python_qt_binding.QtWidgets import QWidget
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel, QPixmap
from qt_gui.plugin import Plugin

from std_msgs.msg import Float32, Bool, String
from std_srvs.srv import SetBool, SetBoolRequest
from clf_speech_msgs.srv import SetFloat32, SetFloat32Request
from clf_speech_msgs.msg import NLU, ASR


class CLFSpeech(Plugin):
    """
    Graphical frontend for managing clf_speech recognition.
    """

    _cm_update_freq = 1  # Hz

    def __init__(self, context):
        super(CLFSpeech, self).__init__(context)
        self.setObjectName("CLFSpeech")

        self.amp_lock = threading.Lock()
        self.last_amp = 0.0
        self.recording_lock = threading.Lock()
        self.recording = False
        self.enabled_lock = threading.Lock()
        self.vad_enabled = False

        # Initialize members
        self._audio_ns = "/silero_vad"  # Namespace of the audio recording
        self._asr_text_topic = "/ros_whisper/text"
        self._asr_topic = "/ros_whisper/asr"
        self._nlu_topic = "/rasa/nlu"
        self._ignore_asr_text = None

        try:
            msg = rospy.wait_for_message(
                f"{self._audio_ns}/min_amp", Float32, timeout=2
            )
            self.min_amp = int(msg.data * 100)
        except rospy.ROSException as e:
            rospy.logwarn(f"{e}", logger_name="CLFSpeech")
            self.min_amp = 0

        # Create QWidget and populate it via the UI file
        self._widget = QWidget()
        rp = rospkg.RosPack()
        path = rp.get_path("clf_speech_rqt")
        ui_file = os.path.join(path, "resource", "clf_speech.ui")
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("CLF Speech UI")

        self._widget.audio_slider.setValue(self.min_amp)

        self._icons = {
            name: QIcon(QPixmap(os.path.join(path, "resource", file)))
            for name, file in zip(
                ["recording", "stopped", "disabled"],
                ["led_green.png", "led_red.png", "led_off.png"],
            )
        }
        self._widget.enabled_button.setIcon(self._icons["disabled"])
        self._widget.enabled_button.pressed.connect(self.toggle_vad_enabled)

        # Add instance ID to window title to distinguish multiple instances
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} {context.serial_number()}"
            )

        self.text_list_model = QStandardItemModel()
        self._widget.asr_list.setModel(self.text_list_model)
        self._widget.text_input.currentIndexChanged.connect(self.send_text)

        self.nlu_model = NLUTableModel()
        self._widget.nlu_table.setModel(self.nlu_model)
        # self._widget.nlu_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._widget.nlu_table.setColumnWidth(0, 180)
        self._widget.nlu_table.setColumnWidth(1, 360)
        self._widget.nlu_table.setColumnWidth(2, 400)

        # Add widget to the user interface
        context.add_widget(self._widget)

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.setInterval(10)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

        self.start_monitor()

        if self.nlu_subscriber.get_num_connections() == 0:
            self._widget.nlu_table.hide()

    def send_text(self):
        if self._widget.text_input.currentIndex() < 0:
            return
        value = self._widget.text_input.currentText()
        rospy.loginfo(logger_name="CLFSpeech", msg=f"sending '{value}'")
        self.publisher.publish(value)

        self._ignore_asr_text = value
        asr = ASR()
        asr.text = value
        asr.conf = 1.0
        asr.lang = asr.EN
        self.asr_publisher.publish(asr)

        # finally clear the text input
        self._widget.text_input.setCurrentIndex(-1)

    def toggle_vad_enabled(self):
        with self.enabled_lock:
            current = self.vad_enabled

        # Call service to enable/disable VAD
        req = SetBoolRequest()
        req.data = not current
        try:
            self.service_enable_vad(req)
        except rospy.ServiceException as e:
            rospy.logerr(f"{e}", logger_name="CLFSpeech")

    def refresh_topics(self):
        # rospy.loginfo(logger_name="CLFSpeech", msg=f"REFRESH")
        if self.min_amp != self._widget.audio_slider.value():
            self.min_amp = self._widget.audio_slider.value()
            amp_rec = SetFloat32Request()
            amp_rec.data = self.min_amp / 100.0
            try:
                self.service_set_amp(amp_rec)
            except rospy.ServiceException as e:
                rospy.logerr(f"{e}", logger_name="CLFSpeech")

        with self.amp_lock:
            self._widget.audio_bar.setValue(int(self.last_amp * 100))

        with self.enabled_lock:
            with self.recording_lock:
                if not self.vad_enabled:
                    self._widget.enabled_button.setIcon(self._icons["disabled"])
                elif self.recording:
                    self._widget.enabled_button.setIcon(self._icons["recording"])
                else:
                    self._widget.enabled_button.setIcon(self._icons["stopped"])

    def start_monitor(self):
        topic = self._audio_ns + "/amp"
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.audio_subscriber = rospy.Subscriber(topic, Float32, self.callback_amp)

        topic = self._audio_ns + "/enabled"
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.enabled_subscriber = rospy.Subscriber(topic, Bool, self.callback_enabled)

        topic = self._audio_ns + "/in_rec"
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.rec_subscriber = rospy.Subscriber(topic, Bool, self.callback_recording)

        topic = self._nlu_topic
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.nlu_subscriber = rospy.Subscriber(topic, NLU, self.callback_nlu)

        topic = self._asr_text_topic
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.asr_text_subscriber = rospy.Subscriber(topic, String, self.callback_asr_text)
        self.publisher = rospy.Publisher(topic, String, queue_size=1)

        topic = self._asr_topic
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.asr_subscriber = rospy.Subscriber(topic, ASR, self.callback_asr)
        self.asr_publisher = rospy.Publisher(topic, ASR, queue_size=1)

        topic = self._audio_ns + "/set_min_amp"
        self.service_set_amp = rospy.ServiceProxy(topic, SetFloat32)

        topic = self._audio_ns + "/enable_vad"
        self.service_enable_vad = rospy.ServiceProxy(topic, SetBool)

        self._timer_refresh_topics.start()

    def callback_asr_text(self, message):
        rospy.loginfo(logger_name="CLFSpeech", msg=f"asr: {message.data}")
        item = QStandardItem(message.data)
        self.text_list_model.insertRow(0, item)
        self.text_list_model.setRowCount(20)  # limit to 20 rows

    def callback_asr(self, message: ASR):
        rospy.loginfo(logger_name="CLFSpeech", msg=f"asr: {message.text}")
        # avoid listing the same text twice
        if message.text == self._ignore_asr_text:
            self._ignore_asr_text = None
            return
        item = QStandardItem(f"({message.lang}) {message.text}")
        self.text_list_model.insertRow(0, item)
        self.text_list_model.setRowCount(20)  # limit to 20 rows

    def callback_nlu(self, message):
        self._widget.nlu_table.show()
        rospy.loginfo(
            f"nlu for : '{message.text}' intent:{message.intent}",
            logger_name="CLFSpeech",
        )
        self.nlu_model.add_nlu(message)
        self._widget.nlu_table.viewport().update()

    def callback_amp(self, message):
        with self.amp_lock:
            self.last_amp = message.data

    def callback_enabled(self, message):
        with self.enabled_lock:
            self.vad_enabled = message.data

    def callback_recording(self, message):
        with self.recording_lock:
            self.recording = message.data

    def shutdown_plugin(self):
        self._timer_refresh_topics.stop()

    def save_settings(self, plugin_settings, instance_settings):
        # Save session settings
        pass

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session
        pass
