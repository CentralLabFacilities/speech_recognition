#!/usr/bin/env python3
import os
import rospkg
import rospy
import threading
from .models import NLUTableModel
from .widgets import FontZoomWidget

from python_qt_binding import loadUi
from python_qt_binding.QtCore import Qt, QTimer, QModelIndex
from python_qt_binding.QtGui import QIcon, QStandardItem, QStandardItemModel, QPixmap
from python_qt_binding.QtGui import QBrush, QColor
from qt_gui.plugin import Plugin

from std_msgs.msg import Float32, Bool, String
from std_srvs.srv import SetBool, SetBoolRequest
from clf_speech_msgs.srv import SetFloat32, SetFloat32Request
from clf_speech_msgs.msg import NLU, ASR


def limit_rows(model, num):
    """Limit the number of rows in the ASR widget to num"""
    rows = model.rowCount(QModelIndex())
    if rows > num:
        model.removeRows(0, rows - num, QModelIndex())


class CLFSpeech(Plugin):
    """
    Graphical frontend for managing clf_speech recognition.
    """

    _cm_update_freq = 1  # Hz

    def __init__(self, context):
        super(CLFSpeech, self).__init__(context)
        self.setObjectName("CLFSpeech")

        self.amp_lock = threading.Lock()
        self.min_amp = 0
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
        self._tts_topic = "/tts"
        self._ignore_asr_text = None
        self._scroll_to_bottom = False

        # Create QWidget and populate it via the UI file
        self._widget = FontZoomWidget()
        rp = rospkg.RosPack()
        path = rp.get_path("clf_speech_rqt")
        ui_file = os.path.join(path, "resource", "clf_speech.ui")
        loadUi(ui_file, self._widget)
        self._widget.setObjectName("CLF Speech UI")

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
        self._widget.chat.setModel(self.text_list_model)

        self.init_text_inputs_from_params()
        self._widget.text_input.currentIndexChanged.connect(self.send_text)

        self.nlu_model = NLUTableModel()
        self._widget.nlu_table.setModel(self.nlu_model)
        # self._widget.nlu_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self._widget.nlu_table.setColumnWidth(0, 180)
        self._widget.nlu_table.setColumnWidth(1, 360)
        self._widget.nlu_table.setColumnWidth(2, 400)

        # Add widget to the user interface
        context.add_widget(self._widget)

        self.subscribe()

        if self.nlu_subscriber.get_num_connections() == 0:
            self._widget.nlu_table.hide()

        if self.audio_subscriber.get_num_connections() > 0:
            try:
                topic = f"{self._audio_ns}/min_amp"
                msg = rospy.wait_for_message(topic, Float32, timeout=2.0)
                self.min_amp = int(msg.data * 100)

            except rospy.ROSException as e:
                rospy.logwarn(f"{e}", logger_name="CLFSpeech")
        self._widget.audio_slider.setValue(self.min_amp)

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.setInterval(10)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)
        self._timer_refresh_topics.start()

    def init_text_inputs_from_params(self):
        """
        Initialize the text input dropdown with values from ROS parameters.
        """
        self._widget.text_input.clear()
        text_inputs = rospy.get_param("~text_inputs", [])
        for text in text_inputs:
            self._widget.text_input.addItem(text)

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

        if self._scroll_to_bottom:
            def scroll_to_bottom(view):
                rows = 0
                if view.model() is not None:
                    rows = view.model().rowCount(QModelIndex())
                if rows > 0:
                    view.scrollTo(view.model().index(rows - 1, 0))

            self._scroll_to_bottom = False
            scroll_to_bottom(self._widget.chat)
            scroll_to_bottom(self._widget.nlu_table)

    def subscribe(self):
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

        topic = self._tts_topic
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.tts_subscriber = rospy.Subscriber(topic, String, self.callback_tts)

        topic = self._audio_ns + "/set_min_amp"
        self.service_set_amp = rospy.ServiceProxy(topic, SetFloat32)

        topic = self._audio_ns + "/enable_vad"
        self.service_enable_vad = rospy.ServiceProxy(topic, SetBool)

    def callback_asr_text(self, message: String):
        rospy.loginfo(logger_name="CLFSpeech", msg=f"asr: {message.data}")
        self.add_chat_item(message.data, user=True)

    def callback_asr(self, message: ASR):
        rospy.loginfo(logger_name="CLFSpeech", msg=f"asr: {message.text}")
        # avoid listing the same text twice
        if message.text == self._ignore_asr_text:
            self._ignore_asr_text = None
            return
        self.add_chat_item(f"({message.lang}) {message.text}", user=True)

    def callback_tts(self, message: String):
        rospy.loginfo(logger_name="CLFSpeech", msg=f"tts: {message.data}")
        self.add_chat_item(message.data, user=False)

    def add_chat_item(self, text, user=True):
        # replace newlines with <br>
        item = QStandardItem(text.replace("\n", "<br>"))
        item.setData(user, role=Qt.UserRole)
        num = self.text_list_model.rowCount(QModelIndex())
        self.text_list_model.insertRow(num, item)
        limit_rows(self.text_list_model, 20)  # limit to 20 rows
        self._scroll_to_bottom = True

    def callback_nlu(self, message):
        self._widget.nlu_table.show()
        rospy.loginfo(
            f"nlu for : '{message.text}' intent:{message.intent}",
            logger_name="CLFSpeech",
        )
        self.nlu_model.add_nlu(message)
        limit_rows(self.nlu_model, 5)
        self._scroll_to_bottom = True

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
        instance_settings.set_value("font_size", self._widget.font().pointSize())

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session
        size = int(instance_settings.value("font_size", self._widget.font().pointSize()))
        self._widget.set_font_size(size)
