#!/usr/bin/env python
import os
import rospkg
import rospy
import threading

from python_qt_binding import loadUi
from python_qt_binding.QtCore import QAbstractTableModel, QModelIndex, Qt,\
                                     QTimer, QVariant, Signal
from python_qt_binding.QtWidgets import QWidget, QFormLayout, QHeaderView,\
					QMenu, QStyledItemDelegate
from python_qt_binding.QtGui import QCursor, QFont, QIcon, QStandardItem, QStandardItemModel, QPixmap
from qt_gui.plugin import Plugin

from std_msgs.msg import Float32, Bool, String
from clf_speech_msgs.srv import SetFloat32, SetFloat32Request, SetFloat32Response


class CLFSpeech(Plugin):
    """
    Graphical frontend for managing clf_speech recognition.
    """
    _cm_update_freq = 1  # Hz

    def __init__(self, context):
        super(CLFSpeech, self).__init__(context)
        self.setObjectName('CLFSpeech')

        self.amp_lock = threading.Lock()
        self.last_amp = 0.0
        self.recording_lock = threading.Lock()
        self.recording = False
        self.enabled_lock = threading.Lock()
        self.vad_enabled = False

        


        # Initialize members
        self._audio_ns = "/silero_vad"  # Namespace of the audio recording
        self._asr_topic = "/ros_whisper/text"

        try:
            self.min_amp = int(rospy.wait_for_message(f'{self._audio_ns}/min_amp', Float32, timeout=10).data * 100)
        except:
            self.min_amp = 0

        # Create QWidget and extend it with all the attributes and children
        # from the UI file
        self._widget = QWidget()
        rp = rospkg.RosPack()
        ui_file = os.path.join(rp.get_path('clf_speech_rqt'),
                               'resource',
                               'clf_speech.ui')
        loadUi(ui_file, self._widget)
        self._widget.setObjectName('ClfSpeechUi')

        self._widget.audio_slider.setValue(self.min_amp)

        path = rp.get_path('clf_speech_rqt')
        self._pixmaps = {'recording': QPixmap(path + '/resource/led_green.png'),
                       'stopped': QPixmap(path + '/resource/led_red.png'),
                       'disabled': QPixmap(path + '/resource/led_off.png')}
        
        self._widget.enabled_label.setPixmap(self._pixmaps['disabled'])
        #self._widget.enabled_label.setScaledContents(True)

        # Show _widget.windowTitle on left-top of each plugin (when
        # it's set in _widget). This is useful when you open multiple
        # plugins at once. Also if you open multiple instances of your
        # plugin at once, these lines add number to make it easy to
        # tell from pane to pane.
        if context.serial_number() > 1:
            self._widget.setWindowTitle(self._widget.windowTitle() +
                                        (' (%d)' % context.serial_number()))
            

        self.text_list_model = QStandardItemModel()
        self._widget.asr_list.setModel(self.text_list_model)
        self._widget.text_input.returnPressed.connect(self.send_text) 
        
        # Add widget to the user interface
        context.add_widget(self._widget)

        # init and start update timer
        self._timer_refresh_topics = QTimer(self)
        self._timer_refresh_topics.setInterval(10)
        self._timer_refresh_topics.timeout.connect(self.refresh_topics)

        self.start_monitor()

    def send_text(self):
        value = self._widget.text_input.text()
        rospy.loginfo(logger_name="CLFSpeech", msg=f"sending {value}")
        self.publisher.publish(value)
        #label.setText(value) 

    def refresh_topics(self):
        #rospy.loginfo(logger_name="CLFSpeech", msg=f"REFRESH")
        if self.min_amp != self._widget.audio_slider.value():
            self.min_amp = self._widget.audio_slider.value()
            amp_rec = SetFloat32Request()
            amp_rec.data = self.min_amp / 100.0
            res = self.service_set_amp(amp_rec)

        with self.amp_lock:
            self._widget.audo_bar.setValue(int(self.last_amp * 100))

        with self.enabled_lock:
            with self.recording_lock:
                if not self.vad_enabled:
                    self._widget.enabled_label.setPixmap(self._pixmaps['disabled'])
                elif self.recording:
                    self._widget.enabled_label.setPixmap(self._pixmaps['recording'])
                else:
                    self._widget.enabled_label.setPixmap(self._pixmaps['stopped'])

    def start_monitor(self):
        topic = self._audio_ns + "/amp"
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.audio_subscriber = rospy.Subscriber(
                topic, Float32, self.callback_amp)
        
        topic = self._audio_ns + "/enabled"
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.enabled_subscriber = rospy.Subscriber(
                topic, Bool, self.callback_enabled)
        
        topic = self._audio_ns + "/in_rec"
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.rec_subscriber = rospy.Subscriber(
                topic, Bool, self.callback_recording)
        
        topic = self._asr_topic
        rospy.loginfo(logger_name="CLFSpeech", msg=f"subscribe to {topic}")
        self.asr_subscriber = rospy.Subscriber(
                topic, String, self.callback_asr)
        self.publisher = rospy.Publisher(topic, String, queue_size=1)

        topic = self._audio_ns + "/set_min_amp"
        self.service_set_amp = rospy.ServiceProxy(topic, SetFloat32)

        self._timer_refresh_topics.start()

    def callback_asr(self, message):
        self.last_asr = message.data
        rospy.loginfo(logger_name="CLFSpeech", msg=f"asr: {self.last_asr}")
        item = QStandardItem(self.last_asr)
        self.text_list_model.insertRow(0, item)
        self.text_list_model.setRowCount(20)
        #self.text_list_model.appendRow(item)
            
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
        instance_settings.set_value('audio_ns', self._audio_ns)

    def restore_settings(self, plugin_settings, instance_settings):
        # Restore last session
        pass

